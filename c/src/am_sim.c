/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Simulated joint: one plant plus one drive model, in virtual time. A line-by-line port of
 * python/am_sdk/backends/sim.py; the two must agree (python/tests/test_cross_check_c.py).
 *
 * One cycle = one drive cycle. The drive computes its command from the feedback sample of the end
 * of the previous cycle (one-cycle delay, as with a bus master), then the plant runs for the cycle
 * with that torque held. Everything here is a model of design targets, not measured hardware.
 */
#include "am_internal.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#define MOTOR_BITS 19
#define OUTPUT_BITS 14
#define PLANT_DT_S 1.0e-4
#define BOOT_CYCLES 3
#define REF_QUALIFY_CYCLES 6
#define T_TRIP_C 130.0
#define T_DERATE_C 105.0
#define TRIP_HYST_K 5.0
#define QUICK_STOP_DONE_RAD_S 0.02
#define ENVELOPE_HZ 60.0
#define FAMILY_WINDING_RISE_K 80.0
#define PEAK_RECHARGE_S_PER_S 0.1
#define PEAK_REARM_S 0.5
#define BUS_SOURCE_R_OHM 0.05

#define FAULT_VBUS 0x1008u
#define FAULT_MOTOR_OVERTEMP 0x2000u
#define FAULT_COMMS_LOST 0x3000u

#define SW_VOLTAGE 0x0010u
#define SW_WARNING 0x0080u

/* ---------------------------------------------------------------- small helpers */
uint32_t am__crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t c = 0xFFFFFFFFu;
    size_t i;
    int k;
    for (i = 0; i < len; i++) {
        c ^= p[i];
        for (k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return c ^ 0xFFFFFFFFu;
}

void am__copy_str(char *dst, size_t cap, const char *src)
{
    size_t n;
    if (!dst || cap == 0) return;
    n = src ? strlen(src) : 0;
    if (n >= cap) n = cap - 1;
    if (n) memcpy(dst, src, n);
    dst[n] = '\0';
}

const am_unit_data *am__unit_find(const char *name, size_t len)
{
    char buf[8];
    size_t i, k = 0;
    while (len && isspace((unsigned char)*name)) { name++; len--; }
    while (len && isspace((unsigned char)name[len - 1])) len--;
    if (len == 0 || len >= sizeof buf) return NULL;
    for (i = 0; i < len; i++) {
        char ch = (char)toupper((unsigned char)name[i]);
        buf[k++] = ch == '_' ? '-' : ch;
    }
    buf[k] = '\0';
    for (i = 0; i < AM_UNIT_COUNT; i++)
        if (strcmp(am_unit_table[i].unit, buf) == 0) return &am_unit_table[i];
    return NULL;
}

size_t am__sim_parse_units(const char *spec, const am_unit_data **units, size_t cap)
{
    const char *p;
    size_t n = 0;
    if (!spec || strncmp(spec, "sim", 3) != 0) return 0;
    if (spec[3] == '\0') {
        if (cap < 1) return 0;
        units[0] = am__unit_find("M", 1);
        return 1;
    }
    if (spec[3] != ':') return 0;
    p = spec + 4;
    while (*p) {
        const char *e = p;
        while (*e && *e != ',' && *e != ';') e++;
        if (e > p) {
            size_t len = (size_t)(e - p), i;
            bool blank = true;
            for (i = 0; i < len; i++) if (!isspace((unsigned char)p[i])) blank = false;
            if (!blank) {
                const am_unit_data *u = am__unit_find(p, len);
                if (!u || n >= cap) return 0;
                units[n++] = u;
            }
        }
        p = *e ? e + 1 : e;
    }
    if (n == 0 && cap >= 1) units[n++] = am__unit_find("M", 1);
    return n;
}

static double clampd(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }

/* Python float modulo: result has the sign of the divisor. */
static double pymod(double x, double y)
{
    double m = fmod(x, y);
    if (m != 0.0 && ((y < 0.0) != (m < 0.0))) m += y;
    return m;
}

/* ---------------------------------------------------------------- unit model (torque_limit, ThermalState) */
static bool torque_limit(const am_curve *c, double speed_rad_s, double *out)
{
    double rpm = fabs(speed_rad_s) * (60.0 / (2.0 * 3.141592653589793));
    int i;
    if (c->n <= 0) return false;
    if (rpm < c->p[0].rpm - 1e-6 || rpm > c->p[c->n - 1].rpm + 1e-6) return false;
    for (i = 0; i + 1 < c->n; i++) {
        double r0 = c->p[i].rpm, t0 = c->p[i].nm, r1 = c->p[i + 1].rpm, t1 = c->p[i + 1].nm;
        if (r0 - 1e-6 <= rpm && rpm <= r1 + 1e-6) {
            *out = (r1 == r0) ? t0 : t0 + (t1 - t0) * (rpm - r0) / (r1 - r0);
            return true;
        }
    }
    return false;
}

/* ---------------------------------------------------------------- events and faults */
static void sim_event(am_sim_joint *sj, am_event_kind kind, uint16_t code)
{
    if (sj->n_ev < AM_SIM_EVQ) {
        sj->ev[sj->n_ev].kind = kind;
        sj->ev[sj->n_ev].code = code;
        sj->ev[sj->n_ev].t_ns = sj->t_ns;
        sj->n_ev++;
    }
}

bool am__sim_pop_event(am_sim_joint *sj, am_sim_ev *out)
{
    if (sj->n_ev == 0) return false;
    *out = sj->ev[0];
    memmove(sj->ev, sj->ev + 1, (sj->n_ev - 1) * sizeof sj->ev[0]);
    sj->n_ev--;
    return true;
}

static int fault_index(const am_sim_joint *sj, uint16_t code)
{
    size_t i;
    for (i = 0; i < sj->n_faults; i++)
        if (sj->faults[i].code == code) return (int)i;
    return -1;
}

static double t_winding(const am_sim_joint *sj) { return sj->ambient + FAMILY_WINDING_RISE_K * sj->thermal_x; }
static double t_ps(const am_sim_joint *sj) { return sj->ambient + 20.0 * sj->thermal_x; }

static void raise_fault(am_sim_joint *sj, uint16_t code, bool cause_active)
{
    int i = fault_index(sj, code);
    if (i >= 0) {
        sj->faults[i].active = cause_active || sj->faults[i].active;
        return;
    }
    if (sj->n_faults < AM_SIM_MAX_FAULTS) {
        sj->faults[sj->n_faults].code = code;
        sj->faults[sj->n_faults].active = cause_active;
        sj->n_faults++;
    }
    if (!sj->first_fault) sj->first_fault = code;
    sj->error_code = code;
    memmove(sj->history + 1, sj->history, (AM_SIM_HIST - 1) * sizeof sj->history[0]);
    sj->history[0].code = code;
    sj->history[0].drive_state = (uint8_t)sj->state;
    sj->history[0].t_ns = sj->t_ns;
    sj->history[0].vbus_V = (float)sj->vbus;
    sj->history[0].iq_A = (float)sj->iq_meas;
    sj->history[0].t_winding_C = (float)t_winding(sj);
    sj->history[0].t_power_stage_C = (float)t_ps(sj);
    if (sj->n_history < AM_SIM_HIST) sj->n_history++;
    sim_event(sj, AM_EV_FAULT, code);
}

static bool any_fault_active(const am_sim_joint *sj)
{
    size_t i;
    for (i = 0; i < sj->n_faults; i++)
        if (sj->faults[i].active) return true;
    return false;
}

static uint32_t fault_word(const am_sim_joint *sj)
{
    uint32_t fw = 0;
    size_t i;
    for (i = 0; i < sj->n_faults; i++) {
        uint16_t c = sj->faults[i].code;
        if (c >= 0x1000u && c < 0x1020u) fw |= 1u << (c - 0x1000u);
    }
    return fw;
}

/* ---------------------------------------------------------------- setup */
static void sim_defaults(am_sim_joint *sj)
{
    const am_unit_data *ud = sj->ud;
    double wn = AM_TWO_PI * ENVELOPE_HZ, jt = ud->armature_kgm2;
    sj->ceiling.torque_pos_Nm = ud->peak_nm;
    sj->ceiling.torque_neg_Nm = ud->peak_nm;
    sj->ceiling.current_A = ud->peak_nm / (ud->kt * ud->ratio * ud->gear_eta);
    sj->ceiling.speed_rad_s = ud->no_load_48;
    sj->ceiling.pos_min_rad = -INFINITY;
    sj->ceiling.pos_max_rad = INFINITY;
    sj->ceiling.kp_max_Nm_per_rad = ud->armature_kgm2 * wn * wn;
    sj->ceiling.kd_max_Nm_s_per_rad = 2.0 * ud->armature_kgm2 * wn;
    sj->ceiling.comms_loss_ms = 1000;
    sj->limits = sj->ceiling;
    sj->limits.comms_loss_ms = 100;
    sj->gains.pos_kp_1_per_s = 30.0;
    sj->gains.vel_kp_Nm_s_per_rad = jt * AM_TWO_PI * 50.0;
    sj->gains.vel_ki_Nm_per_rad = jt * AM_TWO_PI * 50.0 * AM_TWO_PI * 8.0;
    sj->gains.vel_filter_hz = 300.0;
    sj->gains.torque_cmd_filter_hz = 0.0;
    sj->gains.notch_hz = 0.0;
    sj->gains.notch_q = 1.0;
    memset(&sj->gain_ceiling, 0, sizeof sj->gain_ceiling);
    sj->gain_ceiling.pos_kp_1_per_s = 200.0;
    sj->gain_ceiling.vel_kp_Nm_s_per_rad = jt * AM_TWO_PI * 100.0;
    sj->gain_ceiling.vel_ki_Nm_per_rad = jt * AM_TWO_PI * 100.0 * AM_TWO_PI * 20.0;
    sj->gain_ceiling.vel_filter_hz = 1000.0;
    sj->min_filter_hz = 50.0;
}

void am__sim_lim_to_api(const am_sim_lim *in, am_limits *out)
{
    out->torque_pos_Nm = (float)in->torque_pos_Nm;
    out->torque_neg_Nm = (float)in->torque_neg_Nm;
    out->current_A = (float)in->current_A;
    out->speed_rad_s = (float)in->speed_rad_s;
    out->pos_min_rad = in->pos_min_rad;
    out->pos_max_rad = in->pos_max_rad;
    out->kp_max_Nm_per_rad = (float)in->kp_max_Nm_per_rad;
    out->kd_max_Nm_s_per_rad = (float)in->kd_max_Nm_s_per_rad;
    out->comms_loss_ms = in->comms_loss_ms;
}

void am__sim_lim_from_api(const am_limits *in, am_sim_lim *out)
{
    out->torque_pos_Nm = in->torque_pos_Nm;
    out->torque_neg_Nm = in->torque_neg_Nm;
    out->current_A = in->current_A;
    out->speed_rad_s = in->speed_rad_s;
    out->pos_min_rad = in->pos_min_rad;
    out->pos_max_rad = in->pos_max_rad;
    out->kp_max_Nm_per_rad = in->kp_max_Nm_per_rad;
    out->kd_max_Nm_s_per_rad = in->kd_max_Nm_s_per_rad;
    out->comms_loss_ms = in->comms_loss_ms;
}

void am__sim_gain_to_api(const am_sim_gain *in, am_loop_gains *out)
{
    out->pos_kp_1_per_s = (float)in->pos_kp_1_per_s;
    out->vel_kp_Nm_s_per_rad = (float)in->vel_kp_Nm_s_per_rad;
    out->vel_ki_Nm_per_rad = (float)in->vel_ki_Nm_per_rad;
    out->vel_filter_hz = (float)in->vel_filter_hz;
    out->torque_cmd_filter_hz = (float)in->torque_cmd_filter_hz;
    out->notch_hz = (float)in->notch_hz;
    out->notch_q = (float)in->notch_q;
}

void am__sim_gain_from_api(const am_loop_gains *in, am_sim_gain *out)
{
    out->pos_kp_1_per_s = in->pos_kp_1_per_s;
    out->vel_kp_Nm_s_per_rad = in->vel_kp_Nm_s_per_rad;
    out->vel_ki_Nm_per_rad = in->vel_ki_Nm_per_rad;
    out->vel_filter_hz = in->vel_filter_hz;
    out->torque_cmd_filter_hz = in->torque_cmd_filter_hz;
    out->notch_hz = in->notch_hz;
    out->notch_q = in->notch_q;
}

static void quantise_position(am_sim_joint *sj, double q)
{
    double ratio = sj->ud->ratio, turn;
    int64_t enc;
    sj->counts = (int64_t)am__round_even(q * ratio / AM_TWO_PI * (double)(1 << MOTOR_BITS));
    sj->q_meas = (double)sj->counts * AM_TWO_PI / (double)(1 << MOTOR_BITS) / ratio;
    turn = pymod(q, AM_TWO_PI) / AM_TWO_PI;
    enc = (int64_t)(turn * (double)(1 << OUTPUT_BITS));
    enc %= (1 << OUTPUT_BITS);
    sj->enc_out = (double)enc * AM_TWO_PI / (double)(1 << OUTPUT_BITS);
}

static void sample_state(am_sim_joint *sj, double q)
{
    double raw, a, fhz = sj->gains.vel_filter_hz;
    sj->q_prev = sj->q_meas;
    quantise_position(sj, q);
    raw = (sj->q_meas - sj->q_prev) / sj->dt;
    a = 1.0 - exp(-AM_TWO_PI * (fhz > 1.0 ? fhz : 1.0) * sj->dt);
    sj->qd_f += a * (raw - sj->qd_f);
}

static void sample_init(am_sim_joint *sj)
{
    quantise_position(sj, sj->external_plant ? sj->ext_q : sj->q);
    sj->qd_f = 0.0;
    sj->q_prev = sj->q_meas;
    sj->iq_meas = 0.0;
    sj->vbus = sj->vbus_in;
}

void am__sim_power_cycle(am_sim_joint *sj)
{
    const am_sim_options *o = &sj->opts;
    sj->state = AM_DS_NOT_READY_TO_SWITCH_ON;
    sj->boot_count = 0;
    sj->ref_cnt = 0;
    sj->ref_state = AM_REF_INVALID;
    sj->thermal_tau = sj->ud->thermal_tau_s;
    sj->thermal_cont = sj->ud->rated_nm;
    sj->thermal_x = o->thermal_x0;
    sj->ambient = o->ambient_C;
    sj->t_magnet = sj->ambient + 0.6 * FAMILY_WINDING_RISE_K * sj->thermal_x;
    sj->budget = o->peak_budget_s;
    sj->lockout = false;
    sj->bus_nominal = o->bus_V > 42.0 ? 48.0 : 36.0;
    sj->vbus_in = o->bus_V;
    sj->n_faults = 0;
    sj->first_fault = 0;
    sj->error_code = 0;
    sj->warn = 0;
    sj->prev_cw = 0;
    sj->integ = 0.0;
    sj->have_cmd = false;
    memset(&sj->cmd, 0, sizeof sj->cmd);
    sj->hold_q = 0.0;
    sj->mode = AM_MODE_NONE;
    sj->tau_applied = 0.0;
    sj->tau_filt = 0.0;
    sj->limit_active = false;
    sj->authority = sj->authority_cfg;
    if (!sj->plant_init) {
        sj->q = o->q0_rad;
        sj->qd = 0.0;
        sj->plant_init = true;
    }
    sj->ext_q = sj->q;
    sj->ext_qd = sj->qd;
    sample_init(sj);
    sj->qs_active = false;
}

void am__sim_init(am_sim_joint *sj, const am_unit_data *ud, uint16_t address, am_bus cfg_bus, uint32_t cycle_us,
                  bool extended, const am_sim_options *o)
{
    long n;
    memset(sj, 0, sizeof *sj);
    sj->ud = ud;
    sj->address = address;
    if (o) sj->opts = *o; else am_sim_options_default(&sj->opts);
    sj->dt = cycle_us * 1e-6;
    n = (long)am__round_even(sj->dt / PLANT_DT_S);
    sj->n_sub = (int)(n < 1 ? 1 : n > 20 ? 20 : n);
    sj->cfg_bus = cfg_bus;
    sj->extended = extended;
    sj->can_sync = cfg_bus == AM_BUS_CANFD;
    sj->authority = sj->opts.authority_bus != AM_BUS_NONE ? sj->opts.authority_bus : cfg_bus;
    sj->authority_cfg = sj->authority;
    sim_defaults(sj);
    am__sim_power_cycle(sj);
}

/* ---------------------------------------------------------------- envelope */
static double x_limit(const am_sim_joint *sj)
{
    double v = (T_DERATE_C - sj->ambient) / FAMILY_WINDING_RISE_K;
    return v > 1e-3 ? v : 1e-3;
}

static double derate(const am_sim_joint *sj)
{
    double x = sj->thermal_x, xl = x_limit(sj);
    return x <= xl ? 1.0 : 1.0 / sqrt(x / xl);
}

static double peak_headroom_x(const am_sim_joint *sj)
{
    const am_unit_data *ud = sj->ud;
    double r = ud->peak_nm / ud->rated_nm;
    return r * r * (1.0 - exp(-sj->opts.peak_budget_s / ud->thermal_tau_s));
}

static bool peak_available(const am_sim_joint *sj)
{
    return sj->budget > 0.0 && !sj->lockout && sj->thermal_x <= x_limit(sj) - peak_headroom_x(sj);
}

static double curve(const am_sim_joint *sj, double w, int kind)
{
    double v;
    if (!torque_limit(&sj->ud->curve[sj->bus_nominal > 42.0 ? 0 : 1][kind], w, &v)) return 0.0;
    return v;
}

/* peak: -1 = automatic, 0 = continuous, 1 = peak */
static void envelope_at(const am_sim_joint *sj, double w, int peak, double *pos, double *neg)
{
    int use_peak = peak < 0 ? (peak_available(sj) ? 1 : 0) : peak;
    int kind = use_peak ? 0 : 1;
    double d = derate(sj);
    double motoring = curve(sj, w, kind) * d;
    double braking = curve(sj, 0.0, kind) * d;
    if (fabs(w) < 1e-6) {
        *pos = braking;
        *neg = braking;
    } else if (w > 0) {
        *pos = motoring;
        *neg = braking;
    } else {
        *pos = braking;
        *neg = motoring;
    }
}

static double clamp_torque(am_sim_joint *sj, double tau, double w, double q)
{
    double pos, neg, out, kt_tot, icap;
    const am_sim_lim *lim = &sj->limits;
    envelope_at(sj, w, -1, &pos, &neg);
    kt_tot = sj->ud->kt * sj->ud->ratio * sj->ud->gear_eta;
    icap = lim->current_A * kt_tot;
    if (lim->torque_pos_Nm < pos) pos = lim->torque_pos_Nm;
    if (icap < pos) pos = icap;
    if (lim->torque_neg_Nm < neg) neg = lim->torque_neg_Nm;
    if (icap < neg) neg = icap;
    out = clampd(tau, -neg, pos);
    sj->warn = (uint16_t)(sj->warn & ~AM_WARN_SPEED_LIMIT);
    if (fabs(w) > lim->speed_rad_s && out * w > 0.0) {
        out = 0.0;
        sj->warn |= AM_WARN_SPEED_LIMIT;
    }
    if ((q <= lim->pos_min_rad && out < 0.0) || (q >= lim->pos_max_rad && out > 0.0)) out = 0.0;
    return out;
}

/* ---------------------------------------------------------------- state machine */
uint16_t am__sim_statusword(const am_sim_joint *sj)
{
    uint16_t sw = am__state_to_sw(sj->state);
    if (sj->state == AM_DS_READY_TO_SWITCH_ON || sj->state == AM_DS_SWITCHED_ON ||
        sj->state == AM_DS_OPERATION_ENABLED || sj->state == AM_DS_QUICK_STOP_ACTIVE)
        sw |= SW_VOLTAGE;
    if (sj->warn) sw |= SW_WARNING;
    if (sj->authority == sj->cfg_bus) sw |= AM_SW_REMOTE;
    if (sj->limit_active) sw |= AM_SW_INTERNAL_LIMIT;
    if (sj->ref_state == AM_REF_VALID) sw |= AM_SW_REF_VALID;
    if (sj->sto_active) sw |= AM_SW_STO;
    return sw;
}

static void set_state(am_sim_joint *sj, am_drive_state st)
{
    if (st != sj->state) {
        if (sj->state == AM_DS_OPERATION_ENABLED) sj->tau_applied = 0.0;
        sj->state = st;
        if (st == AM_DS_OPERATION_ENABLED) {
            sj->have_cmd = false;
            sj->integ = 0.0;
            sj->hold_q = sj->q_meas;
        }
        sim_event(sj, AM_EV_STATE_CHANGED, (uint16_t)st);
    }
}

static void state_machine(am_sim_joint *sj, uint16_t cw)
{
    am_drive_state st = sj->state;
    bool rising_reset = (cw & AM_CW_FAULT_RESET) && !(sj->prev_cw & AM_CW_FAULT_RESET);
    bool disable_voltage, quick_stop, shutdown, switch_on, enable_op;
    sj->prev_cw = cw;
    if (st == AM_DS_NOT_READY_TO_SWITCH_ON) {
        sj->boot_count++;
        if (sj->boot_count >= BOOT_CYCLES) set_state(sj, AM_DS_SWITCH_ON_DISABLED);
        return;
    }
    if (sj->n_faults && st != AM_DS_FAULT && st != AM_DS_FAULT_REACTION_ACTIVE) {
        set_state(sj, AM_DS_FAULT_REACTION_ACTIVE);
        return;
    }
    if (st == AM_DS_FAULT_REACTION_ACTIVE) {
        set_state(sj, AM_DS_FAULT);
        return;
    }
    if (st == AM_DS_FAULT) {
        if (rising_reset && !any_fault_active(sj)) {
            sj->n_faults = 0;
            sj->first_fault = 0;
            sj->error_code = 0;
            sj->ref_cnt = 0;
            sj->ref_state = AM_REF_INVALID;
            sj->integ = 0.0;
            sim_event(sj, AM_EV_FAULT_CLEARED, 0);
            set_state(sj, AM_DS_SWITCH_ON_DISABLED);
        }
        return;
    }
    if (sj->sto_active) {
        if (st != AM_DS_SWITCH_ON_DISABLED) set_state(sj, AM_DS_SWITCH_ON_DISABLED);
        sj->qs_active = false;
        return;
    }
    if (sj->ref_state != AM_REF_VALID) {
        sj->ref_cnt++;
        sj->ref_state = sj->ref_cnt >= REF_QUALIFY_CYCLES ? AM_REF_VALID : AM_REF_CANDIDATE;
    }
    if (st == AM_DS_QUICK_STOP_ACTIVE) {
        if (fabs(sj->qd_f) < QUICK_STOP_DONE_RAD_S) {
            sj->qs_active = false;
            set_state(sj, AM_DS_SWITCH_ON_DISABLED);
        }
        return;
    }
    disable_voltage = (cw & 0x82) == 0x00;
    quick_stop = (cw & 0x06) == 0x02;
    shutdown = (cw & 0x87) == 0x06;
    switch_on = (cw & 0x8F) == 0x07;
    enable_op = (cw & 0x8F) == 0x0F;
    if (disable_voltage) {
        if (st != AM_DS_SWITCH_ON_DISABLED) set_state(sj, AM_DS_SWITCH_ON_DISABLED);
        return;
    }
    if (quick_stop && (st == AM_DS_OPERATION_ENABLED || st == AM_DS_SWITCHED_ON || st == AM_DS_READY_TO_SWITCH_ON)) {
        if (st == AM_DS_OPERATION_ENABLED) {
            sj->qs_active = true;
            set_state(sj, AM_DS_QUICK_STOP_ACTIVE);
        } else {
            set_state(sj, AM_DS_SWITCH_ON_DISABLED);
        }
        return;
    }
    if (st == AM_DS_SWITCH_ON_DISABLED) {
        if (shutdown) set_state(sj, AM_DS_READY_TO_SWITCH_ON);
    } else if (st == AM_DS_READY_TO_SWITCH_ON) {
        if (switch_on || enable_op) set_state(sj, AM_DS_SWITCHED_ON);
    } else if (st == AM_DS_SWITCHED_ON) {
        if (shutdown) set_state(sj, AM_DS_READY_TO_SWITCH_ON);
        else if (enable_op && sj->ref_state == AM_REF_VALID) set_state(sj, AM_DS_OPERATION_ENABLED);
    } else if (st == AM_DS_OPERATION_ENABLED) {
        if (shutdown || switch_on) set_state(sj, AM_DS_READY_TO_SWITCH_ON);
    }
}

/* ---------------------------------------------------------------- control laws */
static double control(am_sim_joint *sj, am_mode mode, const am_command *fresh)
{
    const am_command *c;
    double q, qd, dt;
    const am_sim_gain *g = &sj->gains;
    if (sj->state == AM_DS_QUICK_STOP_ACTIVE) return g->vel_kp_Nm_s_per_rad * (0.0 - sj->qd_f);
    if (sj->state != AM_DS_OPERATION_ENABLED) return 0.0;
    if (fresh) {
        sj->cmd = *fresh;
        sj->have_cmd = true;
    }
    c = &sj->cmd;
    q = sj->q_meas;
    qd = sj->qd_f;
    dt = sj->dt;
    if (mode == AM_MODE_CST) return sj->have_cmd ? (double)c->tau_ff_Nm : 0.0;
    if (mode == AM_MODE_CURRENT) {
        double kt = sj->ud->kt * sj->ud->ratio * sj->ud->gear_eta;
        return sj->have_cmd ? c->iq_ref_A * kt : 0.0;
    }
    if (mode == AM_MODE_IMPEDANCE) {
        if (!sj->have_cmd) return 0.0;
        return c->kp_Nm_per_rad * (c->q_ref_rad - q) + c->kd_Nm_s_per_rad * (c->qd_ref_rad_s - qd) + c->tau_ff_Nm;
    }
    if (mode == AM_MODE_CSP || mode == AM_MODE_CSV) {
        double qd_cmd, err, tau_ff;
        if (mode == AM_MODE_CSP) {
            double q_ref = sj->have_cmd ? c->q_ref_rad : sj->hold_q;
            qd_cmd = g->pos_kp_1_per_s * (q_ref - q) + (sj->have_cmd ? (double)c->qd_ref_rad_s : 0.0);
        } else {
            qd_cmd = sj->have_cmd ? (double)c->qd_ref_rad_s : 0.0;
        }
        qd_cmd = clampd(qd_cmd, -sj->limits.speed_rad_s, sj->limits.speed_rad_s);
        err = qd_cmd - qd;
        sj->last_err = err;
        sj->integ += err * dt;
        tau_ff = sj->have_cmd ? (double)c->tau_ff_Nm : 0.0;
        return g->vel_kp_Nm_s_per_rad * err + g->vel_ki_Nm_per_rad * sj->integ + tau_ff;
    }
    return 0.0;
}

/* ---------------------------------------------------------------- plant */
static double external_torque(am_sim_joint *sj, double t)
{
    return sj->ext_fn ? sj->ext_fn(t, sj->q, sj->qd, sj->ext_user) : 0.0;
}

static void integrate(am_sim_joint *sj, double tau_motor)
{
    double J = sj->ud->armature_kgm2 + sj->opts.load_inertia_kgm2;
    double F = sj->ud->backdrive_nm * sj->opts.friction_scale;
    double h = sj->dt / sj->n_sub;
    int k;
    for (k = 0; k < sj->n_sub; k++) {
        double t = (double)sj->t_ns * 1e-9 + k * h;
        double tau_t = tau_motor + external_torque(sj, t);
        double v = sj->qd, s, v_new;
        if (v == 0.0 && fabs(tau_t) <= F) continue;
        s = v != 0.0 ? (v > 0 ? 1.0 : -1.0) : (tau_t > 0 ? 1.0 : -1.0);
        v_new = v + h * (tau_t - F * s) / J;
        if (v != 0.0 && v * v_new < 0.0 && fabs(tau_t) <= F) v_new = 0.0;
        sj->qd = v_new;
        sj->q += v_new * h;
    }
}

/* ---------------------------------------------------------------- one cycle */
static void budget_update(am_sim_joint *sj, double tau)
{
    double pos, neg, cont;
    envelope_at(sj, sj->qd_f, 0, &pos, &neg);
    cont = tau >= 0 ? pos : neg;
    if (fabs(tau) > cont * 1.0001 && fabs(tau) > 1e-9) {
        sj->budget = sj->budget - sj->dt > 0.0 ? sj->budget - sj->dt : 0.0;
        if (sj->budget <= 0.0) sj->lockout = true;
    } else {
        double b = sj->budget + PEAK_RECHARGE_S_PER_S * sj->dt;
        sj->budget = b < sj->opts.peak_budget_s ? b : sj->opts.peak_budget_s;
    }
    if (sj->lockout && sj->budget >= PEAK_REARM_S) sj->lockout = false;
    if (sj->lockout && sj->state == AM_DS_OPERATION_ENABLED) sj->warn |= AM_WARN_PEAK_BUDGET_EMPTY;
    else sj->warn = (uint16_t)(sj->warn & ~AM_WARN_PEAK_BUDGET_EMPTY);
}

static void thermal_and_faults(am_sim_joint *sj, double tau)
{
    bool was = (sj->warn & AM_WARN_THERMAL_DERATE) != 0;
    double target = (tau / sj->thermal_cont) * (tau / sj->thermal_cont);
    double kt, ibus, vlo, vhi;
    int i;
    sj->thermal_x = target + (sj->thermal_x - target) * exp(-sj->dt / sj->thermal_tau);
    sj->t_magnet += (sj->ambient + 0.6 * (t_winding(sj) - sj->ambient) - sj->t_magnet) * sj->dt / 600.0;
    if (sj->thermal_x > x_limit(sj)) {
        sj->warn |= AM_WARN_THERMAL_DERATE;
        if (!was) sim_event(sj, AM_EV_WARNING, FAULT_MOTOR_OVERTEMP);
    } else {
        sj->warn = (uint16_t)(sj->warn & ~AM_WARN_THERMAL_DERATE);
    }
    i = fault_index(sj, FAULT_MOTOR_OVERTEMP);
    if (t_winding(sj) >= T_TRIP_C) raise_fault(sj, FAULT_MOTOR_OVERTEMP, true);
    else if (i >= 0 && t_winding(sj) < T_TRIP_C - TRIP_HYST_K) sj->faults[i].active = false;
    kt = sj->ud->kt * sj->ud->ratio * sj->ud->gear_eta;
    sj->iq_meas = am__round_even(tau / kt * 1000.0) / 1000.0;
    ibus = fabs(tau * sj->qd_f) / (sj->vbus_in > 1.0 ? sj->vbus_in : 1.0);
    sj->vbus = sj->vbus_in - BUS_SOURCE_R_OHM * ibus;
    sj->ibus = ibus;
    vlo = 0.75 * sj->bus_nominal;
    vhi = 1.25 * sj->bus_nominal;
    i = fault_index(sj, FAULT_VBUS);
    if (!(vlo <= sj->vbus && sj->vbus <= vhi)) raise_fault(sj, FAULT_VBUS, true);
    else if (i >= 0) sj->faults[i].active = false;
}

static void make_telemetry(const am_sim_joint *sj, am_telemetry *t)
{
    const am_unit_data *ud = sj->ud;
    double kt = ud->kt, wm = sj->qd_f * ud->ratio, iq = sj->iq_meas;
    uint8_t sio = 0;
    memset(t, 0, sizeof *t);
    if (!sj->sto_active) {
        sio = AM_SIO_STO1 | AM_SIO_STO2 | AM_SIO_VM_GD_PRESENT;
        if (sj->state == AM_DS_OPERATION_ENABLED) sio |= AM_SIO_ENABLE_READBACK;
    }
    t->t_ns = sj->t_ns;
    t->sample_counter = sj->sample_counter;
    t->sync_flags = (uint16_t)((sj->can_sync ? AM_SYNC_CAN_LOCKED : AM_SYNC_DC_LOCKED) | AM_SYNC_TIMESTAMP_VALID);
    t->state = sj->state;
    t->mode = sj->mode;
    t->q_rad = sj->q_meas;
    t->qd_rad_s = (float)sj->qd_f;
    t->q_motor_counts = (int32_t)sj->counts;
    t->qd_motor_rad_s = (float)wm;
    t->q_out_encoder_rad = (float)sj->enc_out;
    t->iq_A = (float)iq;
    t->iq_ref_A = (float)(sj->tau_applied / (kt * ud->ratio * ud->gear_eta));
    t->vq_V = (float)(kt * wm + 0.05 * iq);
    t->vbus_V = (float)sj->vbus;
    t->ibus_est_A = (float)sj->ibus;
    t->tau_est_Nm = (float)(iq * kt * ud->ratio * ud->gear_eta);
    t->t_winding_C = (float)t_winding(sj);
    t->t_magnet_C = (float)sj->t_magnet;
    t->t_power_stage_C = (float)t_ps(sj);
    t->t_board_C = (float)(sj->ambient + 8.0 * sj->thermal_x);
    t->fault_word = fault_word(sj);
    t->error_code = sj->error_code;
    t->warning_word = sj->warn;
    t->safety_io = sio;
    t->ref_state = sj->ref_state;
    t->sto_active = sj->sto_active;
    t->internal_limit = sj->limit_active;
    t->extended_valid = sj->extended;
    t->statusword = am__sim_statusword(sj);
    if (!sj->extended) {
        t->id_A = t->vd_V = t->vq_V = t->ibus_est_A = t->twist_rad = 0.0f;
        t->t_magnet_C = t->t_power_stage_C = t->t_board_C = 0.0f;
        t->q_motor_counts = 0;
        t->qd_motor_rad_s = 0.0f;
    }
}

void am__sim_exchange(am_sim_joint *sj, uint16_t cw, am_mode mode, const am_command *cmd, bool fresh,
                      am_telemetry *out)
{
    bool authority_ok;
    double tau_raw, tau;
    am_command fresh_cmd;
    sj->t_ns += (uint64_t)am__round_even(sj->dt * 1e9);
    sj->sample_counter = (uint16_t)(sj->sample_counter + 1u);
    if (mode != sj->mode) {
        sj->integ = 0.0;
        sj->have_cmd = false;
        sj->hold_q = sj->q_meas;
    }
    sj->mode = mode;
    authority_ok = sj->authority == sj->cfg_bus;
    state_machine(sj, cw);
    if (fresh && authority_ok && cmd) fresh_cmd = *cmd;
    tau_raw = control(sj, mode, (fresh && authority_ok && cmd) ? &fresh_cmd : NULL);
    if (sj->state == AM_DS_OPERATION_ENABLED || sj->state == AM_DS_QUICK_STOP_ACTIVE) {
        double fc = sj->gains.torque_cmd_filter_hz;
        if (fc > 0.0) {
            double a = 1.0 - exp(-AM_TWO_PI * fc * sj->dt);
            sj->tau_filt += a * (tau_raw - sj->tau_filt);
            tau_raw = sj->tau_filt;
        } else {
            sj->tau_filt = tau_raw;
        }
        tau = clamp_torque(sj, tau_raw, sj->qd_f, sj->q_meas);
        sj->limit_active = fabs(tau_raw) > fabs(tau) + 1e-9 || derate(sj) < 1.0;
        if (fabs(tau_raw) > fabs(tau) + 1e-9 && sj->state == AM_DS_OPERATION_ENABLED) {
            if ((mode == AM_MODE_CSP || mode == AM_MODE_CSV) && sj->last_err * tau_raw > 0)
                sj->integ -= sj->last_err * sj->dt;
        }
    } else {
        tau = 0.0;
        sj->tau_filt = 0.0;
        sj->limit_active = false;
        sj->integ = 0.0;
    }
    sj->tau_applied = tau;
    budget_update(sj, tau);
    thermal_and_faults(sj, tau);
    if (sj->state != AM_DS_OPERATION_ENABLED && sj->state != AM_DS_QUICK_STOP_ACTIVE) {
        sj->tau_applied = 0.0;
        tau = 0.0;
    }
    if (!sj->external_plant) {
        integrate(sj, tau);
        sample_state(sj, sj->q);
    }
    make_telemetry(sj, out);
}

/* ---------------------------------------------------------------- identity, limits, gains */
void am__sim_identity(const am_sim_joint *sj, am_identity *out)
{
    const am_unit_data *ud = sj->ud;
    char tmp[32];
    size_t i, k = 0;
    memset(out, 0, sizeof *out);
    out->vendor_id = 0;
    out->sku = ud->sku;
    out->revision = (1u << 16) | AM_OD_MAJOR_SUPPORTED;
    out->serial = 1000u + sj->address;
    for (i = 0; ud->unit[i] && k < sizeof tmp - 1; i++)
        if (ud->unit[i] != '-') tmp[k++] = ud->unit[i];
    tmp[k] = '\0';
    {
        char uid[48]; /* "SIM-" + 31-char unit + "-" + up to 5 digits + NUL fits */
        snprintf(uid, sizeof uid, "SIM-%s-%02u", tmp, (unsigned)sj->address);
        k = strlen(uid);
        memcpy(out->uid, uid, k < 12 ? k : 12);
    }
    out->od_version = AM_VERSION_PACK(0, 1, 0);
    out->capabilities = AM_CAP_OUTPUT_ABS_ENCODER | AM_CAP_WINDING_SENSOR | AM_CAP_ETHERCAT | AM_CAP_CANFD_CONTROL |
                        AM_CAP_STO | AM_CAP_MODE_CURRENT;
    out->cal_record_version = 0;
    {
        char key[32];
        snprintf(key, sizeof key, "SIM-CAL-%s", ud->unit);
        out->cal_record_crc32 = am__crc32(key, strlen(key));
    }
    out->ratio_num = ud->ratio_num;
    out->ratio_den = ud->ratio_den;
    out->fw_cm7 = out->fw_cm4 = out->fw_boot = AM_VERSION_PACK(0, 0, 0);
    snprintf(out->device_name, sizeof out->device_name, "AM %s (sim)", ud->unit);
    am__copy_str(out->user_label, sizeof out->user_label, sj->label);
}

void am__sim_transparency(const am_sim_joint *sj, am_transparency *out)
{
    const am_unit_data *ud = sj->ud;
    double w0 = ud->no_load_48;
    out->rated_torque_Nm = (float)ud->rated_nm;
    out->peak_torque_2s_Nm = (float)ud->peak_nm;
    out->rated_speed_rad_s = (float)ud->rated_speed_rad_s;
    out->no_load_speed_48V_rad_s = (float)w0;
    out->reflected_inertia_kgm2 = (float)ud->armature_kgm2;
    out->backdrive_torque_Nm = (float)ud->backdrive_nm;
    out->kinetic_energy_no_load_J = (float)(0.5 * ud->armature_kgm2 * w0 * w0);
    out->stiffness_Nm_per_rad = (float)ud->stiffness;
    out->backlash_rad = (float)ud->backlash;
    out->sto_reaction_time_s = 2.0e-3f;
    out->collision_latency_s = (float)AM_NAN;
    out->basis = AM_BASIS_DESIGN_TARGET;
}

double am__sim_rated_current(const am_sim_joint *sj)
{
    const am_unit_data *ud = sj->ud;
    return ud->rated_nm / (ud->kt * ud->ratio * ud->gear_eta);
}

void am__sim_fault_info(const am_sim_joint *sj, am_fault_info *out)
{
    memset(out, 0, sizeof *out);
    out->error_code = sj->error_code;
    out->fault_word = fault_word(sj);
    out->first_fault = sj->first_fault;
    out->warning_word = sj->warn;
    out->boot_block = 0;
    out->safety_io = sj->sto_active ? 0 : (uint8_t)(AM_SIO_STO1 | AM_SIO_STO2 | AM_SIO_VM_GD_PRESENT);
    out->latched = sj->n_faults > 0;
}

static bool over(double v, double cap) { return (v != v) || v < 0 || v > cap * (1 + 1e-9); }

am_status am__sim_set_limits(am_sim_joint *sj, const am_sim_lim *lim)
{
    const am_sim_lim *c = &sj->ceiling;
    if (over(lim->torque_pos_Nm, c->torque_pos_Nm) || over(lim->torque_neg_Nm, c->torque_neg_Nm) ||
        over(lim->current_A, c->current_A) || over(lim->speed_rad_s, c->speed_rad_s) ||
        over(lim->kp_max_Nm_per_rad, c->kp_max_Nm_per_rad) || over(lim->kd_max_Nm_s_per_rad, c->kd_max_Nm_s_per_rad) ||
        over((double)lim->comms_loss_ms, (double)c->comms_loss_ms)) {
        sj->last_abort = 0x06090031u;
        return AM_ERR_LIMIT;
    }
    if (lim->pos_min_rad > lim->pos_max_rad || lim->pos_min_rad != lim->pos_min_rad ||
        lim->pos_max_rad != lim->pos_max_rad)
        return AM_ERR_PARAM;
    sj->limits = *lim;
    return AM_OK;
}

am_status am__sim_set_loop_gains(am_sim_joint *sj, const am_sim_gain *g)
{
    const am_sim_gain *c = &sj->gain_ceiling;
    double v[3], cap[3];
    int i;
    v[0] = g->pos_kp_1_per_s; cap[0] = c->pos_kp_1_per_s;
    v[1] = g->vel_kp_Nm_s_per_rad; cap[1] = c->vel_kp_Nm_s_per_rad;
    v[2] = g->vel_ki_Nm_per_rad; cap[2] = c->vel_ki_Nm_per_rad;
    for (i = 0; i < 3; i++)
        if (v[i] != v[i] || v[i] < 0 || v[i] > cap[i]) return AM_ERR_LIMIT;
    if (g->vel_filter_hz < sj->min_filter_hz || g->vel_filter_hz > c->vel_filter_hz) return AM_ERR_LIMIT;
    if (g->torque_cmd_filter_hz != 0.0 && g->torque_cmd_filter_hz < sj->min_filter_hz) return AM_ERR_LIMIT;
    sj->gains = *g;
    return AM_OK;
}

am_status am__sim_restore_defaults(am_sim_joint *sj)
{
    if (sj->state == AM_DS_OPERATION_ENABLED) return AM_ERR_STATE;
    sim_defaults(sj);
    sj->label[0] = '\0';
    sj->current_mode_enabled = false;
    return AM_OK;
}

am_status am__sim_set_control_bus(am_sim_joint *sj, am_bus bus)
{
    if (bus != AM_BUS_ETHERCAT && bus != AM_BUS_CANFD && bus != AM_BUS_NONE) return AM_ERR_PARAM;
    if (sj->state != AM_DS_NOT_READY_TO_SWITCH_ON && sj->state != AM_DS_SWITCH_ON_DISABLED) return AM_ERR_STATE;
    sj->authority_cfg = bus;
    return AM_OK;
}

void am__sim_comms_lost(am_sim_joint *sj)
{
    if (sj->state == AM_DS_OPERATION_ENABLED || sj->state == AM_DS_QUICK_STOP_ACTIVE) {
        sj->tau_applied = 0.0;
        set_state(sj, AM_DS_SWITCH_ON_DISABLED);
    }
    sim_event(sj, AM_EV_LINK_LOST, FAULT_COMMS_LOST);
}

/* Shortest decimal that reads back to the same double, as Python repr(). */
static void pyfloat(char *buf, size_t cap, double v)
{
    int p;
    for (p = 1; p <= 17; p++) {
        snprintf(buf, cap, "%.*g", p, v);
        if (strtod(buf, NULL) == v) break;
    }
    if (!strpbrk(buf, ".eEn")) {
        size_t n = strlen(buf);
        if (n + 2 < cap) { buf[n] = '.'; buf[n + 1] = '0'; buf[n + 2] = '\0'; }
    } else {
        char *e = strchr(buf, 'e');     /* Python writes 1e-05, C may write 1e-05 too; keep */
        (void)e;
    }
}

size_t am__sim_calibration(const am_sim_joint *sj, char *buf, size_t cap)
{
    const am_unit_data *ud = sj->ud;
    char a[40], b[40], c[40], d[40], e[40], f[40];
    pyfloat(a, sizeof a, ud->ratio);
    pyfloat(b, sizeof b, ud->armature_kgm2);
    pyfloat(c, sizeof c, ud->backdrive_nm);
    pyfloat(d, sizeof d, ud->kt);
    pyfloat(e, sizeof e, ud->gear_eta);
    pyfloat(f, sizeof f, ud->thermal_tau_s);
    return (size_t)snprintf(buf, cap,
        "{\n \"schema\": \"am_cal_sim_v0\",\n \"unit\": \"%s\",\n \"data_basis\": \"design_target\",\n"
        " \"gear_ratio\": %s,\n \"reflected_inertia_kgm2\": %s,\n \"backdrive_torque_Nm\": %s,\n"
        " \"kt_motor_Nm_per_A\": %s,\n \"gear_efficiency\": %s,\n \"thermal_tau_s\": %s,\n"
        " \"thermal_tau_published\": %s,\n \"motor_encoder_bits\": %d,\n \"output_encoder_bits\": %d\n}",
        ud->unit, a, b, c, d, e, f, ud->thermal_tau_published ? "true" : "false", MOTOR_BITS, OUTPUT_BITS);
}

/* ---------------------------------------------------------------- object dictionary (subset) */
typedef enum { T_U8, T_I8, T_U16, T_I16, T_U32, T_I32, T_F32, T_RAW } od_type;

static int32_t pos_obj(double v)
{
    if (v == -INFINITY) return INT32_MIN;
    if (v == INFINITY) return INT32_MAX;
    return (int32_t)am__round_even(v * 1e6);
}

static uint32_t u32na(double v, double scale)
{
    if (v != v) return 0xFFFFFFFFu;
    return (uint32_t)am__round_even(v * scale);
}

static int32_t pm(double v, double base) { return (int32_t)am__round_even(1000.0 * v / base); }

static am_status put(od_type t, double v, void *buf, size_t cap, size_t *len)
{
    uint8_t tmp[4];
    size_t n = 0;
    switch (t) {
    case T_U8: tmp[0] = (uint8_t)(uint32_t)v; n = 1; break;
    case T_I8: tmp[0] = (uint8_t)(int8_t)v; n = 1; break;
    case T_U16: am__put_u16(tmp, (uint16_t)(uint32_t)v); n = 2; break;
    case T_I16: am__put_u16(tmp, (uint16_t)(int16_t)v); n = 2; break;
    case T_U32: am__put_u32(tmp, (uint32_t)v); n = 4; break;
    case T_I32: am__put_u32(tmp, (uint32_t)(int32_t)v); n = 4; break;
    case T_F32: am__put_f32(tmp, (float)v); n = 4; break;
    case T_RAW: default: return AM_ERR_INTERNAL;
    }
    if (cap < n) return AM_ERR_PARAM;
    memcpy(buf, tmp, n);
    if (len) *len = n;
    return AM_OK;
}

static am_status put_raw(const void *src, size_t n, void *buf, size_t cap, size_t *len)
{
    if (cap < n) return AM_ERR_PARAM;
    if (n) memcpy(buf, src, n);
    if (len) *len = n;
    return AM_OK;
}

static am_status abort_with(am_sim_joint *sj, uint32_t code)
{
    sj->last_abort = code;
    return AM_ERR_SDO_ABORT;
}

#define K(i, s) (((uint32_t)(i) << 8) | (uint32_t)(s))

am_status am__sim_sdo_read(am_sim_joint *sj, uint16_t index, uint8_t sub, void *buf, size_t cap, size_t *len)
{
    const am_unit_data *ud = sj->ud;
    am_identity id;
    am_transparency tr;
    uint8_t raw[24];
    double rated_a = am__sim_rated_current(sj);
    const am_sim_lim *lim = &sj->limits, *c = &sj->ceiling;
    const am_sim_gain *g = &sj->gains, *gc = &sj->gain_ceiling;
    if (!buf) return AM_ERR_PARAM;
    am__sim_identity(sj, &id);
    am__sim_transparency(sj, &tr);
    if (index == 0x2601 && sub >= 1 && sub <= AM_SIM_HIST) {
        const am_fault_record *r;
        if (sub > sj->n_history) return abort_with(sj, 0x06090011u);
        r = &sj->history[sub - 1];
        am__put_u16(raw, r->code);
        raw[2] = r->drive_state;
        raw[3] = 0;
        am__put_u64(raw + 4, r->t_ns);
        am__put_u32(raw + 12, (uint32_t)(int32_t)am__round_even(r->vbus_V * 1e3));
        am__put_u32(raw + 16, (uint32_t)(int32_t)am__round_even(r->iq_A * 1e3));
        am__put_u16(raw + 20, (uint16_t)(int16_t)am__round_even(r->t_winding_C * 10.0));
        am__put_u16(raw + 22, (uint16_t)(int16_t)am__round_even(r->t_power_stage_C * 10.0));
        return put_raw(raw, 24, buf, cap, len);
    }
    switch (K(index, sub)) {
    case K(0x1008, 0): return put_raw(id.device_name, strlen(id.device_name), buf, cap, len);
    case K(0x1010, 1): return put(T_U32, 1, buf, cap, len);
    case K(0x1011, 1): return put(T_U32, 1, buf, cap, len);
    case K(0x1018, 1): return put(T_U32, id.vendor_id, buf, cap, len);
    case K(0x1018, 2): return put(T_U32, id.sku, buf, cap, len);
    case K(0x1018, 3): return put(T_U32, id.revision, buf, cap, len);
    case K(0x1018, 4): return put(T_U32, id.serial, buf, cap, len);
    case K(0x2000, 2): return put(T_U16, id.sku, buf, cap, len);
    case K(0x2000, 5): return put_raw(id.uid, 12, buf, cap, len);
    case K(0x2000, 6): return put(T_U32, id.od_version, buf, cap, len);
    case K(0x2000, 8): return put(T_U32, id.capabilities, buf, cap, len);
    case K(0x2000, 9):
        am__put_u32(raw, id.cal_record_version);
        am__put_u32(raw + 4, id.cal_record_crc32);
        return put_raw(raw, 8, buf, cap, len);
    case K(0x2100, 1): return put(T_U32, id.fw_cm7, buf, cap, len);
    case K(0x2100, 2): return put(T_U32, id.fw_cm4, buf, cap, len);
    case K(0x2100, 3): return put(T_U32, id.fw_boot, buf, cap, len);
    case K(0x2300, 1): return put(T_F32, c->current_A, buf, cap, len);
    case K(0x2300, 4): return put(T_F32, lim->speed_rad_s, buf, cap, len);
    case K(0x2300, 20): return put(T_U32, lim->comms_loss_ms, buf, cap, len);
    case K(0x2401, 1): return put(T_I16, am__round_even(t_winding(sj) * 10.0), buf, cap, len);
    case K(0x2600, 1): return put(T_U32, fault_word(sj), buf, cap, len);
    case K(0x2600, 2): return put(T_U32, sj->first_fault, buf, cap, len);
    case K(0x2601, 0): return put(T_U8, (double)sj->n_history, buf, cap, len);
    case K(0x2602, 1): return put(T_U32, sj->warn, buf, cap, len);
    case K(0x2603, 1): return put(T_U32, 0, buf, cap, len);
    case K(0x2620, 1): {
        uint8_t sio = 0;
        if (!sj->sto_active) {
            sio = AM_SIO_STO1 | AM_SIO_STO2 | AM_SIO_VM_GD_PRESENT;
            if (sj->state == AM_DS_OPERATION_ENABLED) sio |= AM_SIO_ENABLE_READBACK;
        }
        return put(T_U8, sio, buf, cap, len);
    }
    case K(0x2620, 3): return put(T_U32, sj->sto_counter, buf, cap, len);
    case K(0x2901, 1): return put(T_U8, (double)sj->authority_cfg, buf, cap, len);
    case K(0x2901, 3): return put(T_U8, (double)sj->authority, buf, cap, len);
    case K(0x2902, 2): return put(T_U32, 0x1C0 + sj->address / 4, buf, cap, len);
    case K(0x2902, 3): return put(T_U8, sj->address % 4, buf, cap, len);
    case K(0x2A00, 4): return put(T_U32, am__round_even(lim->kp_max_Nm_per_rad * 1e3), buf, cap, len);
    case K(0x2A00, 5): return put(T_U32, am__round_even(lim->kd_max_Nm_s_per_rad * 1e3), buf, cap, len);
    case K(0x2A11, 3): return put(T_U8, sj->current_mode_enabled ? 1 : 0, buf, cap, len);
    case K(0x2A20, 1): return put(T_U16, 20, buf, cap, len);
    case K(0x2A20, 2): return put(T_U16, 1, buf, cap, len);
    case K(0x2A30, 1): return put(T_F32, g->pos_kp_1_per_s, buf, cap, len);
    case K(0x2A30, 2): return put(T_F32, g->vel_kp_Nm_s_per_rad, buf, cap, len);
    case K(0x2A30, 3): return put(T_F32, g->vel_ki_Nm_per_rad, buf, cap, len);
    case K(0x2A40, 1): return put(T_F32, g->vel_filter_hz, buf, cap, len);
    case K(0x2A40, 2): return put(T_F32, g->torque_cmd_filter_hz, buf, cap, len);
    case K(0x2A40, 3): return put(T_F32, g->notch_hz, buf, cap, len);
    case K(0x2A40, 4): return put(T_F32, g->notch_q, buf, cap, len);
    case K(0x2A50, 1): return put(T_U32, am__round_even(c->kp_max_Nm_per_rad * 1e3), buf, cap, len);
    case K(0x2A50, 2): return put(T_U32, am__round_even(c->kd_max_Nm_s_per_rad * 1e3), buf, cap, len);
    case K(0x2A50, 3):
        am__put_f32(raw, 0.0f);
        am__put_f32(raw + 4, (float)gc->pos_kp_1_per_s);
        return put_raw(raw, 8, buf, cap, len);
    case K(0x2A50, 4): return put(T_F32, gc->vel_kp_Nm_s_per_rad, buf, cap, len);
    case K(0x2A50, 5): return put(T_F32, gc->vel_ki_Nm_per_rad, buf, cap, len);
    case K(0x2A50, 6): return put(T_F32, sj->min_filter_hz, buf, cap, len);
    case K(0x2B00, 1): return put(T_U32, id.cal_record_crc32, buf, cap, len);
    case K(0x2D00, 1): return put(T_U16, 1, buf, cap, len);
    case K(0x2D00, 2): return put(T_U32, u32na(tr.peak_torque_2s_Nm, 1e3), buf, cap, len);
    case K(0x2D00, 3): return put(T_U32, u32na(tr.rated_speed_rad_s, 1e3), buf, cap, len);
    case K(0x2D00, 4): return put(T_U32, u32na(tr.no_load_speed_48V_rad_s, 1e3), buf, cap, len);
    case K(0x2D00, 5): return put(T_F32, tr.reflected_inertia_kgm2, buf, cap, len);
    case K(0x2D00, 6): return put(T_U32, u32na(tr.backdrive_torque_Nm, 1e3), buf, cap, len);
    case K(0x2D00, 7): return put(T_F32, tr.kinetic_energy_no_load_J, buf, cap, len);
    case K(0x2D00, 8): return put(T_F32, tr.stiffness_Nm_per_rad, buf, cap, len);
    case K(0x2D00, 9): return put(T_U32, u32na(tr.backlash_rad, 1e6), buf, cap, len);
    case K(0x2D00, 10): return put(T_U32, u32na(tr.sto_reaction_time_s, 1e6), buf, cap, len);
    case K(0x2D00, 11): return put(T_U32, u32na(tr.collision_latency_s, 1e6), buf, cap, len);
    case K(0x2D00, 12): return put(T_U8, tr.basis, buf, cap, len);
    case K(0x2E00, 1): return put_raw(sj->label, strlen(sj->label), buf, cap, len);
    case K(0x603F, 0): return put(T_U16, sj->error_code, buf, cap, len);
    case K(0x6041, 0): return put(T_U16, am__sim_statusword(sj), buf, cap, len);
    case K(0x6061, 0): return put(T_I8, (double)sj->mode, buf, cap, len);
    case K(0x6064, 0): return put(T_I32, am__round_even(sj->q_meas * 1e6), buf, cap, len);
    case K(0x606C, 0): return put(T_I32, am__round_even(sj->qd_f * 1e3), buf, cap, len);
    case K(0x6073, 0): return put(T_U16, pm(lim->current_A, rated_a), buf, cap, len);
    case K(0x6075, 0): return put(T_U32, am__round_even(rated_a * 1e3), buf, cap, len);
    case K(0x6076, 0): return put(T_U32, am__round_even(ud->rated_nm * 1e3), buf, cap, len);
    case K(0x6077, 0): return put(T_I16, pm(sj->tau_applied, ud->rated_nm), buf, cap, len);
    case K(0x607D, 1): return put(T_I32, pos_obj(lim->pos_min_rad), buf, cap, len);
    case K(0x607D, 2): return put(T_I32, pos_obj(lim->pos_max_rad), buf, cap, len);
    case K(0x6091, 1): return put(T_U32, id.ratio_num, buf, cap, len);
    case K(0x6091, 2): return put(T_U32, id.ratio_den, buf, cap, len);
    case K(0x60E0, 0): return put(T_U16, pm(lim->torque_pos_Nm, ud->rated_nm), buf, cap, len);
    case K(0x60E1, 0): return put(T_U16, pm(lim->torque_neg_Nm, ud->rated_nm), buf, cap, len);
    default: break;
    }
    /* object missing (0x06020000) or subindex missing (0x06090011) */
    {
        static const uint16_t known[] = {0x1008, 0x1010, 0x1011, 0x1018, 0x2000, 0x2100, 0x2300, 0x2401, 0x2600,
                                         0x2601, 0x2602, 0x2603, 0x2620, 0x2901, 0x2902, 0x2A00, 0x2A11, 0x2A20,
                                         0x2A30, 0x2A40, 0x2A50, 0x2B00, 0x2D00, 0x2E00, 0x603F, 0x6041, 0x6061,
                                         0x6064, 0x606C, 0x6073, 0x6075, 0x6076, 0x6077, 0x607D, 0x6091, 0x60E0,
                                         0x60E1};
        size_t i;
        for (i = 0; i < sizeof known / sizeof known[0]; i++)
            if (known[i] == index) return abort_with(sj, 0x06090011u);
        return abort_with(sj, 0x06020000u);
    }
}

typedef enum { W_NONE, W_LIMIT, W_GAIN, W_OTHER } w_kind;

am_status am__sim_sdo_write(am_sim_joint *sj, uint16_t index, uint8_t sub, const void *buf, size_t len)
{
    const uint8_t *d = (const uint8_t *)buf;
    uint8_t probe[64];
    size_t plen;
    am_status st;
    od_type t = T_RAW;
    w_kind wk = W_NONE;
    bool rw_any = false;
    uint32_t key = K(index, sub);
    double rated_a = am__sim_rated_current(sj), rated = sj->ud->rated_nm;
    am_sim_lim lim = sj->limits;
    am_sim_gain g = sj->gains;
    if (!buf && len) return AM_ERR_PARAM;
    switch (key) {
    case K(0x1010, 1): case K(0x1011, 1): t = T_U32; wk = W_OTHER; break;
    case K(0x2300, 4): t = T_F32; wk = W_LIMIT; break;
    case K(0x2300, 20): t = T_U32; wk = W_LIMIT; break;
    case K(0x2901, 1): t = T_U8; wk = W_OTHER; break;
    case K(0x2A00, 4): case K(0x2A00, 5): t = T_U32; wk = W_LIMIT; break;
    case K(0x2A11, 3): t = T_U8; wk = W_OTHER; break;
    case K(0x2A30, 1): case K(0x2A30, 2): case K(0x2A30, 3):
    case K(0x2A40, 1): case K(0x2A40, 2): case K(0x2A40, 3): case K(0x2A40, 4): t = T_F32; wk = W_GAIN; break;
    case K(0x2E00, 1): t = T_RAW; wk = W_OTHER; break;
    case K(0x6073, 0): t = T_U16; wk = W_LIMIT; break;
    case K(0x607D, 1): case K(0x607D, 2): t = T_I32; wk = W_LIMIT; break;
    case K(0x60E0, 0): case K(0x60E1, 0): t = T_U16; wk = W_LIMIT; rw_any = true; break;
    default: break;
    }
    if (wk == W_NONE) {
        st = am__sim_sdo_read(sj, index, sub, probe, sizeof probe, &plen);
        if (st != AM_OK && (index == 0x2200 || index == 0x2300 || index == 0x2A50 || index == 0x2B00))
            return abort_with(sj, 0x08000020u);
        if (st != AM_OK) return st;                      /* last_abort set by the read */
        return abort_with(sj, (index == 0x2200 || index == 0x2300 || index == 0x2A50 || index == 0x2B00)
                                  ? 0x08000020u : 0x06010002u);
    }
    if (sj->state == AM_DS_OPERATION_ENABLED && !rw_any) {
        sim_event(sj, AM_EV_WARNING, 0x4000);
        return abort_with(sj, 0x08000022u);
    }
    if (t != T_RAW) {
        size_t need = (t == T_U8) ? 1 : (t == T_U16) ? 2 : 4;
        if (len != need) return abort_with(sj, 0x06070010u);
    }
    if (t == T_F32) {
        float f = am__get_f32(d);
        if (!(f == f) || f > 3.0e38f || f < -3.0e38f) return abort_with(sj, 0x06090030u);
    }
    switch (key) {
    case K(0x1010, 1):
        if (am__get_u32(d) != 0x65766173u) return abort_with(sj, 0x08000020u);
        return AM_OK;
    case K(0x1011, 1):
        if (am__get_u32(d) != 0x64616F6Cu) return abort_with(sj, 0x08000020u);
        st = am__sim_restore_defaults(sj);
        if (st == AM_ERR_STATE) sj->last_abort = 0x08000022u;
        return st;
    case K(0x2901, 1):
        st = am__sim_set_control_bus(sj, (am_bus)d[0]);
        if (st == AM_ERR_PARAM) sj->last_abort = 0x06090030u;
        if (st == AM_ERR_STATE) sj->last_abort = 0x08000022u;
        return st;
    case K(0x2A11, 3): sj->current_mode_enabled = d[0] != 0; return AM_OK;
    case K(0x2E00, 1):
        if (len > 31) { sj->last_abort = 0x06090030u; return AM_ERR_PARAM; }
        memcpy(sj->label, d, len);
        sj->label[len] = '\0';
        return AM_OK;
    default: break;
    }
    if (wk == W_GAIN) {
        float f = am__get_f32(d);
        switch (key) {
        case K(0x2A30, 1): g.pos_kp_1_per_s = f; break;
        case K(0x2A30, 2): g.vel_kp_Nm_s_per_rad = f; break;
        case K(0x2A30, 3): g.vel_ki_Nm_per_rad = f; break;
        case K(0x2A40, 1): g.vel_filter_hz = f; break;
        case K(0x2A40, 2): g.torque_cmd_filter_hz = f; break;
        case K(0x2A40, 3): g.notch_hz = f; break;
        default: g.notch_q = f; break;
        }
        st = am__sim_set_loop_gains(sj, &g);
        if (st == AM_ERR_LIMIT) sj->last_abort = 0x06090031u;
        return st;
    }
    /* limits: the drive accepts the wire representation of its ceiling (half an LSB above it) */
    {
        double v = 0.0, cap = 0.0, tol = 0.0;
        double *fp = NULL;
        switch (key) {
        case K(0x2300, 4): v = am__get_f32(d); fp = &lim.speed_rad_s; cap = sj->ceiling.speed_rad_s; tol = 1e-3; break;
        case K(0x2300, 20): lim.comms_loss_ms = am__get_u32(d); break;
        case K(0x2A00, 4): v = am__get_u32(d) * 1e-3; fp = &lim.kp_max_Nm_per_rad; cap = sj->ceiling.kp_max_Nm_per_rad; tol = 1e-3; break;
        case K(0x2A00, 5): v = am__get_u32(d) * 1e-3; fp = &lim.kd_max_Nm_s_per_rad; cap = sj->ceiling.kd_max_Nm_s_per_rad; tol = 1e-3; break;
        case K(0x6073, 0): v = am__get_u16(d) * rated_a / 1000.0; fp = &lim.current_A; cap = sj->ceiling.current_A; tol = rated_a / 1000.0; break;
        case K(0x60E0, 0): v = am__get_u16(d) * rated / 1000.0; fp = &lim.torque_pos_Nm; cap = sj->ceiling.torque_pos_Nm; tol = rated / 1000.0; break;
        case K(0x60E1, 0): v = am__get_u16(d) * rated / 1000.0; fp = &lim.torque_neg_Nm; cap = sj->ceiling.torque_neg_Nm; tol = rated / 1000.0; break;
        case K(0x607D, 1): case K(0x607D, 2): {
            int32_t r = (int32_t)am__get_u32(d);
            double p = r <= INT32_MIN ? -INFINITY : r >= INT32_MAX ? INFINITY : r * 1e-6;
            if (sub == 1) lim.pos_min_rad = p; else lim.pos_max_rad = p;
            break;
        }
        default: break;
        }
        if (fp) {
            if (cap < v && v <= cap + tol) v = cap;
            *fp = v;
        }
        st = am__sim_set_limits(sj, &lim);
        if (st == AM_ERR_LIMIT) sj->last_abort = 0x06090031u;
        if (st == AM_ERR_PARAM) sj->last_abort = 0x06090030u;
        return st;
    }
}

/* ---------------------------------------------------------------- public simulator controls */
void am_sim_options_default(am_sim_options *o)
{
    if (!o) return;
    o->ambient_C = 25.0;
    o->thermal_x0 = 0.0;
    o->peak_budget_s = 2.0;
    o->load_inertia_kgm2 = 0.0;
    o->q0_rad = 0.0;
    o->bus_V = 48.0;
    o->friction_scale = 1.0;
    o->authority_bus = AM_BUS_NONE;
}

am_status am_sim_configure(am_sim_joint *sj, const am_sim_options *o)
{
    if (!sj || !o) return AM_ERR_PARAM;
    sj->opts = *o;
    sj->authority = o->authority_bus != AM_BUS_NONE ? o->authority_bus : sj->cfg_bus;
    sj->authority_cfg = sj->authority;
    sj->plant_init = false;
    am__sim_power_cycle(sj);
    return AM_OK;
}

am_status am_sim_set_external_torque(am_sim_joint *sj, am_sim_torque_fn f, void *user)
{
    if (!sj) return AM_ERR_PARAM;
    sj->ext_fn = f;
    sj->ext_user = user;
    return AM_OK;
}

am_status am_sim_set_sto(am_sim_joint *sj, bool active)
{
    if (!sj) return AM_ERR_PARAM;
    if (active != sj->sto_active) {
        sj->sto_active = active;
        if (active) sj->sto_counter++;
        sim_event(sj, AM_EV_STO_CHANGED, active ? 1 : 0);
    }
    return AM_OK;
}

am_status am_sim_inject_fault(am_sim_joint *sj, uint16_t code)
{
    if (!sj || code == 0) return AM_ERR_PARAM;
    raise_fault(sj, code, true);
    return AM_OK;
}

am_status am_sim_clear_fault_cause(am_sim_joint *sj, uint16_t code)
{
    size_t i;
    if (!sj) return AM_ERR_PARAM;
    for (i = 0; i < sj->n_faults; i++)
        if (code == 0 || sj->faults[i].code == code) sj->faults[i].active = false;
    return AM_OK;
}

am_status am_sim_set_ambient(am_sim_joint *sj, double ambient_C)
{
    if (!sj || ambient_C != ambient_C) return AM_ERR_PARAM;
    sj->ambient = ambient_C;
    return AM_OK;
}

am_status am_sim_set_bus_voltage(am_sim_joint *sj, double volts)
{
    if (!sj || volts != volts) return AM_ERR_PARAM;
    sj->vbus_in = volts;
    return AM_OK;
}

am_status am_sim_set_load_inertia(am_sim_joint *sj, double kgm2)
{
    if (!sj || !(kgm2 >= 0.0)) return AM_ERR_PARAM;
    sj->opts.load_inertia_kgm2 = kgm2;
    return AM_OK;
}

am_status am_sim_power_cycle(am_sim_joint *sj)
{
    if (!sj) return AM_ERR_PARAM;
    am__sim_power_cycle(sj);
    return AM_OK;
}

am_status am_sim_get_plant(const am_sim_joint *sj, double *q_rad, double *qd_rad_s, double *tau_applied_Nm,
                           double *thermal_x)
{
    if (!sj) return AM_ERR_PARAM;
    if (q_rad) *q_rad = sj->q;
    if (qd_rad_s) *qd_rad_s = sj->qd;
    if (tau_applied_Nm) *tau_applied_Nm = sj->tau_applied;
    if (thermal_x) *thermal_x = sj->thermal_x;
    return AM_OK;
}

am_status am_sim_set_plant(am_sim_joint *sj, double q_rad, double qd_rad_s)
{
    if (!sj || q_rad != q_rad || qd_rad_s != qd_rad_s) return AM_ERR_PARAM;
    sj->q = q_rad;
    sj->qd = qd_rad_s;
    if (sj->external_plant) {
        sj->ext_q = q_rad;
        sj->ext_qd = qd_rad_s;
        sample_state(sj, q_rad);
    }
    return AM_OK;
}

am_status am_sim_set_external_plant(am_sim_joint *sj, bool external)
{
    if (!sj) return AM_ERR_PARAM;
    sj->external_plant = external;
    return AM_OK;
}
