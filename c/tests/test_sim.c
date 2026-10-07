/* SPDX-License-Identifier: Apache-2.0 */
/* L2 session on the built-in simulator. Mirrors python/tests/test_sim_backend.py. */
#include "../src/am_internal.h"
#include "am_test.h"

#include <stdlib.h>
#include <string.h>

#define HEAVY 1.0e4

typedef struct { am_session *s; am_joint *j; am_sim_joint *sj; } rig;

static rig make(const char *unit, am_mode mode, bool enable, uint32_t cycle_us, const am_sim_options *o)
{
    rig r;
    char iface[32];
    am_session_config cfg;
    am_sim_options def;
    memset(&r, 0, sizeof r);
    memset(&cfg, 0, sizeof cfg);
    if (snprintf(iface, sizeof iface, "sim:%s", unit) < 0) iface[0] = '\0';
    cfg.interface = iface;
    cfg.cycle_us = cycle_us;
    cfg.telemetry_extended = true;
    if (am_init(&cfg, &r.s) != AM_OK) return r;
    am_open_joint(r.s, 0, &r.j);
    r.sj = am_sim_of(r.j);
    if (!o) {
        am_sim_options_default(&def);
        o = &def;
    }
    am_sim_configure(r.sj, o);
    am_start(r.s);
    am_set_mode(r.j, mode);
    if (enable) am_enable(r.j, 2000);
    return r;
}

static am_sim_options opts_load(double J)
{
    am_sim_options o;
    am_sim_options_default(&o);
    o.load_inertia_kgm2 = J;
    return o;
}

static double tau_applied(const am_sim_joint *sj)
{
    double t;
    am_sim_get_plant(sj, NULL, NULL, &t, NULL);
    return t;
}

static void test_torque_never_above_peak_envelope(void)
{
    size_t u;
    for (u = 0; u < AM_UNIT_COUNT; u++) {
        am_sim_options o = opts_load(0.2);
        rig r = make(am_unit_table[u].unit, AM_MODE_CST, true, 1000, &o);
        double prev_w = 0.0, peak = am_unit_table[u].peak_nm;
        am_telemetry t;
        int k;
        CHECK(r.j != NULL);
        for (k = 0; k < 3000; k++) {
            double env = 0.0;
            const am_curve *c = &am_unit_table[u].curve[0][0];
            double rpm = fabs(prev_w) * 60.0 / AM_TWO_PI;
            int i;
            am_set_torque(r.j, (float)(10.0 * peak));
            am_cycle(r.s, NULL);
            am_read_telemetry(r.j, &t);
            for (i = 0; i + 1 < c->n; i++)
                if (c->p[i].rpm - 1e-6 <= rpm && rpm <= c->p[i + 1].rpm + 1e-6) {
                    env = c->p[i].nm + (c->p[i + 1].nm - c->p[i].nm) * (rpm - c->p[i].rpm) /
                                           (c->p[i + 1].rpm - c->p[i].rpm);
                    break;
                }
            if (fabs(tau_applied(r.sj)) > env * 1.0001 + 1e-9) {
                CHECK(fabs(tau_applied(r.sj)) <= env * 1.0001 + 1e-9);
                break;
            }
            prev_w = r.sj->qd_f;          /* the drive's own velocity estimate (double) */
        }
        CHECK(t.internal_limit);
        am_deinit(r.s);
    }
}

static void test_peak_two_seconds_then_continuous(void)
{
    am_sim_options o = opts_load(HEAVY);
    rig r = make("M", AM_MODE_CST, true, 1000, &o);
    int k, n_peak = 0;
    double tau500 = 0, tau1900 = 0;
    am_telemetry t;
    for (k = 0; k < 4000; k++) {
        double ta;
        am_set_torque(r.j, 500.0f);
        am_cycle(r.s, NULL);
        ta = tau_applied(r.sj);
        if (k == 500) tau500 = ta;
        if (k == 1900) tau1900 = ta;
        if (k < 2600 && ta > 40.0) n_peak++;
    }
    CHECK_NEAR(tau500, 80.0, 1e-3);
    CHECK_NEAR(tau1900, 80.0, 1e-3);
    CHECK_NEAR(n_peak / 1000.0, 2.0, 0.05);
    am_read_telemetry(r.j, &t);
    CHECK(t.warning_word & AM_WARN_PEAK_BUDGET_EMPTY);
    am_deinit(r.s);
}

static void test_limits_lower_only(void)
{
    am_sim_options o = opts_load(HEAVY);
    rig r = make("M", AM_MODE_CST, false, 1000, &o);
    am_limits eff, ceil, bad, ok;
    am_get_limits(r.j, &eff, &ceil);
    bad = eff;
    bad.torque_pos_Nm = ceil.torque_pos_Nm * 1.01f;
    CHECK_ST(am_set_limits(r.j, &bad), AM_ERR_LIMIT);
    am_get_limits(r.j, &bad, NULL);
    CHECK(bad.torque_pos_Nm == eff.torque_pos_Nm);
    ok = eff;
    ok.torque_pos_Nm = 20.0f;
    ok.torque_neg_Nm = 15.0f;
    CHECK_ST(am_set_limits(r.j, &ok), AM_OK);
    CHECK_ST(am_enable(r.j, 1000), AM_OK);
    am_set_torque(r.j, 500.0f);
    am_cycle(r.s, NULL);
    CHECK_NEAR(tau_applied(r.sj), 20.0, 1e-6);
    am_set_torque(r.j, -500.0f);
    am_cycle(r.s, NULL);
    CHECK_NEAR(tau_applied(r.sj), -15.0, 1e-6);
    CHECK_ST(am_set_limits(r.j, &ok), AM_ERR_STATE);
    am_deinit(r.s);
}

static void test_impedance_envelope_and_params(void)
{
    rig r = make("M", AM_MODE_IMPEDANCE, true, 1000, NULL);
    am_limits eff;
    am_impedance imp;
    memset(&imp, 0, sizeof imp);
    am_get_limits(r.j, &eff, NULL);
    imp.kp_Nm_per_rad = eff.kp_max_Nm_per_rad * 2;
    imp.kd_Nm_s_per_rad = 1.0f;
    CHECK_ST(am_set_impedance(r.j, &imp), AM_ERR_LIMIT);
    imp.kp_Nm_per_rad = NAN;
    CHECK_ST(am_set_impedance(r.j, &imp), AM_ERR_PARAM);
    CHECK_ST(am_set_position(r.j, 0.1, 0, 0), AM_ERR_STATE);
    am_deinit(r.s);
}

static void test_impedance_step_response(void)
{
    am_sim_options o = opts_load(0.1);
    rig r;
    double K = 100.0, D, qmax = 0.0, q = 0.0, step = 0.2;
    int k, t63 = -1;
    am_telemetry t;
    am_impedance imp;
    o.friction_scale = 0.0;
    r = make("M", AM_MODE_IMPEDANCE, true, 1000, &o);
    D = 2.0 * sqrt(K * (0.030 + 0.1));
    memset(&imp, 0, sizeof imp);
    imp.q_ref_rad = step;
    imp.kp_Nm_per_rad = (float)K;
    imp.kd_Nm_s_per_rad = (float)D;
    for (k = 0; k < 3000; k++) {
        am_set_impedance(r.j, &imp);
        am_cycle(r.s, NULL);
        am_read_telemetry(r.j, &t);
        q = t.q_rad;
        if (q > qmax) qmax = q;
        if (t63 < 0 && q >= 0.632 * step) t63 = k;
    }
    CHECK(qmax - step < 0.1 * step);
    CHECK_NEAR(q, step, 0.002);
    CHECK(t63 > 0);
    am_deinit(r.s);
}

static double ramp(double t, double q, double qd, void *user)
{
    (void)q; (void)qd;
    return *(double *)user * t;
}

static void test_unpowered_breakaway_equals_backdrive(void)
{
    rig r = make("M", AM_MODE_CST, false, 1000, NULL);
    double rate = 0.5, qd;
    int k, broke = -1;
    am_sim_set_external_torque(r.sj, ramp, &rate);
    for (k = 0; k < 10000; k++) {
        am_cycle(r.s, NULL);
        am_sim_get_plant(r.sj, NULL, &qd, NULL, NULL);
        if (fabs(qd) > 1e-6) {
            broke = k;
            break;
        }
    }
    CHECK(broke > 0);
    CHECK_NEAR(broke * 1e-3 * 0.5, 1.4, 0.02);
    am_deinit(r.s);
}

static void test_csp_holds_until_first_command(void)
{
    am_sim_options o = opts_load(0.05);
    rig r;
    int k;
    o.q0_rad = 1.0;
    r = make("M", AM_MODE_CSP, true, 1000, &o);
    for (k = 0; k < 20; k++) am_cycle(r.s, NULL);
    CHECK(fabs(tau_applied(r.sj)) < 1e-6);
    for (k = 0; k < 3000; k++) {
        am_set_position(r.j, 0.3, 0, 0);
        am_cycle(r.s, NULL);
    }
    {
        am_telemetry t;
        am_read_telemetry(r.j, &t);
        CHECK_NEAR(t.q_rad, 0.3, 0.01);
    }
    am_deinit(r.s);
}

static void test_state_machine_walk_and_disable(void)
{
    am_sim_options o = opts_load(HEAVY);
    rig r = make("M", AM_MODE_CST, true, 1000, &o);
    am_telemetry t;
    am_read_telemetry(r.j, &t);
    CHECK(t.state == AM_DS_OPERATION_ENABLED);
    CHECK(t.ref_state == AM_REF_VALID);
    CHECK(t.statusword & AM_SW_REF_VALID);
    am_set_torque(r.j, 20.0f);
    am_cycle(r.s, NULL);
    CHECK(tau_applied(r.sj) > 19.0);
    am_disable(r.j);
    am_cycle(r.s, NULL);
    am_cycle(r.s, NULL);
    am_read_telemetry(r.j, &t);
    CHECK(t.state == AM_DS_READY_TO_SWITCH_ON);
    CHECK(tau_applied(r.sj) == 0.0);
    am_deinit(r.s);
}

static void test_quick_stop(void)
{
    am_sim_options o = opts_load(0.1);
    rig r = make("M", AM_MODE_CSV, true, 1000, &o);
    am_telemetry t;
    int k, saw_qs = 0;
    double qd;
    for (k = 0; k < 800; k++) {
        am_set_velocity(r.j, 3.0f, 0.0f);
        am_cycle(r.s, NULL);
    }
    am_sim_get_plant(r.sj, NULL, &qd, NULL, NULL);
    CHECK(qd > 2.0);
    am_quick_stop(r.j);
    for (k = 0; k < 3000; k++) {
        am_cycle(r.s, NULL);
        am_read_telemetry(r.j, &t);
        if (t.state == AM_DS_QUICK_STOP_ACTIVE) saw_qs = 1;
        if (t.state == AM_DS_SWITCH_ON_DISABLED) break;
    }
    CHECK(saw_qs);
    CHECK(t.state == AM_DS_SWITCH_ON_DISABLED);
    for (k = 0; k < 50; k++) am_cycle(r.s, NULL);
    am_read_telemetry(r.j, &t);
    CHECK(t.state == AM_DS_SWITCH_ON_DISABLED);
    am_deinit(r.s);
}

static int n_events[8];
static void count_events(const am_event *ev, void *user)
{
    (void)user;
    if (ev->kind >= 1 && ev->kind <= 7) n_events[ev->kind]++;
}

static void test_sto(void)
{
    am_session_config cfg;
    am_session *s;
    am_joint *j;
    am_sim_joint *sj;
    am_telemetry t;
    int k;
    memset(&cfg, 0, sizeof cfg);
    memset(n_events, 0, sizeof n_events);
    cfg.interface = "sim:M";
    cfg.event_cb = count_events;
    CHECK_ST(am_init(&cfg, &s), AM_OK);
    CHECK_ST(am_open_joint(s, 0, &j), AM_OK);
    sj = am_sim_of(j);
    CHECK_ST(am_start(s), AM_OK);
    CHECK_ST(am_set_mode(j, AM_MODE_CST), AM_OK);
    CHECK_ST(am_enable(j, 1000), AM_OK);
    am_sim_set_load_inertia(sj, HEAVY);
    for (k = 0; k < 10; k++) {
        am_set_torque(j, 20.0f);
        am_cycle(s, NULL);
    }
    CHECK_NEAR(tau_applied(sj), 20.0, 1e-6);
    am_sim_set_sto(sj, true);
    am_set_torque(j, 20.0f);
    am_cycle(s, NULL);
    am_read_telemetry(j, &t);
    CHECK(tau_applied(sj) == 0.0);
    CHECK(t.sto_active && (t.statusword & AM_SW_STO));
    CHECK(t.state == AM_DS_SWITCH_ON_DISABLED);
    CHECK((t.safety_io & (AM_SIO_STO1 | AM_SIO_STO2)) == 0);
    CHECK_ST(am_enable(j, 100), AM_ERR_STO);
    am_sim_set_sto(sj, false);
    for (k = 0; k < 50; k++) {
        am_set_torque(j, 20.0f);
        am_cycle(s, NULL);
    }
    am_read_telemetry(j, &t);
    CHECK(t.state == AM_DS_SWITCH_ON_DISABLED);
    CHECK(tau_applied(sj) == 0.0);
    CHECK_ST(am_enable(j, 1000), AM_OK);
    am_set_torque(j, 20.0f);
    am_cycle(s, NULL);
    CHECK_NEAR(tau_applied(sj), 20.0, 1e-6);
    CHECK(n_events[AM_EV_STO_CHANGED] >= 2);
    am_deinit(s);
}

static void test_fault_latch_clear_and_history(void)
{
    rig r = make("M", AM_MODE_CST, true, 1000, NULL);
    am_telemetry t;
    am_fault_record rec[4];
    size_t n = 0;
    am_fault_info fi;
    int k;
    am_sim_inject_fault(r.sj, 0x100B);
    for (k = 0; k < 5; k++) am_cycle(r.s, NULL);
    am_read_telemetry(r.j, &t);
    CHECK(t.state == AM_DS_FAULT);
    CHECK(t.error_code == 0x100B);
    CHECK(t.fault_word & (1u << 11));
    CHECK_ST(am_set_torque(r.j, 1.0f), AM_ERR_FAULT);
    CHECK_ST(am_get_fault_history(r.j, rec, 4, &n), AM_OK);
    CHECK(n == 1 && rec[0].code == 0x100B);
    am_get_faults(r.j, &fi);
    CHECK(fi.latched && fi.error_code == 0x100B);
    CHECK_ST(am_clear_faults(r.j, 100), AM_ERR_FAULT);          /* cause still active */
    am_sim_clear_fault_cause(r.sj, 0);
    CHECK_ST(am_clear_faults(r.j, 500), AM_OK);
    am_read_telemetry(r.j, &t);
    CHECK(t.state == AM_DS_SWITCH_ON_DISABLED);
    CHECK(t.ref_state == AM_REF_INVALID);
    CHECK_ST(am_enable(r.j, 1000), AM_OK);
    am_deinit(r.s);
}

static void test_over_temperature_trip(void)
{
    am_sim_options o;
    rig r;
    am_telemetry t;
    int k;
    am_sim_options_default(&o);
    o.thermal_x0 = 1.5;
    r = make("M", AM_MODE_CST, false, 20000, &o);
    for (k = 0; k < 3; k++) am_cycle(r.s, NULL);
    am_read_telemetry(r.j, &t);
    CHECK(t.state == AM_DS_FAULT);
    CHECK(t.error_code == 0x2000);
    CHECK_ST(am_enable(r.j, 100), AM_ERR_FAULT);
    for (k = 0; k < 3000; k++) am_cycle(r.s, NULL);
    CHECK_ST(am_clear_faults(r.j, 500), AM_OK);
    CHECK_ST(am_enable(r.j, 2000), AM_OK);
    am_deinit(r.s);
}

static void test_current_mode_needs_enable_object(void)
{
    rig r = make("M", AM_MODE_CST, false, 1000, NULL);
    uint8_t one = 1;
    am_telemetry t;
    int k;
    CHECK_ST(am_set_mode(r.j, AM_MODE_CURRENT), AM_ERR_NOT_SUPPORTED);
    CHECK_ST(am_sdo_write(r.j, 0x2A11, 3, &one, 1), AM_OK);
    CHECK_ST(am_set_mode(r.j, AM_MODE_CURRENT), AM_OK);
    CHECK_ST(am_enable(r.j, 1000), AM_OK);
    for (k = 0; k < 10; k++) {
        am_set_current(r.j, 5.0f);
        am_cycle(r.s, NULL);
    }
    am_read_telemetry(r.j, &t);
    CHECK_NEAR(t.iq_A, 5.0, 0.01);
    CHECK_ST(am_set_mode(r.j, AM_MODE_CURRENT), AM_ERR_STATE);
    am_deinit(r.s);
}

static void test_control_authority(void)
{
    rig r = make("M", AM_MODE_CST, false, 1000, NULL);
    am_bus c, e;
    CHECK_ST(am_get_control_bus(r.j, &c, &e), AM_OK);
    CHECK(c == AM_BUS_ETHERCAT && e == AM_BUS_ETHERCAT);
    CHECK_ST(am_set_control_bus(r.j, AM_BUS_CANFD), AM_OK);
    am_get_control_bus(r.j, &c, &e);
    CHECK(c == AM_BUS_CANFD && e == AM_BUS_ETHERCAT);
    am_sim_power_cycle(r.sj);
    CHECK_ST(am_set_torque(r.j, 1.0f), AM_ERR_AUTHORITY);
    am_deinit(r.s);
}

static void test_identity_all_units(void)
{
    am_session_config cfg;
    am_session *s;
    am_device_info dev[8];
    size_t n = 0, i;
    memset(&cfg, 0, sizeof cfg);
    cfg.interface = "sim:S-S,S-D,M,H-F,H-T,H-X";
    CHECK_ST(am_init(&cfg, &s), AM_OK);
    CHECK_ST(am_scan(s, dev, 8, &n), AM_OK);
    CHECK(n == 6);
    CHECK(dev[0].product_code == 2 && dev[1].product_code == 1 && dev[5].product_code == 6);
    for (i = 0; i < n; i++) {
        am_joint *j;
        am_identity id;
        am_transparency tr;
        CHECK_ST(am_open_joint(s, dev[i].address, &j), AM_OK);
        am_get_identity(j, &id);
        am_get_transparency(j, &tr);
        CHECK(tr.basis == AM_BASIS_DESIGN_TARGET);
        CHECK(tr.peak_torque_2s_Nm > tr.rated_torque_Nm);
        CHECK(tr.backdrive_torque_Nm > 0.3f);
        CHECK(id.ratio_den > 0);
        if (id.sku == AM_SKU_M) {
            uint8_t b[8];
            size_t len = 0;
            char cal[1024];
            CHECK_NEAR(tr.peak_torque_2s_Nm, 80.0, 1e-6);
            CHECK_NEAR(tr.reflected_inertia_kgm2, 0.030, 1e-7);
            CHECK_ST(am_sdo_read(j, 0x2D00, 2, b, sizeof b, &len), AM_OK);
            CHECK(len == 4 && am__get_u32(b) == 80000);
            CHECK_ST(am_sdo_write(j, 0x6064, 0, b, 4), AM_ERR_SDO_ABORT);
            CHECK(am_last_abort_code(j) == 0x06010002u);
            CHECK_ST(am_sdo_read(j, 0x9999, 0, b, sizeof b, &len), AM_ERR_SDO_ABORT);
            CHECK_ST(am_read_calibration(j, cal, sizeof cal, &len), AM_OK);
            CHECK(strstr(cal, "\"data_basis\": \"design_target\"") != NULL);
            CHECK_ST(am_set_user_label(j, "left knee"), AM_OK);
            am_get_identity(j, &id);
            CHECK(strcmp(id.user_label, "left knee") == 0);
        }
    }
    {
        am_session_config bad;
        am_session *x = NULL;
        memset(&bad, 0, sizeof bad);
        bad.interface = "sim:Q-Z";
        CHECK_ST(am_init(&bad, &x), AM_ERR_PARAM);
        bad.interface = "hardware";
        CHECK_ST(am_init(&bad, &x), AM_ERR_PARAM);
    }
    am_deinit(s);
}

static void test_trace_not_supported_and_stale(void)
{
    rig r = make("M", AM_MODE_CST, false, 1000, NULL);
    am_telemetry t;
    CHECK_ST(am_trace_arm(r.j, 0, 0.0f, 100, 0), AM_ERR_NOT_SUPPORTED);
    CHECK_ST(am_read_telemetry(r.j, &t), AM_OK);
    am_stop(r.s);
    CHECK_ST(am_cycle(r.s, NULL), AM_ERR_STATE);
    am_deinit(r.s);
}

int main(void)
{
    RUN(test_torque_never_above_peak_envelope);
    RUN(test_peak_two_seconds_then_continuous);
    RUN(test_limits_lower_only);
    RUN(test_impedance_envelope_and_params);
    RUN(test_impedance_step_response);
    RUN(test_unpowered_breakaway_equals_backdrive);
    RUN(test_csp_holds_until_first_command);
    RUN(test_state_machine_walk_and_disable);
    RUN(test_quick_stop);
    RUN(test_sto);
    RUN(test_fault_latch_clear_and_history);
    RUN(test_over_temperature_trip);
    RUN(test_current_mode_needs_enable_object);
    RUN(test_control_authority);
    RUN(test_identity_all_units);
    RUN(test_trace_not_supported_and_stale);
    TEST_EXIT();
}
