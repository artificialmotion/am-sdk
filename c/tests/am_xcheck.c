/* SPDX-License-Identifier: Apache-2.0 */
/*
 * am_xcheck: runs a command script against the C library and prints the results, so the Python
 * test suite can run the same script on the Python SDK and compare (python/tests/test_cross_check_c.py).
 *
 * Script lines (stdin), one command per line, '#' starts a comment:
 *   session <units> <cycle_us> <bus> [wire]   open a session ("wire" = through am_sim_transport_open)
 *   opt <addr> <key> <value>                  sim option before open: load_inertia q0 ambient thermal_x0
 *                                             bus_V friction_scale peak_budget
 *   open <addr> | start | stop
 *   mode <addr> <am_mode> | enable <addr> <ms> | disable <addr> | qstop <addr> | clear <addr> <ms>
 *   imp <addr> q qd kp kd tau | pos <addr> q qd tau | vel <addr> qd tau | tor <addr> tau | cur <addr> iq
 *   sdow <addr> <index> <sub> <hex>  | od <addr> <index> <sub>
 *   ext <addr> <N*m>                  constant external torque on the plant
 *   sto <addr> <0|1> | fault <addr> <code> | clearcause <addr>
 *   cycle <n>                         run n cycles; print one T line per open joint per cycle
 *   limits <addr> | gains <addr> | ident <addr>
 *   pack1601 <rated_mNm> <rated_mA> <cw> <mode> q qd tau kp kd iq
 *   pack1600 <rated_mNm> <rated_mA> <cw> <mode> q qd tau
 *   packslot <rated_mNm> <rated_mA> <control> <counter> <mode> q qd tau kp kd iq
 *   unpacktx <rated_mNm> <rated_mA> <hex of 24 or 84 bytes>
 *   unpackfb <rated_mNm> <rated_mA> <hex of 16 or 32 bytes>
 * Every command prints "R <command> <status>"; data lines start with T, O, L, G, I, P or U.
 */
#define _CRT_SECURE_NO_WARNINGS 1     /* MSVC runtime: allow sscanf */
#include "am_api.h"
#include "am_sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAXJ 16

static am_session *S;
static am_joint *J[MAXJ];
static am_sim_options OPT[MAXJ];
static double EXT[MAXJ];
static const am_transport_ops *TP_OPS;
static void *TP_USER;

static double ext_const(double t, double q, double qd, void *user)
{
    (void)t; (void)q; (void)qd;
    return *(const double *)user;
}

static am_sim_joint *simj(int a)
{
    if (TP_USER) return am_sim_transport_joint(TP_USER, (uint16_t)a);
    return J[a] ? am_sim_of(J[a]) : NULL;
}

static void hex(const uint8_t *p, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) printf("%02x", p[i]);
}

static size_t unhex(const char *s, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (s[0] && s[1] && n < cap) {
        unsigned v;
        if (sscanf(s, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

static void print_tlm(unsigned long long cyc, int a, const am_telemetry *t)
{
    printf("T %llu %d %d %d %.17g %.9g %.9g %.9g %.9g %.9g %u %u %u %d %d %.9g %llu\n", cyc, a, (int)t->state,
           (int)t->mode, t->q_rad, t->qd_rad_s, t->iq_A, t->tau_est_Nm, t->t_winding_C, t->vbus_V, t->statusword,
           t->warning_word, t->error_code, (int)t->ref_state, t->sto_active ? 1 : 0, t->q_out_encoder_rad,
           (unsigned long long)t->t_ns);
}

static void print_limits(const char *tag, const am_limits *l)
{
    printf("L %s %.9g %.9g %.9g %.9g %.17g %.17g %.9g %.9g %u\n", tag, l->torque_pos_Nm, l->torque_neg_Nm, l->current_A,
           l->speed_rad_s, l->pos_min_rad, l->pos_max_rad, l->kp_max_Nm_per_rad, l->kd_max_Nm_s_per_rad,
           l->comms_loss_ms);
}

static void set_opt(am_sim_options *o, const char *k, double v)
{
    if (!strcmp(k, "load_inertia")) o->load_inertia_kgm2 = v;
    else if (!strcmp(k, "q0")) o->q0_rad = v;
    else if (!strcmp(k, "ambient")) o->ambient_C = v;
    else if (!strcmp(k, "thermal_x0")) o->thermal_x0 = v;
    else if (!strcmp(k, "bus_V")) o->bus_V = v;
    else if (!strcmp(k, "friction_scale")) o->friction_scale = v;
    else if (!strcmp(k, "peak_budget")) o->peak_budget_s = v;
}

int main(void)
{
    char line[4096];
    unsigned long long cycles = 0;
    int i;
    for (i = 0; i < MAXJ; i++) am_sim_options_default(&OPT[i]);
    while (fgets(line, sizeof line, stdin)) {
        char cmd[32] = "", a1[256] = "", rest[4096] = "";
        int a = 0;
        am_status st = AM_OK;
        char *nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        if (line[0] == '#' || sscanf(line, "%31s", cmd) != 1) continue;
        if (!strcmp(cmd, "session")) {
            char units[256], wire[16] = "";
            unsigned cyc = 1000;
            int bus = 1;
            am_session_config cfg;
            memset(&cfg, 0, sizeof cfg);
            sscanf(line, "%*s %255s %u %d %15s", units, &cyc, &bus, wire);
            cfg.bus = (am_bus)bus;
            cfg.cycle_us = cyc;
            cfg.telemetry_extended = true;
            if (!strcmp(wire, "wire")) {
                st = am_sim_transport_open(units, cfg.bus, cyc, &TP_OPS, &TP_USER);
                cfg.transport = TP_OPS;
                cfg.transport_user = TP_USER;
                cfg.interface = "sim-wire";
            } else {
                if (snprintf(a1, sizeof a1, "sim:%s", units) < 0) a1[0] = '\0';
                cfg.interface = a1;
            }
            if (st == AM_OK) st = am_init(&cfg, &S);
        } else if (!strcmp(cmd, "opt")) {
            char key[64];
            double v;
            sscanf(line, "%*s %d %63s %lf", &a, key, &v);
            set_opt(&OPT[a], key, v);
        } else if (!strcmp(cmd, "open")) {
            sscanf(line, "%*s %d", &a);
            if (TP_USER) am_sim_configure(am_sim_transport_joint(TP_USER, (uint16_t)a), &OPT[a]);
            st = am_open_joint(S, (uint16_t)a, &J[a]);
            if (st == AM_OK && !TP_USER) st = am_sim_configure(am_sim_of(J[a]), &OPT[a]);
        } else if (!strcmp(cmd, "start")) {
            st = am_start(S);
        } else if (!strcmp(cmd, "stop")) {
            st = am_stop(S);
        } else if (!strcmp(cmd, "mode")) {
            int m;
            sscanf(line, "%*s %d %d", &a, &m);
            st = am_set_mode(J[a], (am_mode)m);
        } else if (!strcmp(cmd, "enable")) {
            unsigned ms;
            sscanf(line, "%*s %d %u", &a, &ms);
            st = am_enable(J[a], ms);
        } else if (!strcmp(cmd, "disable")) {
            sscanf(line, "%*s %d", &a);
            st = am_disable(J[a]);
        } else if (!strcmp(cmd, "qstop")) {
            sscanf(line, "%*s %d", &a);
            st = am_quick_stop(J[a]);
        } else if (!strcmp(cmd, "clear")) {
            unsigned ms;
            sscanf(line, "%*s %d %u", &a, &ms);
            st = am_clear_faults(J[a], ms);
        } else if (!strcmp(cmd, "imp")) {
            am_impedance imp;
            double q, qd, kp, kd, tau;
            sscanf(line, "%*s %d %lf %lf %lf %lf %lf", &a, &q, &qd, &kp, &kd, &tau);
            imp.q_ref_rad = q; imp.qd_ref_rad_s = (float)qd; imp.kp_Nm_per_rad = (float)kp;
            imp.kd_Nm_s_per_rad = (float)kd; imp.tau_ff_Nm = (float)tau;
            st = am_set_impedance(J[a], &imp);
        } else if (!strcmp(cmd, "pos")) {
            double q, qd, tau;
            sscanf(line, "%*s %d %lf %lf %lf", &a, &q, &qd, &tau);
            st = am_set_position(J[a], q, (float)qd, (float)tau);
        } else if (!strcmp(cmd, "vel")) {
            double qd, tau;
            sscanf(line, "%*s %d %lf %lf", &a, &qd, &tau);
            st = am_set_velocity(J[a], (float)qd, (float)tau);
        } else if (!strcmp(cmd, "tor")) {
            double tau;
            sscanf(line, "%*s %d %lf", &a, &tau);
            st = am_set_torque(J[a], (float)tau);
        } else if (!strcmp(cmd, "cur")) {
            double iq;
            sscanf(line, "%*s %d %lf", &a, &iq);
            st = am_set_current(J[a], (float)iq);
        } else if (!strcmp(cmd, "sdow")) {
            unsigned idx, sub;
            uint8_t b[64];
            size_t n;
            sscanf(line, "%*s %d %x %u %255s", &a, &idx, &sub, a1);
            n = unhex(a1, b, sizeof b);
            st = am_sdo_write(J[a], (uint16_t)idx, (uint8_t)sub, b, n);
        } else if (!strcmp(cmd, "od")) {
            unsigned idx, sub;
            uint8_t b[64];
            size_t n = 0;
            sscanf(line, "%*s %d %x %u", &a, &idx, &sub);
            st = am_sdo_read(J[a], (uint16_t)idx, (uint8_t)sub, b, sizeof b, &n);
            printf("O %04x %u ", idx, sub);
            if (st == AM_OK) hex(b, n); else printf("abort:%08lx", (unsigned long)am_last_abort_code(J[a]));
            printf("\n");
        } else if (!strcmp(cmd, "ext")) {
            double v = 0.0;
            sscanf(line, "%*s %d %lf", &a, &v);
            EXT[a] = v;
            st = am_sim_set_external_torque(simj(a), ext_const, &EXT[a]);
        } else if (!strcmp(cmd, "sto")) {
            int on;
            sscanf(line, "%*s %d %d", &a, &on);
            st = am_sim_set_sto(simj(a), on != 0);
        } else if (!strcmp(cmd, "fault")) {
            unsigned code;
            sscanf(line, "%*s %d %x", &a, &code);
            st = am_sim_inject_fault(simj(a), (uint16_t)code);
        } else if (!strcmp(cmd, "clearcause")) {
            sscanf(line, "%*s %d", &a);
            st = am_sim_clear_fault_cause(simj(a), 0);
        } else if (!strcmp(cmd, "cycle")) {
            unsigned long n, k;
            sscanf(line, "%*s %lu", &n);
            for (k = 0; k < n && st == AM_OK; k++) {
                st = am_cycle(S, NULL);
                cycles++;
                for (i = 0; i < MAXJ; i++) {
                    am_telemetry t;
                    if (J[i] && am_read_telemetry(J[i], &t) == AM_OK) print_tlm(cycles, i, &t);
                }
            }
        } else if (!strcmp(cmd, "limits")) {
            am_limits e, c;
            sscanf(line, "%*s %d", &a);
            st = am_get_limits(J[a], &e, &c);
            if (st == AM_OK) { print_limits("eff", &e); print_limits("ceil", &c); }
        } else if (!strcmp(cmd, "gains")) {
            am_loop_gains g;
            sscanf(line, "%*s %d", &a);
            st = am_get_loop_gains(J[a], &g);
            if (st == AM_OK)
                printf("G %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", g.pos_kp_1_per_s, g.vel_kp_Nm_s_per_rad,
                       g.vel_ki_Nm_per_rad, g.vel_filter_hz, g.torque_cmd_filter_hz, g.notch_hz, g.notch_q);
        } else if (!strcmp(cmd, "ident")) {
            am_identity id;
            am_transparency tr;
            memset(&tr, 0, sizeof tr);
            sscanf(line, "%*s %d", &a);
            st = am_get_identity(J[a], &id);
            if (st == AM_OK) st = am_get_transparency(J[a], &tr);
            if (st == AM_OK) {
                printf("I %d %u %u %u %u %s|", (int)id.sku, id.ratio_num, id.ratio_den, id.serial, id.od_version,
                       id.device_name);
                hex(id.uid, 12);
                printf(" %08x %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d\n", id.cal_record_crc32, tr.rated_torque_Nm,
                       tr.peak_torque_2s_Nm, tr.rated_speed_rad_s, tr.no_load_speed_48V_rad_s, tr.reflected_inertia_kgm2,
                       tr.backdrive_torque_Nm, tr.kinetic_energy_no_load_J, (int)tr.basis);
            }
        } else if (!strcmp(cmd, "pack1601") || !strcmp(cmd, "pack1600") || !strcmp(cmd, "packslot")) {
            unsigned rt, ra, cw, ctl = 0, cnt = 0;
            int m;
            double q, qd, tau, kp = 0, kd = 0, iq = 0;
            am_codec c;
            am_command x;
            uint8_t b[32];
            size_t n;
            if (!strcmp(cmd, "packslot")) {
                sscanf(line, "%*s %u %u %u %u %d %lf %lf %lf %lf %lf %lf", &rt, &ra, &ctl, &cnt, &m, &q, &qd, &tau, &kp,
                       &kd, &iq);
                cw = 0;
            } else {
                sscanf(line, "%*s %u %u %u %d %lf %lf %lf %lf %lf %lf", &rt, &ra, &cw, &m, &q, &qd, &tau, &kp, &kd, &iq);
            }
            am_codec_init(&c, rt, ra, 20, 1, 0);
            memset(&x, 0, sizeof x);
            x.mode = (am_mode)m; x.q_ref_rad = q; x.qd_ref_rad_s = (float)qd; x.tau_ff_Nm = (float)tau;
            x.kp_Nm_per_rad = (float)kp; x.kd_Nm_s_per_rad = (float)kd; x.iq_ref_A = (float)iq;
            if (!strcmp(cmd, "pack1601")) { st = am_codec_pack_rx1601(&c, (am_controlword)cw, &x, b); n = 28; }
            else if (!strcmp(cmd, "pack1600")) { st = am_codec_pack_rx1600(&c, (am_controlword)cw, &x, b); n = 20; }
            else { st = am_codec_pack_cfd_slot(&c, (uint8_t)ctl, (uint8_t)cnt, &x, b); n = 16; }
            printf("P ");
            if (st == AM_OK) hex(b, n);
            printf("\n");
        } else if (!strcmp(cmd, "unpacktx") || !strcmp(cmd, "unpackfb")) {
            unsigned rt, ra;
            am_codec c;
            am_telemetry t;
            uint8_t b[128], echo = 0;
            size_t n;
            sscanf(line, "%*s %u %u %255s", &rt, &ra, rest);
            n = unhex(rest, b, sizeof b);
            am_codec_init(&c, rt, ra, 20, 1, 0);
            memset(&t, 0, sizeof t);
            if (!strcmp(cmd, "unpacktx")) {
                st = am_codec_unpack_tx1a00(&c, b, &t);
                if (st == AM_OK && n >= 84) st = am_codec_unpack_tx1a01(&c, b + 24, &t);
            } else {
                st = am_codec_unpack_cfd_fb(&c, b, n, &t, &echo);
            }
            if (st == AM_OK)
                printf("U %d %d %.17g %.9g %.9g %.9g %.9g %u %d %d %llu %d %.9g %.9g %u %u\n", (int)t.state, (int)t.mode,
                       t.q_rad, t.qd_rad_s, t.tau_est_Nm, t.iq_A, t.t_winding_C, t.statusword, (int)t.ref_state,
                       t.extended_valid ? 1 : 0, (unsigned long long)t.t_ns, (int)t.q_motor_counts, t.vbus_V,
                       t.q_out_encoder_rad, t.warning_word, echo);
        } else {
            st = AM_ERR_PARAM;
        }
        printf("R %s %d\n", cmd, (int)st);
    }
    am_deinit(S);
    if (TP_USER) am_sim_transport_close(TP_USER);
    return 0;
}
