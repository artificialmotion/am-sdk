/* SPDX-License-Identifier: Apache-2.0 */
/* L2 session over a transport: the wire-level simulator behind am_transport_ops.
 * Every cycle goes through the codec and every configuration call is an object access. */
#include "../src/am_internal.h"
#include "am_test.h"

#include <string.h>

typedef struct { am_session *s; am_joint *j; am_sim_joint *sj; const am_transport_ops *ops; void *user; } rig;

static rig make_wire(const char *units, am_bus bus, bool compact, bool extended, double load)
{
    rig r;
    am_session_config cfg;
    am_sim_options o;
    memset(&r, 0, sizeof r);
    memset(&cfg, 0, sizeof cfg);
    am_sim_transport_open(units, bus, 1000, &r.ops, &r.user);
    am_sim_options_default(&o);
    o.load_inertia_kgm2 = load;
    am_sim_configure(am_sim_transport_joint(r.user, 0), &o);
    cfg.bus = bus;
    cfg.canfd_compact = compact;
    cfg.telemetry_extended = extended;
    cfg.transport = r.ops;
    cfg.transport_user = r.user;
    am_init(&cfg, &r.s);
    am_open_joint(r.s, 0, &r.j);
    r.sj = am_sim_transport_joint(r.user, 0);
    return r;
}

static rig make_direct(const char *unit, double load)
{
    rig r;
    char iface[32];
    am_session_config cfg;
    am_sim_options o;
    memset(&r, 0, sizeof r);
    memset(&cfg, 0, sizeof cfg);
    if (snprintf(iface, sizeof iface, "sim:%s", unit) < 0) iface[0] = '\0';
    cfg.interface = iface;
    cfg.telemetry_extended = true;
    am_init(&cfg, &r.s);
    am_open_joint(r.s, 0, &r.j);
    r.sj = am_sim_of(r.j);
    am_sim_options_default(&o);
    o.load_inertia_kgm2 = load;
    am_sim_configure(r.sj, &o);
    return r;
}

static void close_rig(rig *r)
{
    am_deinit(r->s);
    if (r->user) am_sim_transport_close(r->user);
}

/* impedance sine through both paths must give the same motion */
static void run_sine(rig *r, double *q_out, int n)
{
    int k;
    am_impedance imp;
    am_telemetry t;
    am_start(r->s);
    am_set_mode(r->j, AM_MODE_IMPEDANCE);
    am_enable(r->j, 2000);
    memset(&imp, 0, sizeof imp);
    imp.kp_Nm_per_rad = 120.0f;
    imp.kd_Nm_s_per_rad = 5.0f;
    for (k = 0; k < n; k++) {
        double tt = k * 1e-3;
        imp.q_ref_rad = 0.4 * sin(AM_TWO_PI * tt);
        imp.qd_ref_rad_s = (float)(0.4 * AM_TWO_PI * cos(AM_TWO_PI * tt));
        am_set_impedance(r->j, &imp);
        am_cycle(r->s, NULL);
        q_out[k] = am_read_telemetry(r->j, &t) == AM_OK ? t.q_rad : 1e9;
    }
}

static void test_wire_matches_direct(void)
{
    static double a[1500], b[1500], c[1500];
    rig d = make_direct("M", 0.1), w = make_wire("M", AM_BUS_ETHERCAT, false, true, 0.1),
        cf = make_wire("M", AM_BUS_CANFD, true, true, 0.1);
    double worst = 0, worst_c = 0;
    int k;
    run_sine(&d, a, 1500);
    run_sine(&w, b, 1500);
    run_sine(&cf, c, 1500);
    for (k = 0; k < 1500; k++) {
        if (fabs(a[k] - b[k]) > worst) worst = fabs(a[k] - b[k]);
        if (fabs(a[k] - c[k]) > worst_c) worst_c = fabs(a[k] - c[k]);
    }
    CHECK(worst < 2e-4);        /* EtherCAT PDOs: urad position, mNm torque */
    CHECK(worst_c < 2e-3);      /* compact: 10 mrad/s, 10 mNm, 20 mNm/rad steps */
    close_rig(&d);
    close_rig(&w);
    close_rig(&cf);
}

static void test_identity_over_the_wire(void)
{
    rig d = make_direct("H-F", 0), w = make_wire("H-F", AM_BUS_ETHERCAT, false, true, 0);
    am_identity a, b;
    am_transparency ta, tb;
    am_device_info dev[4];
    size_t n = 0;
    CHECK_ST(am_scan(w.s, dev, 4, &n), AM_OK);
    CHECK(n == 1 && dev[0].product_code == 4 && dev[0].is_am_unit && dev[0].has_control_authority);
    am_get_identity(d.j, &a);
    am_get_identity(w.j, &b);
    CHECK(a.sku == b.sku && a.ratio_num == b.ratio_num && a.ratio_den == b.ratio_den && a.serial == b.serial);
    CHECK(memcmp(a.uid, b.uid, 12) == 0 && a.cal_record_crc32 == b.cal_record_crc32);
    CHECK(strcmp(a.device_name, b.device_name) == 0);
    am_get_transparency(d.j, &ta);
    am_get_transparency(w.j, &tb);
    CHECK_NEAR(ta.rated_torque_Nm, tb.rated_torque_Nm, 1e-3);
    CHECK_NEAR(ta.peak_torque_2s_Nm, tb.peak_torque_2s_Nm, 1e-3);
    CHECK_NEAR(ta.backdrive_torque_Nm, tb.backdrive_torque_Nm, 1e-3);
    CHECK_NEAR(ta.reflected_inertia_kgm2, tb.reflected_inertia_kgm2, 1e-7);
    CHECK(tb.basis == AM_BASIS_DESIGN_TARGET);
    CHECK(tb.collision_latency_s != tb.collision_latency_s);     /* NaN: not available */
    close_rig(&d);
    close_rig(&w);
}

static void test_limits_gains_label_over_the_wire(void)
{
    rig w = make_wire("H-F", AM_BUS_ETHERCAT, false, true, 0);
    am_limits eff, ceil, bad, ok, back;
    am_loop_gains g, g2;
    am_identity id;
    char cal[1024];
    size_t len = 0;
    CHECK_ST(am_get_limits(w.j, &eff, &ceil), AM_OK);
    CHECK_ST(am_set_limits(w.j, &eff), AM_OK);              /* read-back values can be written again */
    bad = eff;
    bad.torque_pos_Nm = ceil.torque_pos_Nm * 1.02f;
    CHECK_ST(am_set_limits(w.j, &bad), AM_ERR_LIMIT);
    ok = eff;
    ok.torque_pos_Nm = 20.0f;
    ok.torque_neg_Nm = 15.0f;
    ok.pos_min_rad = -1.0;
    ok.pos_max_rad = 1.0;
    CHECK_ST(am_set_limits(w.j, &ok), AM_OK);
    am_get_limits(w.j, &back, NULL);
    CHECK_NEAR(back.torque_pos_Nm, 20.0, 0.06);
    CHECK_NEAR(back.pos_max_rad, 1.0, 1e-6);
    CHECK_ST(am_get_loop_gains(w.j, &g), AM_OK);
    g.pos_kp_1_per_s = 20.0f;
    CHECK_ST(am_set_loop_gains(w.j, &g), AM_OK);
    am_get_loop_gains(w.j, &g2);
    CHECK_NEAR(g2.pos_kp_1_per_s, 20.0, 1e-6);
    g.vel_filter_hz = 5.0f;
    CHECK_ST(am_set_loop_gains(w.j, &g), AM_ERR_LIMIT);
    CHECK_ST(am_set_user_label(w.j, "left knee, joint 4"), AM_OK);
    am_get_identity(w.j, &id);
    CHECK(strcmp(id.user_label, "left knee, joint 4") == 0);
    CHECK_ST(am_read_calibration(w.j, cal, sizeof cal, &len), AM_OK);
    CHECK(strstr(cal, "design_target") != NULL);
    CHECK_ST(am_store_parameters(w.j), AM_OK);
    CHECK_ST(am_restore_defaults(w.j), AM_OK);
    am_get_identity(w.j, &id);
    CHECK(id.user_label[0] == '\0');
    close_rig(&w);
}

static int n_ev[8];
static void on_event(const am_event *ev, void *user)
{
    (void)user;
    if (ev->kind >= 1 && ev->kind <= 7) n_ev[ev->kind]++;
}

static void test_faults_sto_events_over_the_wire(void)
{
    rig w;
    am_session_config cfg;
    am_telemetry t;
    am_fault_info fi;
    am_fault_record rec[2];
    size_t n = 0;
    int k;
    memset(n_ev, 0, sizeof n_ev);
    memset(&w, 0, sizeof w);
    memset(&cfg, 0, sizeof cfg);
    am_sim_transport_open("M", AM_BUS_ETHERCAT, 1000, &w.ops, &w.user);
    {
        am_sim_options o;
        am_sim_options_default(&o);
        o.load_inertia_kgm2 = 1e4;
        am_sim_configure(am_sim_transport_joint(w.user, 0), &o);
    }
    cfg.transport = w.ops;
    cfg.transport_user = w.user;
    cfg.telemetry_extended = true;
    cfg.event_cb = on_event;
    CHECK_ST(am_init(&cfg, &w.s), AM_OK);
    CHECK_ST(am_open_joint(w.s, 0, &w.j), AM_OK);
    w.sj = am_sim_transport_joint(w.user, 0);
    CHECK_ST(am_start(w.s), AM_OK);
    CHECK_ST(am_set_mode(w.j, AM_MODE_CST), AM_OK);
    CHECK_ST(am_enable(w.j, 1000), AM_OK);
    for (k = 0; k < 5; k++) {
        am_set_torque(w.j, 20.0f);
        am_cycle(w.s, NULL);
    }
    CHECK_NEAR(w.sj->tau_applied, 20.0, 1e-3);
    am_sim_set_sto(w.sj, true);
    am_cycle(w.s, NULL);
    CHECK(w.sj->tau_applied == 0.0);
    am_read_telemetry(w.j, &t);
    CHECK(t.sto_active);
    CHECK_ST(am_enable(w.j, 50), AM_ERR_STO);
    am_sim_set_sto(w.sj, false);
    am_cycle(w.s, NULL);
    CHECK_ST(am_enable(w.j, 1000), AM_OK);
    am_sim_inject_fault(w.sj, 0x100B);
    for (k = 0; k < 3; k++) am_cycle(w.s, NULL);
    am_read_telemetry(w.j, &t);
    CHECK(t.state == AM_DS_FAULT && t.error_code == 0x100B);
    CHECK_ST(am_get_faults(w.j, &fi), AM_OK);
    CHECK(fi.latched && (fi.fault_word & (1u << 11)));
    CHECK_ST(am_get_fault_history(w.j, rec, 2, &n), AM_OK);
    CHECK(n == 1 && rec[0].code == 0x100B);
    am_sim_clear_fault_cause(w.sj, 0);
    CHECK_ST(am_clear_faults(w.j, 500), AM_OK);
    CHECK_ST(am_enable(w.j, 1000), AM_OK);
    CHECK(n_ev[AM_EV_STO_CHANGED] >= 2 && n_ev[AM_EV_FAULT] >= 1 && n_ev[AM_EV_FAULT_CLEARED] >= 1);
    am_stop(w.s);
    CHECK(w.sj->state == AM_DS_SWITCH_ON_DISABLED);
    CHECK(n_ev[AM_EV_LINK_LOST] >= 1);
    close_rig(&w);
}

static void test_csp_holds_and_stale(void)
{
    rig w = make_wire("M", AM_BUS_ETHERCAT, false, false, 0.05);
    am_telemetry t;
    am_sim_options o;
    int k;
    am_sim_options_default(&o);
    o.load_inertia_kgm2 = 0.05;
    o.q0_rad = 1.0;
    am_sim_configure(w.sj, &o);
    CHECK_ST(am_start(w.s), AM_OK);
    CHECK_ST(am_set_mode(w.j, AM_MODE_CSP), AM_OK);
    CHECK_ST(am_enable(w.j, 1000), AM_OK);
    for (k = 0; k < 20; k++) am_cycle(w.s, NULL);
    CHECK(fabs(w.sj->tau_applied) < 1e-3);
    am_read_telemetry(w.j, &t);
    CHECK_NEAR(t.q_rad, 1.0, 1e-5);
    CHECK(!t.extended_valid);                  /* 0x1A01 not assigned */
    close_rig(&w);
}

int main(void)
{
    RUN(test_wire_matches_direct);
    RUN(test_identity_over_the_wire);
    RUN(test_limits_gains_label_over_the_wire);
    RUN(test_faults_sto_events_over_the_wire);
    RUN(test_csp_holds_and_stale);
    TEST_EXIT();
}
