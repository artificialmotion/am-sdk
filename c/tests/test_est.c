/* SPDX-License-Identifier: Apache-2.0 */
/* Torque estimator and collision detector (am_est.h). Mirrors python/tests/test_estimation.py.
 * SIMULATED values only; limits here are regression limits, not product accuracy. */
#include "am_est.h"
#include "am_sim.h"
#include "am_test.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TWO_PI 6.283185307179586

static const char *const UNITS[] = {"S-S", "S-D", "M", "H-F", "H-T", "H-X"};

static void test_calibration_defaults_and_check(void)
{
    am_est_calibration cal, bad;
    size_t u;
    for (u = 0; u < sizeof UNITS / sizeof UNITS[0]; u++) {
        CHECK_ST(am_est_calibration_default(UNITS[u], &cal), AM_OK);
        CHECK_ST(am_est_calibration_check(&cal), AM_OK);
        CHECK(cal.basis == AM_BASIS_DESIGN_TARGET);
        CHECK(cal.breakaway_Nm > 0.0 && cal.coulomb_Nm < cal.breakaway_Nm);
        CHECK(cal.eta_reverse <= cal.eta_forward);
    }
    CHECK_ST(am_est_calibration_default("X-9", &cal), AM_ERR_PARAM);
    CHECK_ST(am_est_calibration_default(NULL, &cal), AM_ERR_PARAM);
    CHECK_ST(am_est_calibration_check(NULL), AM_ERR_PARAM);
    am_est_calibration_default("M", &cal);
    bad = cal;
    bad.coulomb_Nm = 99.0;
    CHECK_ST(am_est_calibration_check(&bad), AM_ERR_PARAM);
    bad = cal;
    bad.eta_forward = 1.2;
    CHECK_ST(am_est_calibration_check(&bad), AM_ERR_PARAM);
    bad = cal;
    bad.friction_map_n = 2;
    bad.friction_map_w_rad_s[0] = 1.0;
    bad.friction_map_w_rad_s[1] = 0.5;
    CHECK_ST(am_est_calibration_check(&bad), AM_ERR_PARAM);
    bad = cal;
    bad.friction_map_n = AM_EST_FRICTION_MAP_MAX + 1;
    CHECK_ST(am_est_calibration_check(&bad), AM_ERR_PARAM);
    bad = cal;
    bad.kt_Nm_per_A = NAN;
    CHECK_ST(am_est_calibration_check(&bad), AM_ERR_PARAM);
    bad = cal;
    bad.eff_n_temps = 1;     /* loads missing */
    CHECK_ST(am_est_calibration_check(&bad), AM_ERR_PARAM);
}

/* Steady current that delivers tau_t at the gear output at speed w (the model's forward direction). */
static double forward_iq(const am_est_calibration *cal, double tau_t, double w, double T)
{
    double g = tau_t * w > 0 ? 1.0 / cal->eta_forward : cal->eta_reverse;
    double tm = am_est_friction_at(cal, w, T, 0.003) + g * tau_t;
    return tm / (cal->gear_ratio * am_est_kt_at(cal, T, 0.0));
}

static void test_current_model_inverts_forward_model(void)
{
    am_est_calibration cal;
    am_est_torque e;
    const double ws[] = {-3.0, -0.5, 0.4, 2.0};
    const double taus[] = {-12.0, -3.0, 2.0, 10.0};
    size_t a, b;
    am_est_calibration_default("S-S", &cal);
    cal.bearing_Nm = 0.04;
    for (a = 0; a < 4; a++) {
        for (b = 0; b < 4; b++) {
            double iq = forward_iq(&cal, taus[b], ws[a], 50.0);
            CHECK_ST(am_est_current_model(&cal, NULL, iq, ws[a], 0.0, 50.0, false, &e), AM_OK);
            CHECK_NEAR(e.tau_out_Nm, taus[b] - 0.04 * tanh(ws[a] / 0.01), 1e-3 * fabs(taus[b]) + 1e-6);
            CHECK(e.quality == AM_EST_Q_OK);
            CHECK(e.bound_Nm > 0.0);
        }
    }
}

static void test_stick_band_and_temperature(void)
{
    am_est_calibration cal;
    am_est_torque e, hot, cold, none, ref;
    double tm, band, lo, hi;
    am_est_calibration_default("S-S", &cal);
    am_est_current_model(&cal, NULL, 1.0, 0.0, 0.0, 40.0, false, &e);
    CHECK(e.quality & AM_EST_Q_STICK);
    tm = cal.gear_ratio * am_est_kt_at(&cal, 40.0, 1.0);
    band = am_est_stick_band(&cal, tm, 40.0);
    lo = (tm - cal.breakaway_Nm) * cal.eta_forward;
    hi = (tm + cal.breakaway_Nm) / cal.eta_reverse;
    CHECK_NEAR(band, 0.5 * (hi - lo), 1e-9);
    CHECK(e.bound_Nm >= band);
    am_est_current_model(&cal, NULL, 1.0, 0.0, 0.0, 40.0, true, &e);
    CHECK(e.quality & AM_EST_Q_DITHER);
    CHECK(!(e.quality & AM_EST_Q_STICK));
    am_est_current_model(&cal, NULL, 2.0, 1.0, 0.0, 100.0, false, &hot);
    am_est_current_model(&cal, NULL, 2.0, 1.0, 0.0, 20.0, false, &cold);
    CHECK(hot.tau_motor_Nm < cold.tau_motor_Nm);
    am_est_current_model(&cal, NULL, 2.0, 1.0, 0.0, NAN, false, &none);
    am_est_current_model(&cal, NULL, 2.0, 1.0, 0.0, cal.kt_ref_temp_C, false, &ref);
    CHECK(none.quality & AM_EST_Q_NO_TEMPERATURE);
    CHECK_NEAR(none.tau_out_Nm, ref.tau_out_Nm, 1e-12);
    CHECK(none.bound_Nm > ref.bound_Nm);
    CHECK_NEAR(am_est_dither_amplitude_max(&cal), fmin(1.5 * cal.breakaway_Nm, 0.1 * cal.rated_torque_Nm), 1e-12);
    CHECK(fabs(am_est_dither_torque(&cal, 100.0, 60.0, 1.0 / 240.0)) <= am_est_dither_amplitude_max(&cal) + 1e-12);
}

/* Golden values from the Python implementation (am_sdk.estimation.current_model). The full
 * sequence comparison is python/tests/test_cross_check_est.py. */
#define GOLD_M_TAU_OUT 7.7153439516397775
#define GOLD_M_TAU_DRIVE 8.025368751639776
#define GOLD_M_BOUND 0.40799200153361465
#define GOLD_HX_TAU_OUT -16.500651411346034
#define GOLD_HX_BOUND 2.664259448237209

static void test_golden_values_match_python(void)
{
    am_est_calibration cal;
    am_est_torque e;
    am_est_calibration_default("M", &cal);
    am_est_current_model(&cal, NULL, 3.25, 0.8, 12.0, 47.5, false, &e);
    CHECK_NEAR(e.tau_out_Nm, GOLD_M_TAU_OUT, 1e-9);
    CHECK_NEAR(e.tau_drive_Nm, GOLD_M_TAU_DRIVE, 1e-9);
    CHECK_NEAR(e.bound_Nm, GOLD_M_BOUND, 1e-9);
    CHECK(e.quality == AM_EST_Q_OK);
    am_est_calibration_default("H-X", &cal);
    am_est_current_model(&cal, NULL, -6.0, -0.02, 0.0, 70.0, false, &e);
    CHECK_NEAR(e.tau_out_Nm, GOLD_HX_TAU_OUT, 1e-9);
    CHECK_NEAR(e.bound_Nm, GOLD_HX_BOUND, 1e-9);
    CHECK(e.quality == AM_EST_Q_AMBIGUOUS);
}

static void test_estimator_reversal_hold_and_invalid(void)
{
    am_est_calibration cal;
    am_est est;
    am_est_input in;
    am_est_torque e;
    int k, first_moving = -1;
    uint32_t last = 0, at_first = 0;
    am_est_calibration_default("S-D", &cal);
    CHECK_ST(am_est_init(&est, &cal, NULL), AM_OK);
    memset(&in, 0, sizeof in);
    in.dt_s = 1e-3;
    in.iq_A = 0.5;
    in.temp_C = 40.0;
    in.valid = true;
    am_est_update(&est, &in, &e);
    CHECK(e.quality & AM_EST_Q_STICK);
    for (k = 0; k < 100; k++) {
        in.q_rad += 0.5e-3;
        am_est_update(&est, &in, &e);
        if (first_moving < 0 && !(e.quality & AM_EST_Q_STICK)) {
            first_moving = k;
            at_first = e.quality;
        }
        last = e.quality;
    }
    CHECK(first_moving >= 0);
    CHECK(at_first & AM_EST_Q_AMBIGUOUS);
    CHECK(last == AM_EST_Q_OK);
    in.q_rad = NAN;
    CHECK_ST(am_est_update(&est, &in, &e), AM_OK);
    CHECK(e.quality == AM_EST_Q_INVALID);
    CHECK(e.confidence == 0.0);
    CHECK_ST(am_est_init(&est, NULL, NULL), AM_ERR_PARAM);
    cal.eta_forward = 2.0;
    CHECK_ST(am_est_init(&est, &cal, NULL), AM_ERR_PARAM);
}

static void test_collision_step_and_release(void)
{
    am_collision det;
    am_collision_config cc;
    am_collision_output o;
    const double J = 0.07, dt = 1e-3;
    double qd = 1.0, r199 = 0.0, r299 = 0.0;
    int k, onsets = 0, onset_k = -1;
    am_collision_config_default(&cc);
    CHECK_ST(am_collision_init(&det, &cc), AM_ERR_PARAM);     /* inertia missing */
    cc.inertia_kgm2 = J;
    cc.threshold_Nm = 1.0;
    cc.min_hold_s = 0.02;
    CHECK_ST(am_collision_init(&det, &cc), AM_OK);
    for (k = 0; k < 600; k++) {
        double contact = (k >= 200 && k < 300) ? -3.0 : 0.0;
        qd += dt * (0.5 + contact) / J;
        CHECK_ST(am_collision_update(&det, dt, qd, 0.5, 0.0, 0, &o), AM_OK);
        if (o.onset) {
            onsets++;
            onset_k = k;
        }
        if (k == 199) r199 = o.residual_Nm;
        if (k == 299) r299 = o.residual_Nm;
    }
    CHECK(onsets == 1);
    CHECK(onset_k >= 200 && onset_k <= 203);
    CHECK_NEAR(r299, -3.0, 0.01);
    CHECK(fabs(r199) < 1e-9);
    CHECK(!det.active);
    CHECK(det.events == 1);
    am_collision_update(&det, dt, qd, 0.0, 0.0, AM_EST_Q_STICK, &o);
    CHECK_NEAR(o.threshold_Nm, 2.0, 1e-12);
    {
        const double r[] = {0.1, -0.5, NAN, 0.3};
        CHECK_NEAR(am_collision_calibrate_threshold(r, 4, 1.1), 0.55, 1e-12);
    }
}

/* On the built-in simulator: impedance sine, then an external torque step. The default simulator plant
 * has Coulomb friction only and no gear loss, so the calibration is set to match it. */
static double g_contact = 0.0;
static double contact_fn(double t, double q, double qd, void *user)
{
    (void)q;
    (void)qd;
    (void)user;
    return t >= 2.6 ? g_contact : 0.0;
}

static void test_detects_contact_on_the_simulator(void)
{
    am_session_config cfg;
    am_session *s;
    am_joint *j;
    am_sim_options so;
    am_est_calibration cal;
    am_est est;
    am_collision det;
    am_collision_config cc;
    double rmax = 0.0;
    int k, early = 0, armed = 0;
    double onset_t = -1.0;
    memset(&cfg, 0, sizeof cfg);
    cfg.interface = "sim:M";
    cfg.cycle_us = 1000;
    cfg.telemetry_extended = true;
    CHECK_ST(am_init(&cfg, &s), AM_OK);
    am_open_joint(s, 0, &j);
    am_sim_options_default(&so);
    so.load_inertia_kgm2 = 0.1;
    am_sim_configure(am_sim_of(j), &so);
    am_sim_set_external_torque(am_sim_of(j), contact_fn, NULL);
    am_start(s);
    am_set_mode(j, AM_MODE_IMPEDANCE);
    CHECK_ST(am_enable(j, 2000), AM_OK);
    am_est_calibration_default("M", &cal);
    cal.kt_Nm_per_A *= 0.926;                          /* the sim folds the gear efficiency into Kt */
    cal.kt_temp_coeff_per_K = 0.0;
    cal.eta_forward = cal.eta_reverse = 1.0;
    cal.coulomb_Nm = cal.breakaway_Nm;
    cal.viscous_Nm_s_per_rad = 0.0;
    CHECK_ST(am_est_init(&est, &cal, NULL), AM_OK);
    am_collision_config_default(&cc);
    cc.inertia_kgm2 = cal.reflected_inertia_kgm2 + 0.1;
    am_collision_init(&det, &cc);
    g_contact = -0.3 * cal.rated_torque_Nm;
    for (k = 0; k < 3000; k++) {
        double t = k * 1e-3;
        am_impedance imp;
        am_telemetry tl;
        am_est_input in;
        am_est_torque e;
        am_collision_output o;
        double ts;
        memset(&imp, 0, sizeof imp);
        imp.q_ref_rad = 0.3 * sin(TWO_PI * 0.5 * t);
        imp.qd_ref_rad_s = (float)(0.3 * TWO_PI * 0.5 * cos(TWO_PI * 0.5 * t));
        imp.kp_Nm_per_rad = 150.0f;
        imp.kd_Nm_s_per_rad = 6.0f;
        am_set_impedance(j, &imp);
        am_cycle(s, NULL);
        am_read_telemetry(j, &tl);
        am_est_input_from_telemetry(&tl, 1e-3, &in);
        CHECK(in.temp_C == in.temp_C);                 /* extended telemetry: temperature present */
        am_est_update(&est, &in, &e);
        am_collision_update(&det, 1e-3, tl.qd_rad_s, e.tau_drive_Nm, 0.0, e.quality, &o);
        ts = tl.t_ns * 1e-9;                           /* simulator time (the enable walk took a few cycles) */
        if (!armed && ts >= 2.2) {                     /* threshold from one contact-free period */
            armed = 1;
            det.cfg.threshold_Nm = 1.1 * rmax;
            CHECK(det.cfg.threshold_Nm < 0.25 * cal.rated_torque_Nm);
        }
        if (ts > 0.2 && ts < 2.2 && fabs(o.residual_Nm) > rmax) rmax = fabs(o.residual_Nm);
        if (o.onset) {
            if (ts < 2.6) early++;
            else if (onset_t < 0) onset_t = ts;
        }
    }
    CHECK(early == 0);
    CHECK(onset_t >= 2.6 && onset_t <= 2.6 + 0.0105);  /* simulator regression limit: 10 ms */
    am_deinit(s);
}

int main(void)
{
    RUN(test_calibration_defaults_and_check);
    RUN(test_current_model_inverts_forward_model);
    RUN(test_stick_band_and_temperature);
    RUN(test_golden_values_match_python);
    RUN(test_estimator_reversal_hold_and_invalid);
    RUN(test_collision_step_and_release);
    RUN(test_detects_contact_on_the_simulator);
    TEST_EXIT();
}
