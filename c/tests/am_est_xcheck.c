/* SPDX-License-Identifier: Apache-2.0 */
/* Estimator cross-check runner: runs a fixed synthetic input sequence through am_est and
 * am_collision and prints every output with 17 significant digits. python/tests/test_cross_check_est.py
 * generates the same inputs, runs am_sdk.estimation and compares line by line.
 * Usage: am_est_xcheck <unit> <variant 0|1> */
#include "am_est.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define XC_TWO_PI 6.283185307179586

static void variant_cal(am_est_calibration *cal, int variant)
{
    if (variant != 1) return;
    cal->kt_sat_frac = 0.05;
    cal->kt_sat_current_A = 1.5;
    cal->bearing_Nm = 0.03;
    cal->friction_map_n = 3;
    cal->friction_map_w_rad_s[0] = 0.0;
    cal->friction_map_w_rad_s[1] = 0.5;
    cal->friction_map_w_rad_s[2] = 3.0;
    cal->friction_map_Nm[0] = 0.6;
    cal->friction_map_Nm[1] = 0.45;
    cal->friction_map_Nm[2] = 0.8;
    cal->eff_n_temps = 2;
    cal->eff_n_loads = 3;
    cal->eff_temps_C[0] = 20.0;
    cal->eff_temps_C[1] = 80.0;
    cal->eff_loads_Nm[0] = 0.0;
    cal->eff_loads_Nm[1] = 5.0;
    cal->eff_loads_Nm[2] = 20.0;
    cal->eff_forward[0][0] = 0.88; cal->eff_forward[0][1] = 0.92; cal->eff_forward[0][2] = 0.94;
    cal->eff_forward[1][0] = 0.90; cal->eff_forward[1][1] = 0.93; cal->eff_forward[1][2] = 0.95;
    cal->eff_reverse[0][0] = 0.78; cal->eff_reverse[0][1] = 0.84; cal->eff_reverse[0][2] = 0.86;
    cal->eff_reverse[1][0] = 0.80; cal->eff_reverse[1][1] = 0.85; cal->eff_reverse[1][2] = 0.88;
}

int main(int argc, char **argv)
{
    am_est_calibration cal;
    am_est est;
    am_collision det;
    am_collision_config cc;
    int variant, k;
    const double dt = 1e-3;
    if (argc < 3) {
        fprintf(stderr, "usage: am_est_xcheck <unit> <variant>\n");
        return 2;
    }
    variant = atoi(argv[2]);
    if (am_est_calibration_default(argv[1], &cal) != AM_OK) return 2;
    variant_cal(&cal, variant);
    if (am_est_init(&est, &cal, NULL) != AM_OK) return 3;
    am_collision_config_default(&cc);
    cc.inertia_kgm2 = cal.reflected_inertia_kgm2 + 0.05;
    cc.threshold_Nm = 0.8;
    if (am_collision_init(&det, &cc) != AM_OK) return 4;
    for (k = 0; k < 3000; k++) {
        double t = k * dt;
        int hold = k >= 1200 && k < 1500;                 /* integer bounds: no float edge cases */
        double tq = (k < 1200 ? k : (hold ? 1200 : k - 300)) * dt;
        am_est_input in;
        am_est_torque e;
        am_collision_output o;
        double qd_c;
        in.dt_s = dt;
        in.q_rad = 0.4 * sin(XC_TWO_PI * 0.7 * tq) + 0.05 * sin(XC_TWO_PI * 3.1 * tq);
        in.iq_A = 2.0 * sin(XC_TWO_PI * 1.3 * t) + 0.3;
        in.temp_C = (k % 500 < 450) ? 30.0 + 20.0 * t : NAN;
        in.dither_active = k >= 1350 && k < 1450;
        in.saturated = false;
        in.valid = k != 2900;
        am_est_update(&est, &in, &e);
        qd_c = !hold ? 0.4 * XC_TWO_PI * 0.7 * cos(XC_TWO_PI * 0.7 * tq) +
                                           0.05 * XC_TWO_PI * 3.1 * cos(XC_TWO_PI * 3.1 * tq)
                                     : 0.0;
        am_collision_update(&det, dt, qd_c, e.tau_drive_Nm, -1.5 * cos(in.q_rad), e.quality, &o);
        printf("%d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %u %.17g %.17g %d %d\n", k, e.tau_out_Nm,
               e.tau_drive_Nm, e.tau_motor_Nm, e.friction_Nm, e.qd_rad_s, e.qdd_rad_s2, e.bound_Nm, e.confidence,
               (unsigned)e.quality, o.residual_Nm, o.threshold_Nm, o.active ? 1 : 0, o.onset ? 1 : 0);
    }
    return 0;
}
