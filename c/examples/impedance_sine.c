/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Impedance control of one M joint following a sine (same as python/examples/01_impedance_sine.py).
 *
 * The control loop below is the code that drives a real joint. Only the session configuration
 * changes: set cfg.transport to your EtherCAT master or CAN driver instead of interface "sim:M".
 */
#include "am_api.h"
#include "am_sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define K 150.0f        /* N*m/rad */
#define D 6.0f          /* N*m*s/rad */
#define AMP 0.5         /* rad */
#define FREQ 0.5        /* Hz */
#define LOAD_J 0.15     /* kg*m^2 of link the joint carries */
#define PI 3.141592653589793

#define TRY(x) do { am_status st_ = (x); if (st_ != AM_OK) { \
    fprintf(stderr, "%s: %s\n", #x, am_status_str(st_)); return 1; } } while (0)

int main(void)
{
    am_session_config cfg;
    am_session *s;
    am_joint *joint;
    am_sim_options opt;
    am_impedance imp;
    am_telemetry t;
    am_transparency tr;
    double err_sq = 0.0, peak_tau = 0.0, peak_temp = 0.0;
    int k, n = 0;

    memset(&cfg, 0, sizeof cfg);
    cfg.bus = AM_BUS_ETHERCAT;
    cfg.interface = "sim:M";            /* one M joint in the built-in simulator */
    cfg.cycle_us = 1000;                 /* 1 kHz */
    cfg.telemetry_extended = true;
    TRY(am_init(&cfg, &s));
    TRY(am_open_joint(s, 0, &joint));

    am_sim_options_default(&opt);        /* simulator only: the link the joint carries */
    opt.load_inertia_kgm2 = LOAD_J;
    TRY(am_sim_configure(am_sim_of(joint), &opt));

    TRY(am_start(s));
    TRY(am_set_mode(joint, AM_MODE_IMPEDANCE));
    TRY(am_enable(joint, 1000));         /* Shutdown, Switch on, Enable operation */

    memset(&imp, 0, sizeof imp);
    imp.kp_Nm_per_rad = K;
    imp.kd_Nm_s_per_rad = D;
    for (k = 0; k < 6000; k++) {         /* 6 s */
        double time_s = k * 1e-3;
        imp.q_ref_rad = AMP * sin(2 * PI * FREQ * time_s);
        imp.qd_ref_rad_s = (float)(AMP * 2 * PI * FREQ * cos(2 * PI * FREQ * time_s));
        TRY(am_set_impedance(joint, &imp));
        TRY(am_cycle(s, NULL));          /* in your RT thread: once per cycle */
        TRY(am_read_telemetry(joint, &t));
        if (time_s > 2.0) {              /* skip the start-up transient */
            err_sq += (imp.q_ref_rad - t.q_rad) * (imp.q_ref_rad - t.q_rad);
            n++;
        }
        if (fabs(t.tau_est_Nm) > peak_tau) peak_tau = fabs(t.tau_est_Nm);
        if (t.t_winding_C > peak_temp) peak_temp = t.t_winding_C;
    }
    TRY(am_get_transparency(joint, &tr));
    printf("M joint, impedance K=%g D=%g, sine %.2f rad at %.1f Hz, load %.2f kg m^2\n", K, D, AMP, FREQ, LOAD_J);
    printf("RMS tracking error  %.4f rad\n", sqrt(err_sq / n));
    printf("peak torque         %.1f N*m   (peak rating %.0f N*m)\n", peak_tau, tr.peak_torque_2s_Nm);
    printf("winding temperature %.2f C (model)\n", peak_temp);
    printf("final state         %d (4 = Operation enabled)\n", (int)t.state);
    am_deinit(s);
    return 0;
}
