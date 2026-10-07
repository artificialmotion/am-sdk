/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Two-joint leg: hip (H-F) and knee (M) of a planar two-link leg doing a squat
 * (same as python/examples/02_leg_two_joints.py).
 *
 * Gravity acts on the links through the simulator's external-torque hook. The controller is a
 * joint-space impedance law with gravity feed-forward, written only with am_* calls.
 */
#include "am_api.h"
#include "am_sim.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define G 9.81
#define M1 4.0
#define L1 0.40          /* thigh */
#define M2 3.0
#define L2 0.40          /* shank */
#define PI 3.141592653589793

#define TRY(x) do { am_status st_ = (x); if (st_ != AM_OK) { \
    fprintf(stderr, "%s: %s\n", #x, am_status_str(st_)); return 1; } } while (0)

/* angles from the downward vertical; knee angle relative to the thigh */
static void gravity(double q1, double q2, double *tau_hip, double *tau_knee)
{
    *tau_knee = M2 * G * (L2 / 2) * sin(q1 + q2);
    *tau_hip = G * (M1 * L1 / 2 + M2 * L1) * sin(q1) + *tau_knee;
}

static am_sim_joint *SIM[2];

/* the plant sees gravity from both joints' true positions */
static double grav_torque(double t, double q, double qd, void *user)
{
    double q1, q2, th, tk;
    int which = *(int *)user;
    (void)t; (void)q; (void)qd;
    am_sim_get_plant(SIM[0], &q1, NULL, NULL, NULL);
    am_sim_get_plant(SIM[1], &q2, NULL, NULL, NULL);
    gravity(q1, q2, &th, &tk);
    return which == 0 ? -th : -tk;
}

int main(void)
{
    static int idx[2] = {0, 1};
    const float KP[2] = {600.0f, 400.0f}, KD[2] = {25.0f, 15.0f};
    am_session_config cfg;
    am_session *s;
    am_joint *j[2];
    am_sim_options o;
    double worst[2] = {0.0, 0.0};
    int k, i;

    memset(&cfg, 0, sizeof cfg);
    cfg.interface = "sim:H-F,M";
    cfg.cycle_us = 1000;
    cfg.telemetry_extended = true;
    TRY(am_init(&cfg, &s));
    for (i = 0; i < 2; i++) {
        TRY(am_open_joint(s, (uint16_t)i, &j[i]));
        SIM[i] = am_sim_of(j[i]);
        am_sim_options_default(&o);
        o.load_inertia_kgm2 = i == 0 ? M1 * L1 * L1 / 3 + M2 * L1 * L1 : M2 * L2 * L2 / 3;
        TRY(am_sim_configure(SIM[i], &o));
        TRY(am_sim_set_external_torque(SIM[i], grav_torque, &idx[i]));
    }
    TRY(am_start(s));
    for (i = 0; i < 2; i++) {
        TRY(am_set_mode(j[i], AM_MODE_IMPEDANCE));
        TRY(am_enable(j[i], 1000));
    }
    for (k = 0; k < 6000; k++) {
        double t_s = k * 1e-3, squat = 0.5 * (1 - cos(2 * PI * 0.25 * t_s));
        double q_ref[2], g[2];
        am_telemetry th, tk;
        q_ref[0] = -0.6 * squat;           /* thigh back */
        q_ref[1] = 1.2 * squat;            /* knee bent */
        TRY(am_read_telemetry(j[0], &th));
        TRY(am_read_telemetry(j[1], &tk));
        gravity(th.q_rad, tk.q_rad, &g[0], &g[1]);
        for (i = 0; i < 2; i++) {
            am_impedance imp;
            double q = i == 0 ? th.q_rad : tk.q_rad;
            memset(&imp, 0, sizeof imp);
            imp.q_ref_rad = q_ref[i];
            imp.kp_Nm_per_rad = KP[i];
            imp.kd_Nm_s_per_rad = KD[i];
            imp.tau_ff_Nm = (float)g[i];
            TRY(am_set_impedance(j[i], &imp));
            if (fabs(q_ref[i] - q) > worst[i]) worst[i] = fabs(q_ref[i] - q);
        }
        TRY(am_cycle(s, NULL));
    }
    for (i = 0; i < 2; i++) {
        am_telemetry t;
        TRY(am_read_telemetry(j[i], &t));
        printf("%s max tracking error %.3f rad, torque now %.1f N*m, winding %.2f C, state %d\n",
               i == 0 ? "hip  (H-F)" : "knee (M)  ", worst[i], t.tau_est_Nm, t.t_winding_C, (int)t.state);
    }
    am_deinit(s);
    return 0;
}
