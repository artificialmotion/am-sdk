/*
 * am_est.h - Artificial Motion joint SDK, torque estimator and collision detector (C API 0.2.0)
 * SPDX-License-Identifier: Apache-2.0
 *
 * SIMULATED, PRE-HARDWARE. The joints have no output torque sensor. These functions estimate the
 * output torque from the phase current, the output position and the temperature, and detect
 * unexpected contact with a momentum observer. Every torque here is an ESTIMATE with an uncertainty
 * bound, a confidence value and a quality flag. No accuracy is claimed before the evaluation-unit
 * dyno; docs/ESTIMATION.md lists the targets, to be confirmed on hardware.
 *
 * The collision detector is NOT a safety function. It does not replace STO or a safety controller.
 *
 * Same model and same numbers as the Python module am_sdk.estimation (cross-checked in CI).
 *
 * Conventions: SI units, output-referred unless the name says _motor. Every struct is fixed-size;
 * nothing allocates. All am_est_* and am_collision_* functions are [RT] except where noted, and
 * none touches a session: call them from your control loop with your own telemetry.
 *
 *   am_est_calibration cal;  am_est est;  am_collision det;  am_collision_config cc;
 *   am_est_calibration_default("S-S", &cal);
 *   am_est_init(&est, &cal, NULL);
 *   am_collision_config_default(&cc);  cc.inertia_kgm2 = cal.reflected_inertia_kgm2 + link_J;
 *   cc.threshold_Nm = thr;  am_collision_init(&det, &cc);
 *   loop:  am_read_telemetry(j, &t);
 *          am_est_input in;  am_est_input_from_telemetry(&t, 0.001, &in);
 *          am_est_torque e;  am_est_update(&est, &in, &e);
 *          am_collision_output o;
 *          am_collision_update(&det, 0.001, t.qd_rad_s, e.tau_drive_Nm, gravity(t.q_rad), e.quality, &o);
 *
 * C99. Compiles with gcc/clang -std=c99 -Wall -Wextra -pedantic and as C++.
 */
#ifndef AM_EST_H
#define AM_EST_H

#include "am_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Calibration record am_est_cal_v1 (python/am_sdk/estimation/               */
/* estimator_calibration.schema.json; JSON field names = struct field names)*/
/* ------------------------------------------------------------------------ */

#define AM_EST_CAL_SCHEMA_VERSION 1
#define AM_EST_FRICTION_MAP_MAX   16
#define AM_EST_EFF_TEMPS_MAX      4
#define AM_EST_EFF_LOADS_MAX      4

typedef struct am_est_calibration {
    uint32_t schema_version;          /**< AM_EST_CAL_SCHEMA_VERSION                          */
    am_data_basis basis;              /**< DESIGN_TARGET until measured at end of line        */
    uint32_t cal_epoch;
    double gear_ratio;
    double kt_Nm_per_A;               /**< motor side, at kt_ref_temp_C (0x2200:2)            */
    double kt_ref_temp_C;             /**< 0x2200:3                                           */
    double kt_temp_coeff_per_K;       /**< 0x2200:4                                           */
    double kt_sat_frac;               /**< Kt drop at kt_sat_current_A, quadratic in current  */
    double kt_sat_current_A;          /**< 0 = no saturation term                             */
    double reflected_inertia_kgm2;    /**< armature, output-referred                          */
    double rated_torque_Nm;           /**< normalises the confidence value                    */
    double coulomb_Nm;                /**< friction, output-referred                          */
    double breakaway_Nm;              /**< static level; base of the stick band               */
    double stribeck_vel_rad_s;
    double viscous_Nm_s_per_rad;
    double friction_ref_temp_C;
    double coulomb_temp_coeff_per_K;  /**< + per K below friction_ref_temp_C                  */
    double viscous_temp_exp_per_K;    /**< viscous * exp(c * (Tref - T))                      */
    double bearing_Nm;                /**< output bearing, downstream of the gear             */
    uint32_t friction_map_n;          /**< 0 = parametric friction                            */
    double friction_map_w_rad_s[AM_EST_FRICTION_MAP_MAX];   /**< increasing, >= 0           */
    double friction_map_Nm[AM_EST_FRICTION_MAP_MAX];
    double eta_forward, eta_reverse;  /**< used when eff_n_temps == 0                         */
    uint32_t eff_n_temps, eff_n_loads;/**< efficiency grid, bilinear, clamped                 */
    double eff_temps_C[AM_EST_EFF_TEMPS_MAX];
    double eff_loads_Nm[AM_EST_EFF_LOADS_MAX];
    double eff_forward[AM_EST_EFF_TEMPS_MAX][AM_EST_EFF_LOADS_MAX];
    double eff_reverse[AM_EST_EFF_TEMPS_MAX][AM_EST_EFF_LOADS_MAX];
    double unc_kt_rel, unc_temp_C, unc_friction_rel, unc_eta_abs, unc_inertia_rel;
} am_est_calibration;

/** Design-target record from the published unit data ("S-S", "M", ...); same values as
 *  EstimatorCalibration.from_unit(). Friction shape and reverse efficiency are assumed. [NRT] */
AM_API am_status am_est_calibration_default(const char *unit, am_est_calibration *cal);
/** AM_OK when the record is usable (same rules as validate_dict in Python), else AM_ERR_PARAM. */
AM_API am_status am_est_calibration_check(const am_est_calibration *cal);

/* ------------------------------------------------------------------------ */
/* Estimator                                                                 */
/* ------------------------------------------------------------------------ */

/* Quality bits (EST_* in Python). */
#define AM_EST_Q_OK              0u
#define AM_EST_Q_STICK           (1u << 0)  /**< at rest: torque known only to the stick band   */
#define AM_EST_Q_AMBIGUOUS       (1u << 1)  /**< Stribeck region or just after a reversal       */
#define AM_EST_Q_NO_TEMPERATURE  (1u << 2)  /**< no temperature: Kt at reference, wider bound   */
#define AM_EST_Q_SATURATED       (1u << 3)  /**< current beyond the Kt saturation point         */
#define AM_EST_Q_INVALID         (1u << 4)  /**< inputs held, stale or not finite               */
#define AM_EST_Q_DITHER          (1u << 5)  /**< dither active: dither-period average           */

typedef struct am_est_config {
    double svf_hz;                    /**< differentiator bandwidth; 40                       */
    double svf_zeta;                  /**< 0.8                                                */
    double output_lp_hz;              /**< output filter, 0 = off; 100                        */
    double dither_lp_hz;              /**< output filter while dithering; 5                   */
    double stick_vel_rad_s;           /**< 0.01                                               */
    double ambiguous_vel_factor;      /**< x Stribeck velocity; 2                             */
    double reversal_hold_s;           /**< 0.02                                               */
    double w_eps_rad_s;               /**< friction sign regularisation; 0.003                */
    double power_eps_W;               /**< driving / back-driven blend; 0.02                  */
    double bearing_w_eps_rad_s;       /**< 0.01                                               */
    double confidence_band_frac_rated;/**< confidence = 1 - bound / (this * rated); 0.1       */
} am_est_config;

/** One estimate. tau_* are ESTIMATED torques, never measurements. */
typedef struct am_est_torque {
    double tau_out_Nm;                /**< estimated torque at the output flange              */
    double tau_drive_Nm;              /**< same without the armature inertia term             */
    double tau_motor_Nm;              /**< N * Kt(T) * iq                                     */
    double friction_Nm;
    double qd_rad_s, qdd_rad_s2;      /**< differentiator outputs                             */
    double bound_Nm;                  /**< model uncertainty half-width (about 2 sigma)       */
    double confidence;                /**< 0..1                                               */
    uint32_t quality;                 /**< AM_EST_Q_* bits                                    */
} am_est_torque;

typedef struct am_est_input {
    double dt_s;
    double q_rad;                     /**< output angle (telemetry q_rad)                     */
    double iq_A;                      /**< measured q-axis current                            */
    double temp_C;                    /**< magnet or winding temperature; NaN = none          */
    bool dither_active;
    bool saturated;                   /**< caller-side saturation flag                        */
    bool valid;                       /**< false: inputs held or stale                        */
} am_est_input;

/** Estimator state. Fields are private; the caller owns the storage. */
typedef struct am_est {
    am_est_calibration cal;
    am_est_config cfg;
    double p, v, a, tau_f, tau_d, since_stick;
    bool initialised;
} am_est;

AM_API void am_est_config_default(am_est_config *cfg);
/** cal is copied; cfg NULL = defaults. */
AM_API am_status am_est_init(am_est *e, const am_est_calibration *cal, const am_est_config *cfg);
/** Restart the differentiator and filters at the next update. */
AM_API void am_est_reset(am_est *e);
/** One cycle. Non-finite inputs give AM_EST_Q_INVALID and the held estimate (AM_OK). */
AM_API am_status am_est_update(am_est *e, const am_est_input *in, am_est_torque *out);
/** Inputs from one telemetry sample: magnet temperature when the extended fields are fresh, else
 *  winding temperature, else NaN. */
AM_API am_status am_est_input_from_telemetry(const am_telemetry *t, double dt_s, am_est_input *in);

/** Pure current model for one sample (no filtering, no reversal hold). temp_C NaN = none. */
AM_API am_status am_est_current_model(const am_est_calibration *cal, const am_est_config *cfg, double iq_A,
                                      double qd_rad_s, double qdd_rad_s2, double temp_C, bool dither,
                                      am_est_torque *out);
AM_API double am_est_kt_at(const am_est_calibration *cal, double temp_C, double iq_A);
AM_API double am_est_friction_at(const am_est_calibration *cal, double w_rad_s, double temp_C, double w_eps);
AM_API void am_est_efficiency_at(const am_est_calibration *cal, double temp_C, double load_Nm,
                                 double *eta_forward, double *eta_reverse);
/** Half-width of the torque band consistent with a joint at rest. */
AM_API double am_est_stick_band(const am_est_calibration *cal, double tau_motor_Nm, double temp_C);

/** Optional bounded dither (off unless you add it to your torque feed-forward). The amplitude is
 *  capped at min(1.5 x breakaway, 10 % of rated). */
AM_API double am_est_dither_amplitude_max(const am_est_calibration *cal);
AM_API double am_est_dither_torque(const am_est_calibration *cal, double amplitude_Nm, double freq_Hz, double t_s);

/* ------------------------------------------------------------------------ */
/* Momentum-observer collision detector (non-safety)                        */
/* ------------------------------------------------------------------------ */

typedef struct am_collision_config {
    double inertia_kgm2;              /**< armature + link inertia seen by the joint          */
    double gain_hz;                   /**< observer bandwidth; 30                             */
    double threshold_Nm;              /**< INFINITY = residual only; calibrate per robot      */
    double stick_scale;               /**< threshold x this while STICK/AMBIGUOUS; 2          */
    double release_ratio;             /**< clear below ratio x threshold ...; 0.5             */
    double min_hold_s;                /**< ... for this long; 0.05                            */
} am_collision_config;

typedef struct am_collision_output {
    double residual_Nm;               /**< estimated unexpected (contact) torque               */
    double threshold_Nm;              /**< threshold in force for this sample                  */
    bool active;
    bool onset;                       /**< first sample of a detection                         */
    double t_s;
} am_collision_output;

typedef struct am_collision {
    am_collision_config cfg;
    double p0, integ, r, hold, t;
    bool active, initialised;
    uint32_t events;                  /**< number of onsets since init                         */
} am_collision;

AM_API void am_collision_config_default(am_collision_config *cfg);
AM_API am_status am_collision_init(am_collision *c, const am_collision_config *cfg);
AM_API void am_collision_reset(am_collision *c);
/** qd: the drive's velocity (telemetry qd_rad_s); tau_drive: am_est_torque.tau_drive_Nm;
 *  tau_known: what your link model explains (gravity, coupling); quality: am_est_torque.quality. */
AM_API am_status am_collision_update(am_collision *c, double dt_s, double qd_rad_s, double tau_drive_Nm,
                                     double tau_known_Nm, uint32_t quality, am_collision_output *out);
/** margin x max |residual| over a contact-free run (NaN entries skipped). */
AM_API double am_collision_calibrate_threshold(const double *residuals, size_t n, double margin);

#ifdef __cplusplus
}
#endif

#endif /* AM_EST_H */
