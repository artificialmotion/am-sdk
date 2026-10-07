/*
 * am_sim.h - Artificial Motion joint SDK, simulator controls (C API 0.1.0)
 * SPDX-License-Identifier: Apache-2.0
 *
 * The built-in simulator runs when am_session_config.interface is "sim" or "sim:<units>",
 * for example "sim:H-F,M" (addresses 0 and 1). Units: S-S, S-D, M, H-F, H-T, H-X.
 * It is the same joint model as the Python package (am_sdk.SimBackend): reflected inertia,
 * backdrive friction, torque clamp from the published torque-speed curves, 2 s peak budget,
 * first-order thermal model with derate and trip, CiA 402 state machine, STO and faults,
 * 19-bit motor and 14-bit output encoders. Values are design targets, not measurements.
 *
 * The functions below set test conditions. They are not part of the bus API: on hardware
 * the same conditions come from the physical world.
 *
 * am_sim_transport_open() gives the same joints behind a byte-level am_transport_ops, so you
 * can run the full transport path (codec, process image, object access) without hardware.
 */
#ifndef AM_SIM_H
#define AM_SIM_H

#include "am_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One simulated joint. */
typedef struct am_sim_joint am_sim_joint;

/** Per-joint simulator settings. Fill with am_sim_options_default() first. */
typedef struct am_sim_options {
    double ambient_C;          /**< 25 */
    double thermal_x0;         /**< initial normalised winding heat, 1.0 = continuous limit; 0 */
    double peak_budget_s;      /**< 2 */
    double load_inertia_kgm2;  /**< added to the reflected inertia; 0 */
    double q0_rad;             /**< initial output angle; 0 */
    double bus_V;              /**< 48 or 36 selects the published curve set; 48 */
    double friction_scale;     /**< scales the backdrive friction; 1 */
    am_bus authority_bus;      /**< bus the unit obeys; AM_BUS_NONE = the session bus */
} am_sim_options;

/** External torque on the output (gravity, contact), N*m, evaluated every plant step. */
typedef double (*am_sim_torque_fn)(double t_s, double q_rad, double qd_rad_s, void *user);

AM_API void am_sim_options_default(am_sim_options *o);

/** The simulated joint behind a session joint, or NULL when the session is not a simulator. */
AM_API am_sim_joint *am_sim_of(am_joint *j);

/** Apply options and reset the simulated drive (power cycle; the plant position is set to q0). */
AM_API am_status am_sim_configure(am_sim_joint *sj, const am_sim_options *o);
AM_API am_status am_sim_set_external_torque(am_sim_joint *sj, am_sim_torque_fn f, void *user);
/** De-energise (true) or restore (false) both STO inputs. */
AM_API am_status am_sim_set_sto(am_sim_joint *sj, bool active);
/** Latch a fault code as a persistent cause (clear the cause with am_sim_clear_fault_cause). */
AM_API am_status am_sim_inject_fault(am_sim_joint *sj, uint16_t code);
/** Remove a fault cause (code 0 = all); the fault stays latched until am_clear_faults(). */
AM_API am_status am_sim_clear_fault_cause(am_sim_joint *sj, uint16_t code);
AM_API am_status am_sim_set_ambient(am_sim_joint *sj, double ambient_C);
AM_API am_status am_sim_set_bus_voltage(am_sim_joint *sj, double volts);
AM_API am_status am_sim_set_load_inertia(am_sim_joint *sj, double kgm2);
/** Drive reset: state, faults, thermal state, reference. The plant keeps its position. */
AM_API am_status am_sim_power_cycle(am_sim_joint *sj);
/** True plant state and the torque the drive applied in the last cycle. Any pointer may be NULL. */
AM_API am_status am_sim_get_plant(const am_sim_joint *sj, double *q_rad, double *qd_rad_s,
                                  double *tau_applied_Nm, double *thermal_x);
/** Overwrite the plant state (tests, external physics engines). */
AM_API am_status am_sim_set_plant(am_sim_joint *sj, double q_rad, double qd_rad_s);
/** Hand the plant to an external physics engine: the sim then only samples the state you set
 *  with am_sim_set_plant() and reports the torque to apply (am_sim_get_plant). */
AM_API am_status am_sim_set_external_plant(am_sim_joint *sj, bool external);

/** Simulated joints behind a byte-level transport. units: "M,H-F"; addresses 0..n-1.
 *  bus and cycle_us must match the session configuration that uses it; the process-image
 *  layout (EtherCAT PDOs or CAN-FD compact) follows from the sizes passed to start(). */
AM_API am_status am_sim_transport_open(const char *units, am_bus bus, uint32_t cycle_us,
                                       const am_transport_ops **ops, void **user);
AM_API am_sim_joint *am_sim_transport_joint(void *user, uint16_t address);
AM_API void am_sim_transport_close(void *user);

#ifdef __cplusplus
}
#endif

#endif /* AM_SIM_H */
