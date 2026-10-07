/* SPDX-License-Identifier: Apache-2.0 */
/* Internal declarations of the reference implementation. Not part of the public API. */
#ifndef AM_INTERNAL_H
#define AM_INTERNAL_H

#include "am_api.h"
#include "am_sim.h"

#include <math.h>
#include <string.h>

#ifndef NAN
#define NAN (0.0f / 0.0f)
#endif
#ifndef INFINITY
#define INFINITY (1.0 / 0.0)
#endif
#define AM_NAN ((double)NAN)
#define AM_TWO_PI 6.283185307179586

/* ---------------------------------------------------------------- unit data */
#define AM_UNIT_COUNT 6

typedef struct am_curve_pt { double rpm, nm; } am_curve_pt;
typedef struct am_curve { const am_curve_pt *p; int n; } am_curve;

typedef struct am_unit_data {
    const char *unit;
    am_sku sku;
    double ratio, armature_kgm2, mass_kg, peak_nm, rated_nm, rated_speed_rad_s;
    double no_load_48, no_load_36, backdrive_nm, thermal_tau_s;
    int thermal_tau_published;
    double kt, gear_eta;
    uint32_t ratio_num, ratio_den;
    double stiffness, backlash;
    am_curve curve[2][2];      /* [0 = 48 V, 1 = 36 V][0 = peak, 1 = continuous] */
} am_unit_data;

extern const am_unit_data am_unit_table[AM_UNIT_COUNT];

/* Unit by name ("M", "h-f", "H_X"); len = number of characters; NULL when unknown. */
const am_unit_data *am__unit_find(const char *name, size_t len);

/* ---------------------------------------------------------------- numeric helpers */
/* Round half away from zero (the codec rule, same as the Python codec). */
double am__round_away(double x);
/* Python round(): ties to even. */
double am__round_even(double x);
uint32_t am__crc32(const void *data, size_t len);
void am__copy_str(char *dst, size_t cap, const char *src);

/* little-endian helpers */
void am__put_u16(uint8_t *p, uint16_t v);
void am__put_u32(uint8_t *p, uint32_t v);
void am__put_u64(uint8_t *p, uint64_t v);
void am__put_f32(uint8_t *p, float v);
uint16_t am__get_u16(const uint8_t *p);
uint32_t am__get_u32(const uint8_t *p);
uint64_t am__get_u64(const uint8_t *p);
float am__get_f32(const uint8_t *p);

/* ---------------------------------------------------------------- codec, drive side */
uint16_t am__state_to_sw(am_drive_state st);
void am__drive_unpack_rx1601(const uint8_t in[AM_PDO_RX_1601_LEN], uint16_t *cw, am_command *cmd);
void am__drive_pack_tx1a00(const am_codec *c, const am_telemetry *t, uint8_t out[AM_PDO_TX_1A00_LEN]);
void am__drive_pack_tx1a01(const am_telemetry *t, uint8_t out[AM_PDO_TX_1A01_LEN]);
void am__drive_unpack_cfd_slot(const am_codec *c, const uint8_t in[AM_CFD_CMD_SLOT_LEN], uint8_t *control,
                               uint8_t *counter, am_command *cmd);
void am__drive_pack_cfd_fb(const am_telemetry *t, uint8_t counter, bool authority, bool extended, uint8_t *out);
am_mode am__mode_from_compact(uint8_t code);
#define AM_CW_DISABLE_VOLTAGE 0x0000u
#define AM_CW_SHUTDOWN 0x0006u
#define AM_CW_SWITCH_ON 0x0007u
#define AM_CW_ENABLE_OPERATION 0x000Fu
#define AM_CW_QUICK_STOP 0x0002u
#define AM_CW_FAULT_RESET 0x0080u

/* ---------------------------------------------------------------- simulated joint */
#define AM_SIM_EVQ 64
#define AM_SIM_HIST 32
#define AM_SIM_MAX_FAULTS 16

typedef struct am_sim_ev { am_event_kind kind; uint16_t code; uint64_t t_ns; } am_sim_ev;
/* Double-precision forms of am_limits / am_loop_gains, as in the Python simulator. */
typedef struct am_sim_lim {
    double torque_pos_Nm, torque_neg_Nm, current_A, speed_rad_s, pos_min_rad, pos_max_rad;
    double kp_max_Nm_per_rad, kd_max_Nm_s_per_rad;
    uint32_t comms_loss_ms;
} am_sim_lim;
typedef struct am_sim_gain {
    double pos_kp_1_per_s, vel_kp_Nm_s_per_rad, vel_ki_Nm_per_rad, vel_filter_hz, torque_cmd_filter_hz, notch_hz, notch_q;
} am_sim_gain;
typedef struct am_sim_fault { uint16_t code; bool active; } am_sim_fault;

struct am_sim_joint {
    const am_unit_data *ud;
    uint16_t address;
    am_sim_options opts;
    am_sim_torque_fn ext_fn;
    void *ext_user;
    bool external_plant;
    double dt;
    int n_sub;
    am_bus cfg_bus;
    bool extended, can_sync;
    am_bus authority, authority_cfg;
    char label[32];
    uint32_t last_abort;
    am_fault_record history[AM_SIM_HIST];
    size_t n_history;
    am_sim_ev ev[AM_SIM_EVQ];
    size_t n_ev;
    uint32_t sto_counter;
    bool sto_active;
    am_sim_fault faults[AM_SIM_MAX_FAULTS];
    size_t n_faults;
    uint16_t first_fault, error_code;
    bool current_mode_enabled;
    /* envelope and settings */
    am_sim_lim ceiling, limits;
    am_sim_gain gains, gain_ceiling;
    double min_filter_hz;
    /* drive state */
    am_drive_state state;
    int boot_count, ref_cnt;
    am_ref_state ref_state;
    double thermal_x, thermal_tau, thermal_cont;
    double ambient, t_magnet, budget;
    bool lockout;
    double bus_nominal, vbus_in, vbus, ibus;
    uint16_t warn;
    uint16_t prev_cw;
    double integ, last_err;
    bool have_cmd;
    am_command cmd;
    double hold_q;
    am_mode mode;
    double tau_applied, tau_filt;
    bool limit_active;
    uint64_t t_ns;
    uint16_t sample_counter;
    bool qs_active;
    /* plant and sensing */
    bool plant_init;
    double q, qd, ext_q, ext_qd;
    int64_t counts;
    double q_meas, q_prev, enc_out, qd_f, iq_meas;
};

void am__sim_init(am_sim_joint *sj, const am_unit_data *ud, uint16_t address, am_bus cfg_bus, uint32_t cycle_us,
                  bool extended, const am_sim_options *o);
void am__sim_power_cycle(am_sim_joint *sj);
void am__sim_exchange(am_sim_joint *sj, uint16_t cw, am_mode mode, const am_command *cmd, bool fresh,
                      am_telemetry *out);
void am__sim_identity(const am_sim_joint *sj, am_identity *out);
void am__sim_transparency(const am_sim_joint *sj, am_transparency *out);
void am__sim_fault_info(const am_sim_joint *sj, am_fault_info *out);
am_status am__sim_set_limits(am_sim_joint *sj, const am_sim_lim *lim);
am_status am__sim_set_loop_gains(am_sim_joint *sj, const am_sim_gain *g);
void am__sim_lim_to_api(const am_sim_lim *in, am_limits *out);
void am__sim_lim_from_api(const am_limits *in, am_sim_lim *out);
void am__sim_gain_to_api(const am_sim_gain *in, am_loop_gains *out);
void am__sim_gain_from_api(const am_loop_gains *in, am_sim_gain *out);
am_status am__sim_restore_defaults(am_sim_joint *sj);
am_status am__sim_set_control_bus(am_sim_joint *sj, am_bus bus);
am_status am__sim_sdo_read(am_sim_joint *sj, uint16_t index, uint8_t sub, void *buf, size_t cap, size_t *len);
am_status am__sim_sdo_write(am_sim_joint *sj, uint16_t index, uint8_t sub, const void *buf, size_t len);
size_t am__sim_calibration(const am_sim_joint *sj, char *buf, size_t cap);
void am__sim_comms_lost(am_sim_joint *sj);
bool am__sim_pop_event(am_sim_joint *sj, am_sim_ev *out);
uint16_t am__sim_statusword(const am_sim_joint *sj);
double am__sim_rated_current(const am_sim_joint *sj);

/* parse "sim", "sim:M,H-F" into units; returns count (0 on error) */
size_t am__sim_parse_units(const char *spec, const am_unit_data **units, size_t cap);

#endif /* AM_INTERNAL_H */
