/*
 * am_api.h - Artificial Motion joint SDK, C API 0.2.0
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pre-hardware release: the API runs against the built-in simulator and against your own
 * transport; the object dictionary and every unit value are design values until evaluation units ship.
 *
 * Specification: docs/API.md. Object dictionary: docs/object_dictionary.csv.
 *
 * Layers
 *   L1 codec   am_codec_*   pure functions, no I/O, no allocation, no locks.
 *                           Use them inside your own EtherCAT master or CAN task.
 *   L2 session am_*         session and joint handles over a pluggable transport
 *                           (am_transport_ops) or the built-in simulator ("sim:M,H-F").
 *   Python     am_sdk       the same calls and the same simulator (python/).
 *
 * Conventions
 *   - Every function returns am_status; AM_OK == 0, errors are negative.
 *   - SDK units are SI: rad, rad/s, N*m, A, V, degC, s. Output-shaft side unless the name
 *     says _motor. The codec converts to the object-dictionary integer units (urad, mrad/s,
 *     mNm, per mille, mA, 0.1 degC) on the wire.
 *   - Structs passed in carry no size field. ABI growth is handled by appending new
 *     functions, never by changing a released struct (docs/API.md 4.7).
 *   - [RT] functions take bounded time, allocate nothing and make no blocking system call.
 *     [NRT] functions may block (object access, file transfer, the enable walk).
 *
 * C99. Compiles with gcc/clang -std=c99 -Wall -Wextra -pedantic and as C++.
 */
#ifndef AM_API_H
#define AM_API_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Versioning (docs/API.md 4.7)                                             */
/* ------------------------------------------------------------------------ */

/** SDK API semantic version. 0.y.z = pre-release, no compatibility promise. */
#define AM_API_VERSION_MAJOR 0
#define AM_API_VERSION_MINOR 2
#define AM_API_VERSION_PATCH 0

/** Object-dictionary major version this SDK build understands (0x2000:6). */
#define AM_OD_MAJOR_SUPPORTED 0

/** Pack major.minor.patch into one word (8.8.16 bits), as in 0x2000:6. */
#define AM_VERSION_PACK(maj, min, pat) \
    ((((uint32_t)(maj) & 0xFFu) << 24) | (((uint32_t)(min) & 0xFFu) << 16) | ((uint32_t)(pat) & 0xFFFFu))

#if defined(_WIN32) && defined(AM_BUILD_SHARED)
#define AM_API __declspec(dllexport)
#elif defined(_WIN32) && defined(AM_USE_SHARED)
#define AM_API __declspec(dllimport)
#elif defined(__GNUC__) && (__GNUC__ >= 4)
#define AM_API __attribute__((visibility("default")))
#else
#define AM_API
#endif

/* ------------------------------------------------------------------------ */
/* Status codes (docs/API.md 4.5)                                           */
/* ------------------------------------------------------------------------ */

typedef enum am_status {
    AM_OK                 = 0,
    AM_ERR_PARAM          = -1,   /**< NULL pointer, NaN, value outside type range         */
    AM_ERR_STATE          = -2,   /**< call not allowed in the current drive/session state  */
    AM_ERR_TIMEOUT        = -3,   /**< no answer within the configured timeout              */
    AM_ERR_BUS            = -4,   /**< link down, frame lost, working-counter error         */
    AM_ERR_NO_DEVICE      = -5,   /**< address not found / not an AM unit                    */
    AM_ERR_SDO_ABORT      = -6,   /**< SDO abort; am_last_abort_code() has the code          */
    AM_ERR_NOT_SUPPORTED  = -7,   /**< feature absent on this unit, bus, firmware or SDK    */
    AM_ERR_LIMIT          = -8,   /**< value outside the AM safe envelope; NOT clamped      */
    AM_ERR_AUTHORITY      = -9,   /**< this bus does not hold control authority (0x2901)    */
    AM_ERR_FAULT          = -10,  /**< drive is in Fault; read am_get_faults()               */
    AM_ERR_VERSION        = -11,  /**< OD major / SDK major incompatible                     */
    AM_ERR_IMAGE          = -12,  /**< firmware image rejected                               */
    AM_ERR_BUSY           = -13,  /**< another non-RT operation is in progress on the joint  */
    AM_ERR_NO_MEMORY      = -14,
    AM_ERR_STALE          = -15,  /**< telemetry older than the stale limit, or cycle missed */
    AM_ERR_STO            = -16,  /**< STO active: torque cannot be enabled                  */
    AM_ERR_REFERENCE      = -17,  /**< absolute output reference not valid (statusword b14)  */
    AM_ERR_INTERNAL       = -99
} am_status;

/** Static English text for a status code. Never NULL. [RT] */
AM_API const char *am_status_str(am_status st);

/* ------------------------------------------------------------------------ */
/* Enumerations                                                             */
/* ------------------------------------------------------------------------ */

/** Physical channel. Only EtherCAT or CAN-FD can hold control authority. Value 3 is reserved. */
typedef enum am_bus {
    AM_BUS_NONE        = 0,
    AM_BUS_ETHERCAT    = 1,
    AM_BUS_CANFD       = 2
} am_bus;

/** Modes of operation (0x6060 values). */
typedef enum am_mode {
    AM_MODE_NONE      = 0,
    AM_MODE_CSP       = 8,   /**< cyclic synchronous position                        */
    AM_MODE_CSV       = 9,   /**< cyclic synchronous velocity                        */
    AM_MODE_CST       = 10,  /**< cyclic synchronous torque (open-loop estimate)     */
    AM_MODE_IMPEDANCE = -1,  /**< AM: tau = K(q*-q) + D(qd*-qd) + tau_ff              */
    AM_MODE_CURRENT   = -2   /**< AM: cyclic q-axis current (advanced)                */
} am_mode;

/** CiA 402 power state (decoded from 0x6041). */
typedef enum am_drive_state {
    AM_DS_NOT_READY_TO_SWITCH_ON = 0,
    AM_DS_SWITCH_ON_DISABLED     = 1,
    AM_DS_READY_TO_SWITCH_ON     = 2,
    AM_DS_SWITCHED_ON            = 3,
    AM_DS_OPERATION_ENABLED      = 4,
    AM_DS_QUICK_STOP_ACTIVE      = 5,
    AM_DS_FAULT_REACTION_ACTIVE  = 6,
    AM_DS_FAULT                  = 7
} am_drive_state;

/** Product codes (0x1018:2). */
typedef enum am_sku {
    AM_SKU_UNKNOWN = 0,
    AM_SKU_S_D     = 1,
    AM_SKU_S_S     = 2,
    AM_SKU_M       = 3,
    AM_SKU_H_F     = 4,
    AM_SKU_H_T     = 5,
    AM_SKU_H_X     = 6
} am_sku;

/** Output reference state (0x2500:1). */
typedef enum am_ref_state {
    AM_REF_INVALID   = 0,
    AM_REF_CANDIDATE = 1,
    AM_REF_VALID     = 2
} am_ref_state;

/** Where a value comes from (transparency data, docs/API.md 2.7). */
typedef enum am_data_basis {
    AM_BASIS_NOT_AVAILABLE = 0,
    AM_BASIS_DESIGN_TARGET = 1,  /**< design value, not yet measured          */
    AM_BASIS_MEASURED_TYPE = 2,  /**< measured on a sample of the type        */
    AM_BASIS_MEASURED_UNIT = 3   /**< measured on this unit at end of line    */
} am_data_basis;

/* ------------------------------------------------------------------------ */
/* Opaque handles                                                           */
/* ------------------------------------------------------------------------ */

/** One bus connection plus its process image. */
typedef struct am_session am_session;
/** One actuator on a session. Owned by the session; freed by am_deinit(). */
typedef struct am_joint am_joint;

/* ------------------------------------------------------------------------ */
/* Plain data types                                                         */
/* ------------------------------------------------------------------------ */

/** Identity (0x1018, 0x2000, 0x2100). */
typedef struct am_identity {
    uint32_t vendor_id;            /**< 0x1018:1                                 */
    am_sku   sku;                  /**< 0x1018:2                                 */
    uint32_t revision;             /**< 0x1018:3 hw rev x 0x10000 + OD major     */
    uint32_t serial;               /**< 0x1018:4                                 */
    uint8_t  uid[12];              /**< 0x2000:5                                 */
    uint32_t od_version;           /**< 0x2000:6 AM_VERSION_PACK form            */
    uint32_t capabilities;         /**< 0x2000:8, AM_CAP_* bits                  */
    uint32_t cal_record_version;   /**< 0x2000:9 calibration record version      */
    uint32_t cal_record_crc32;     /**< 0x2000:9 calibration record CRC32        */
    uint32_t ratio_num, ratio_den; /**< 0x6091:1/:2 exact gear ratio            */
    uint32_t fw_main;              /**< 0x2100:1 firmware version, main core     */
    uint32_t fw_motor;             /**< 0x2100:2 firmware version, motor-control core */
    uint32_t fw_boot;              /**< 0x2100:3 bootloader version              */
    char     device_name[32];      /**< 0x1008                                   */
    char     user_label[32];       /**< 0x2E00:1 (customer-writable)             */
} am_identity;

/* Capability bits (0x2000:8). */
#define AM_CAP_OUTPUT_ABS_ENCODER  (1u << 0)
#define AM_CAP_WINDING_SENSOR      (1u << 1)
#define AM_CAP_ETHERCAT            (1u << 2)
#define AM_CAP_CANFD_CONTROL       (1u << 3)
#define AM_CAP_STO                 (1u << 4)
#define AM_CAP_BRAKE               (1u << 5)  /**< 0 in v1                     */
#define AM_CAP_MODE_CURRENT        (1u << 6)
#define AM_CAP_TRACE_BUFFER        (1u << 7)
#define AM_CAP_BUS_FW_UPDATE       (1u << 8)

/** Published transparency and rating data (0x2D00, 0x6076). NaN = not available. */
typedef struct am_transparency {
    float rated_torque_Nm;            /**< 0x6076 headline continuous           */
    float peak_torque_2s_Nm;          /**< 0x2D00:2                             */
    float rated_speed_rad_s;          /**< 0x2D00:3                             */
    float no_load_speed_48V_rad_s;    /**< 0x2D00:4                             */
    float reflected_inertia_kgm2;     /**< 0x2D00:5 at the output               */
    float backdrive_torque_Nm;        /**< 0x2D00:6                             */
    float kinetic_energy_no_load_J;   /**< 0x2D00:7                             */
    float stiffness_Nm_per_rad;       /**< 0x2D00:8                             */
    float backlash_rad;               /**< 0x2D00:9                             */
    float sto_reaction_time_s;        /**< 0x2D00:10 (specified value)          */
    float collision_latency_s;        /**< 0x2D00:11 (non-safety)               */
    am_data_basis basis;              /**< 0x2D00:12 applies to all fields      */
} am_transparency;

/** Cyclic command. Fields not used by the active mode are ignored. */
typedef struct am_command {
    am_mode mode;            /**< must equal the mode set by am_set_mode()          */
    double  q_ref_rad;       /**< CSP / impedance target position (output)        */
    float   qd_ref_rad_s;    /**< CSV target / CSP & impedance velocity ref         */
    float   tau_ff_Nm;       /**< CST target; feed-forward in CSP/CSV/impedance     */
    float   kp_Nm_per_rad;   /**< impedance stiffness K   (0x2A00:1)               */
    float   kd_Nm_s_per_rad; /**< impedance damping  D    (0x2A00:2)               */
    float   iq_ref_A;        /**< current mode only (0x2A11:1)                      */
} am_command;

/** Impedance set-point (convenience form of am_command for mode -1). */
typedef struct am_impedance {
    double q_ref_rad;
    float  qd_ref_rad_s;
    float  kp_Nm_per_rad;
    float  kd_Nm_s_per_rad;
    float  tau_ff_Nm;
} am_impedance;

/** One synchronised telemetry sample (TxPDO 0x1A00 + 0x1A01). */
typedef struct am_telemetry {
    uint64_t t_ns;              /**< 0x2800:1 sample time (DC system time, or
                                     CAN sync-relative host time, see sync_flags) */
    uint16_t sample_counter;    /**< 0x2800:2                                       */
    uint16_t sync_flags;        /**< 0x2800:3 AM_SYNC_* bits                        */
    am_drive_state state;       /**< decoded 0x6041                                 */
    am_mode  mode;              /**< 0x6061                                         */
    double   q_rad;             /**< 0x6064 output, absolute (valid with ref)       */
    float    qd_rad_s;          /**< 0x606C output                                  */
    int32_t  q_motor_counts;    /**< 0x2801:1, 2^19 per motor turn                  */
    float    qd_motor_rad_s;    /**< 0x2801:2                                       */
    float    q_out_encoder_rad; /**< 0x2802:1 calibrated single-turn output angle   */
    float    twist_rad;         /**< 0x2802:3 twist residual                        */
    float    id_A, iq_A;        /**< 0x2803:1/2 measured, 1 ms mean                 */
    float    id_ref_A, iq_ref_A;/**< 0x2803:3/4                                     */
    float    vd_V, vq_V;        /**< 0x2803:5/6                                     */
    float    vbus_V;            /**< 0x2804:1                                       */
    float    ibus_est_A;        /**< 0x2804:2 power-balance estimate                */
    float    tau_est_Nm;        /**< 0x2805:1 open-loop kt(T)*iq*r*eta estimate     */
    float    t_winding_C;       /**< 0x2804:3                                       */
    float    t_magnet_C;        /**< 0x2804:4 model                                 */
    float    t_power_stage_C;   /**< 0x2804:5                                       */
    float    t_board_C;         /**< 0x2804:6                                       */
    uint32_t fault_word;        /**< 0x2600:1                                       */
    uint16_t error_code;        /**< 0x603F                                         */
    uint16_t warning_word;      /**< 0x2805:2 AM_WARN_* bits                        */
    uint8_t  safety_io;         /**< 0x2620:1 AM_SIO_* bits                         */
    am_ref_state ref_state;     /**< 0x2500:1                                       */
    bool     sto_active;        /**< statusword bit 15                              */
    bool     internal_limit;    /**< statusword bit 11                              */
    bool     extended_valid;    /**< false when only the status PDO arrived:
                                     motor, voltage, temperature and estimator
                                     fields above are then held, not fresh        */
    uint16_t statusword;        /**< raw 0x6041                                     */
} am_telemetry;

/* Sync flags (0x2800:3). */
#define AM_SYNC_DC_LOCKED        (1u << 0)
#define AM_SYNC_TIMESTAMP_VALID  (1u << 1)
#define AM_SYNC_CAN_LOCKED       (1u << 2)   /**< CAN group-frame sync locked     */
#define AM_SYNC_MISSED_MASK      (0xFu << 4) /**< missed/held cycles, saturating  */

/* Safety I/O bits (0x2620:1). */
#define AM_SIO_STO1              (1u << 0)   /**< 1 = input energised (run)          */
#define AM_SIO_STO2              (1u << 1)
#define AM_SIO_GATE_SUPPLY_PRESENT (1u << 2) /**< gate-driver supply present     */
#define AM_SIO_ENABLE_READBACK   (1u << 3)
#define AM_SIO_DISCREPANCY       (1u << 4)

/* Warning word bits (0x2805:2). */
#define AM_WARN_THERMAL_DERATE   (1u << 0)   /**< winding above the derate onset     */
#define AM_WARN_PEAK_BUDGET_EMPTY (1u << 1)  /**< peak budget used up; continuous   */
#define AM_WARN_SPEED_LIMIT      (1u << 2)   /**< speed limit removed motoring torque */

/* Statusword bits beyond CiA 402 (0x6041). */
#define AM_SW_REMOTE             (1u << 9)   /**< this channel holds control authority */
#define AM_SW_INTERNAL_LIMIT     (1u << 11)
#define AM_SW_REF_VALID          (1u << 14)  /**< absolute output reference valid     */
#define AM_SW_STO                (1u << 15)  /**< STO active                          */

/** Customer limits. All may only LOWER the AM ceiling (docs/API.md 5). */
typedef struct am_limits {
    float torque_pos_Nm;        /**< 0x60E0 (stored as per mille of 0x6076)         */
    float torque_neg_Nm;        /**< 0x60E1                                         */
    float current_A;            /**< 0x6073 (per mille of 0x6075)                   */
    float speed_rad_s;          /**< 0x2300:4                                       */
    double pos_min_rad;         /**< 0x607D:1 software position limit (-inf = none) */
    double pos_max_rad;         /**< 0x607D:2                         (+inf = none) */
    float kp_max_Nm_per_rad;    /**< 0x2A00:4 customer cap <= 0x2A50:1               */
    float kd_max_Nm_s_per_rad;  /**< 0x2A00:5 customer cap <= 0x2A50:2               */
    uint32_t comms_loss_ms;     /**< 0x2300:20 T_loss before disarm                 */
} am_limits;

/** CSP/CSV loop gains and filters, customer-tunable inside 0x2A50 bounds. */
typedef struct am_loop_gains {
    float pos_kp_1_per_s;       /**< 0x2A30:1 position loop gain                    */
    float vel_kp_Nm_s_per_rad;  /**< 0x2A30:2 velocity loop proportional            */
    float vel_ki_Nm_per_rad;    /**< 0x2A30:3 velocity loop integral                */
    float vel_filter_hz;        /**< 0x2A40:1 velocity feedback low-pass            */
    float torque_cmd_filter_hz; /**< 0x2A40:2 torque command low-pass, 0 = off       */
    float notch_hz;             /**< 0x2A40:3 notch centre, 0 = off                 */
    float notch_q;              /**< 0x2A40:4                                       */
} am_loop_gains;

/** Fault snapshot (0x603F, 0x2600, 0x2602, 0x2603, 0x2620). */
typedef struct am_fault_info {
    uint16_t error_code;        /**< 0x603F                                          */
    uint32_t fault_word;        /**< 0x2600:1, bit n = family code 0x1000 + n         */
    uint32_t first_fault;       /**< 0x2600:2                                        */
    uint32_t warning_word;      /**< 0x2602:1                                        */
    uint32_t boot_block;        /**< 0x2603:1                                        */
    uint8_t  safety_io;         /**< 0x2620:1                                        */
    bool     latched;           /**< true: needs am_clear_faults()                   */
} am_fault_info;

/** One fault-history entry (0x2601). */
typedef struct am_fault_record {
    uint16_t code;
    uint8_t  drive_state;
    uint64_t t_ns;
    float    vbus_V, iq_A, t_winding_C, t_power_stage_C;
} am_fault_record;

/** Scan result. */
typedef struct am_device_info {
    am_bus   bus;
    uint16_t address;           /**< EtherCAT ring position (0-based) or CAN node id */
    uint32_t vendor_id, product_code, revision, serial;
    bool     is_am_unit;        /**< AM objects present; other slaves are listed too */
    bool     od_compatible;     /**< OD major supported by this SDK                  */
    bool     has_control_authority; /**< this bus is the unit's 0x2901 authority     */
} am_device_info;

/* ------------------------------------------------------------------------ */
/* Callbacks                                                                */
/* ------------------------------------------------------------------------ */

typedef enum am_event_kind {
    AM_EV_FAULT          = 1,   /**< new fault                                     */
    AM_EV_FAULT_CLEARED  = 2,
    AM_EV_WARNING        = 3,
    AM_EV_STO_CHANGED    = 4,
    AM_EV_LINK_LOST      = 5,
    AM_EV_LINK_RESTORED  = 6,
    AM_EV_STATE_CHANGED  = 7
} am_event_kind;

typedef struct am_event {
    am_event_kind kind;
    am_joint     *joint;        /**< NULL for session-level events                */
    uint16_t      code;         /**< fault/warning code where applicable           */
    uint64_t      t_ns;
} am_event;

/** Called at the end of am_cycle() (and am_stop()) on the thread that runs the cycle.
 *  Must not call am_cycle(). Keep it short; copy and return. */
typedef void (*am_event_cb)(const am_event *ev, void *user);

/** Trace transfer progress, 0..1. Return false to abort. */
typedef bool (*am_progress_cb)(float fraction, void *user);

typedef enum am_log_level { AM_LOG_ERROR = 0, AM_LOG_WARN, AM_LOG_INFO, AM_LOG_DEBUG } am_log_level;
typedef void (*am_log_cb)(am_log_level lvl, const char *msg, void *user);

/* ------------------------------------------------------------------------ */
/* Transport plug-in (your EtherCAT master or CAN driver)                    */
/* ------------------------------------------------------------------------ */

/**
 * Transport operations. Set am_session_config.transport to run the session over your
 * own master or CAN driver; leave it NULL and set interface = "sim:<units>" for the
 * built-in simulator. The SDK then never touches a NIC or socket itself.
 *
 * Process image per joint, concatenated in am_open_joint() order:
 *   EtherCAT, or CAN-FD with canfd_compact == false:
 *       out RxPDO 0x1601 (28 B); in TxPDO 0x1A00 (24 B), followed by TxPDO 0x1A01 (60 B)
 *       when telemetry_extended is set.
 *   CAN-FD with canfd_compact == true:
 *       out one group-frame slot (16 B); in compact (16 B) or estimator-lite (32 B) feedback.
 * pdo_exchange is expected to return when the cycle's inputs are in (or the cycle is over);
 * the SDK counts cycles for its timeouts. Zero-fill a joint's input that did not arrive: its
 * telemetry then turns stale (AM_ERR_STALE) after stale_limit_cycles.
 */
typedef struct am_transport_ops {
    /** Exchange one cycle of process data. t_ns (optional) returns the cycle time. [RT] */
    am_status (*pdo_exchange)(void *user, const uint8_t *out, size_t out_len,
                              uint8_t *in, size_t in_len, uint64_t *t_ns);
    /** Confirmed object read (CoE SDO on EtherCAT, SDO on CAN-FD). Return AM_ERR_SDO_ABORT
     *  with *abort_code set, or AM_ERR_NO_DEVICE / AM_ERR_TIMEOUT when nobody answers. [NRT] */
    am_status (*sdo_read)(void *user, uint16_t address, uint16_t index, uint8_t sub,
                          void *buf, size_t cap, size_t *len, uint32_t *abort_code);
    /** Confirmed object write. [NRT] */
    am_status (*sdo_write)(void *user, uint16_t address, uint16_t index, uint8_t sub,
                           const void *buf, size_t len, uint32_t *abort_code);
    /** File transfer (FoE on EtherCAT). May be NULL. [NRT] */
    am_status (*file_write)(void *user, uint16_t address, const char *name,
                            const uint8_t *data, size_t len,
                            am_progress_cb cb, void *cb_user);
    /** File read, used by am_read_calibration() ("am_cal.json"). May be NULL. [NRT] */
    am_status (*file_read)(void *user, uint16_t address, const char *name,
                           uint8_t *buf, size_t cap, size_t *len);
    /** Called by am_start() with the opened joints in process-image order and the per-joint
     *  image sizes; configure PDO assignment and bring the bus to cyclic operation here.
     *  May be NULL. [NRT] */
    am_status (*start)(void *user, const uint16_t *addresses, size_t n_joints,
                       size_t out_len_per_joint, size_t in_len_per_joint);
    /** Called by am_stop() and am_deinit(). May be NULL. [NRT] */
    am_status (*stop)(void *user);
} am_transport_ops;

/** Session configuration. Zero-initialise, then set what you need. */
typedef struct am_session_config {
    am_bus      bus;              /**< AM_BUS_ETHERCAT (0 is read as EtherCAT) or AM_BUS_CANFD */
    const char *interface;        /**< "sim" or "sim:M,H-F" for the built-in simulator;
                                       free text for your transport (copied)             */
    uint32_t    cycle_us;         /**< 0 = 1000; 100..100000 accepted                  */
    uint32_t    sdo_timeout_ms;   /**< 0 = default 100 ms                           */
    uint32_t    stale_limit_cycles; /**< am_read_telemetry() returns AM_ERR_STALE
                                         beyond this; 0 = default 3                 */
    uint32_t    canfd_nominal_bps;/**< 0 = 1 000 000                                */
    uint32_t    canfd_data_bps;   /**< 0 = 5 000 000                                */
    bool        canfd_compact;    /**< true = AM compact group profile (API.md 3.4) */
    bool        telemetry_extended; /**< assign TxPDO 0x1A01 (EtherCAT) or the 32 B
                                         estimator-lite feedback (CAN-FD)           */
    const am_transport_ops *transport;  /**< NULL = built-in simulator          */
    void       *transport_user;
    am_event_cb event_cb;  void *event_user;
    am_log_cb   log_cb;    void *log_user;
} am_session_config;

/* ------------------------------------------------------------------------ */
/* L2 session API                                                           */
/* ------------------------------------------------------------------------ */

/** SDK version as AM_VERSION_PACK. [RT] */
AM_API uint32_t am_api_version(void);

/** Open a bus session. Does not change any drive state. [NRT] */
AM_API am_status am_init(const am_session_config *cfg, am_session **out);

/** Disable every joint opened on the session (Disable operation, not STO),
 *  stop the exchange and free all memory. Safe on NULL. [NRT] */
AM_API am_status am_deinit(am_session *s);

/** Enumerate devices on the bus. Reads identity only; changes nothing. [NRT] */
AM_API am_status am_scan(am_session *s, am_device_info *list, size_t cap, size_t *found);

/** Attach to one actuator (address from am_scan). Checks the OD major and reads the
 *  identity, transparency and codec scaling. Fails with AM_ERR_VERSION on an unsupported
 *  OD major. Call before am_start(). [NRT] */
AM_API am_status am_open_joint(am_session *s, uint16_t address, am_joint **out);

/** Start cyclic exchange and run a few cycles so the drives report their state. [NRT] */
AM_API am_status am_start(am_session *s);

/** Stop cyclic exchange. Drives see comms loss -> zero torque -> disarm. [NRT] */
AM_API am_status am_stop(am_session *s);

/**
 * Run one cycle: send the latest command of every joint, receive the feedback,
 * update the session process image, then deliver events. Call exactly once per
 * cycle from your RT thread. t_ns (optional) returns the cycle reference time. [RT]
 */
AM_API am_status am_cycle(am_session *s, uint64_t *t_ns);

/** Identity block (cached at am_open_joint). [RT] */
AM_API am_status am_get_identity(const am_joint *j, am_identity *out);

/** Transparency / rating data (cached at am_open_joint). [RT] */
AM_API am_status am_get_transparency(const am_joint *j, am_transparency *out);

/** Select the mode used from the next cycle. CSP, CSV, CST and impedance can be switched
 *  while Operation enabled (bumpless). AM_MODE_CURRENT also requires AM_CAP_MODE_CURRENT and
 *  the enable object 0x2A11:3 = 1 (AM_ERR_NOT_SUPPORTED otherwise), and entry before
 *  Operation enabled (AM_ERR_STATE otherwise). The command is reset: until the next setter
 *  call the joint holds position in CSP and applies zero torque in the other modes. [NRT] */
AM_API am_status am_set_mode(am_joint *j, am_mode mode);

/** Walk Shutdown -> Switch on -> Enable operation. Needs a mode, reference valid, STO
 *  released and no latched fault. Runs session cycles itself while it waits (all joints
 *  of the session advance with their latest commands), up to timeout_ms of cycle time.
 *  Do not call it while another thread runs am_cycle() on the same session. [NRT] */
AM_API am_status am_enable(am_joint *j, uint32_t timeout_ms);

/** Disable operation (torque off, bridge open, joint coasts). Takes effect in the next
 *  cycle; the drive lands in Ready to switch on. [RT] */
AM_API am_status am_disable(am_joint *j);

/** Quick stop (torque ramp 0x6087 per 0x605A, then Switch on disabled; not safety-rated). Follow with
 *  STO from your safety controller where the risk assessment requires it. [RT] */
AM_API am_status am_quick_stop(am_joint *j);

/** Set the full cyclic command for the next am_cycle(). Values beyond the effective
 *  limits are clamped by the drive and flagged (statusword bit 11); values beyond the
 *  SDK sanity range return AM_ERR_PARAM; impedance K or D above the effective caps
 *  return AM_ERR_LIMIT. [RT] */
AM_API am_status am_set_command(am_joint *j, const am_command *cmd);

/** Convenience: impedance set-point (mode must be AM_MODE_IMPEDANCE). [RT] */
AM_API am_status am_set_impedance(am_joint *j, const am_impedance *imp);

/** Convenience setters for the single-quantity modes. [RT] */
AM_API am_status am_set_position(am_joint *j, double q_rad, float qd_ff_rad_s, float tau_ff_Nm);
AM_API am_status am_set_velocity(am_joint *j, float qd_rad_s, float tau_ff_Nm);
AM_API am_status am_set_torque(am_joint *j, float tau_Nm);
AM_API am_status am_set_current(am_joint *j, float iq_A);

/** Latest telemetry sample of this joint (copy from the process image).
 *  Returns AM_ERR_STALE if older than stale_limit_cycles. [RT] */
AM_API am_status am_read_telemetry(const am_joint *j, am_telemetry *out);

/** Current fault snapshot. [NRT] */
AM_API am_status am_get_faults(am_joint *j, am_fault_info *out);

/** Clear latched faults (controlword bit 7 edge). The drive re-qualifies the output
 *  reference from the output encoder; walk am_enable() again afterwards. Runs session
 *  cycles itself while it waits. [NRT] */
AM_API am_status am_clear_faults(am_joint *j, uint32_t timeout_ms);

/** Fault history (0x2601), newest first. [NRT] */
AM_API am_status am_get_fault_history(am_joint *j, am_fault_record *buf, size_t cap, size_t *n);

/** Static name of a family fault code (0x1000..), e.g. "HOST_STALE". Never NULL. [RT] */
AM_API const char *am_fault_name(uint16_t code);

/** Last SDO abort code seen on this joint (CiA 301 numbering). [RT] */
AM_API uint32_t am_last_abort_code(const am_joint *j);

/* --- limits, gains, filters (customer-modifiable, docs/API.md 5) -------- */

/** Write customer limits. Every value must be <= the AM ceiling; a value above it
 *  returns AM_ERR_LIMIT and nothing is written (no silent clamp).
 *  Not allowed while Operation enabled. [NRT] */
AM_API am_status am_set_limits(am_joint *j, const am_limits *lim);

/** Read the effective (customer) limits and the AM ceilings. Either pointer may be NULL. [NRT] */
AM_API am_status am_get_limits(am_joint *j, am_limits *effective, am_limits *ceiling);

/** CSP/CSV gains and filters, inside 0x2A50 bounds. Not while enabled. [NRT] */
AM_API am_status am_set_loop_gains(am_joint *j, const am_loop_gains *g);
AM_API am_status am_get_loop_gains(am_joint *j, am_loop_gains *g);

/** Persist customer parameters (0x1010:1 "save"). [NRT] */
AM_API am_status am_store_parameters(am_joint *j);

/** Restore customer defaults (0x1011:1); factory data untouched. [NRT] */
AM_API am_status am_restore_defaults(am_joint *j);

/** Customer label stored on the drive (0x2E00:1, <= 31 chars). [NRT] */
AM_API am_status am_set_user_label(am_joint *j, const char *label);

/* --- control bus selection (docs/API.md 3.6) ----------------------------- */

/** Read configured and effective control authority (0x2901:1 / 0x2901:3). [NRT] */
AM_API am_status am_get_control_bus(am_joint *j, am_bus *configured, am_bus *effective);

/** Configure the control bus. Refused unless the drive is in Switch on
 *  disabled or lower; takes effect at the next drive reset only. [NRT] */
AM_API am_status am_set_control_bus(am_joint *j, am_bus bus);

/* --- raw object access ---------------------------------------------------- */

/** Raw SDO access to any object the customer class may read. [NRT] */
AM_API am_status am_sdo_read(am_joint *j, uint16_t index, uint8_t sub,
                             void *buf, size_t cap, size_t *len);
/** Raw write. Factory and AM-internal objects abort 0x08000020/22. [NRT] */
AM_API am_status am_sdo_write(am_joint *j, uint16_t index, uint8_t sub,
                              const void *buf, size_t len);

/* --- calibration and trace ----------------------------------------------- */

/** Read the calibration record (am_cal.json) into buf, NUL-terminated when it fits. [NRT] */
AM_API am_status am_read_calibration(am_joint *j, char *buf, size_t cap, size_t *len);

/** Arm the 20 kHz trace buffer (0x2810). Returns AM_ERR_NOT_SUPPORTED in SDK 0.x. [NRT] */
AM_API am_status am_trace_arm(am_joint *j, uint16_t trigger_signal, float trigger_level,
                              uint16_t length, uint16_t pretrigger);
/** Read a completed trace. Returns AM_ERR_NOT_SUPPORTED in SDK 0.x. [NRT] */
AM_API am_status am_trace_read(am_joint *j, float *buf, size_t cap_samples, size_t *n_samples,
                               am_progress_cb cb, void *user);

/* ------------------------------------------------------------------------ */
/* L1 codec (no I/O). For customers who run their own master.               */
/* ------------------------------------------------------------------------ */

/** Per-joint scaling needed to convert SI <-> OD integers. Fill it once with
 *  am_codec_init() from the joint's objects and keep it with your joint. */
typedef struct am_codec {
    float rated_torque_Nm;      /**< 0x6076, for per mille torque objects         */
    float rated_current_A;      /**< 0x6075, for per mille current objects        */
    float compact_k_scale;      /**< 0x2A20:1, N*m/rad per LSB of compact K       */
    float compact_d_scale;      /**< 0x2A20:2, N*m*s/rad per LSB of compact D     */
    uint8_t compact_slot;       /**< 0x2902:3, slot in the group command frame    */
} am_codec;

/** Returns AM_ERR_PARAM on a zero scale or a slot above 3. [RT] */
AM_API am_status am_codec_init(am_codec *c, uint32_t rated_torque_mNm, uint32_t rated_current_mA,
                               uint16_t compact_k_mNm_per_rad_lsb, uint16_t compact_d_mNm_s_per_rad_lsb,
                               uint8_t compact_slot);

/** Byte sizes of the fixed PDOs and the CAN-FD compact PDOs. */
#define AM_PDO_RX_1600_LEN   20u
#define AM_PDO_RX_1601_LEN   28u
#define AM_PDO_TX_1A00_LEN   24u
#define AM_PDO_TX_1A01_LEN   60u
#define AM_CFD_CMD_SLOT_LEN  16u   /**< one joint in the 64 B group command frame */
#define AM_CFD_GROUP_LEN     64u   /**< 4 slots                                   */
#define AM_CFD_FB_LEN        16u   /**< compact feedback                          */
#define AM_CFD_FB_EXT_LEN    32u   /**< estimator-lite feedback                    */

/** Controlword request carried with a command (CiA 402 0x6040 value). */
typedef uint16_t am_controlword;

/* Pack functions return AM_ERR_PARAM when a value is NaN or does not fit its wire
 * field (nothing is clamped), and leave out[] unspecified in that case. */

/** Pack RxPDO 0x1601 (little-endian): any mode. [RT] */
AM_API am_status am_codec_pack_rx1601(const am_codec *c, am_controlword cw,
                                      const am_command *cmd, uint8_t out[AM_PDO_RX_1601_LEN]);
/** Pack RxPDO 0x1600 (CiA 402 generic; torque in per mille). CSP, CSV and CST only;
 *  other modes return AM_ERR_NOT_SUPPORTED. [RT] */
AM_API am_status am_codec_pack_rx1600(const am_codec *c, am_controlword cw,
                                      const am_command *cmd, uint8_t out[AM_PDO_RX_1600_LEN]);
/** Unpack TxPDO 0x1A00 into the status part of am_telemetry. [RT] */
AM_API am_status am_codec_unpack_tx1a00(const am_codec *c, const uint8_t in[AM_PDO_TX_1A00_LEN],
                                        am_telemetry *t);
/** Unpack TxPDO 0x1A01 into the estimator part of am_telemetry. [RT] */
AM_API am_status am_codec_unpack_tx1a01(const am_codec *c, const uint8_t in[AM_PDO_TX_1A01_LEN],
                                        am_telemetry *t);

/** Compact control byte (CAN-FD compact profile, docs/API.md 3.4). */
#define AM_CC_ENABLE        (1u << 0) /**< level; rising edge requests the enable walk */
#define AM_CC_QUICK_STOP    (1u << 1)
#define AM_CC_FAULT_RESET   (1u << 2) /**< rising edge                                 */
#define AM_CC_ZERO_TORQUE   (1u << 3) /**< hold enabled, command zero torque           */
#define AM_CC_MODE_SHIFT    4u        /**< bits 4..7: 0 keep, 1 CST, 2 CSV, 3 CSP,
                                           4 impedance, 5 current                      */

/** Compact mode code (control bits 4..7) for a mode; 0 for AM_MODE_NONE. [RT] */
AM_API uint8_t am_codec_compact_mode(am_mode mode);

/** Pack one joint slot of the CAN-FD group command frame. counter is the
 *  8-bit cycle counter echoed in the feedback. K and D are sent in impedance mode only. [RT] */
AM_API am_status am_codec_pack_cfd_slot(const am_codec *c, uint8_t control, uint8_t counter,
                                        const am_command *cmd, uint8_t out[AM_CFD_CMD_SLOT_LEN]);
/** Unpack compact (16 B) or estimator-lite (32 B) CAN-FD feedback. [RT] */
AM_API am_status am_codec_unpack_cfd_fb(const am_codec *c, const uint8_t *in, size_t len,
                                        am_telemetry *t, uint8_t *echoed_counter);

/** Decode the CiA 402 state from a statusword. [RT] */
AM_API am_drive_state am_codec_decode_state(uint16_t statusword);

/** Next controlword for walking toward Operation enabled (or toward
 *  Switch on disabled when enable == false). [RT] */
AM_API am_controlword am_codec_next_controlword(uint16_t statusword, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* AM_API_H */
