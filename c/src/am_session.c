/* SPDX-License-Identifier: Apache-2.0 */
/*
 * L2 session API. Two backends behind one handle model:
 *   - the built-in simulator ("sim:<units>"), which passes commands to the joint model directly,
 *     exactly as python/am_sdk/backends/sim.py does;
 *   - a transport (am_transport_ops), where every cycle goes through the L1 codec and every
 *     configuration call is an object read or write.
 * The reference implementation is single-threaded: call everything from the thread that runs
 * am_cycle(), or serialise the calls yourself.
 */
#include "am_internal.h"

#include <stdio.h>
#include <stdlib.h>

#define AM_MAX_JOINTS 64
#define AM_MAX_SIM_UNITS 64
#define SANITY_TORQUE_NM 1.0e4
#define SANITY_RATE 1.0e4
#define SANITY_Q 1.0e6
#define STORE_SIGNATURE 0x65766173u
#define RESTORE_SIGNATURE 0x64616F6Cu

typedef enum { BK_SIM, BK_TRANSPORT } backend_kind;

struct am_joint {
    am_session *s;
    uint16_t address;
    size_t index;
    am_sim_joint *sim;
    am_codec codec;
    am_identity ident;
    am_transparency tr;
    am_limits limits_cache;
    am_mode mode;
    am_command cmd;
    bool cmd_fresh, enable_req, quick_stop_req, reset_req;
    am_telemetry tlm;
    bool have_tlm;
    uint64_t tlm_cycle;
    uint32_t last_abort;
    /* transport path */
    bool have_cmd;
    double neutral_q;
    am_mode last_mode;
    uint8_t counter, echo;
    am_telemetry prev;
    bool have_prev;
    am_sim_ev tev[16];
    size_t n_tev;
};

struct am_session {
    am_session_config cfg;
    char iface[128];
    backend_kind kind;
    am_sim_joint *sims;
    size_t n_sims;
    am_joint *joints[AM_MAX_JOINTS];
    size_t n_joints;
    bool started, closed;
    uint64_t cycle_idx, t_ns;
    uint8_t *out, *in;
    size_t out_per, in_per;
    bool compact;
};

/* ---------------------------------------------------------------- strings */
const char *am_status_str(am_status st)
{
    switch (st) {
    case AM_OK: return "ok";
    case AM_ERR_PARAM: return "invalid parameter";
    case AM_ERR_STATE: return "call not allowed in the current drive or session state";
    case AM_ERR_TIMEOUT: return "timeout";
    case AM_ERR_BUS: return "bus error";
    case AM_ERR_NO_DEVICE: return "no such device";
    case AM_ERR_SDO_ABORT: return "SDO abort";
    case AM_ERR_NOT_SUPPORTED: return "not supported on this unit, bus or firmware";
    case AM_ERR_LIMIT: return "value outside the safe envelope (not clamped, nothing written)";
    case AM_ERR_AUTHORITY: return "this bus does not hold control authority";
    case AM_ERR_FAULT: return "drive is in fault";
    case AM_ERR_VERSION: return "incompatible object dictionary or SDK version";
    case AM_ERR_IMAGE: return "firmware image rejected";
    case AM_ERR_BUSY: return "another operation is in progress on the joint";
    case AM_ERR_NO_MEMORY: return "out of memory";
    case AM_ERR_STALE: return "telemetry stale";
    case AM_ERR_STO: return "STO active";
    case AM_ERR_REFERENCE: return "absolute output reference not valid";
    case AM_ERR_INTERNAL: return "internal error";
    default: return "unknown status";
    }
}

const char *am_fault_name(uint16_t code)
{
    static const struct { uint16_t c; const char *n; } t[] = {
        {0x1000, "DRV_NFAULT"}, {0x1001, "OVERCURRENT"}, {0x1002, "CUR_SAT"}, {0x1003, "CUR_IMPLAUS"},
        {0x1004, "CUR_KCL"}, {0x1005, "CUR_INVALID"}, {0x1006, "POS_INVALID"}, {0x1007, "POS_STALE"},
        {0x1008, "VBUS"}, {0x1009, "ISR_OVERRUN"}, {0x100A, "DRV_READBACK"}, {0x100B, "HOST_STALE"},
        {0x100C, "CLOCK_ORDER"}, {0x100D, "SEQUENCE"}, {0x100E, "RAIL"}, {0x100F, "NOCAL"},
        {0x1010, "PT1000"}, {0x1011, "INPUT_CURRENT"},
        {0x2000, "MOTOR_OVERTEMP_WINDING"}, {0x2001, "MAGNET_OVERTEMP_CEILING"}, {0x2002, "POWERSTAGE_OVERTEMP"},
        {0x2003, "TEMP_MODEL_INVALID"}, {0x2100, "KT_RECORD_INVALID"}, {0x2200, "CONFIG_CRC_FAIL"},
        {0x2201, "IMAGE_INCOMPATIBLE"}, {0x2300, "ENC_CROSSCHECK_MISMATCH"}, {0x2301, "OUTPUT_ENCODER_LOST"},
        {0x2302, "REFERENCE_LOST"}, {0x3000, "COMMS_LOST_BUS"}, {0x3001, "CONTROL_AUTHORITY_CONFLICT"},
        {0x3002, "CANFD_CONFIG_INVALID"}, {0x4000, "SERVICE_WRITE_DENIED_ARMED"},
    };
    size_t i;
    for (i = 0; i < sizeof t / sizeof t[0]; i++)
        if (t[i].c == code) return t[i].n;
    return "UNKNOWN";
}

uint32_t am_api_version(void)
{
    return AM_VERSION_PACK(AM_API_VERSION_MAJOR, AM_API_VERSION_MINOR, AM_API_VERSION_PATCH);
}

/* ---------------------------------------------------------------- object access helpers */
static am_status j_rd(am_joint *j, uint16_t idx, uint8_t sub, void *buf, size_t cap, size_t *len)
{
    am_session *s = j->s;
    uint32_t abort_code = 0;
    am_status st;
    size_t n = 0;
    if (s->kind == BK_SIM) {
        st = am__sim_sdo_read(j->sim, idx, sub, buf, cap, &n);
        if (st == AM_ERR_SDO_ABORT) j->last_abort = j->sim->last_abort;
    } else {
        st = s->cfg.transport->sdo_read(s->cfg.transport_user, j->address, idx, sub, buf, cap, &n, &abort_code);
        if (st == AM_ERR_SDO_ABORT) j->last_abort = abort_code;
    }
    if (len) *len = n;
    return st;
}

static am_status j_wr(am_joint *j, uint16_t idx, uint8_t sub, const void *buf, size_t len)
{
    am_session *s = j->s;
    uint32_t abort_code = 0;
    am_status st;
    if (s->kind == BK_SIM) {
        st = am__sim_sdo_write(j->sim, idx, sub, buf, len);
        if (st != AM_OK) j->last_abort = j->sim->last_abort;
        return st;
    }
    st = s->cfg.transport->sdo_write(s->cfg.transport_user, j->address, idx, sub, buf, len, &abort_code);
    if (st == AM_ERR_SDO_ABORT) j->last_abort = abort_code;
    return st;
}

/* Typed reads. An abort or a short answer is an error. */
static am_status rd_u(am_joint *j, uint16_t idx, uint8_t sub, size_t size, uint32_t *v)
{
    uint8_t b[8] = {0};
    size_t n = 0;
    am_status st = j_rd(j, idx, sub, b, sizeof b, &n);
    if (st != AM_OK) return st;
    if (n < size) return AM_ERR_BUS;
    *v = size == 1 ? b[0] : size == 2 ? am__get_u16(b) : am__get_u32(b);
    return AM_OK;
}

static am_status rd_f(am_joint *j, uint16_t idx, uint8_t sub, float *v)
{
    uint8_t b[8] = {0};
    size_t n = 0;
    am_status st = j_rd(j, idx, sub, b, sizeof b, &n);
    if (st != AM_OK) return st;
    if (n < 4) return AM_ERR_BUS;
    *v = am__get_f32(b);
    return AM_OK;
}

#define TRY(x) do { am_status st_ = (x); if (st_ != AM_OK) return st_; } while (0)

/* Raw transport read before a joint exists (scan, open). */
static am_status tp_rd_u32(am_session *s, uint16_t addr, uint16_t idx, uint8_t sub, uint32_t *v)
{
    uint8_t b[8] = {0};
    size_t n = 0;
    uint32_t ab = 0;
    am_status st = s->cfg.transport->sdo_read(s->cfg.transport_user, addr, idx, sub, b, sizeof b, &n, &ab);
    if (st != AM_OK) return st;
    *v = n >= 4 ? am__get_u32(b) : n == 2 ? am__get_u16(b) : n == 1 ? b[0] : 0;
    return AM_OK;
}

/* ---------------------------------------------------------------- lifecycle */
am_status am_init(const am_session_config *cfg, am_session **out)
{
    am_session *s;
    const char *iface;
    if (!cfg || !out) return AM_ERR_PARAM;
    *out = NULL;
    if (cfg->bus != AM_BUS_NONE && cfg->bus != AM_BUS_ETHERCAT && cfg->bus != AM_BUS_CANFD) return AM_ERR_PARAM;
    if (cfg->cycle_us != 0 && (cfg->cycle_us < 100 || cfg->cycle_us > 100000)) return AM_ERR_PARAM;
    if (cfg->transport && (!cfg->transport->pdo_exchange || !cfg->transport->sdo_read || !cfg->transport->sdo_write))
        return AM_ERR_PARAM;
    s = (am_session *)calloc(1, sizeof *s);
    if (!s) return AM_ERR_NO_MEMORY;
    s->cfg = *cfg;
    if (s->cfg.bus == AM_BUS_NONE) s->cfg.bus = AM_BUS_ETHERCAT;
    if (s->cfg.cycle_us == 0) s->cfg.cycle_us = 1000;
    if (s->cfg.sdo_timeout_ms == 0) s->cfg.sdo_timeout_ms = 100;
    if (s->cfg.stale_limit_cycles == 0) s->cfg.stale_limit_cycles = 3;
    if (s->cfg.canfd_nominal_bps == 0) s->cfg.canfd_nominal_bps = 1000000;
    if (s->cfg.canfd_data_bps == 0) s->cfg.canfd_data_bps = 5000000;
    iface = cfg->interface ? cfg->interface : (cfg->transport ? "" : "sim");
    am__copy_str(s->iface, sizeof s->iface, iface);
    s->cfg.interface = s->iface;
    s->compact = s->cfg.bus == AM_BUS_CANFD && s->cfg.canfd_compact;
    if (cfg->transport) {
        s->kind = BK_TRANSPORT;
    } else {
        const am_unit_data *units[AM_MAX_SIM_UNITS];
        size_t n = am__sim_parse_units(s->iface, units, AM_MAX_SIM_UNITS), i;
        if (n == 0) {
            free(s);
            return AM_ERR_PARAM;
        }
        s->kind = BK_SIM;
        s->sims = (am_sim_joint *)calloc(n, sizeof *s->sims);
        if (!s->sims) {
            free(s);
            return AM_ERR_NO_MEMORY;
        }
        s->n_sims = n;
        for (i = 0; i < n; i++)
            am__sim_init(&s->sims[i], units[i], (uint16_t)i, s->cfg.bus, s->cfg.cycle_us, s->cfg.telemetry_extended, NULL);
    }
    *out = s;
    return AM_OK;
}

static void dispatch(am_session *s, am_joint *j)
{
    am_sim_ev ev;
    am_event e;
    size_t i;
    if (s->kind == BK_SIM) {
        while (am__sim_pop_event(j->sim, &ev)) {
            if (s->cfg.event_cb) {
                e.kind = ev.kind; e.joint = j; e.code = ev.code; e.t_ns = ev.t_ns;
                s->cfg.event_cb(&e, s->cfg.event_user);
            }
        }
        return;
    }
    for (i = 0; i < j->n_tev; i++) {
        if (s->cfg.event_cb) {
            e.kind = j->tev[i].kind; e.joint = j; e.code = j->tev[i].code; e.t_ns = j->tev[i].t_ns;
            s->cfg.event_cb(&e, s->cfg.event_user);
        }
    }
    j->n_tev = 0;
}

am_status am_stop(am_session *s)
{
    size_t i;
    if (!s) return AM_ERR_PARAM;
    if (!s->started) return AM_OK;
    if (s->kind == BK_SIM) {
        for (i = 0; i < s->n_joints; i++) am__sim_comms_lost(s->joints[i]->sim);
    } else {
        if (s->cfg.transport->stop) s->cfg.transport->stop(s->cfg.transport_user);
        for (i = 0; i < s->n_joints; i++) {
            am_joint *j = s->joints[i];
            if (j->n_tev < 16) {
                j->tev[j->n_tev].kind = AM_EV_LINK_LOST;
                j->tev[j->n_tev].code = 0x3000;
                j->tev[j->n_tev].t_ns = j->tlm.t_ns;
                j->n_tev++;
            }
        }
    }
    for (i = 0; i < s->n_joints; i++) {
        s->joints[i]->enable_req = false;
        s->joints[i]->quick_stop_req = false;
        dispatch(s, s->joints[i]);
    }
    s->started = false;
    return AM_OK;
}

am_status am_deinit(am_session *s)
{
    size_t i;
    if (!s) return AM_OK;
    if (s->started) {
        for (i = 0; i < s->n_joints; i++) {
            s->joints[i]->enable_req = false;
            s->joints[i]->quick_stop_req = false;
        }
        (void)am_cycle(s, NULL);
        (void)am_stop(s);
    }
    for (i = 0; i < s->n_joints; i++) free(s->joints[i]);
    free(s->sims);
    free(s->out);
    free(s->in);
    free(s);
    return AM_OK;
}

/* ---------------------------------------------------------------- scan and open */
am_status am_scan(am_session *s, am_device_info *list, size_t cap, size_t *found)
{
    size_t n = 0, i;
    if (!s || (!list && cap) || !found) return AM_ERR_PARAM;
    if (s->kind == BK_SIM) {
        for (i = 0; i < s->n_sims; i++) {
            if (n < cap) {
                am_identity id;
                am_device_info *d = &list[n];
                am__sim_identity(&s->sims[i], &id);
                memset(d, 0, sizeof *d);
                d->bus = s->cfg.bus;
                d->address = (uint16_t)i;
                d->vendor_id = id.vendor_id;
                d->product_code = (uint32_t)id.sku;
                d->revision = id.revision;
                d->serial = id.serial;
                d->is_am_unit = true;
                d->od_compatible = true;
                d->has_control_authority = s->sims[i].authority == s->cfg.bus;
            }
            n++;
        }
        *found = n;
        return n > cap ? AM_ERR_NO_MEMORY : AM_OK;
    }
    {
        uint32_t a_first = s->cfg.bus == AM_BUS_CANFD ? 1 : 0, a_last = s->cfg.bus == AM_BUS_CANFD ? 63 : 255, a;
        for (a = a_first; a <= a_last; a++) {
            uint32_t vid, pc, rev, ser, odv = 0, eff = 0;
            am_status st = tp_rd_u32(s, (uint16_t)a, 0x1018, 1, &vid);
            bool is_am;
            if (st != AM_OK) {
                if (s->cfg.bus == AM_BUS_CANFD) continue;
                break;
            }
            if (tp_rd_u32(s, (uint16_t)a, 0x1018, 2, &pc) != AM_OK || tp_rd_u32(s, (uint16_t)a, 0x1018, 3, &rev) != AM_OK ||
                tp_rd_u32(s, (uint16_t)a, 0x1018, 4, &ser) != AM_OK)
                continue;
            is_am = tp_rd_u32(s, (uint16_t)a, 0x2000, 6, &odv) == AM_OK && tp_rd_u32(s, (uint16_t)a, 0x2901, 3, &eff) == AM_OK;
            if (n < cap) {
                am_device_info *d = &list[n];
                memset(d, 0, sizeof *d);
                d->bus = s->cfg.bus;
                d->address = (uint16_t)a;
                d->vendor_id = vid;
                d->product_code = pc;
                d->revision = rev;
                d->serial = ser;
                d->is_am_unit = is_am;
                d->od_compatible = is_am && ((odv >> 24) & 0xFFu) == AM_OD_MAJOR_SUPPORTED;
                d->has_control_authority = is_am && (eff & 0xFFu) == (uint32_t)s->cfg.bus;
            }
            n++;
        }
    }
    *found = n;
    return n > cap ? AM_ERR_NO_MEMORY : AM_OK;
}

static am_status read_identity_transport(am_joint *j)
{
    uint8_t b[64];
    size_t n;
    uint32_t v, rated_mNm, rated_mA, k_lsb = 20, d_lsb = 1, slot = 0;
    am_identity *id = &j->ident;
    am_transparency *tr = &j->tr;
    memset(id, 0, sizeof *id);
    TRY(rd_u(j, 0x1018, 1, 4, &id->vendor_id));
    TRY(rd_u(j, 0x1018, 2, 4, &v));
    id->sku = v <= 6 ? (am_sku)v : AM_SKU_UNKNOWN;
    TRY(rd_u(j, 0x1018, 3, 4, &id->revision));
    TRY(rd_u(j, 0x1018, 4, 4, &id->serial));
    TRY(j_rd(j, 0x2000, 5, b, sizeof b, &n));
    memcpy(id->uid, b, n < 12 ? n : 12);
    TRY(rd_u(j, 0x2000, 6, 4, &id->od_version));
    TRY(rd_u(j, 0x2000, 8, 4, &id->capabilities));
    TRY(j_rd(j, 0x2000, 9, b, sizeof b, &n));
    if (n >= 8) {
        id->cal_record_version = am__get_u32(b);
        id->cal_record_crc32 = am__get_u32(b + 4);
    }
    TRY(rd_u(j, 0x6091, 1, 4, &id->ratio_num));
    TRY(rd_u(j, 0x6091, 2, 4, &id->ratio_den));
    TRY(rd_u(j, 0x2100, 1, 4, &id->fw_cm7));
    TRY(rd_u(j, 0x2100, 2, 4, &id->fw_cm4));
    TRY(rd_u(j, 0x2100, 3, 4, &id->fw_boot));
    TRY(j_rd(j, 0x1008, 0, b, sizeof id->device_name - 1, &n));
    memcpy(id->device_name, b, n);
    TRY(j_rd(j, 0x2E00, 1, b, sizeof id->user_label - 1, &n));
    memcpy(id->user_label, b, n);
    TRY(rd_u(j, 0x6076, 0, 4, &rated_mNm));
    TRY(rd_u(j, 0x6075, 0, 4, &rated_mA));
    (void)rd_u(j, 0x2A20, 1, 2, &k_lsb);
    (void)rd_u(j, 0x2A20, 2, 2, &d_lsb);
    (void)rd_u(j, 0x2902, 3, 1, &slot);
    TRY(am_codec_init(&j->codec, rated_mNm, rated_mA, (uint16_t)(k_lsb ? k_lsb : 20), (uint16_t)(d_lsb ? d_lsb : 1),
                      (uint8_t)(slot & 3u)));
    {
        static const struct { uint8_t sub; double scale; int is_float; } m[] = {
            {2, 1e-3, 0}, {3, 1e-3, 0}, {4, 1e-3, 0}, {5, 0, 1}, {6, 1e-3, 0}, {7, 0, 1}, {8, 0, 1},
            {9, 1e-6, 0}, {10, 1e-6, 0}, {11, 1e-6, 0}};
        float *dst[10];
        size_t i;
        dst[0] = &tr->peak_torque_2s_Nm; dst[1] = &tr->rated_speed_rad_s; dst[2] = &tr->no_load_speed_48V_rad_s;
        dst[3] = &tr->reflected_inertia_kgm2; dst[4] = &tr->backdrive_torque_Nm; dst[5] = &tr->kinetic_energy_no_load_J;
        dst[6] = &tr->stiffness_Nm_per_rad; dst[7] = &tr->backlash_rad; dst[8] = &tr->sto_reaction_time_s;
        dst[9] = &tr->collision_latency_s;
        tr->rated_torque_Nm = (float)(rated_mNm * 1e-3);
        for (i = 0; i < 10; i++) {
            *dst[i] = (float)AM_NAN;
            if (m[i].is_float) {
                float f;
                if (rd_f(j, 0x2D00, m[i].sub, &f) == AM_OK) *dst[i] = f;
            } else if (rd_u(j, 0x2D00, m[i].sub, 4, &v) == AM_OK && v != 0xFFFFFFFFu) {
                *dst[i] = (float)(v * m[i].scale);
            }
        }
        tr->basis = AM_BASIS_NOT_AVAILABLE;
        if (rd_u(j, 0x2D00, 12, 1, &v) == AM_OK && v <= 3) tr->basis = (am_data_basis)v;
    }
    return AM_OK;
}

static am_status get_limits_impl(am_joint *j, am_limits *eff, am_limits *ceil);

am_status am_open_joint(am_session *s, uint16_t address, am_joint **out)
{
    am_joint *j;
    size_t i;
    am_status st;
    if (!s || !out) return AM_ERR_PARAM;
    *out = NULL;
    if (s->closed) return AM_ERR_STATE;
    for (i = 0; i < s->n_joints; i++)
        if (s->joints[i]->address == address) {
            *out = s->joints[i];
            return AM_OK;
        }
    if (s->started) return AM_ERR_STATE;
    if (s->n_joints >= AM_MAX_JOINTS) return AM_ERR_NO_MEMORY;
    if (s->kind == BK_SIM && address >= s->n_sims) return AM_ERR_NO_DEVICE;
    j = (am_joint *)calloc(1, sizeof *j);
    if (!j) return AM_ERR_NO_MEMORY;
    j->s = s;
    j->address = address;
    j->index = s->n_joints;
    j->tlm_cycle = 0;
    if (s->kind == BK_SIM) {
        j->sim = &s->sims[address];
        am__sim_identity(j->sim, &j->ident);
        am__sim_transparency(j->sim, &j->tr);
        (void)am_codec_init(&j->codec, (uint32_t)am__round_even(j->sim->ud->rated_nm * 1e3),
                            (uint32_t)am__round_even(am__sim_rated_current(j->sim) * 1e3), 20, 1, (uint8_t)(address % 4));
    } else {
        uint32_t probe;
        if (tp_rd_u32(s, address, 0x1018, 1, &probe) != AM_OK) {
            free(j);
            return AM_ERR_NO_DEVICE;
        }
        st = read_identity_transport(j);
        if (st != AM_OK) {
            free(j);
            return st;
        }
    }
    if (((j->ident.od_version >> 24) & 0xFFu) != AM_OD_MAJOR_SUPPORTED) {
        free(j);
        return AM_ERR_VERSION;
    }
    st = get_limits_impl(j, &j->limits_cache, NULL);
    if (st != AM_OK) {
        free(j);
        return st;
    }
    s->joints[s->n_joints++] = j;
    *out = j;
    return AM_OK;
}

am_status am_start(am_session *s)
{
    int k;
    if (!s) return AM_ERR_PARAM;
    if (s->closed) return AM_ERR_STATE;
    if (s->started) return AM_OK;
    if (s->kind == BK_TRANSPORT) {
        size_t i, n = s->n_joints;
        uint16_t addr[AM_MAX_JOINTS];
        s->out_per = s->compact ? AM_CFD_CMD_SLOT_LEN : AM_PDO_RX_1601_LEN;
        s->in_per = s->compact ? (s->cfg.telemetry_extended ? AM_CFD_FB_EXT_LEN : AM_CFD_FB_LEN)
                               : AM_PDO_TX_1A00_LEN + (s->cfg.telemetry_extended ? AM_PDO_TX_1A01_LEN : 0u);
        free(s->out);
        free(s->in);
        s->out = (uint8_t *)calloc(n ? n : 1, s->out_per);
        s->in = (uint8_t *)calloc(n ? n : 1, s->in_per);
        if (!s->out || !s->in) return AM_ERR_NO_MEMORY;
        for (i = 0; i < n; i++) addr[i] = s->joints[i]->address;
        if (s->cfg.transport->start) TRY(s->cfg.transport->start(s->cfg.transport_user, addr, n, s->out_per, s->in_per));
    }
    s->started = true;
    for (k = 0; k < 5; k++) TRY(am_cycle(s, NULL));
    return AM_OK;
}

/* ---------------------------------------------------------------- cycle */
static am_controlword controlword(am_joint *j)
{
    uint16_t sw = j->have_tlm ? j->tlm.statusword : 0;
    am_drive_state st = am_codec_decode_state(sw);
    if (j->reset_req) {
        j->reset_req = false;
        return AM_CW_FAULT_RESET;
    }
    if (j->quick_stop_req) return AM_CW_QUICK_STOP;
    if (j->enable_req) return am_codec_next_controlword(sw, true);
    if (st == AM_DS_READY_TO_SWITCH_ON || st == AM_DS_SWITCHED_ON || st == AM_DS_OPERATION_ENABLED)
        return AM_CW_SHUTDOWN;
    return AM_CW_DISABLE_VOLTAGE;
}

static void push_tev(am_joint *j, am_event_kind k, uint16_t code, uint64_t t)
{
    if (j->n_tev < sizeof j->tev / sizeof j->tev[0]) {
        j->tev[j->n_tev].kind = k;
        j->tev[j->n_tev].code = code;
        j->tev[j->n_tev].t_ns = t;
        j->n_tev++;
    }
}

static bool is_fault(am_drive_state st) { return st == AM_DS_FAULT || st == AM_DS_FAULT_REACTION_ACTIVE; }

static void transport_events(am_joint *j, const am_telemetry *b)
{
    const am_telemetry *a = j->have_prev ? &j->prev : NULL;
    uint16_t new_w;
    if (!a || a->state != b->state) push_tev(j, AM_EV_STATE_CHANGED, (uint16_t)b->state, b->t_ns);
    if (a && a->sto_active != b->sto_active) push_tev(j, AM_EV_STO_CHANGED, b->sto_active ? 1 : 0, b->t_ns);
    if (is_fault(b->state) && !(a && is_fault(a->state))) push_tev(j, AM_EV_FAULT, b->error_code, b->t_ns);
    if (a && is_fault(a->state) && !is_fault(b->state)) push_tev(j, AM_EV_FAULT_CLEARED, 0, b->t_ns);
    new_w = (uint16_t)(b->warning_word & ~(a ? a->warning_word : 0u));
    if (new_w) push_tev(j, AM_EV_WARNING, new_w, b->t_ns);
    j->prev = *b;
    j->have_prev = true;
}

/* Transport path: what to send. Until the host sets a command after enable or a mode change,
 * send a neutral one: hold the latched position in CSP, zero torque in the other modes. */
static am_status pack_joint(am_joint *j, am_controlword cw, uint8_t *out)
{
    am_session *s = j->s;
    am_command send;
    const am_telemetry *t = j->have_tlm ? &j->tlm : NULL;
    if (!t || t->state != AM_DS_OPERATION_ENABLED) {
        j->have_cmd = false;
        j->neutral_q = t ? t->q_rad : 0.0;
    }
    if (j->mode != j->last_mode) {
        j->have_cmd = false;
        j->neutral_q = t ? t->q_rad : 0.0;
        j->last_mode = j->mode;
    }
    if (j->cmd_fresh) j->have_cmd = true;
    if (j->have_cmd) {
        send = j->cmd;
    } else {
        memset(&send, 0, sizeof send);
        send.mode = j->mode;
        if (j->mode == AM_MODE_CSP) send.q_ref_rad = j->neutral_q;
    }
    if (s->compact) {
        am_drive_state st = t ? t->state : AM_DS_NOT_READY_TO_SWITCH_ON;
        bool enable = cw == AM_CW_SWITCH_ON || cw == AM_CW_ENABLE_OPERATION ||
                      (cw == AM_CW_SHUTDOWN && st == AM_DS_SWITCH_ON_DISABLED);
        uint8_t ctl = (uint8_t)(am_codec_compact_mode(j->mode) << AM_CC_MODE_SHIFT);
        if (cw == AM_CW_FAULT_RESET) ctl |= AM_CC_FAULT_RESET;
        else if (cw == AM_CW_QUICK_STOP) ctl |= AM_CC_QUICK_STOP;
        else if (enable) ctl |= AM_CC_ENABLE;
        j->counter = (uint8_t)(j->counter + 1u);
        return am_codec_pack_cfd_slot(&j->codec, ctl, j->counter, &send, out);
    }
    return am_codec_pack_rx1601(&j->codec, cw, &send, out);
}

static bool all_zero(const uint8_t *p, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++)
        if (p[i]) return false;
    return true;
}

static bool unpack_joint(am_joint *j, const uint8_t *in, uint64_t t_ns)
{
    am_session *s = j->s;
    am_telemetry t;
    if (all_zero(in, s->in_per)) return false;          /* nothing arrived for this joint */
    t = j->have_tlm ? j->tlm : j->tlm;
    if (s->compact) {
        if (am_codec_unpack_cfd_fb(&j->codec, in, s->in_per, &t, &j->echo) != AM_OK) return false;
        t.sync_flags = AM_SYNC_CAN_LOCKED | AM_SYNC_TIMESTAMP_VALID;
        if (!t.extended_valid) t.t_ns = t_ns;
    } else {
        if (am_codec_unpack_tx1a00(&j->codec, in, &t) != AM_OK) return false;
        if (s->in_per >= AM_PDO_TX_1A00_LEN + AM_PDO_TX_1A01_LEN) {
            (void)am_codec_unpack_tx1a01(&j->codec, in + AM_PDO_TX_1A00_LEN, &t);
        } else {
            t.extended_valid = false;
            t.t_ns = t_ns;
        }
    }
    transport_events(j, &t);
    j->tlm = t;
    return true;
}

am_status am_cycle(am_session *s, uint64_t *t_ns)
{
    size_t i;
    if (!s) return AM_ERR_PARAM;
    if (s->closed || !s->started) return AM_ERR_STATE;
    s->cycle_idx++;
    s->t_ns = s->cycle_idx * (uint64_t)s->cfg.cycle_us * 1000u;
    if (s->kind == BK_SIM) {
        for (i = 0; i < s->n_joints; i++) {
            am_joint *j = s->joints[i];
            am_controlword cw = controlword(j);
            am__sim_exchange(j->sim, cw, j->mode, &j->cmd, j->cmd_fresh, &j->tlm);
            j->have_tlm = true;
            j->tlm_cycle = s->cycle_idx;
        }
    } else {
        uint64_t tt = 0;
        am_status st;
        for (i = 0; i < s->n_joints; i++) {
            am_joint *j = s->joints[i];
            TRY(pack_joint(j, controlword(j), s->out + i * s->out_per));
        }
        memset(s->in, 0, s->n_joints * s->in_per);
        st = s->cfg.transport->pdo_exchange(s->cfg.transport_user, s->out, s->n_joints * s->out_per, s->in,
                                            s->n_joints * s->in_per, &tt);
        for (i = 0; i < s->n_joints; i++) s->joints[i]->cmd_fresh = false;
        if (st != AM_OK) return st;
        for (i = 0; i < s->n_joints; i++) {
            am_joint *j = s->joints[i];
            if (unpack_joint(j, s->in + i * s->in_per, tt ? tt : s->t_ns)) {
                j->have_tlm = true;
                j->tlm_cycle = s->cycle_idx;
            }
        }
    }
    for (i = 0; i < s->n_joints; i++) {
        am_joint *j = s->joints[i];
        const am_telemetry *t = &j->tlm;
        j->cmd_fresh = false;
        if (!j->have_tlm || j->tlm_cycle != s->cycle_idx) continue;
        if (t->sto_active || is_fault(t->state) || t->state == AM_DS_QUICK_STOP_ACTIVE) j->enable_req = false;
        if (t->state == AM_DS_SWITCH_ON_DISABLED && j->quick_stop_req) j->quick_stop_req = false;
    }
    for (i = 0; i < s->n_joints; i++) dispatch(s, s->joints[i]);
    if (t_ns) *t_ns = s->t_ns;
    return AM_OK;
}

/* ---------------------------------------------------------------- identity */
am_status am_get_identity(const am_joint *j, am_identity *out)
{
    if (!j || !out) return AM_ERR_PARAM;
    if (j->sim) am__sim_identity(j->sim, out);
    else *out = j->ident;
    return AM_OK;
}

am_status am_get_transparency(const am_joint *j, am_transparency *out)
{
    if (!j || !out) return AM_ERR_PARAM;
    *out = j->tr;
    return AM_OK;
}

/* ---------------------------------------------------------------- mode and state */
static am_status get_control_bus_impl(am_joint *j, am_bus *configured, am_bus *effective)
{
    uint32_t c, e;
    if (j->sim) {
        *configured = j->sim->authority_cfg;
        *effective = j->sim->authority;
        return AM_OK;
    }
    TRY(rd_u(j, 0x2901, 1, 1, &c));
    TRY(rd_u(j, 0x2901, 3, 1, &e));
    *configured = (am_bus)c;
    *effective = (am_bus)e;
    return AM_OK;
}

static am_status check_active(am_joint *j)
{
    am_session *s = j->s;
    if (s->closed) return AM_ERR_STATE;
    if (j->have_tlm && is_fault(j->tlm.state)) return AM_ERR_FAULT;
    if (s->kind == BK_TRANSPORT && j->have_tlm) {
        if (!(j->tlm.statusword & AM_SW_REMOTE)) return AM_ERR_AUTHORITY;
    } else {
        am_bus c, e;
        TRY(get_control_bus_impl(j, &c, &e));
        if (e != s->cfg.bus) return AM_ERR_AUTHORITY;
    }
    return AM_OK;
}

am_status am_set_mode(am_joint *j, am_mode mode)
{
    if (!j) return AM_ERR_PARAM;
    if (mode != AM_MODE_CSP && mode != AM_MODE_CSV && mode != AM_MODE_CST && mode != AM_MODE_IMPEDANCE &&
        mode != AM_MODE_CURRENT)
        return AM_ERR_PARAM;
    if (mode == AM_MODE_CURRENT) {
        uint32_t en = 0;
        if (!(j->ident.capabilities & AM_CAP_MODE_CURRENT)) return AM_ERR_NOT_SUPPORTED;
        TRY(rd_u(j, 0x2A11, 3, 1, &en));
        if (!en) return AM_ERR_NOT_SUPPORTED;
        if (j->have_tlm && j->tlm.state == AM_DS_OPERATION_ENABLED) return AM_ERR_STATE;
    }
    j->mode = mode;
    memset(&j->cmd, 0, sizeof j->cmd);
    j->cmd.mode = mode;
    j->cmd_fresh = false;
    return AM_OK;
}

am_status am_enable(am_joint *j, uint32_t timeout_ms)
{
    am_session *s;
    uint64_t n, k;
    if (!j) return AM_ERR_PARAM;
    s = j->s;
    if (!s->started) return AM_ERR_STATE;
    if (j->mode == AM_MODE_NONE) return AM_ERR_STATE;
    TRY(check_active(j));
    if (j->have_tlm && j->tlm.sto_active) return AM_ERR_STO;
    j->enable_req = true;
    j->quick_stop_req = false;
    n = (uint64_t)timeout_ms * 1000u / s->cfg.cycle_us;
    if (n < 1) n = 1;
    for (k = 0; k < n; k++) {
        am_status st = am_cycle(s, NULL);
        if (st != AM_OK) {
            j->enable_req = false;
            return st;
        }
        if (!j->have_tlm) continue;
        if (j->tlm.state == AM_DS_OPERATION_ENABLED) return AM_OK;
        if (j->tlm.sto_active) {
            j->enable_req = false;
            return AM_ERR_STO;
        }
        if (is_fault(j->tlm.state)) {
            j->enable_req = false;
            return AM_ERR_FAULT;
        }
    }
    j->enable_req = false;
    if (!j->have_tlm) return AM_ERR_BUS;
    if (j->tlm.ref_state != AM_REF_VALID) return AM_ERR_REFERENCE;
    return AM_ERR_TIMEOUT;
}

am_status am_disable(am_joint *j)
{
    if (!j) return AM_ERR_PARAM;
    j->enable_req = false;
    j->quick_stop_req = false;
    return AM_OK;
}

am_status am_quick_stop(am_joint *j)
{
    if (!j) return AM_ERR_PARAM;
    j->quick_stop_req = true;
    j->enable_req = false;
    return AM_OK;
}

/* ---------------------------------------------------------------- commands */
static bool finite_d(double v) { return v == v && v < 1e300 && v > -1e300; }

am_status am_set_command(am_joint *j, const am_command *cmd)
{
    if (!j || !cmd) return AM_ERR_PARAM;
    TRY(check_active(j));
    if (cmd->mode != j->mode) return AM_ERR_STATE;
    if (!finite_d(cmd->q_ref_rad) || !finite_d(cmd->qd_ref_rad_s) || !finite_d(cmd->tau_ff_Nm) ||
        !finite_d(cmd->kp_Nm_per_rad) || !finite_d(cmd->kd_Nm_s_per_rad) || !finite_d(cmd->iq_ref_A))
        return AM_ERR_PARAM;
    if (fabs(cmd->q_ref_rad) > SANITY_Q || fabs(cmd->qd_ref_rad_s) > SANITY_RATE ||
        fabs(cmd->tau_ff_Nm) > SANITY_TORQUE_NM || fabs(cmd->iq_ref_A) > SANITY_TORQUE_NM)
        return AM_ERR_PARAM;
    if (cmd->kp_Nm_per_rad < 0 || cmd->kd_Nm_s_per_rad < 0) return AM_ERR_PARAM;
    if (j->mode == AM_MODE_IMPEDANCE && (cmd->kp_Nm_per_rad > j->limits_cache.kp_max_Nm_per_rad ||
                                         cmd->kd_Nm_s_per_rad > j->limits_cache.kd_max_Nm_s_per_rad))
        return AM_ERR_LIMIT;
    j->cmd = *cmd;
    j->cmd_fresh = true;
    return AM_OK;
}

am_status am_set_impedance(am_joint *j, const am_impedance *imp)
{
    am_command c;
    if (!j || !imp) return AM_ERR_PARAM;
    if (j->mode != AM_MODE_IMPEDANCE) return AM_ERR_STATE;
    memset(&c, 0, sizeof c);
    c.mode = AM_MODE_IMPEDANCE;
    c.q_ref_rad = imp->q_ref_rad;
    c.qd_ref_rad_s = imp->qd_ref_rad_s;
    c.kp_Nm_per_rad = imp->kp_Nm_per_rad;
    c.kd_Nm_s_per_rad = imp->kd_Nm_s_per_rad;
    c.tau_ff_Nm = imp->tau_ff_Nm;
    return am_set_command(j, &c);
}

am_status am_set_position(am_joint *j, double q_rad, float qd_ff_rad_s, float tau_ff_Nm)
{
    am_command c;
    if (!j) return AM_ERR_PARAM;
    if (j->mode != AM_MODE_CSP) return AM_ERR_STATE;
    memset(&c, 0, sizeof c);
    c.mode = AM_MODE_CSP;
    c.q_ref_rad = q_rad;
    c.qd_ref_rad_s = qd_ff_rad_s;
    c.tau_ff_Nm = tau_ff_Nm;
    return am_set_command(j, &c);
}

am_status am_set_velocity(am_joint *j, float qd_rad_s, float tau_ff_Nm)
{
    am_command c;
    if (!j) return AM_ERR_PARAM;
    if (j->mode != AM_MODE_CSV) return AM_ERR_STATE;
    memset(&c, 0, sizeof c);
    c.mode = AM_MODE_CSV;
    c.qd_ref_rad_s = qd_rad_s;
    c.tau_ff_Nm = tau_ff_Nm;
    return am_set_command(j, &c);
}

am_status am_set_torque(am_joint *j, float tau_Nm)
{
    am_command c;
    if (!j) return AM_ERR_PARAM;
    if (j->mode != AM_MODE_CST) return AM_ERR_STATE;
    memset(&c, 0, sizeof c);
    c.mode = AM_MODE_CST;
    c.tau_ff_Nm = tau_Nm;
    return am_set_command(j, &c);
}

am_status am_set_current(am_joint *j, float iq_A)
{
    am_command c;
    if (!j) return AM_ERR_PARAM;
    if (j->mode != AM_MODE_CURRENT) return AM_ERR_STATE;
    memset(&c, 0, sizeof c);
    c.mode = AM_MODE_CURRENT;
    c.iq_ref_A = iq_A;
    return am_set_command(j, &c);
}

am_status am_read_telemetry(const am_joint *j, am_telemetry *out)
{
    if (!j || !out) return AM_ERR_PARAM;
    if (!j->have_tlm) return AM_ERR_STALE;
    if (j->s->cycle_idx - j->tlm_cycle > j->s->cfg.stale_limit_cycles) return AM_ERR_STALE;
    *out = j->tlm;
    return AM_OK;
}

/* ---------------------------------------------------------------- faults */
am_status am_get_faults(am_joint *j, am_fault_info *out)
{
    uint32_t v;
    if (!j || !out) return AM_ERR_PARAM;
    if (j->sim) {
        am__sim_fault_info(j->sim, out);
        return AM_OK;
    }
    memset(out, 0, sizeof *out);
    TRY(rd_u(j, 0x603F, 0, 2, &v)); out->error_code = (uint16_t)v;
    TRY(rd_u(j, 0x2600, 1, 4, &out->fault_word));
    TRY(rd_u(j, 0x2600, 2, 4, &out->first_fault));
    TRY(rd_u(j, 0x2602, 1, 4, &out->warning_word));
    TRY(rd_u(j, 0x2603, 1, 4, &out->boot_block));
    TRY(rd_u(j, 0x2620, 1, 1, &v)); out->safety_io = (uint8_t)v;
    out->latched = out->fault_word != 0 || out->error_code != 0 || (j->have_tlm && is_fault(j->tlm.state));
    return AM_OK;
}

am_status am_clear_faults(am_joint *j, uint32_t timeout_ms)
{
    am_session *s;
    uint64_t n, k;
    if (!j) return AM_ERR_PARAM;
    s = j->s;
    if (!s->started) return AM_ERR_STATE;
    if (j->have_tlm && !is_fault(j->tlm.state)) return AM_OK;
    j->reset_req = true;
    n = (uint64_t)timeout_ms * 1000u / s->cfg.cycle_us;
    if (n < 2) n = 2;
    for (k = 0; k < n; k++) {
        TRY(am_cycle(s, NULL));
        if (j->have_tlm && !is_fault(j->tlm.state)) return AM_OK;
        j->reset_req = j->have_tlm && j->tlm.state == AM_DS_FAULT && (s->cycle_idx % 2 == 0);
    }
    return AM_ERR_FAULT;
}

am_status am_get_fault_history(am_joint *j, am_fault_record *buf, size_t cap, size_t *n)
{
    uint32_t count, i;
    if (!j || !n || (!buf && cap)) return AM_ERR_PARAM;
    *n = 0;
    TRY(rd_u(j, 0x2601, 0, 1, &count));
    for (i = 1; i <= count && i <= AM_SIM_HIST && *n < cap; i++) {
        uint8_t b[32];
        size_t len = 0;
        am_fault_record *r = &buf[*n];
        TRY(j_rd(j, 0x2601, (uint8_t)i, b, sizeof b, &len));
        if (len < 24) return AM_ERR_BUS;
        r->code = am__get_u16(b);
        r->drive_state = b[2];
        r->t_ns = am__get_u64(b + 4);
        r->vbus_V = (float)((int32_t)am__get_u32(b + 12) * 1e-3);
        r->iq_A = (float)((int32_t)am__get_u32(b + 16) * 1e-3);
        r->t_winding_C = (float)((int16_t)am__get_u16(b + 20) * 0.1);
        r->t_power_stage_C = (float)((int16_t)am__get_u16(b + 22) * 0.1);
        (*n)++;
    }
    return AM_OK;
}

uint32_t am_last_abort_code(const am_joint *j) { return j ? j->last_abort : 0; }

/* ---------------------------------------------------------------- limits and gains */
static double pos_from_obj(int32_t v)
{
    if (v == INT32_MIN) return -INFINITY;
    if (v == INT32_MAX) return INFINITY;
    return v * 1e-6;
}

static am_status get_limits_impl(am_joint *j, am_limits *eff, am_limits *ceil)
{
    uint32_t v;
    float f;
    const am_codec *c = &j->codec;
    if (j->sim) {
        if (eff) am__sim_lim_to_api(&j->sim->limits, eff);
        if (ceil) am__sim_lim_to_api(&j->sim->ceiling, ceil);
        return AM_OK;
    }
    if (eff) {
        TRY(rd_u(j, 0x60E0, 0, 2, &v)); eff->torque_pos_Nm = (float)(v * (double)c->rated_torque_Nm * 1e-3);
        TRY(rd_u(j, 0x60E1, 0, 2, &v)); eff->torque_neg_Nm = (float)(v * (double)c->rated_torque_Nm * 1e-3);
        TRY(rd_u(j, 0x6073, 0, 2, &v)); eff->current_A = (float)(v * (double)c->rated_current_A * 1e-3);
        TRY(rd_f(j, 0x2300, 4, &f)); eff->speed_rad_s = f;
        TRY(rd_u(j, 0x607D, 1, 4, &v)); eff->pos_min_rad = pos_from_obj((int32_t)v);
        TRY(rd_u(j, 0x607D, 2, 4, &v)); eff->pos_max_rad = pos_from_obj((int32_t)v);
        TRY(rd_u(j, 0x2A00, 4, 4, &v)); eff->kp_max_Nm_per_rad = (float)(v * 1e-3);
        TRY(rd_u(j, 0x2A00, 5, 4, &v)); eff->kd_max_Nm_s_per_rad = (float)(v * 1e-3);
        TRY(rd_u(j, 0x2300, 20, 4, &eff->comms_loss_ms));
    }
    if (ceil) {
        ceil->torque_pos_Nm = j->tr.peak_torque_2s_Nm;
        ceil->torque_neg_Nm = j->tr.peak_torque_2s_Nm;
        TRY(rd_f(j, 0x2300, 1, &f)); ceil->current_A = f;
        ceil->speed_rad_s = j->tr.no_load_speed_48V_rad_s;
        ceil->pos_min_rad = -INFINITY;
        ceil->pos_max_rad = INFINITY;
        TRY(rd_u(j, 0x2A50, 1, 4, &v)); ceil->kp_max_Nm_per_rad = (float)(v * 1e-3);
        TRY(rd_u(j, 0x2A50, 2, 4, &v)); ceil->kd_max_Nm_s_per_rad = (float)(v * 1e-3);
        ceil->comms_loss_ms = 1000;
    }
    return AM_OK;
}

am_status am_get_limits(am_joint *j, am_limits *effective, am_limits *ceiling)
{
    if (!j) return AM_ERR_PARAM;
    return get_limits_impl(j, effective, ceiling);
}

static bool above(double v, double cap, double tol) { return !(v == v) || v < 0 || v > cap + tol; }

static am_status wr_u(am_joint *j, uint16_t idx, uint8_t sub, uint32_t v, size_t size)
{
    uint8_t b[4];
    am__put_u32(b, v);
    return j_wr(j, idx, sub, b, size);
}

static am_status wr_f(am_joint *j, uint16_t idx, uint8_t sub, float v)
{
    uint8_t b[4];
    am__put_f32(b, v);
    return j_wr(j, idx, sub, b, 4);
}

/* Map a write failure inside a limits or gains call: range aborts mean AM_ERR_LIMIT. */
static am_status lim_status(am_joint *j, am_status st)
{
    if (st == AM_ERR_SDO_ABORT) {
        uint32_t a = j->last_abort;
        if (a == 0x06090030u || a == 0x06090031u || a == 0x06090032u || a == 0x06090036u) return AM_ERR_LIMIT;
        if (a == 0x08000022u) return AM_ERR_STATE;
    }
    return st;
}

static am_status enc_u(double v, double scale, double hi, uint32_t *out)
{
    double r = am__round_away(v * scale);
    if (!(r >= 0.0) || r > hi) return AM_ERR_PARAM;
    *out = (uint32_t)r;
    return AM_OK;
}

static am_status enc_pos(double v, uint32_t *out)
{
    double r;
    if (v == -INFINITY) { *out = (uint32_t)INT32_MIN; return AM_OK; }
    if (v == INFINITY) { *out = (uint32_t)INT32_MAX; return AM_OK; }
    if (!(v == v)) return AM_ERR_PARAM;
    r = am__round_away(v * 1e6);
    if (r <= -2147483648.0 || r >= 2147483647.0) return AM_ERR_PARAM;
    *out = (uint32_t)(int32_t)r;
    return AM_OK;
}

am_status am_set_limits(am_joint *j, const am_limits *lim)
{
    am_limits eff, c;
    uint32_t tp, tn, cur, kp, kd, pmin, pmax;
    double rt, ra;
    if (!j || !lim) return AM_ERR_PARAM;
    if (j->have_tlm && j->tlm.state == AM_DS_OPERATION_ENABLED) return AM_ERR_STATE;
    if (j->sim) {
        /* am_limits carries floats: a value within float rounding of the ceiling is the ceiling */
        const am_sim_lim *cl = &j->sim->ceiling;
        am_sim_lim l;
        am__sim_lim_from_api(lim, &l);
#define SNAP(f) if (cl->f < l.f && l.f <= cl->f * (1.0 + 1e-6)) l.f = cl->f
        SNAP(torque_pos_Nm); SNAP(torque_neg_Nm); SNAP(current_A); SNAP(speed_rad_s);
        SNAP(kp_max_Nm_per_rad); SNAP(kd_max_Nm_s_per_rad);
#undef SNAP
        TRY(am__sim_set_limits(j->sim, &l));
        am__sim_lim_to_api(&j->sim->limits, &j->limits_cache);
        return AM_OK;
    }
    TRY(get_limits_impl(j, &eff, &c));
    rt = j->codec.rated_torque_Nm;
    ra = j->codec.rated_current_A;
    /* tolerance = one LSB of the wire unit, so limits read back can always be written again */
    if (above(lim->torque_pos_Nm, c.torque_pos_Nm, rt * 1e-3) || above(lim->torque_neg_Nm, c.torque_neg_Nm, rt * 1e-3) ||
        above(lim->current_A, c.current_A, ra * 1e-3) || above(lim->speed_rad_s, c.speed_rad_s, 1e-3) ||
        above(lim->kp_max_Nm_per_rad, c.kp_max_Nm_per_rad, 1e-3) ||
        above(lim->kd_max_Nm_s_per_rad, c.kd_max_Nm_s_per_rad, 1e-3) || lim->comms_loss_ms > c.comms_loss_ms)
        return AM_ERR_LIMIT;
    if (lim->pos_min_rad > lim->pos_max_rad) return AM_ERR_PARAM;
    TRY(enc_u(lim->torque_pos_Nm, 1000.0 / rt, 65535.0, &tp));
    TRY(enc_u(lim->torque_neg_Nm, 1000.0 / rt, 65535.0, &tn));
    TRY(enc_u(lim->current_A, 1000.0 / ra, 65535.0, &cur));
    TRY(enc_u(lim->kp_max_Nm_per_rad, 1e3, 4294967295.0, &kp));
    TRY(enc_u(lim->kd_max_Nm_s_per_rad, 1e3, 4294967295.0, &kd));
    TRY(enc_pos(lim->pos_min_rad, &pmin));
    TRY(enc_pos(lim->pos_max_rad, &pmax));
    TRY(lim_status(j, wr_u(j, 0x60E0, 0, tp, 2)));
    TRY(lim_status(j, wr_u(j, 0x60E1, 0, tn, 2)));
    TRY(lim_status(j, wr_u(j, 0x6073, 0, cur, 2)));
    TRY(lim_status(j, wr_f(j, 0x2300, 4, lim->speed_rad_s)));
    TRY(lim_status(j, wr_u(j, 0x2A00, 4, kp, 4)));
    TRY(lim_status(j, wr_u(j, 0x2A00, 5, kd, 4)));
    TRY(lim_status(j, wr_u(j, 0x2300, 20, lim->comms_loss_ms, 4)));
    if (lim->pos_min_rad > eff.pos_max_rad) {
        TRY(lim_status(j, wr_u(j, 0x607D, 2, pmax, 4)));
        TRY(lim_status(j, wr_u(j, 0x607D, 1, pmin, 4)));
    } else {
        TRY(lim_status(j, wr_u(j, 0x607D, 1, pmin, 4)));
        TRY(lim_status(j, wr_u(j, 0x607D, 2, pmax, 4)));
    }
    return get_limits_impl(j, &j->limits_cache, NULL);
}

am_status am_get_loop_gains(am_joint *j, am_loop_gains *g)
{
    if (!j || !g) return AM_ERR_PARAM;
    if (j->sim) {
        am__sim_gain_to_api(&j->sim->gains, g);
        return AM_OK;
    }
    TRY(rd_f(j, 0x2A30, 1, &g->pos_kp_1_per_s));
    TRY(rd_f(j, 0x2A30, 2, &g->vel_kp_Nm_s_per_rad));
    TRY(rd_f(j, 0x2A30, 3, &g->vel_ki_Nm_per_rad));
    TRY(rd_f(j, 0x2A40, 1, &g->vel_filter_hz));
    TRY(rd_f(j, 0x2A40, 2, &g->torque_cmd_filter_hz));
    TRY(rd_f(j, 0x2A40, 3, &g->notch_hz));
    TRY(rd_f(j, 0x2A40, 4, &g->notch_q));
    return AM_OK;
}

am_status am_set_loop_gains(am_joint *j, const am_loop_gains *g)
{
    if (!j || !g) return AM_ERR_PARAM;
    if (j->have_tlm && j->tlm.state == AM_DS_OPERATION_ENABLED) return AM_ERR_STATE;
    if (!finite_d(g->pos_kp_1_per_s) || !finite_d(g->vel_kp_Nm_s_per_rad) || !finite_d(g->vel_ki_Nm_per_rad) ||
        !finite_d(g->vel_filter_hz) || !finite_d(g->torque_cmd_filter_hz) || !finite_d(g->notch_hz) ||
        !finite_d(g->notch_q))
        return AM_ERR_PARAM;
    if (j->sim) {
        am_sim_gain sg;
        am__sim_gain_from_api(g, &sg);
        return am__sim_set_loop_gains(j->sim, &sg);
    }
    {
        uint8_t b[16];
        size_t n = 0;
        float lo, hi, vp, vi, fmin;
        TRY(j_rd(j, 0x2A50, 3, b, sizeof b, &n));
        if (n < 8) return AM_ERR_BUS;
        lo = am__get_f32(b);
        hi = am__get_f32(b + 4);
        TRY(rd_f(j, 0x2A50, 4, &vp));
        TRY(rd_f(j, 0x2A50, 5, &vi));
        TRY(rd_f(j, 0x2A50, 6, &fmin));
        if (g->pos_kp_1_per_s < lo || g->pos_kp_1_per_s > hi || g->vel_kp_Nm_s_per_rad < 0 ||
            g->vel_kp_Nm_s_per_rad > vp || g->vel_ki_Nm_per_rad < 0 || g->vel_ki_Nm_per_rad > vi ||
            g->vel_filter_hz < fmin || (g->torque_cmd_filter_hz != 0.0f && g->torque_cmd_filter_hz < fmin))
            return AM_ERR_LIMIT;
    }
    TRY(lim_status(j, wr_f(j, 0x2A30, 1, g->pos_kp_1_per_s)));
    TRY(lim_status(j, wr_f(j, 0x2A30, 2, g->vel_kp_Nm_s_per_rad)));
    TRY(lim_status(j, wr_f(j, 0x2A30, 3, g->vel_ki_Nm_per_rad)));
    TRY(lim_status(j, wr_f(j, 0x2A40, 1, g->vel_filter_hz)));
    TRY(lim_status(j, wr_f(j, 0x2A40, 2, g->torque_cmd_filter_hz)));
    TRY(lim_status(j, wr_f(j, 0x2A40, 3, g->notch_hz)));
    TRY(lim_status(j, wr_f(j, 0x2A40, 4, g->notch_q)));
    return AM_OK;
}

am_status am_store_parameters(am_joint *j)
{
    if (!j) return AM_ERR_PARAM;
    return lim_status(j, wr_u(j, 0x1010, 1, STORE_SIGNATURE, 4));
}

am_status am_restore_defaults(am_joint *j)
{
    uint8_t b[32];
    size_t n = 0;
    if (!j) return AM_ERR_PARAM;
    TRY(lim_status(j, wr_u(j, 0x1011, 1, RESTORE_SIGNATURE, 4)));
    if (j_rd(j, 0x2E00, 1, b, sizeof j->ident.user_label - 1, &n) == AM_OK) {
        memcpy(j->ident.user_label, b, n);
        j->ident.user_label[n] = '\0';
    }
    return get_limits_impl(j, &j->limits_cache, NULL);
}

am_status am_set_user_label(am_joint *j, const char *label)
{
    size_t n;
    if (!j || !label) return AM_ERR_PARAM;
    n = strlen(label);
    if (n > 31) return AM_ERR_PARAM;
    TRY(lim_status(j, j_wr(j, 0x2E00, 1, label, n)));
    am__copy_str(j->ident.user_label, sizeof j->ident.user_label, label);
    return AM_OK;
}

am_status am_get_control_bus(am_joint *j, am_bus *configured, am_bus *effective)
{
    am_bus c, e;
    if (!j) return AM_ERR_PARAM;
    TRY(get_control_bus_impl(j, &c, &e));
    if (configured) *configured = c;
    if (effective) *effective = e;
    return AM_OK;
}

am_status am_set_control_bus(am_joint *j, am_bus bus)
{
    uint8_t b;
    if (!j) return AM_ERR_PARAM;
    if (bus != AM_BUS_NONE && bus != AM_BUS_ETHERCAT && bus != AM_BUS_CANFD) return AM_ERR_PARAM;
    if (j->sim) return am__sim_set_control_bus(j->sim, bus);
    b = (uint8_t)bus;
    return lim_status(j, j_wr(j, 0x2901, 1, &b, 1));
}

am_status am_sdo_read(am_joint *j, uint16_t index, uint8_t sub, void *buf, size_t cap, size_t *len)
{
    if (!j || !buf) return AM_ERR_PARAM;
    return j_rd(j, index, sub, buf, cap, len);
}

am_status am_sdo_write(am_joint *j, uint16_t index, uint8_t sub, const void *buf, size_t len)
{
    if (!j || (!buf && len)) return AM_ERR_PARAM;
    return j_wr(j, index, sub, buf, len);
}

am_status am_read_calibration(am_joint *j, char *buf, size_t cap, size_t *len)
{
    am_session *s;
    size_t n = 0;
    if (!j || !buf || cap == 0) return AM_ERR_PARAM;
    s = j->s;
    if (j->sim) {
        n = am__sim_calibration(j->sim, buf, cap);
        if (len) *len = n;
        return n < cap ? AM_OK : AM_ERR_NO_MEMORY;
    }
    if (!s->cfg.transport->file_read) return AM_ERR_NOT_SUPPORTED;
    TRY(s->cfg.transport->file_read(s->cfg.transport_user, j->address, "am_cal.json", (uint8_t *)buf, cap - 1, &n));
    buf[n] = '\0';
    if (len) *len = n;
    return AM_OK;
}

am_status am_trace_arm(am_joint *j, uint16_t trigger_signal, float trigger_level, uint16_t length, uint16_t pretrigger)
{
    (void)j; (void)trigger_signal; (void)trigger_level; (void)length; (void)pretrigger;
    return AM_ERR_NOT_SUPPORTED;
}

am_status am_trace_read(am_joint *j, float *buf, size_t cap_samples, size_t *n_samples, am_progress_cb cb, void *user)
{
    (void)j; (void)buf; (void)cap_samples; (void)cb; (void)user;
    if (n_samples) *n_samples = 0;
    return AM_ERR_NOT_SUPPORTED;
}

/* ---------------------------------------------------------------- simulator access */
am_sim_joint *am_sim_of(am_joint *j) { return j ? j->sim : NULL; }

/* ---------------------------------------------------------------- wire-level simulator transport */
typedef struct sim_tp {
    am_sim_joint *nodes;
    size_t n;
    am_codec *codecs;
    uint16_t order[AM_MAX_JOINTS];
    size_t n_order, out_per, in_per;
    am_mode keep_mode[AM_MAX_JOINTS];
} sim_tp;

static am_status stp_pdo(void *user, const uint8_t *out, size_t out_len, uint8_t *in, size_t in_len, uint64_t *t_ns)
{
    sim_tp *tp = (sim_tp *)user;
    size_t i;
    if (out_len < tp->n_order * tp->out_per || in_len < tp->n_order * tp->in_per) return AM_ERR_PARAM;
    for (i = 0; i < tp->n_order; i++) {
        uint16_t a = tp->order[i];
        am_sim_joint *sj = &tp->nodes[a];
        const uint8_t *o = out + i * tp->out_per;
        uint8_t *p = in + i * tp->in_per;
        am_telemetry t;
        if (tp->out_per == AM_PDO_RX_1601_LEN) {
            uint16_t cw;
            am_command cmd;
            am__drive_unpack_rx1601(o, &cw, &cmd);
            am__sim_exchange(sj, cw, cmd.mode, &cmd, true, &t);
            am__drive_pack_tx1a00(&tp->codecs[a], &t, p);
            if (tp->in_per >= AM_PDO_TX_1A00_LEN + AM_PDO_TX_1A01_LEN) am__drive_pack_tx1a01(&t, p + AM_PDO_TX_1A00_LEN);
        } else {
            uint8_t control, counter;
            am_command cmd;
            uint16_t cw, sw = am__sim_statusword(sj);
            am_drive_state st = sj->state;
            am__drive_unpack_cfd_slot(&tp->codecs[a], o, &control, &counter, &cmd);
            if (cmd.mode == AM_MODE_NONE) cmd.mode = tp->keep_mode[i];
            tp->keep_mode[i] = cmd.mode;
            if (control & AM_CC_FAULT_RESET) cw = AM_CW_FAULT_RESET;
            else if (control & AM_CC_QUICK_STOP) cw = AM_CW_QUICK_STOP;
            else if (control & AM_CC_ENABLE) cw = am_codec_next_controlword(sw, true);
            else if (st == AM_DS_READY_TO_SWITCH_ON || st == AM_DS_SWITCHED_ON || st == AM_DS_OPERATION_ENABLED)
                cw = AM_CW_SHUTDOWN;
            else cw = AM_CW_DISABLE_VOLTAGE;
            if (control & AM_CC_ZERO_TORQUE) {
                am_mode m = cmd.mode;
                memset(&cmd, 0, sizeof cmd);
                cmd.mode = m;
            }
            am__sim_exchange(sj, cw, cmd.mode, &cmd, true, &t);
            am__drive_pack_cfd_fb(&t, counter, sj->authority == sj->cfg_bus, tp->in_per == AM_CFD_FB_EXT_LEN, p);
        }
    }
    if (t_ns) *t_ns = tp->n_order ? tp->nodes[tp->order[0]].t_ns : 0;
    return AM_OK;
}

static am_status stp_sdo_read(void *user, uint16_t address, uint16_t index, uint8_t sub, void *buf, size_t cap,
                              size_t *len, uint32_t *abort_code)
{
    sim_tp *tp = (sim_tp *)user;
    am_status st;
    if (address >= tp->n) return AM_ERR_NO_DEVICE;
    st = am__sim_sdo_read(&tp->nodes[address], index, sub, buf, cap, len);
    if (st == AM_ERR_SDO_ABORT && abort_code) *abort_code = tp->nodes[address].last_abort;
    return st;
}

static am_status stp_sdo_write(void *user, uint16_t address, uint16_t index, uint8_t sub, const void *buf, size_t len,
                               uint32_t *abort_code)
{
    sim_tp *tp = (sim_tp *)user;
    am_status st;
    if (address >= tp->n) return AM_ERR_NO_DEVICE;
    st = am__sim_sdo_write(&tp->nodes[address], index, sub, buf, len);
    if (st != AM_OK) {
        if (abort_code) *abort_code = tp->nodes[address].last_abort;
        return AM_ERR_SDO_ABORT;
    }
    return AM_OK;
}

static am_status stp_file_read(void *user, uint16_t address, const char *name, uint8_t *buf, size_t cap, size_t *len)
{
    sim_tp *tp = (sim_tp *)user;
    size_t n;
    char tmp[1024];
    if (address >= tp->n) return AM_ERR_NO_DEVICE;
    if (strcmp(name, "am_cal.json") != 0) return AM_ERR_NOT_SUPPORTED;
    n = am__sim_calibration(&tp->nodes[address], tmp, sizeof tmp);
    if (n >= sizeof tmp || n > cap) return AM_ERR_NO_MEMORY;
    memcpy(buf, tmp, n);
    if (len) *len = n;
    return AM_OK;
}

static am_status stp_start(void *user, const uint16_t *addresses, size_t n, size_t out_per, size_t in_per)
{
    sim_tp *tp = (sim_tp *)user;
    size_t i;
    if (n > AM_MAX_JOINTS) return AM_ERR_PARAM;
    for (i = 0; i < n; i++) {
        if (addresses[i] >= tp->n) return AM_ERR_NO_DEVICE;
        tp->order[i] = addresses[i];
        tp->keep_mode[i] = AM_MODE_NONE;
        tp->nodes[addresses[i]].extended =
            in_per == AM_PDO_TX_1A00_LEN + AM_PDO_TX_1A01_LEN || in_per == AM_CFD_FB_EXT_LEN;
    }
    tp->n_order = n;
    tp->out_per = out_per;
    tp->in_per = in_per;
    return AM_OK;
}

static am_status stp_stop(void *user)
{
    sim_tp *tp = (sim_tp *)user;
    size_t i;
    for (i = 0; i < tp->n_order; i++) am__sim_comms_lost(&tp->nodes[tp->order[i]]);
    return AM_OK;
}

static const am_transport_ops sim_tp_ops = {stp_pdo, stp_sdo_read, stp_sdo_write, NULL, stp_file_read, stp_start,
                                            stp_stop};

am_status am_sim_transport_open(const char *units, am_bus bus, uint32_t cycle_us, const am_transport_ops **ops,
                                void **user)
{
    char spec[256];
    const am_unit_data *u[AM_MAX_SIM_UNITS];
    size_t n, i;
    sim_tp *tp;
    if (!units || !ops || !user) return AM_ERR_PARAM;
    if (bus == AM_BUS_NONE) bus = AM_BUS_ETHERCAT;
    if (cycle_us == 0) cycle_us = 1000;
    snprintf(spec, sizeof spec, "sim:%s", units);
    n = am__sim_parse_units(spec, u, AM_MAX_SIM_UNITS);
    if (n == 0) return AM_ERR_PARAM;
    tp = (sim_tp *)calloc(1, sizeof *tp);
    if (!tp) return AM_ERR_NO_MEMORY;
    tp->nodes = (am_sim_joint *)calloc(n, sizeof *tp->nodes);
    tp->codecs = (am_codec *)calloc(n, sizeof *tp->codecs);
    if (!tp->nodes || !tp->codecs) {
        am_sim_transport_close(tp);
        return AM_ERR_NO_MEMORY;
    }
    tp->n = n;
    for (i = 0; i < n; i++) {
        am__sim_init(&tp->nodes[i], u[i], (uint16_t)i, bus, cycle_us, true, NULL);
        (void)am_codec_init(&tp->codecs[i], (uint32_t)am__round_even(u[i]->rated_nm * 1e3),
                            (uint32_t)am__round_even(am__sim_rated_current(&tp->nodes[i]) * 1e3), 20, 1,
                            (uint8_t)(i % 4));
    }
    *ops = &sim_tp_ops;
    *user = tp;
    return AM_OK;
}

am_sim_joint *am_sim_transport_joint(void *user, uint16_t address)
{
    sim_tp *tp = (sim_tp *)user;
    if (!tp || address >= tp->n) return NULL;
    return &tp->nodes[address];
}

void am_sim_transport_close(void *user)
{
    sim_tp *tp = (sim_tp *)user;
    if (!tp) return;
    free(tp->nodes);
    free(tp->codecs);
    free(tp);
}
