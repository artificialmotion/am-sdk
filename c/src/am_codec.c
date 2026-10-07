/* SPDX-License-Identifier: Apache-2.0 */
/* L1 codec: pure functions, no I/O, no allocation. Byte layouts: docs/API.md 2.4 and 3.4. */
#include "am_internal.h"

/* ---------------------------------------------------------------- helpers */
double am__round_away(double x)
{
    double r = floor(fabs(x) + 0.5);
    return x < 0.0 ? -r : r;
}

double am__round_even(double x)
{
    double f = floor(x);
    double d = x - f;
    if (d > 0.5) return f + 1.0;
    if (d < 0.5) return f;
    return fmod(f, 2.0) == 0.0 ? f : f + 1.0;
}

void am__put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void am__put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
void am__put_u64(uint8_t *p, uint64_t v)
{
    am__put_u32(p, (uint32_t)v);
    am__put_u32(p + 4, (uint32_t)(v >> 32));
}
void am__put_f32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    am__put_u32(p, u);
}
uint16_t am__get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
uint32_t am__get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
uint64_t am__get_u64(const uint8_t *p) { return (uint64_t)am__get_u32(p) | ((uint64_t)am__get_u32(p + 4) << 32); }
float am__get_f32(const uint8_t *p)
{
    uint32_t u = am__get_u32(p);
    float v;
    memcpy(&v, &u, 4);
    return v;
}

/* Scale, round half away from zero, check the range. */
static bool q_int(double v, double scale, double lo, double hi, int64_t *out)
{
    double x;
    if (!(v == v) || v > 1e300 || v < -1e300) return false;
    x = am__round_away(v * scale);
    if (x < lo || x > hi) return false;
    *out = (int64_t)x;
    return true;
}

/* Drive-side encoding of measured values: round half away from zero, saturate. */
static int64_t sat(double x, double lo, double hi)
{
    double r;
    if (!(x == x)) return 0;
    r = am__round_away(x);
    if (r < lo) r = lo;
    if (r > hi) r = hi;
    return (int64_t)r;
}

#define I16_LO -32768.0
#define I16_HI 32767.0
#define U16_HI 65535.0
#define I32_LO -2147483648.0
#define I32_HI 2147483647.0
#define U32_HI 4294967295.0

static bool mode_known(am_mode m)
{
    return m == AM_MODE_NONE || m == AM_MODE_CSP || m == AM_MODE_CSV || m == AM_MODE_CST ||
           m == AM_MODE_IMPEDANCE || m == AM_MODE_CURRENT;
}

static am_mode mode_from_i8(int8_t v)
{
    am_mode m = (am_mode)v;
    return mode_known(m) ? m : AM_MODE_NONE;
}

uint8_t am_codec_compact_mode(am_mode mode)
{
    switch (mode) {
    case AM_MODE_CST: return 1;
    case AM_MODE_CSV: return 2;
    case AM_MODE_CSP: return 3;
    case AM_MODE_IMPEDANCE: return 4;
    case AM_MODE_CURRENT: return 5;
    case AM_MODE_NONE:
    default: return 0;
    }
}

am_mode am__mode_from_compact(uint8_t code)
{
    switch (code) {
    case 1: return AM_MODE_CST;
    case 2: return AM_MODE_CSV;
    case 3: return AM_MODE_CSP;
    case 4: return AM_MODE_IMPEDANCE;
    case 5: return AM_MODE_CURRENT;
    default: return AM_MODE_NONE;
    }
}

uint16_t am__state_to_sw(am_drive_state st)
{
    switch (st) {
    case AM_DS_SWITCH_ON_DISABLED: return 0x0040;
    case AM_DS_READY_TO_SWITCH_ON: return 0x0021;
    case AM_DS_SWITCHED_ON: return 0x0023;
    case AM_DS_OPERATION_ENABLED: return 0x0027;
    case AM_DS_QUICK_STOP_ACTIVE: return 0x0007;
    case AM_DS_FAULT_REACTION_ACTIVE: return 0x000F;
    case AM_DS_FAULT: return 0x0008;
    case AM_DS_NOT_READY_TO_SWITCH_ON:
    default: return 0x0000;
    }
}

/* ---------------------------------------------------------------- init and state machine */
am_status am_codec_init(am_codec *c, uint32_t rated_torque_mNm, uint32_t rated_current_mA,
                        uint16_t compact_k_mNm_per_rad_lsb, uint16_t compact_d_mNm_s_per_rad_lsb,
                        uint8_t compact_slot)
{
    if (!c || rated_torque_mNm == 0 || rated_current_mA == 0 || compact_k_mNm_per_rad_lsb == 0 ||
        compact_d_mNm_s_per_rad_lsb == 0 || compact_slot > 3)
        return AM_ERR_PARAM;
    c->rated_torque_Nm = (float)(rated_torque_mNm * 1e-3);
    c->rated_current_A = (float)(rated_current_mA * 1e-3);
    c->compact_k_scale = (float)(compact_k_mNm_per_rad_lsb * 1e-3);
    c->compact_d_scale = (float)(compact_d_mNm_s_per_rad_lsb * 1e-3);
    c->compact_slot = compact_slot;
    return AM_OK;
}

am_drive_state am_codec_decode_state(uint16_t sw)
{
    if ((sw & 0x4F) == 0x00) return AM_DS_NOT_READY_TO_SWITCH_ON;
    if ((sw & 0x4F) == 0x40) return AM_DS_SWITCH_ON_DISABLED;
    if ((sw & 0x6F) == 0x21) return AM_DS_READY_TO_SWITCH_ON;
    if ((sw & 0x6F) == 0x23) return AM_DS_SWITCHED_ON;
    if ((sw & 0x6F) == 0x27) return AM_DS_OPERATION_ENABLED;
    if ((sw & 0x6F) == 0x07) return AM_DS_QUICK_STOP_ACTIVE;
    if ((sw & 0x4F) == 0x0F) return AM_DS_FAULT_REACTION_ACTIVE;
    if ((sw & 0x4F) == 0x08) return AM_DS_FAULT;
    return AM_DS_NOT_READY_TO_SWITCH_ON;
}

am_controlword am_codec_next_controlword(uint16_t sw, bool enable)
{
    am_drive_state st = am_codec_decode_state(sw);
    if (!enable) return AM_CW_DISABLE_VOLTAGE;
    if (st == AM_DS_SWITCH_ON_DISABLED) return AM_CW_SHUTDOWN;
    if (st == AM_DS_READY_TO_SWITCH_ON) return AM_CW_SWITCH_ON;
    if (st == AM_DS_SWITCHED_ON || st == AM_DS_OPERATION_ENABLED) return AM_CW_ENABLE_OPERATION;
    return AM_CW_DISABLE_VOLTAGE;
}

/* ---------------------------------------------------------------- host side: EtherCAT PDOs */
am_status am_codec_pack_rx1601(const am_codec *c, am_controlword cw, const am_command *cmd,
                               uint8_t out[AM_PDO_RX_1601_LEN])
{
    int64_t q, qd, tau, k, d, iq;
    if (!c || !cmd || !out || !mode_known(cmd->mode)) return AM_ERR_PARAM;
    if (!(cmd->kp_Nm_per_rad >= 0.0f) || !(cmd->kd_Nm_s_per_rad >= 0.0f)) return AM_ERR_PARAM;
    if (!q_int(cmd->q_ref_rad, 1e6, I32_LO, I32_HI, &q) || !q_int(cmd->qd_ref_rad_s, 1e3, I32_LO, I32_HI, &qd) ||
        !q_int(cmd->tau_ff_Nm, 1e3, I32_LO, I32_HI, &tau) || !q_int(cmd->kp_Nm_per_rad, 1e3, 0.0, U32_HI, &k) ||
        !q_int(cmd->kd_Nm_s_per_rad, 1e3, 0.0, U32_HI, &d) || !q_int(cmd->iq_ref_A, 1e3, I32_LO, I32_HI, &iq))
        return AM_ERR_PARAM;
    am__put_u16(out, cw);
    out[2] = (uint8_t)(int8_t)cmd->mode;
    out[3] = 0;
    am__put_u32(out + 4, (uint32_t)(int32_t)q);
    am__put_u32(out + 8, (uint32_t)(int32_t)qd);
    am__put_u32(out + 12, (uint32_t)(int32_t)tau);
    am__put_u32(out + 16, (uint32_t)k);
    am__put_u32(out + 20, (uint32_t)d);
    am__put_u32(out + 24, (uint32_t)(int32_t)iq);
    return AM_OK;
}

am_status am_codec_pack_rx1600(const am_codec *c, am_controlword cw, const am_command *cmd,
                               uint8_t out[AM_PDO_RX_1600_LEN])
{
    int64_t q = 0, vt = 0, vo = 0, tt = 0, to = 0;
    double pm;
    am_mode m;
    if (!c || !cmd || !out || !mode_known(cmd->mode) || !(c->rated_torque_Nm > 0.0f)) return AM_ERR_PARAM;
    m = cmd->mode;
    if (m == AM_MODE_IMPEDANCE || m == AM_MODE_CURRENT) return AM_ERR_NOT_SUPPORTED;
    pm = 1000.0 / c->rated_torque_Nm;
    if (m == AM_MODE_CSP) {
        if (!q_int(cmd->q_ref_rad, 1e6, I32_LO, I32_HI, &q) || !q_int(cmd->qd_ref_rad_s, 1e3, I32_LO, I32_HI, &vo))
            return AM_ERR_PARAM;
    }
    if (m == AM_MODE_CSV && !q_int(cmd->qd_ref_rad_s, 1e3, I32_LO, I32_HI, &vt)) return AM_ERR_PARAM;
    if (m == AM_MODE_CST && !q_int(cmd->tau_ff_Nm, pm, I16_LO, I16_HI, &tt)) return AM_ERR_PARAM;
    if ((m == AM_MODE_CSP || m == AM_MODE_CSV) && !q_int(cmd->tau_ff_Nm, pm, I16_LO, I16_HI, &to))
        return AM_ERR_PARAM;
    am__put_u16(out, cw);
    out[2] = (uint8_t)(int8_t)m;
    out[3] = 0;
    am__put_u32(out + 4, (uint32_t)(int32_t)q);
    am__put_u32(out + 8, (uint32_t)(int32_t)vt);
    am__put_u16(out + 12, (uint16_t)(int16_t)tt);
    am__put_u16(out + 14, (uint16_t)(int16_t)to);
    am__put_u32(out + 16, (uint32_t)(int32_t)vo);
    return AM_OK;
}

am_status am_codec_unpack_tx1a00(const am_codec *c, const uint8_t in[AM_PDO_TX_1A00_LEN], am_telemetry *t)
{
    uint16_t sw;
    uint8_t ref;
    if (!c || !in || !t) return AM_ERR_PARAM;
    sw = am__get_u16(in);
    t->statusword = sw;
    t->state = am_codec_decode_state(sw);
    t->sto_active = (sw & AM_SW_STO) != 0;
    t->internal_limit = (sw & AM_SW_INTERNAL_LIMIT) != 0;
    t->mode = mode_from_i8((int8_t)in[2]);
    t->q_rad = (int32_t)am__get_u32(in + 4) * 1e-6;
    t->qd_rad_s = (float)((int32_t)am__get_u32(in + 8) * 1e-3);
    t->tau_est_Nm = (float)((int16_t)am__get_u16(in + 12) * (double)c->rated_torque_Nm * 1e-3);
    t->error_code = am__get_u16(in + 14);
    t->fault_word = am__get_u32(in + 16);
    t->safety_io = in[20];
    ref = in[21];
    t->ref_state = ref <= 2 ? (am_ref_state)ref : AM_REF_INVALID;
    t->t_winding_C = (float)((int16_t)am__get_u16(in + 22) * 0.1);
    return AM_OK;
}

am_status am_codec_unpack_tx1a01(const am_codec *c, const uint8_t in[AM_PDO_TX_1A01_LEN], am_telemetry *t)
{
    if (!c || !in || !t) return AM_ERR_PARAM;
    t->t_ns = am__get_u64(in);
    t->sample_counter = am__get_u16(in + 8);
    t->sync_flags = am__get_u16(in + 10);
    t->q_motor_counts = (int32_t)am__get_u32(in + 12);
    t->qd_motor_rad_s = (float)((int32_t)am__get_u32(in + 16) * 1e-3);
    t->q_out_encoder_rad = (float)(am__get_u16(in + 20) * AM_TWO_PI / 65536.0);
    /* in + 22: output encoder status (low byte) and reference state (high byte) */
    t->twist_rad = (float)((int32_t)am__get_u32(in + 24) * 1e-6);
    t->id_A = (float)((int16_t)am__get_u16(in + 28) * 0.01);
    t->iq_A = (float)((int16_t)am__get_u16(in + 30) * 0.01);
    t->id_ref_A = (float)((int16_t)am__get_u16(in + 32) * 0.01);
    t->iq_ref_A = (float)((int16_t)am__get_u16(in + 34) * 0.01);
    t->vd_V = (float)((int16_t)am__get_u16(in + 36) * 0.01);
    t->vq_V = (float)((int16_t)am__get_u16(in + 38) * 0.01);
    t->vbus_V = (float)(am__get_u16(in + 40) * 1e-3);
    t->ibus_est_A = (float)((int16_t)am__get_u16(in + 42) * 0.01);
    t->t_winding_C = (float)((int16_t)am__get_u16(in + 44) * 0.1);
    t->t_magnet_C = (float)((int16_t)am__get_u16(in + 46) * 0.1);
    t->t_power_stage_C = (float)((int16_t)am__get_u16(in + 48) * 0.1);
    t->t_board_C = (float)((int16_t)am__get_u16(in + 50) * 0.1);
    t->tau_est_Nm = (float)((int32_t)am__get_u32(in + 52) * 1e-3);
    t->warning_word = am__get_u16(in + 56);
    t->extended_valid = true;
    return AM_OK;
}

/* ---------------------------------------------------------------- host side: CAN-FD compact */
am_status am_codec_pack_cfd_slot(const am_codec *c, uint8_t control, uint8_t counter, const am_command *cmd,
                                 uint8_t out[AM_CFD_CMD_SLOT_LEN])
{
    int64_t q, qd, third, k = 0, d = 0;
    if (!c || !cmd || !out || !mode_known(cmd->mode)) return AM_ERR_PARAM;
    if (!(cmd->kp_Nm_per_rad >= 0.0f) || !(cmd->kd_Nm_s_per_rad >= 0.0f)) return AM_ERR_PARAM;
    if (!(c->compact_k_scale > 0.0f) || !(c->compact_d_scale > 0.0f)) return AM_ERR_PARAM;
    if (!q_int(cmd->q_ref_rad, 1e6, I32_LO, I32_HI, &q) || !q_int(cmd->qd_ref_rad_s, 100.0, I16_LO, I16_HI, &qd))
        return AM_ERR_PARAM;
    if (cmd->mode == AM_MODE_CURRENT) {
        if (!q_int(cmd->iq_ref_A, 100.0, I16_LO, I16_HI, &third)) return AM_ERR_PARAM;
    } else if (!q_int(cmd->tau_ff_Nm, 100.0, I16_LO, I16_HI, &third)) {
        return AM_ERR_PARAM;
    }
    if (cmd->mode == AM_MODE_IMPEDANCE) {
        if (!q_int(cmd->kp_Nm_per_rad, 1.0 / c->compact_k_scale, 0.0, U16_HI, &k) ||
            !q_int(cmd->kd_Nm_s_per_rad, 1.0 / c->compact_d_scale, 0.0, U16_HI, &d))
            return AM_ERR_PARAM;
    }
    am__put_u32(out, (uint32_t)(int32_t)q);
    am__put_u16(out + 4, (uint16_t)(int16_t)qd);
    am__put_u16(out + 6, (uint16_t)(int16_t)third);
    am__put_u16(out + 8, (uint16_t)k);
    am__put_u16(out + 10, (uint16_t)d);
    out[12] = control;
    out[13] = counter;
    out[14] = 0;
    out[15] = 0;
    return AM_OK;
}

#define CFS_WARNING (1u << 3)
#define CFS_STO (1u << 4)
#define CFS_REF_VALID (1u << 5)
#define CFS_INTERNAL_LIMIT (1u << 6)
#define CFS_AUTHORITY (1u << 7)
#define CFS_MODE_SHIFT 8
#define CFS_TIME_VALID (1u << 12)
#define CFS_FAULT (1u << 13)

am_status am_codec_unpack_cfd_fb(const am_codec *c, const uint8_t *in, size_t len, am_telemetry *t,
                                 uint8_t *echoed_counter)
{
    uint16_t st, sw;
    am_drive_state ds;
    if (!c || !in || !t || (len != AM_CFD_FB_LEN && len != AM_CFD_FB_EXT_LEN)) return AM_ERR_PARAM;
    st = am__get_u16(in + 10);
    ds = (am_drive_state)(st & 0x7u);
    sw = am__state_to_sw(ds);
    if (ds == AM_DS_READY_TO_SWITCH_ON || ds == AM_DS_SWITCHED_ON || ds == AM_DS_OPERATION_ENABLED ||
        ds == AM_DS_QUICK_STOP_ACTIVE)
        sw |= 0x0010u;
    if (st & CFS_WARNING) sw |= 0x0080u;
    if (st & CFS_AUTHORITY) sw |= AM_SW_REMOTE;
    if (st & CFS_INTERNAL_LIMIT) sw |= AM_SW_INTERNAL_LIMIT;
    if (st & CFS_REF_VALID) sw |= AM_SW_REF_VALID;
    if (st & CFS_STO) sw |= AM_SW_STO;
    t->statusword = sw;
    t->state = ds;
    t->sto_active = (st & CFS_STO) != 0;
    t->internal_limit = (st & CFS_INTERNAL_LIMIT) != 0;
    t->ref_state = (st & CFS_REF_VALID) ? AM_REF_VALID : AM_REF_INVALID;
    t->mode = am__mode_from_compact((uint8_t)((st >> CFS_MODE_SHIFT) & 0xFu));
    t->q_rad = (int32_t)am__get_u32(in) * 1e-6;
    t->qd_rad_s = (float)((int16_t)am__get_u16(in + 4) * 0.01);
    t->tau_est_Nm = (float)((int16_t)am__get_u16(in + 6) * 0.01);
    t->iq_A = (float)((int16_t)am__get_u16(in + 8) * 0.01);
    if (echoed_counter) *echoed_counter = in[12];
    t->t_winding_C = (float)((double)in[13] - 40.0);
    t->vbus_V = (float)(am__get_u16(in + 14) * 0.01);
    if (len == AM_CFD_FB_EXT_LEN) {
        t->t_ns = am__get_u32(in + 16);
        t->q_motor_counts = (int32_t)am__get_u32(in + 20);
        t->q_out_encoder_rad = (float)(am__get_u16(in + 24) * AM_TWO_PI / 65536.0);
        t->id_A = (float)((int16_t)am__get_u16(in + 26) * 0.01);
        t->t_power_stage_C = (float)((int16_t)am__get_u16(in + 28) * 0.1);
        t->warning_word = am__get_u16(in + 30);
        t->extended_valid = true;
    } else {
        t->extended_valid = false;
    }
    return AM_OK;
}

/* ---------------------------------------------------------------- drive side (simulator) */
void am__drive_unpack_rx1601(const uint8_t in[AM_PDO_RX_1601_LEN], uint16_t *cw, am_command *cmd)
{
    *cw = am__get_u16(in);
    cmd->mode = mode_from_i8((int8_t)in[2]);
    cmd->q_ref_rad = (int32_t)am__get_u32(in + 4) * 1e-6;
    cmd->qd_ref_rad_s = (float)((int32_t)am__get_u32(in + 8) * 1e-3);
    cmd->tau_ff_Nm = (float)((int32_t)am__get_u32(in + 12) * 1e-3);
    cmd->kp_Nm_per_rad = (float)(am__get_u32(in + 16) * 1e-3);
    cmd->kd_Nm_s_per_rad = (float)(am__get_u32(in + 20) * 1e-3);
    cmd->iq_ref_A = (float)((int32_t)am__get_u32(in + 24) * 1e-3);
}

void am__drive_pack_tx1a00(const am_codec *c, const am_telemetry *t, uint8_t out[AM_PDO_TX_1A00_LEN])
{
    am__put_u16(out, t->statusword);
    out[2] = (uint8_t)(int8_t)t->mode;
    out[3] = 0;
    am__put_u32(out + 4, (uint32_t)(int32_t)sat(t->q_rad * 1e6, I32_LO, I32_HI));
    am__put_u32(out + 8, (uint32_t)(int32_t)sat(t->qd_rad_s * 1e3, I32_LO, I32_HI));
    am__put_u16(out + 12, (uint16_t)(int16_t)sat(t->tau_est_Nm * 1e3 / c->rated_torque_Nm, I16_LO, I16_HI));
    am__put_u16(out + 14, t->error_code);
    am__put_u32(out + 16, t->fault_word);
    out[20] = t->safety_io;
    out[21] = (uint8_t)t->ref_state;
    am__put_u16(out + 22, (uint16_t)(int16_t)sat(t->t_winding_C * 10.0, I16_LO, I16_HI));
}

static uint16_t angle_u16(double rad)
{
    int64_t v = sat(rad / AM_TWO_PI * 65536.0, -1099511627776.0, 1099511627776.0);
    v %= 65536;
    if (v < 0) v += 65536;
    return (uint16_t)v;
}

void am__drive_pack_tx1a01(const am_telemetry *t, uint8_t out[AM_PDO_TX_1A01_LEN])
{
    am__put_u64(out, t->t_ns);
    am__put_u16(out + 8, t->sample_counter);
    am__put_u16(out + 10, t->sync_flags);
    am__put_u32(out + 12, (uint32_t)(int32_t)sat((double)t->q_motor_counts, I32_LO, I32_HI));
    am__put_u32(out + 16, (uint32_t)(int32_t)sat(t->qd_motor_rad_s * 1e3, I32_LO, I32_HI));
    am__put_u16(out + 20, angle_u16(t->q_out_encoder_rad));
    am__put_u16(out + 22, (uint16_t)((unsigned)t->ref_state << 8));
    am__put_u32(out + 24, (uint32_t)(int32_t)sat(t->twist_rad * 1e6, I32_LO, I32_HI));
    am__put_u16(out + 28, (uint16_t)(int16_t)sat(t->id_A * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 30, (uint16_t)(int16_t)sat(t->iq_A * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 32, (uint16_t)(int16_t)sat(t->id_ref_A * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 34, (uint16_t)(int16_t)sat(t->iq_ref_A * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 36, (uint16_t)(int16_t)sat(t->vd_V * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 38, (uint16_t)(int16_t)sat(t->vq_V * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 40, (uint16_t)sat(t->vbus_V * 1e3, 0.0, U16_HI));
    am__put_u16(out + 42, (uint16_t)(int16_t)sat(t->ibus_est_A * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 44, (uint16_t)(int16_t)sat(t->t_winding_C * 10.0, I16_LO, I16_HI));
    am__put_u16(out + 46, (uint16_t)(int16_t)sat(t->t_magnet_C * 10.0, I16_LO, I16_HI));
    am__put_u16(out + 48, (uint16_t)(int16_t)sat(t->t_power_stage_C * 10.0, I16_LO, I16_HI));
    am__put_u16(out + 50, (uint16_t)(int16_t)sat(t->t_board_C * 10.0, I16_LO, I16_HI));
    am__put_u32(out + 52, (uint32_t)(int32_t)sat(t->tau_est_Nm * 1e3, I32_LO, I32_HI));
    am__put_u16(out + 56, t->warning_word);
    out[58] = 0;
    out[59] = 0;
}

void am__drive_unpack_cfd_slot(const am_codec *c, const uint8_t in[AM_CFD_CMD_SLOT_LEN], uint8_t *control,
                               uint8_t *counter, am_command *cmd)
{
    int16_t third = (int16_t)am__get_u16(in + 6);
    *control = in[12];
    *counter = in[13];
    cmd->mode = am__mode_from_compact((uint8_t)((in[12] >> AM_CC_MODE_SHIFT) & 0xFu));
    cmd->q_ref_rad = (int32_t)am__get_u32(in) * 1e-6;
    cmd->qd_ref_rad_s = (float)((int16_t)am__get_u16(in + 4) * 0.01);
    cmd->tau_ff_Nm = cmd->mode == AM_MODE_CURRENT ? 0.0f : (float)(third * 0.01);
    cmd->iq_ref_A = cmd->mode == AM_MODE_CURRENT ? (float)(third * 0.01) : 0.0f;
    cmd->kp_Nm_per_rad = (float)(am__get_u16(in + 8) * (double)c->compact_k_scale);
    cmd->kd_Nm_s_per_rad = (float)(am__get_u16(in + 10) * (double)c->compact_d_scale);
}

void am__drive_pack_cfd_fb(const am_telemetry *t, uint8_t counter, bool authority, bool extended, uint8_t *out)
{
    uint16_t st = (uint16_t)((unsigned)t->state & 0x7u);
    if (t->statusword & 0x0080u) st |= CFS_WARNING;
    if (t->sto_active) st |= CFS_STO;
    if (t->ref_state == AM_REF_VALID) st |= CFS_REF_VALID;
    if (t->internal_limit) st |= CFS_INTERNAL_LIMIT;
    if (authority) st |= CFS_AUTHORITY;
    st = (uint16_t)(st | ((unsigned)am_codec_compact_mode(t->mode) << CFS_MODE_SHIFT));
    if (t->sync_flags & AM_SYNC_TIMESTAMP_VALID) st |= CFS_TIME_VALID;
    if (t->state == AM_DS_FAULT || t->state == AM_DS_FAULT_REACTION_ACTIVE) st |= CFS_FAULT;
    am__put_u32(out, (uint32_t)(int32_t)sat(t->q_rad * 1e6, I32_LO, I32_HI));
    am__put_u16(out + 4, (uint16_t)(int16_t)sat(t->qd_rad_s * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 6, (uint16_t)(int16_t)sat(t->tau_est_Nm * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 8, (uint16_t)(int16_t)sat(t->iq_A * 100.0, I16_LO, I16_HI));
    am__put_u16(out + 10, st);
    out[12] = counter;
    out[13] = (uint8_t)sat(t->t_winding_C + 40.0, 0.0, 255.0);
    am__put_u16(out + 14, (uint16_t)sat(t->vbus_V * 100.0, 0.0, U16_HI));
    if (extended) {
        am__put_u32(out + 16, (uint32_t)(t->t_ns & 0xFFFFFFFFu));
        am__put_u32(out + 20, (uint32_t)(int32_t)sat((double)t->q_motor_counts, I32_LO, I32_HI));
        am__put_u16(out + 24, angle_u16(t->q_out_encoder_rad));
        am__put_u16(out + 26, (uint16_t)(int16_t)sat(t->id_A * 100.0, I16_LO, I16_HI));
        am__put_u16(out + 28, (uint16_t)(int16_t)sat(t->t_power_stage_C * 10.0, I16_LO, I16_HI));
        am__put_u16(out + 30, t->warning_word);
    }
}
