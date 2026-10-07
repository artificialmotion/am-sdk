/* SPDX-License-Identifier: Apache-2.0 */
/* L1 codec: byte layouts, ranges, round trips, state walk. */
#include "../src/am_internal.h"
#include "am_test.h"

#include <string.h>

static am_codec C;

static void setup(void) { am_codec_init(&C, 36000, 13530, 20, 1, 2); }

static void test_init_rejects_bad_scaling(void)
{
    am_codec c;
    CHECK_ST(am_codec_init(&c, 0, 1, 20, 1, 0), AM_ERR_PARAM);
    CHECK_ST(am_codec_init(&c, 1, 1, 20, 1, 4), AM_ERR_PARAM);
    CHECK_ST(am_codec_init(NULL, 1, 1, 20, 1, 0), AM_ERR_PARAM);
    CHECK_ST(am_codec_init(&c, 36000, 13530, 20, 1, 3), AM_OK);
    CHECK_NEAR(c.rated_torque_Nm, 36.0, 1e-6);
    CHECK_NEAR(c.compact_k_scale, 0.02, 1e-9);
}

static void test_rx1601_golden(void)
{
    /* same layout as python/tests/test_wire.py Codec.test_rx1601_layout (tau 3.5 is exact in float) */
    static const uint8_t want[28] = {0x0F, 0x00, 0xFF, 0x00, 0x20, 0xA1, 0x07, 0x00, 0x1E, 0xFB, 0xFF, 0xFF, 0xAC, 0x0D,
                                     0x00, 0x00, 0xF0, 0x49, 0x02, 0x00, 0x70, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    am_command cmd;
    uint8_t b[28];
    uint16_t cw;
    am_command back;
    memset(&cmd, 0, sizeof cmd);
    cmd.mode = AM_MODE_IMPEDANCE;
    cmd.q_ref_rad = 0.5;
    cmd.qd_ref_rad_s = -1.25f;
    cmd.tau_ff_Nm = 3.5f;
    cmd.kp_Nm_per_rad = 150.0f;
    cmd.kd_Nm_s_per_rad = 6.0f;
    CHECK_ST(am_codec_pack_rx1601(&C, 0x000F, &cmd, b), AM_OK);
    CHECK(memcmp(b, want, 28) == 0);
    am__drive_unpack_rx1601(b, &cw, &back);
    CHECK(cw == 0x000F);
    CHECK(back.mode == AM_MODE_IMPEDANCE);
    CHECK_NEAR(back.q_ref_rad, 0.5, 1e-9);
    CHECK_NEAR(back.kp_Nm_per_rad, 150.0, 1e-6);
}

static void test_rx1601_ranges(void)
{
    am_command cmd;
    uint8_t b[28];
    memset(&cmd, 0, sizeof cmd);
    cmd.mode = AM_MODE_CST;
    cmd.tau_ff_Nm = NAN;
    CHECK_ST(am_codec_pack_rx1601(&C, 0, &cmd, b), AM_ERR_PARAM);
    cmd.tau_ff_Nm = 0.0f;
    cmd.q_ref_rad = 3000.0;                 /* 3e9 urad does not fit INT32 */
    CHECK_ST(am_codec_pack_rx1601(&C, 0, &cmd, b), AM_ERR_PARAM);
    cmd.q_ref_rad = 0.0;
    cmd.kp_Nm_per_rad = -1.0f;
    CHECK_ST(am_codec_pack_rx1601(&C, 0, &cmd, b), AM_ERR_PARAM);
    cmd.kp_Nm_per_rad = 0.0f;
    cmd.mode = (am_mode)77;
    CHECK_ST(am_codec_pack_rx1601(&C, 0, &cmd, b), AM_ERR_PARAM);
}

static void test_rx1600(void)
{
    am_command cmd;
    uint8_t b[20];
    memset(&cmd, 0, sizeof cmd);
    cmd.mode = AM_MODE_CST;
    cmd.tau_ff_Nm = 18.0f;                  /* 500 per mille of 36 N*m */
    CHECK_ST(am_codec_pack_rx1600(&C, 0x000F, &cmd, b), AM_OK);
    CHECK(b[2] == 10);
    CHECK(am__get_u16(b + 12) == 500);
    cmd.tau_ff_Nm = 36.0f * 40.0f;          /* 40000 per mille: does not fit INT16 */
    CHECK_ST(am_codec_pack_rx1600(&C, 0, &cmd, b), AM_ERR_PARAM);
    cmd.mode = AM_MODE_IMPEDANCE;
    CHECK_ST(am_codec_pack_rx1600(&C, 0, &cmd, b), AM_ERR_NOT_SUPPORTED);
    memset(&cmd, 0, sizeof cmd);
    cmd.mode = AM_MODE_CSP;
    cmd.q_ref_rad = -0.25;
    cmd.qd_ref_rad_s = 1.5f;
    cmd.tau_ff_Nm = 3.6f;
    CHECK_ST(am_codec_pack_rx1600(&C, 0x000F, &cmd, b), AM_OK);
    CHECK((int32_t)am__get_u32(b + 4) == -250000);
    CHECK((int16_t)am__get_u16(b + 14) == 100);
    CHECK((int32_t)am__get_u32(b + 16) == 1500);
}

static void test_tx_roundtrip(void)
{
    am_telemetry t, u;
    uint8_t b[84];
    memset(&t, 0, sizeof t);
    t.state = AM_DS_OPERATION_ENABLED;
    t.mode = AM_MODE_CSP;
    t.statusword = (uint16_t)(0x0027 | 0x0010 | AM_SW_REF_VALID);
    t.q_rad = -1.234567;
    t.qd_rad_s = 2.5f;
    t.tau_est_Nm = 12.3f;
    t.safety_io = 0x0F;
    t.ref_state = AM_REF_VALID;
    t.t_winding_C = 55.5f;
    t.t_ns = 123456789012ull;
    t.sample_counter = 7;
    t.sync_flags = 3;
    t.q_motor_counts = -123456;
    t.qd_motor_rad_s = 54.0f;
    t.q_out_encoder_rad = 1.0f;
    t.iq_A = -4.2f;
    t.vbus_V = 47.9f;
    t.warning_word = 5;
    am__drive_pack_tx1a00(&C, &t, b);
    am__drive_pack_tx1a01(&t, b + 24);
    memset(&u, 0, sizeof u);
    CHECK_ST(am_codec_unpack_tx1a00(&C, b, &u), AM_OK);
    CHECK_ST(am_codec_unpack_tx1a01(&C, b + 24, &u), AM_OK);
    CHECK(u.state == AM_DS_OPERATION_ENABLED);
    CHECK(u.mode == AM_MODE_CSP);
    CHECK_NEAR(u.q_rad, -1.234567, 1e-9);
    CHECK_NEAR(u.tau_est_Nm, 12.3, 1e-3);
    CHECK_NEAR(u.t_winding_C, 55.5, 1e-4);
    CHECK(u.t_ns == 123456789012ull);
    CHECK(u.q_motor_counts == -123456);
    CHECK_NEAR(u.iq_A, -4.2, 1e-4);
    CHECK_NEAR(u.vbus_V, 47.9, 1e-4);
    CHECK_NEAR(u.q_out_encoder_rad, 1.0, 2 * 3.1416 / 65536);
    CHECK(u.extended_valid);
    CHECK(u.ref_state == AM_REF_VALID);
}

static void test_compact(void)
{
    am_command cmd, back;
    am_telemetry t, u;
    uint8_t slot[16], fb[32], ctl, cnt, echo = 0;
    memset(&cmd, 0, sizeof cmd);
    cmd.mode = AM_MODE_IMPEDANCE;
    cmd.q_ref_rad = 0.25;
    cmd.qd_ref_rad_s = 1.0f;
    cmd.tau_ff_Nm = -2.5f;
    cmd.kp_Nm_per_rad = 100.0f;
    cmd.kd_Nm_s_per_rad = 5.0f;
    CHECK(am_codec_compact_mode(AM_MODE_IMPEDANCE) == 4);
    CHECK_ST(am_codec_pack_cfd_slot(&C, (uint8_t)(AM_CC_ENABLE | (4u << AM_CC_MODE_SHIFT)), 9, &cmd, slot), AM_OK);
    am__drive_unpack_cfd_slot(&C, slot, &ctl, &cnt, &back);
    CHECK(cnt == 9 && back.mode == AM_MODE_IMPEDANCE && (ctl & AM_CC_ENABLE));
    CHECK_NEAR(back.kp_Nm_per_rad, 100.0, 1e-4);
    CHECK_NEAR(back.tau_ff_Nm, -2.5, 1e-6);
    cmd.kp_Nm_per_rad = 2000.0f;                   /* above the 16-bit compact range */
    CHECK_ST(am_codec_pack_cfd_slot(&C, 0, 0, &cmd, slot), AM_ERR_PARAM);
    memset(&t, 0, sizeof t);
    t.state = AM_DS_SWITCHED_ON;
    t.mode = AM_MODE_IMPEDANCE;
    t.q_rad = 0.1;
    t.qd_rad_s = -0.5f;
    t.tau_est_Nm = 1.5f;
    t.iq_A = 0.4f;
    t.t_winding_C = 31.0f;
    t.vbus_V = 48.0f;
    t.ref_state = AM_REF_VALID;
    t.t_ns = 5000;
    t.q_motor_counts = 99;
    t.warning_word = 2;
    am__drive_pack_cfd_fb(&t, 9, true, true, fb);
    memset(&u, 0, sizeof u);
    CHECK_ST(am_codec_unpack_cfd_fb(&C, fb, 32, &u, &echo), AM_OK);
    CHECK(echo == 9);
    CHECK(u.state == AM_DS_SWITCHED_ON && u.mode == AM_MODE_IMPEDANCE && u.ref_state == AM_REF_VALID);
    CHECK(u.statusword & AM_SW_REMOTE);
    CHECK_NEAR(u.t_winding_C, 31.0, 1e-6);
    CHECK(u.warning_word == 2 && u.extended_valid);
    CHECK_ST(am_codec_unpack_cfd_fb(&C, fb, 20, &u, &echo), AM_ERR_PARAM);
}

static void test_state_walk(void)
{
    CHECK(am_codec_next_controlword(0x0040, true) == 0x06);
    CHECK(am_codec_next_controlword(0x0021, true) == 0x07);
    CHECK(am_codec_next_controlword(0x0023, true) == 0x0F);
    CHECK(am_codec_next_controlword(0x0027, true) == 0x0F);
    CHECK(am_codec_next_controlword(0x0027, false) == 0x00);
    CHECK(am_codec_decode_state(0x0040) == AM_DS_SWITCH_ON_DISABLED);
    CHECK(am_codec_decode_state(0x0027 | AM_SW_STO) == AM_DS_OPERATION_ENABLED);
    CHECK(am_codec_decode_state(0x0008) == AM_DS_FAULT);
    CHECK(am_codec_decode_state(0x0007) == AM_DS_QUICK_STOP_ACTIVE);
}

static void test_strings(void)
{
    CHECK(strcmp(am_fault_name(0x100B), "HOST_STALE") == 0);
    CHECK(strcmp(am_fault_name(0x7777), "UNKNOWN") == 0);
    CHECK(strstr(am_status_str(AM_ERR_STO), "STO") != NULL);
    CHECK(am_api_version() == AM_VERSION_PACK(0, 2, 0));
}

int main(void)
{
    setup();
    RUN(test_init_rejects_bad_scaling);
    RUN(test_rx1601_golden);
    RUN(test_rx1601_ranges);
    RUN(test_rx1600);
    RUN(test_tx_roundtrip);
    RUN(test_compact);
    RUN(test_state_walk);
    RUN(test_strings);
    TEST_EXIT();
}
