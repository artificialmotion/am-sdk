// SPDX-License-Identifier: Apache-2.0
// The public headers compile and link as C++.
#include "am_api.h"
#include "am_est.h"
#include "am_sim.h"

#include <stdio.h>     // C headers keep this test free of the C++ standard library
#include <string.h>

int main()
{
    am_session_config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.interface = "sim:M";
    am_session *s = nullptr;
    am_joint *j = nullptr;
    if (am_init(&cfg, &s) != AM_OK || am_open_joint(s, 0, &j) != AM_OK || am_start(s) != AM_OK) return 1;
    if (am_set_mode(j, AM_MODE_CST) != AM_OK || am_enable(j, 1000) != AM_OK) return 2;
    am_set_torque(j, 1.0f);
    am_cycle(s, nullptr);
    am_telemetry t;
    if (am_read_telemetry(j, &t) != AM_OK || t.state != AM_DS_OPERATION_ENABLED) return 3;
    am_codec c;
    am_codec_init(&c, 36000, 13530, 20, 1, 0);
    am_command cmd = {};
    cmd.mode = AM_MODE_CST;
    cmd.tau_ff_Nm = 1.0f;
    uint8_t b[AM_PDO_RX_1601_LEN];
    if (am_codec_pack_rx1601(&c, 0x000F, &cmd, b) != AM_OK) return 4;
    am_est_calibration cal;
    am_est est;
    am_est_input in;
    am_est_torque e;
    if (am_est_calibration_default("M", &cal) != AM_OK || am_est_init(&est, &cal, nullptr) != AM_OK) return 5;
    if (am_est_input_from_telemetry(&t, 1e-3, &in) != AM_OK || am_est_update(&est, &in, &e) != AM_OK) return 6;
    am_deinit(s);
    printf("C++ build ok\n");
    return 0;
}
