# SPDX-License-Identifier: Apache-2.0
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import am_sdk as am  # noqa: E402


def make(unit="M", mode=am.AmMode.CST, enable=True, cycle_us=1000, **opts):
    """One-joint sim session; returns (session, joint, sim_joint)."""
    cfg = am.AmSessionConfig(interface="sim:" + unit, cycle_us=cycle_us, sim_options=[am.SimOptions(**opts)])
    s = am.am_init(cfg)
    j = am.am_open_joint(s, 0)
    am.am_start(s)
    am.am_set_mode(j, mode)
    if enable:
        am.am_enable(j, 2000)
    return s, j, s.backend.joints[0]
