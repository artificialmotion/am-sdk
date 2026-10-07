# SPDX-License-Identifier: Apache-2.0
"""Impedance control of one M joint following a sine.

Run: python examples/01_impedance_sine.py
The controller code below is the same code that drives a real unit; only the session
configuration (interface) changes.
"""
import math

import _path  # noqa: F401
import am_sdk as am

K = 150.0          # N*m/rad
D = 6.0            # N*m*s/rad
AMP, FREQ = 0.5, 0.5     # rad, Hz
LOAD_J = 0.15      # kg*m^2 of link the joint carries

cfg = am.AmSessionConfig(interface="sim:M", cycle_us=1000,
                         sim_options=[am.SimOptions(load_inertia_kgm2=LOAD_J)])
s = am.am_init(cfg)
joint = am.am_open_joint(s, 0)
am.am_start(s)
am.am_set_mode(joint, am.AmMode.IMPEDANCE)
am.am_enable(joint, timeout_ms=1000)

t_end, n = 6.0, 0
err_sq, peak_tau, peak_temp = 0.0, 0.0, 0.0
for k in range(int(t_end * 1000)):
    t = k * 1e-3
    q_ref = AMP * math.sin(2 * math.pi * FREQ * t)
    qd_ref = AMP * 2 * math.pi * FREQ * math.cos(2 * math.pi * FREQ * t)
    am.am_set_impedance(joint, am.AmImpedance(q_ref_rad=q_ref, qd_ref_rad_s=qd_ref,
                                              kp_Nm_per_rad=K, kd_Nm_s_per_rad=D))
    am.am_cycle(s)
    tlm = am.am_read_telemetry(joint)
    if t > 2.0:                                   # skip the start-up transient
        err_sq += (q_ref - tlm.q_rad) ** 2
        n += 1
    peak_tau = max(peak_tau, abs(tlm.tau_est_Nm))
    peak_temp = max(peak_temp, tlm.t_winding_C)

print("M joint, impedance K=%g D=%g, sine %.2f rad at %.1f Hz, load %.2f kg m^2" % (K, D, AMP, FREQ, LOAD_J))
print("RMS tracking error  %.4f rad" % math.sqrt(err_sq / n))
print("peak torque         %.1f N*m   (peak rating %.0f N*m)" % (peak_tau, am.am_get_transparency(joint).peak_torque_2s_Nm))
print("winding temperature %.2f C (model)" % peak_temp)
print("final state         %s" % tlm.state.name)
am.am_deinit(s)
