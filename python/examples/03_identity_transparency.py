"""Read identity and transparency data from every unit, then measure backdrive breakaway.

The transparency record is what the drive itself publishes (object 0x2D00); the SDK returns
it as am_transparency. Data basis 1 means a design-target value.
"""
import math

import _path  # noqa: F401
import am_sdk as am

s = am.am_init(am.AmSessionConfig(interface="sim:S-S,S-D,M,H-F,H-T,H-X"))
print("%-5s %-4s %7s %7s %8s %9s %9s %7s %s" % ("unit", "ratio", "rated", "peak2s", "inertia", "backdrive",
                                                 "w_nl48", "KE", "basis"))
for dev in am.am_scan(s):
    j = am.am_open_joint(s, dev.address)
    ident, tr = am.am_get_identity(j), am.am_get_transparency(j)
    bd = "%.2f" % tr.backdrive_torque_Nm
    print("%-5s %-4s %7.1f %7.0f %8.4f %9s %9.2f %7.2f %s  serial %d  %s" % (
        ident.device_name.split()[1], "%d/%d" % (ident.ratio_num, ident.ratio_den), tr.rated_torque_Nm,
        tr.peak_torque_2s_Nm, tr.reflected_inertia_kgm2, bd, tr.no_load_speed_48V_rad_s,
        tr.kinetic_energy_no_load_J, tr.basis.name, ident.serial, hex(ident.capabilities)))

# breakaway torque of an unpowered M joint: push with a slowly rising external torque
s = am.am_init(am.AmSessionConfig(interface="sim:M"))
j = am.am_open_joint(s, 0)
am.am_start(s)
sim = s.backend.joints[0]
sim.set_external_torque(lambda t, q, qd: 0.25 * t)      # 0.25 N*m per second
for k in range(20000):
    am.am_cycle(s)
    if abs(sim.qd) > 1e-6:
        print("\nM unpowered: moves at %.2f N*m applied from outside (published backdrive 1.4 N*m)" % (0.25 * k * 1e-3))
        break
print("calibration record (sim):")
print(am.am_read_calibration(j))
am.am_deinit(s)
