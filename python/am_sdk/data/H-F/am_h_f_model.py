"""Artificial Motion H-F actuator model (dependency-free).
Data revision 2026-10-05
Product preview: specifications are design targets for evaluation units shipping in Q2 2027.

torque_limit(speed_rad_s, bus='48', kind='peak')
    Interpolates the published torque-speed curve at the OUTPUT shaft. Returns Nm, or None where the
    published data do not support a value (outside the curve, in a gap, or between non-interpolable peak markers).
    kind: 'peak' (2 s event from a warm start) or 'continuous' (thermal steady state, 25 C, declared mount).
    Some curves (e.g. M peak) are exact markers only: by default (markers='conservative') a speed between markers
    returns the lower of the two neighbouring markers (non-interpolated lower bound); markers='exact' returns None there.
ThermalState(tau_s, cont_torque)
    First-order normalised heating  x' = ((torque/cont_torque)^2 - x) / tau_s.  x = 1.0 is the continuous limit.
    It does not model the 2 s peak allowance or ambient changes.
"""
import json
import math
import warnings

_DATA = json.loads(r'''{"id":"H-F","curves":{"36":{"peak":{"points":[[0,120],[5,120],[10,120],[15,120],[20,120],[25,120],[30,115.4],[40,108.6],[45,106],[50,103.9333],[60,99.8],[67.0236,97.7983],[70,90],[80,63.8],[90,35.6],[96.1,15]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,55.2],[5,55.2],[10,55.2],[15,55.45],[20,55.7],[25,55.95],[30,56.2],[40,56.5],[45,56.7],[50,56.9],[51.7117,56.9342],[60,56.18],[67.0236,55.4987],[70,55.21],[80,54.19],[89.1263,38.0638],[90,35.6],[90.8017,32.8926],[96.1,8.92]],"interpolate":true,"hold_Nm":null}},"48":{"peak":{"points":[[0,120],[5,120],[10,120],[15,120],[20,120],[25,120],[30,115.4],[40,108.6],[45,106],[50,103.9333],[60,99.8],[70,96.95],[80,94.1],[90,91.25],[99.0582,88.6684],[100,86.7],[110,65.4],[115,54.1],[120,42.4],[128.7,0]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,55.2],[5,55.2],[10,55.2],[15,55.45],[20,55.7],[25,55.95],[30,56.2],[40,56.5],[45,56.7],[50,56.9],[51.9298,56.9386],[60,56.18],[70,55.21],[80,54.19],[90,53.15],[99.0582,52.1536],[100,52.05],[110,50.46],[115,48.85],[117.5888,48.04220799999999],[120,42.4],[128.7,0]],"interpolate":true,"hold_Nm":null}}},"cont_torque_nm":56.18,"peak_nm":120,"no_load_rad_s":{"36":10.063568466999303,"48":13.47743248390021},"thermal_tau_s":null,"family_tau_s":210}''')
ID = _DATA["id"]
CONT_TORQUE_NM = _DATA["cont_torque_nm"]
PEAK_NM = _DATA["peak_nm"]
NO_LOAD_RAD_S = _DATA["no_load_rad_s"]
THERMAL_TAU_S = _DATA["thermal_tau_s"]
THERMAL_TAU_IS_FAMILY_VALUE = THERMAL_TAU_S is None
_RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


def torque_limit(speed_rad_s, bus='48', kind='peak', markers='conservative'):
    if str(bus) not in _DATA["curves"] or kind not in ("peak", "continuous"):
        raise ValueError("bus must be '48' or '36'; kind must be 'peak' or 'continuous'")
    c = _DATA["curves"][str(bus)][kind]
    pts = c["points"]
    if not pts:
        return None
    rpm = abs(speed_rad_s) * _RPM_PER_RAD_S
    if c["hold_Nm"] is not None and rpm < 1e-6:
        return c["hold_Nm"]
    if rpm < pts[0][0] - 1e-6 or rpm > pts[-1][0] + 1e-6:
        return None
    for (r0, t0), (r1, t1) in zip(pts, pts[1:]):
        if r0 - 1e-6 <= rpm <= r1 + 1e-6:
            if c["interpolate"]:
                return t0 if r1 == r0 else t0 + (t1 - t0) * (rpm - r0) / (r1 - r0)
            if abs(rpm - r0) <= 0.5:
                return t0
            if abs(rpm - r1) <= 0.5:
                return t1
            return min(t0, t1) if markers == 'conservative' else None
    return None


class ThermalState:
    def __init__(self, tau_s=None, cont_torque=None, x0=0.0):
        if tau_s is None:
            tau_s = THERMAL_TAU_S
            if tau_s is None:
                tau_s = _DATA["family_tau_s"]
                warnings.warn("%s: no published thermal time constant; using the conservative family value tau=%g s "
                              "(shortest published family value)." % (ID, tau_s))
        self.tau_s = float(tau_s)
        self.cont_torque = float(CONT_TORQUE_NM if cont_torque is None else cont_torque)
        self.x = float(x0)

    def step(self, torque_nm, dt):
        """Advance dt seconds at constant torque (exact solution of the first-order ODE). Returns x."""
        target = (torque_nm / self.cont_torque) ** 2
        self.x = target + (self.x - target) * math.exp(-dt / self.tau_s)
        return self.x

    @property
    def over_limit(self):
        return self.x > 1.0


if __name__ == "__main__":
    print("%s: peak %.4g Nm, continuous %.4g Nm, no-load 48 V %.4g rad/s" % (ID, PEAK_NM, CONT_TORQUE_NM, NO_LOAD_RAD_S["48"]))
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0, 1.1):
        w = frac * NO_LOAD_RAD_S["48"]
        print("  w=%7.3f rad/s  peak48=%s  cont48=%s  peak36=%s" % (w, torque_limit(w, '48', 'peak'),
              torque_limit(w, '48', 'continuous'), torque_limit(w, '36', 'peak')))
    ts = ThermalState()
    print("  thermal tau %.1f s%s" % (ts.tau_s, " (family value)" if THERMAL_TAU_IS_FAMILY_VALUE else ""))
    t = 0.0
    while not ts.over_limit and t < 3600:
        ts.step(1.3 * ts.cont_torque, 1.0)
        t += 1.0
    print("  holding 1.3x continuous torque reaches x=1 after ~%d s (x=%.3f)" % (t, ts.x))
