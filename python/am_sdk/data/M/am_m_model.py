"""Artificial Motion M actuator model (dependency-free).
Data revision 2026-10-05
Product preview: specifications are design targets for evaluation units shipping in Q1 2027.

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

_DATA = json.loads(r'''{"id":"M","curves":{"36":{"peak":{"points":[[0,80],[30,80],[40,79.3],[50,66.2],[60,52],[70,37.8],[75,30.6],[80,23.1],[90.1,6.4]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,36.3],[10,36.3],[20,36.5],[30,36.7],[40,36.8667],[42.45,36.9075],[50,36.53],[55.3597,36.2513],[60,36.01],[70,35.48],[71.8097,35.194],[75,30.6],[80,23.1],[85.6766,13.7139],[90.1,3.4]],"interpolate":true,"hold_Nm":null}},"48":{"peak":{"points":[[0,80],[10,80],[20,80],[30,80],[38.6,80],[40,79.3],[45,77.4],[50,75.8],[60,73.2],[70,71],[75,66.2],[80,60.3],[90,48.4],[100,36.4],[110,23.9],[120.6,8.1]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,36.3],[10,36.3],[20,36.5],[30,36.7],[38.6,36.8433],[40,36.8667],[40.5882,36.8765],[45,36.7],[50,36.5],[55.539,36.223],[60,36],[70,35.5],[75,35.2],[80,34.9],[90,34.3],[91.8421,34.2079],[100,33.49],[100.9422,33.3383],[103.4211,32.1236],[110,23.9],[117.9104,12.109],[120.6,6.4]],"interpolate":true,"hold_Nm":null}}},"cont_torque_nm":36,"peak_nm":80,"no_load_rad_s":{"36":9.435249936281345,"48":12.629202467430968},"thermal_tau_s":null,"family_tau_s":210}''')
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
