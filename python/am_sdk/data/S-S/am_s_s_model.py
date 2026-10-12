"""Artificial Motion S-S actuator model (dependency-free).
Data revision 2026-10-12
Product preview: fully designed and simulated. Specifications are design targets for evaluation units shipping in Q1 2027.

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

_DATA = json.loads(r'''{"id":"S-S","curves":{"36":{"peak":{"points":[[0,25.5],[10,30],[20,30],[30,30],[40,30],[47.1273,30],[50,29.1345],[60,26.1216],[70,23.1087],[80,20.0959],[90,17.083],[95.49,15.4289],[100,14.278],[110,11.941],[120,8.573],[130,5.0315],[140,2.0186],[146.7,0]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,14.406],[10,15.585],[20,15.475],[30,15.332],[40,15.187],[50,15.04],[60,14.891],[70,14.741],[80,14.588],[90,14.434],[95.49,14.349],[100,14.278],[110,11.941],[120,8.573],[130,4.053],[140,0],[146.7,0]],"interpolate":true,"hold_Nm":null}},"48":{"peak":{"points":[[0,25.5],[10,30],[20,30],[30,30],[40,30],[50,30],[60,30],[70,30],[80,30],[90,30],[90.828,30],[95.49,28.6864],[100,27.4156],[110,24.598],[120,21.7804],[130,18.9627],[140,16.1451],[150,13.3274],[160,10.5098],[170,7.6922],[180,4.8745],[190,2.0569],[197.3,0]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,14.36],[10,15.536],[20,15.407],[30,15.264],[40,15.119],[50,14.972],[60,14.824],[70,14.674],[80,14.523],[90,14.369],[95.49,14.284],[100,14.214],[110,14.057],[120,13.899],[130,13.738],[140,13.576],[150,12.932],[160,10.369],[170,7.014],[180,2.456],[190,0],[197.3,0]],"interpolate":true,"hold_Nm":null}}},"cont_torque_nm":14.28,"peak_nm":30,"no_load_rad_s":{"36":15.362388076054087,"48":20.661207685108874},"thermal_tau_s":225.97888140789075,"family_tau_s":210}''')
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
