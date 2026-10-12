"""Artificial Motion S-D actuator model (dependency-free).
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

_DATA = json.loads(r'''{"id":"S-D","curves":{"36":{"peak":{"points":[[0,12],[10,15],[20,15],[40,15],[60,15],[66.5725,15],[80,14.1495],[100,12.8828],[114.59,11.9587],[120,11.6161],[140,10.3493],[160,9.0826],[180,7.8158],[200,6.917],[220,6.869],[240,5.891],[260,4.559],[280,2.828],[300,0.535],[303.4,0.4441]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,6.718],[20,7.286],[40,7.256],[60,7.225],[80,7.185],[100,7.143],[114.59,7.111],[120,7.099],[140,7.055],[160,7.01],[180,6.964],[200,6.917],[220,6.869],[240,5.891],[260,4.559],[280,2.828],[300,0.535],[303.4,0.4441]],"interpolate":true,"hold_Nm":null}},"48":{"peak":{"points":[[0,12],[10,15],[20,15],[40,15],[60,15],[80,15],[100,15],[114.59,15],[120,15],[140,15],[159.5253,15],[160,14.9714],[180,13.765],[200,12.5586],[220,11.3522],[240,10.1458],[260,8.9394],[280,7.733],[300,6.642],[320,6.59],[340,5.784],[360,4.61],[380,3.115],[400,1.146],[408.2,0.6761]],"interpolate":true,"hold_Nm":null},"continuous":{"points":[[0,6.696],[20,7.263],[40,7.233],[60,7.195],[80,7.154],[100,7.111],[114.59,7.079],[120,7.068],[140,7.023],[160,6.979],[180,6.933],[200,6.886],[220,6.839],[240,6.791],[260,6.742],[280,6.692],[300,6.642],[320,6.59],[340,5.784],[360,4.61],[380,3.115],[400,1.146],[408.2,0.6761]],"interpolate":true,"hold_Nm":null}}},"cont_torque_nm":7.08,"peak_nm":15,"no_load_rad_s":{"36":31.77197370330477,"48":42.74660403984512},"thermal_tau_s":210.01602668279628,"family_tau_s":210}''')
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
