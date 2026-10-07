# SPDX-License-Identifier: Apache-2.0
"""The C estimator (c/src/am_est.c) and the Python estimator (am_sdk.estimation) must agree.

c/tests/am_est_xcheck runs a fixed synthetic sequence (moving, a hold, missing temperature, dither,
an invalid sample) through am_est and am_collision; this test feeds the same inputs to the Python
classes and compares every output within float rounding and every quality bit exactly.
The C binary is found through AM_SDK_EST_XCHECK or in c/build*/ and build*/; skipped when not built.
"""
import glob
import math
import os
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import am  # noqa: E402,F401
from am_sdk import estimation as E  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TWO_PI = 6.283185307179586


def find_tool():
    env = os.environ.get("AM_SDK_EST_XCHECK")
    if env and os.path.isfile(env):
        return env
    for base in (os.path.join(ROOT, "c"), ROOT):
        for name in ("am_est_xcheck", "am_est_xcheck.exe"):
            for pat in (os.path.join(base, "build*", name), os.path.join(base, "build*", "*", name)):
                hits = sorted(glob.glob(pat))
                if hits:
                    return hits[0]
    return None


TOOL = find_tool()


def variant_cal(cal, variant):
    if variant != 1:
        return cal
    cal.kt_sat_frac, cal.kt_sat_current_A, cal.bearing_Nm = 0.05, 1.5, 0.03
    cal.friction_map = [[0.0, 0.6], [0.5, 0.45], [3.0, 0.8]]
    cal.efficiency_map = E.EfficiencyMap(temps_C=[20.0, 80.0], loads_Nm=[0.0, 5.0, 20.0],
                                         eta_forward=[[0.88, 0.92, 0.94], [0.90, 0.93, 0.95]],
                                         eta_reverse=[[0.78, 0.84, 0.86], [0.80, 0.85, 0.88]])
    return cal


def python_lines(unit, variant):
    cal = variant_cal(E.EstimatorCalibration.from_unit(unit), variant)
    est = E.TorqueEstimator(cal)
    det = E.CollisionDetector(E.CollisionConfig(inertia_kgm2=cal.reflected_inertia_kgm2 + 0.05, threshold_Nm=0.8))
    dt = 1e-3
    out = []
    for k in range(3000):
        t = k * dt
        hold = 1200 <= k < 1500
        tq = (k if k < 1200 else (1200 if hold else k - 300)) * dt
        q = 0.4 * math.sin(TWO_PI * 0.7 * tq) + 0.05 * math.sin(TWO_PI * 3.1 * tq)
        iq = 2.0 * math.sin(TWO_PI * 1.3 * t) + 0.3
        temp = 30.0 + 20.0 * t if k % 500 < 450 else None
        e = est.update(dt, q, iq, temp, dither_active=1350 <= k < 1450, valid=k != 2900)
        if not hold:
            qd_c = 0.4 * TWO_PI * 0.7 * math.cos(TWO_PI * 0.7 * tq) + 0.05 * TWO_PI * 3.1 * math.cos(TWO_PI * 3.1 * tq)
        else:
            qd_c = 0.0
        o = det.update(dt, qd_c, e.tau_drive_Nm, -1.5 * math.cos(q), e.quality)
        out.append([k, e.tau_out_Nm, e.tau_drive_Nm, e.tau_motor_Nm, e.friction_Nm, e.qd_rad_s, e.qdd_rad_s2,
                    e.bound_Nm, e.confidence, e.quality, o.residual_Nm, o.threshold_Nm, int(o.active), int(o.onset)])
    return out


@unittest.skipUnless(TOOL, "am_est_xcheck not built (cmake --build c/build), or set AM_SDK_EST_XCHECK")
class EstimatorCrossCheck(unittest.TestCase):
    def compare(self, unit, variant):
        txt = subprocess.run([TOOL, unit, str(variant)], capture_output=True, text=True, timeout=120,
                             check=True).stdout.split("\n")
        c_rows = [ln.split() for ln in txt if ln.strip()]
        py = python_lines(unit, variant)
        self.assertEqual(len(c_rows), len(py))
        ints = {0, 9, 12, 13}
        stick = sum(1 for r in py if r[9] & E.EST_STICK)
        self.assertGreater(stick, 50)                 # the hold section is exercised
        self.assertTrue(any(r[9] & E.EST_DITHER for r in py))
        self.assertTrue(any(r[9] & E.EST_NO_TEMPERATURE for r in py))
        self.assertTrue(any(r[13] for r in py))       # at least one detection onset
        for c, p in zip(c_rows, py):
            for i, (cv, pv) in enumerate(zip(c, p)):
                if i in ints:
                    self.assertEqual(int(cv), pv, "%s v%d row %s col %d" % (unit, variant, p[0], i))
                else:
                    cv = float(cv)
                    if math.isinf(pv):
                        self.assertTrue(math.isinf(cv) and (cv > 0) == (pv > 0))
                    else:
                        self.assertLessEqual(abs(cv - pv), 1e-9 * max(1.0, abs(pv)),
                                             "%s v%d row %s col %d: C %r Python %r" % (unit, variant, p[0], i, cv, pv))

    def test_s_s_parametric(self):
        self.compare("S-S", 0)

    def test_m_maps_and_saturation(self):
        self.compare("M", 1)

    def test_h_x_parametric(self):
        self.compare("H-X", 0)


if __name__ == "__main__":
    unittest.main()
