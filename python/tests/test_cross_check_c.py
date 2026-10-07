"""The C and Python simulators must agree.

Each scenario is a command script. The C library runs it through c/tests/am_xcheck (built by CMake);
the Python SDK runs the same script through the small interpreter below. Every command must return
the same status, and every cycle must give the same telemetry: identical state, mode, statusword,
warning and error words, and the same position, velocity, current, torque and temperature within
float rounding. The codec part checks byte-identical PDOs.

The C binary is found through the AM_SDK_XCHECK environment variable or in c/build*/ and build*/;
the test is skipped when it is not built.
"""
import glob
import math
import os
import random
import struct
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import am  # noqa: E402
from am_sdk import codec as C  # noqa: E402
from am_sdk.backends import SimWireTransport, TransportBackend  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def find_xcheck():
    env = os.environ.get("AM_SDK_XCHECK")
    if env and os.path.isfile(env):
        return env
    pats = []
    for base in (os.path.join(ROOT, "c"), ROOT):
        for name in ("am_xcheck", "am_xcheck.exe"):
            pats += [os.path.join(base, "build*", name), os.path.join(base, "build*", "*", name)]
    for p in pats:
        hits = sorted(glob.glob(p))
        if hits:
            return hits[0]
    return None


XCHECK = find_xcheck()


def f32(x):
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


def run_c(script):
    out = subprocess.run([XCHECK], input=script, capture_output=True, text=True, timeout=600, check=True).stdout
    return out.splitlines()


class PyRunner:
    """Executes the am_xcheck command language on the Python SDK and prints the same lines."""

    def __init__(self):
        self.out = []
        self.s = None
        self.j = {}
        self.opts = {}
        self.pending = None
        self.cycles = 0
        self.tp = None

    def sim(self, a):
        if self.tp is not None:
            return self.tp.nodes[a].joint
        return self.s.backend.joints[a]

    def tlm_line(self, a, t):
        return "T %d %d %d %d %.17g %.9g %.9g %.9g %.9g %.9g %d %d %d %d %d %.9g %d" % (
            self.cycles, a, int(t.state), int(t.mode), t.q_rad, f32(t.qd_rad_s), f32(t.iq_A), f32(t.tau_est_Nm),
            f32(t.t_winding_C), f32(t.vbus_V), t.statusword, t.warning_word, t.error_code, int(t.ref_state),
            1 if t.sto_active else 0, f32(t.q_out_encoder_rad), t.t_ns)

    def ensure_session(self):
        if self.s is None:
            units, cyc, bus, wire = self.pending
            n = len(units)
            opts = [am.SimOptions(**self.opts.get(i, {})) for i in range(n)]
            cfg = am.AmSessionConfig(bus=am.AmBus(bus), cycle_us=cyc, sim_units=units, sim_options=opts,
                                     telemetry_extended=True)
            if wire:
                self.tp = SimWireTransport(units, opts)
                cfg.backend = TransportBackend(self.tp)
            self.s = am.am_init(cfg)

    def do(self, line):
        p = line.split()
        cmd = p[0]
        st = 0
        try:
            if cmd == "session":
                self.pending = (p[1].split(","), int(p[2]), int(p[3]), len(p) > 4 and p[4] == "wire")
            elif cmd == "opt":
                key = {"load_inertia": "load_inertia_kgm2", "q0": "q0_rad", "ambient": "ambient_C",
                       "thermal_x0": "thermal_x0", "bus_V": "bus_V", "friction_scale": "friction_scale",
                       "peak_budget": "peak_budget_s"}[p[2]]
                self.opts.setdefault(int(p[1]), {})[key] = float(p[3])
            elif cmd == "open":
                self.ensure_session()
                self.j[int(p[1])] = am.am_open_joint(self.s, int(p[1]))
            elif cmd == "start":
                am.am_start(self.s)
            elif cmd == "stop":
                am.am_stop(self.s)
            elif cmd == "mode":
                am.am_set_mode(self.j[int(p[1])], int(p[2]))
            elif cmd == "enable":
                am.am_enable(self.j[int(p[1])], int(p[2]))
            elif cmd == "disable":
                am.am_disable(self.j[int(p[1])])
            elif cmd == "qstop":
                am.am_quick_stop(self.j[int(p[1])])
            elif cmd == "clear":
                am.am_clear_faults(self.j[int(p[1])], int(p[2]))
            elif cmd == "imp":
                q, qd, kp, kd, tau = map(float, p[2:7])
                am.am_set_impedance(self.j[int(p[1])], am.AmImpedance(q_ref_rad=q, qd_ref_rad_s=f32(qd), kp_Nm_per_rad=f32(kp),
                                                                      kd_Nm_s_per_rad=f32(kd), tau_ff_Nm=f32(tau)))
            elif cmd == "pos":
                am.am_set_position(self.j[int(p[1])], float(p[2]), f32(float(p[3])), f32(float(p[4])))
            elif cmd == "vel":
                am.am_set_velocity(self.j[int(p[1])], f32(float(p[2])), f32(float(p[3])))
            elif cmd == "tor":
                am.am_set_torque(self.j[int(p[1])], f32(float(p[2])))
            elif cmd == "cur":
                am.am_set_current(self.j[int(p[1])], f32(float(p[2])))
            elif cmd == "sdow":
                am.am_sdo_write(self.j[int(p[1])], int(p[2], 16), int(p[3]), bytes.fromhex(p[4]))
            elif cmd == "od":
                a = int(p[1])
                try:
                    v = am.am_sdo_read(self.j[a], int(p[2], 16), int(p[3])).hex()
                except am.AmError as e:
                    v = "abort:%08x" % am.am_last_abort_code(self.j[a])
                    st = int(e.status)
                self.out.append("O %04x %d %s" % (int(p[2], 16), int(p[3]), v))
            elif cmd == "ext":
                v = float(p[2])
                self.sim(int(p[1])).set_external_torque(v)
            elif cmd == "sto":
                self.sim(int(p[1])).set_sto(p[2] != "0")
            elif cmd == "fault":
                self.sim(int(p[1])).inject_fault(int(p[2], 16))
            elif cmd == "clearcause":
                self.sim(int(p[1])).clear_fault_cause()
            elif cmd == "cycle":
                for _ in range(int(p[1])):
                    am.am_cycle(self.s)
                    self.cycles += 1
                    for a in sorted(self.j):
                        try:
                            self.out.append(self.tlm_line(a, am.am_read_telemetry(self.j[a])))
                        except am.AmError:
                            pass
            elif cmd == "limits":
                e, c = am.am_get_limits(self.j[int(p[1])])
                for tag, l in (("eff", e), ("ceil", c)):
                    self.out.append("L %s %.9g %.9g %.9g %.9g %.17g %.17g %.9g %.9g %d" % (
                        tag, f32(l.torque_pos_Nm), f32(l.torque_neg_Nm), f32(l.current_A), f32(l.speed_rad_s),
                        l.pos_min_rad, l.pos_max_rad, f32(l.kp_max_Nm_per_rad), f32(l.kd_max_Nm_s_per_rad),
                        l.comms_loss_ms))
            elif cmd == "gains":
                g = am.am_get_loop_gains(self.j[int(p[1])])
                self.out.append("G " + " ".join("%.9g" % f32(v) for v in (
                    g.pos_kp_1_per_s, g.vel_kp_Nm_s_per_rad, g.vel_ki_Nm_per_rad, g.vel_filter_hz,
                    g.torque_cmd_filter_hz, g.notch_hz, g.notch_q)))
            elif cmd == "ident":
                j = self.j[int(p[1])]
                i, tr = am.am_get_identity(j), am.am_get_transparency(j)
                self.out.append("I %d %d %d %d %d %s|%s %08x %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d" % (
                    int(i.sku), i.ratio_num, i.ratio_den, i.serial, i.od_version, i.device_name, i.uid.hex(),
                    i.cal_record_crc32, f32(tr.rated_torque_Nm), f32(tr.peak_torque_2s_Nm), f32(tr.rated_speed_rad_s),
                    f32(tr.no_load_speed_48V_rad_s), f32(tr.reflected_inertia_kgm2), f32(tr.backdrive_torque_Nm),
                    f32(tr.kinetic_energy_no_load_J), int(tr.basis)))
            else:
                st = int(am.AmStatus.AM_ERR_PARAM)
        except am.AmError as e:
            st = int(e.status)
        self.out.append("R %s %d" % (cmd, st))

    def run(self, script):
        for line in script.splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                self.do(line)
        return self.out


def compare(tc, c_lines, py_lines, q_tol=1e-9, rel=2e-6, atol=1e-6, name=""):
    tc.assertEqual(len(c_lines), len(py_lines), "%s: line count C %d, Python %d" % (name, len(c_lines), len(py_lines)))
    worst = 0.0
    for k, (a, b) in enumerate(zip(c_lines, py_lines)):
        ta, tb = a.split(), b.split()
        tc.assertEqual(ta[0], tb[0], "%s line %d: %s | %s" % (name, k, a, b))
        if ta[0] == "T":
            tc.assertEqual(ta[1:5], tb[1:5], "%s line %d (cycle, joint, state, mode): %s | %s" % (name, k, a, b))
            tc.assertEqual(ta[11:16], tb[11:16], "%s line %d (statusword, warning, error, ref, sto): %s | %s"
                           % (name, k, a, b))
            tc.assertEqual(ta[17], tb[17], "%s line %d (t_ns)" % (name, k))
            dq = abs(float(ta[5]) - float(tb[5]))
            worst = max(worst, dq)
            tc.assertLessEqual(dq, q_tol, "%s line %d q: %s | %s" % (name, k, a, b))
            for i in (6, 7, 8, 9, 10, 16):
                x, y = float(ta[i]), float(tb[i])
                tc.assertLessEqual(abs(x - y), rel * max(1.0, abs(x), abs(y)) + atol,
                                   "%s line %d field %d: %s | %s" % (name, k, i, a, b))
        elif ta[0] in ("L", "G", "I"):
            for x, y in zip(ta[1:], tb[1:]):
                if x == y:
                    continue
                try:
                    fx, fy = float(x), float(y)
                    if math.isnan(fx) and math.isnan(fy):
                        continue
                    tc.assertLessEqual(abs(fx - fy), 1e-6 * max(1.0, abs(fx)), "%s: %s | %s" % (name, a, b))
                except ValueError:
                    tc.assertEqual(x, y, "%s: %s | %s" % (name, a, b))
        else:
            tc.assertEqual(a, b, "%s line %d" % (name, k))
    return worst


def sine_script(units="M", load=0.15, n=3000, wire=""):
    s = ["session %s 1000 1 %s" % (units, wire), "opt 0 load_inertia %r" % load, "open 0", "start", "mode 0 -1",
         "enable 0 1000"]
    for k in range(n):
        t = k * 1e-3
        q = 0.5 * math.sin(2 * math.pi * 0.5 * t)
        qd = 0.5 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t)
        s += ["imp 0 %.17g %.9g 150 6 0" % (q, f32(qd)), "cycle 1"]
    s += ["ident 0", "limits 0", "gains 0"]
    return "\n".join(s) + "\n"


@unittest.skipUnless(XCHECK, "C cross-check tool not built (cmake --build c/build), or set AM_SDK_XCHECK")
class CrossCheck(unittest.TestCase):
    def check(self, script, name, **kw):
        c = run_c(script)
        py = PyRunner().run(script)
        return compare(self, c, py, name=name, **kw)

    def test_impedance_sine(self):
        self.check(sine_script(), "impedance sine M")

    def test_two_joints_csp_cst_and_gravity(self):
        s = ["session H-F,M 1000 1", "opt 0 load_inertia 1.1", "opt 1 load_inertia 0.16", "opt 1 q0 0.2",
             "open 0", "open 1", "ext 0 -12.5", "ext 1 4.25", "start", "mode 0 8", "mode 1 10", "enable 0 1000",
             "enable 1 1000", "cycle 30"]
        for k in range(2000):
            s += ["pos 0 %.17g 0 0" % (0.4 * (1 - math.cos(2 * math.pi * 0.5 * k * 1e-3))),
                  "tor 1 %r" % (6.0 if k < 1000 else -6.0), "cycle 1"]
        s += ["disable 0", "cycle 5", "mode 0 9", "enable 0 1000"]
        for k in range(800):
            s += ["vel 0 1.5 0", "cycle 1"]
        s += ["qstop 0", "cycle 400", "limits 1"]
        self.check("\n".join(s) + "\n", "two joints")

    def test_peak_budget_and_limits(self):
        s = ["session M 1000 1", "opt 0 load_inertia 10000", "open 0", "start", "mode 0 10",
             "limits 0", "enable 0 1000"]
        s += ["tor 0 500", "cycle 1"] * 3500
        s += ["tor 0 0", "cycle 1"] * 500
        s += ["disable 0", "cycle 3", "limits 0"]
        self.check("\n".join(s) + "\n", "peak budget")

    def test_sto_fault_and_clear(self):
        s = ["session M 1000 1", "opt 0 load_inertia 10000", "open 0", "start", "mode 0 10", "enable 0 1000"]
        s += ["tor 0 20", "cycle 1"] * 10
        s += ["sto 0 1", "tor 0 20", "cycle 5", "enable 0 100", "sto 0 0", "cycle 20", "enable 0 1000",
              "tor 0 20", "cycle 5", "fault 0 100b", "cycle 5", "tor 0 1", "clear 0 100", "clearcause 0",
              "clear 0 500", "enable 0 1000", "cycle 5", "stop", "start", "cycle 3"]
        s += ["od 0 %s %d" % (i, k) for i, k in (("2601", 0), ("2601", 1), ("2600", 1), ("2600", 2), ("603f", 0))]
        self.check("\n".join(s) + "\n", "sto and faults")

    def test_thermal_derate_and_trip_hx(self):
        s = ["session H-X 20000 1", "opt 0 load_inertia 10000", "open 0", "ext 0 -131.7", "start", "mode 0 10",
             "enable 0 2000"]
        s += ["tor 0 500", "cycle 1"] * 6000
        self.check("\n".join(s) + "\n", "thermal H-X")
        s = ["session M 20000 1", "opt 0 thermal_x0 1.5", "opt 0 ambient 40", "open 0", "start", "mode 0 10",
             "cycle 3", "enable 0 100", "cycle 3000", "clear 0 500", "enable 0 2000"]
        self.check("\n".join(s) + "\n", "trip M")

    def test_current_mode_and_object_dictionary(self):
        objs = [("1018", 1), ("1018", 2), ("1018", 3), ("1018", 4), ("2000", 2), ("2000", 5), ("2000", 6),
                ("2000", 8), ("2000", 9), ("2100", 1), ("2300", 1), ("2300", 4), ("2300", 20), ("2401", 1),
                ("2620", 1), ("2901", 1), ("2901", 3), ("2902", 2), ("2902", 3), ("2a00", 4), ("2a00", 5),
                ("2a11", 3), ("2a20", 1), ("2a30", 1), ("2a30", 2), ("2a30", 3), ("2a40", 1), ("2a50", 1),
                ("2a50", 2), ("2a50", 3), ("2a50", 4), ("2a50", 5), ("2a50", 6), ("2b00", 1), ("2d00", 1),
                ("2d00", 2), ("2d00", 3), ("2d00", 4), ("2d00", 5), ("2d00", 6), ("2d00", 7), ("2d00", 8),
                ("2d00", 9), ("2d00", 10), ("2d00", 11), ("2d00", 12), ("6041", 0), ("6061", 0), ("6064", 0),
                ("606c", 0), ("6073", 0), ("6075", 0), ("6076", 0), ("6077", 0), ("607d", 1), ("607d", 2),
                ("6091", 1), ("6091", 2), ("60e0", 0), ("60e1", 0), ("1008", 0), ("9999", 0), ("2d00", 99)]
        for unit in ("S-S", "S-D", "M", "H-F", "H-T", "H-X"):
            s = ["session %s 1000 1" % unit, "open 0", "start", "mode 0 -2", "sdow 0 2a11 3 01", "mode 0 -2",
                 "enable 0 1000"]
            s += ["cur 0 2.5", "cycle 1"] * 50
            s += ["od 0 %s %d" % o for o in objs]
            s += ["disable 0", "cycle 3", "sdow 0 60e0 0 f401", "sdow 0 2a30 1 0000a041", "sdow 0 2a40 1 0000a040",
                  "sdow 0 6064 0 00000000", "sdow 0 2e00 1 6b6e6565", "od 0 60e0 0", "od 0 2e00 1", "limits 0",
                  "gains 0", "ident 0"]
            self.check("\n".join(s) + "\n", "current + OD %s" % unit)

    def test_c_wire_path_matches_python_direct(self):
        c = run_c(sine_script(n=1500, wire="wire"))
        py = PyRunner().run(sine_script(n=1500))
        compare(self, [l for l in c if l[0] in "TR"], [l for l in py if l[0] in "TR"], q_tol=2e-4, rel=5e-2,
                name="C wire vs Python direct")

    def test_c_wire_matches_python_wire(self):
        c = run_c(sine_script(n=1500, wire="wire"))
        py = PyRunner().run(sine_script(n=1500, wire="wire"))
        # telemetry is float in C and double in Python: a value on a rounding boundary of its wire field
        # (10 mA, 1 urad) can land one LSB apart
        compare(self, [l for l in c if l[0] in "TR"], [l for l in py if l[0] in "TR"], q_tol=2e-6, atol=0.0101,
                name="wire vs wire")


@unittest.skipUnless(XCHECK, "C cross-check tool not built")
class CodecCrossCheck(unittest.TestCase):
    def test_pack_bytes_identical(self):
        rnd = random.Random(7)
        lines, expect = [], []
        rated_mNm, rated_mA = 36000, 13530
        c = am.am_codec_init(rated_mNm, rated_mA, 20, 1, 0)
        for k in range(400):
            def r(lo, hi, step=1 / 1024):
                return round(rnd.uniform(lo, hi) / step) * step
            mode = rnd.choice([8, 9, 10, -1, -2])
            q, qd, tau = r(-3, 3, 1e-6), r(-20, 20), r(-60, 60)
            kp, kd, iq = r(0, 500), r(0, 30), r(-20, 20)
            if k % 50 == 0:
                tau = 4000.0          # out of range for the per mille and compact fields
            cmd = am.AmCommand(mode=am.AmMode(mode), q_ref_rad=q, qd_ref_rad_s=qd, tau_ff_Nm=tau, kp_Nm_per_rad=kp,
                               kd_Nm_s_per_rad=kd, iq_ref_A=iq)
            vals = "%r %r %r %r %r %r" % (q, qd, tau, kp, kd, iq)
            lines.append("pack1601 %d %d 15 %d %s" % (rated_mNm, rated_mA, mode, vals))
            try:
                expect.append(am.am_codec_pack_rx1601(c, 15, cmd).hex())
            except am.AmError:
                expect.append("")
            lines.append("pack1600 %d %d 15 %d %s" % (rated_mNm, rated_mA, mode, vals))
            try:
                expect.append(am.am_codec_pack_rx1600(c, 15, cmd).hex())
            except am.AmError:
                expect.append("")
            ctl = rnd.randrange(256)
            lines.append("packslot %d %d %d %d %d %s" % (rated_mNm, rated_mA, ctl, k & 0xFF, mode, vals))
            try:
                expect.append(am.am_codec_pack_cfd_slot(c, ctl, k & 0xFF, cmd).hex())
            except am.AmError:
                expect.append("")
        got = [l[2:].strip() for l in run_c("\n".join(lines) + "\n") if l.startswith("P")]
        self.assertEqual(len(got), len(expect))
        for k, (g, e) in enumerate(zip(got, expect)):
            self.assertEqual(g, e, "line %d: %s" % (k, lines[k]))

    def test_unpack_identical(self):
        s = am.am_init(am.AmSessionConfig(interface="sim:M", sim_options=[am.SimOptions(load_inertia_kgm2=0.1)]))
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        am.am_set_mode(j, am.AmMode.CST)
        am.am_enable(j, 1000)
        c = am.am_codec_init(36000, 13530, 20, 1, 0)
        lines, want = [], []
        for k in range(300):
            am.am_set_torque(j, 15.0 if k < 150 else -15.0)
            am.am_cycle(s)
            t = am.am_read_telemetry(j)
            tx = C.drive_pack_tx1a00(c, t) + C.drive_pack_tx1a01(t)
            fb = C.drive_pack_cfd_fb(t, k & 0xFF, True, k % 2 == 0)
            lines += ["unpacktx 36000 13530 " + tx.hex(), "unpackfb 36000 13530 " + fb.hex()]
            for kind, data in (("tx", tx), ("fb", fb)):
                if kind == "tx":
                    u = am.am_codec_unpack_tx1a00(c, data[:24])
                    am.am_codec_unpack_tx1a01(c, data[24:], u)
                    echo = 0
                else:
                    u, echo = am.am_codec_unpack_cfd_fb(c, data)
                want.append((int(u.state), int(u.mode), u.q_rad, u.qd_rad_s, u.tau_est_Nm, u.iq_A, u.t_winding_C,
                             u.statusword, int(u.ref_state), 1 if u.extended_valid else 0, u.t_ns, u.q_motor_counts,
                             u.vbus_V, u.q_out_encoder_rad, u.warning_word, echo))
        got = [l.split()[1:] for l in run_c("\n".join(lines) + "\n") if l.startswith("U")]
        self.assertEqual(len(got), len(want))
        for g, w in zip(got, want):
            self.assertEqual([int(x) for x in g[:2]], list(w[:2]))
            self.assertEqual([int(x) for x in (g[7], g[8], g[9], g[10], g[11], g[14], g[15])],
                             [w[7], w[8], w[9], w[10], w[11], w[14], w[15]])
            for x, y in zip((g[2], g[3], g[4], g[5], g[6], g[12], g[13]), (w[2], w[3], w[4], w[5], w[6], w[12], w[13])):
                self.assertLessEqual(abs(float(x) - y), 1e-6 * max(1.0, abs(y)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
