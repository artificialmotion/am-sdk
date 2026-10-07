"""Bus path tests: L1 codec, the wire-level simulator, EtherCAT over a fake pysoem, CAN-FD over python-can's
virtual bus. The same controller code runs on every backend and must give the same motion."""
import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from helpers import am  # noqa: E402
from am_sdk import codec as C  # noqa: E402
from am_sdk.backends import SimWireTransport, TransportBackend  # noqa: E402


def has(mod):
    import importlib.util
    return importlib.util.find_spec(mod) is not None


class Codec(unittest.TestCase):
    def setUp(self):
        self.c = am.am_codec_init(36000, 13530, 20, 1, 2)

    def test_rx1601_layout(self):
        cmd = am.AmCommand(mode=am.AmMode.IMPEDANCE, q_ref_rad=0.5, qd_ref_rad_s=-1.25, tau_ff_Nm=3.0005,
                           kp_Nm_per_rad=150.0, kd_Nm_s_per_rad=6.0, iq_ref_A=0.0)
        b = am.am_codec_pack_rx1601(self.c, 0x000F, cmd)
        self.assertEqual(len(b), 28)
        self.assertEqual(struct.unpack("<HbxiiiIIi", b), (0x000F, -1, 500000, -1250, 3001, 150000, 6000, 0))
        cw, back = C.drive_unpack_rx1601(b)
        self.assertEqual(cw, 0x000F)
        self.assertAlmostEqual(back.q_ref_rad, 0.5)
        self.assertAlmostEqual(back.kp_Nm_per_rad, 150.0)

    def test_rx1600_modes_and_range(self):
        b = am.am_codec_pack_rx1600(self.c, 0x000F, am.AmCommand(mode=am.AmMode.CST, tau_ff_Nm=18.0))
        self.assertEqual(struct.unpack("<Hbxiihhi", b), (0x000F, 10, 0, 0, 500, 0, 0))
        with self.assertRaises(am.AmError) as cm:
            am.am_codec_pack_rx1600(self.c, 0, am.AmCommand(mode=am.AmMode.IMPEDANCE))
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_NOT_SUPPORTED)
        with self.assertRaises(am.AmError) as cm:
            am.am_codec_pack_rx1600(self.c, 0, am.AmCommand(mode=am.AmMode.CST, tau_ff_Nm=36.0 * 40))  # > 32767 permille
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_PARAM)
        with self.assertRaises(am.AmError):
            am.am_codec_pack_rx1601(self.c, 0, am.AmCommand(mode=am.AmMode.CST, tau_ff_Nm=float("nan")))

    def test_tx_roundtrip(self):
        t = am.AmTelemetry(state=am.AmDriveState.OPERATION_ENABLED, mode=am.AmMode.CSP, q_rad=-1.234567,
                           qd_rad_s=2.5, tau_est_Nm=12.3, error_code=0, fault_word=0, safety_io=0x0F,
                           ref_state=am.AmRefState.VALID, t_winding_C=55.5, t_ns=123456789012, sample_counter=7,
                           sync_flags=3, q_motor_counts=-123456, qd_motor_rad_s=54.0, q_out_encoder_rad=1.0,
                           id_A=0.1, iq_A=-4.2, iq_ref_A=-4.0, vq_V=12.5, vbus_V=47.9, ibus_est_A=1.5,
                           t_magnet_C=40.0, t_power_stage_C=45.0, t_board_C=33.0, warning_word=0x5,
                           statusword=C.STATE_TO_SW[am.AmDriveState.OPERATION_ENABLED] | C.SW_REF_VALID | C.SW_VOLTAGE)
        b = C.drive_pack_tx1a00(self.c, t) + C.drive_pack_tx1a01(t)
        self.assertEqual(len(b), 84)
        u = am.am_codec_unpack_tx1a00(self.c, b[:24])
        am.am_codec_unpack_tx1a01(self.c, b[24:], u)
        self.assertEqual(u.state, am.AmDriveState.OPERATION_ENABLED)
        self.assertEqual(u.mode, am.AmMode.CSP)
        self.assertAlmostEqual(u.q_rad, -1.234567, places=6)
        self.assertAlmostEqual(u.tau_est_Nm, 12.3, places=3)
        self.assertAlmostEqual(u.t_winding_C, 55.5, places=1)
        self.assertEqual(u.t_ns, 123456789012)
        self.assertEqual(u.q_motor_counts, -123456)
        self.assertAlmostEqual(u.iq_A, -4.2, places=2)
        self.assertAlmostEqual(u.vbus_V, 47.9, places=3)
        self.assertAlmostEqual(u.q_out_encoder_rad, 1.0, delta=2 * math.pi / 65536)
        self.assertTrue(u.extended_valid)

    def test_compact_roundtrip(self):
        cmd = am.AmCommand(mode=am.AmMode.IMPEDANCE, q_ref_rad=0.25, qd_ref_rad_s=1.0, tau_ff_Nm=-2.5,
                           kp_Nm_per_rad=100.0, kd_Nm_s_per_rad=5.0)
        ctl = C.compact_control(C.CW_ENABLE_OPERATION, am.AmMode.IMPEDANCE, True)
        b = am.am_codec_pack_cfd_slot(self.c, ctl, 9, cmd)
        self.assertEqual(len(b), 16)
        control, counter, back = C.drive_unpack_cfd_slot(self.c, b)
        self.assertEqual((control, counter, back.mode), (ctl, 9, am.AmMode.IMPEDANCE))
        self.assertAlmostEqual(back.kp_Nm_per_rad, 100.0)
        self.assertAlmostEqual(back.tau_ff_Nm, -2.5)
        with self.assertRaises(am.AmError):        # K above the 16-bit compact range (1310 N*m/rad)
            am.am_codec_pack_cfd_slot(self.c, ctl, 0, am.AmCommand(mode=am.AmMode.IMPEDANCE, kp_Nm_per_rad=2000.0))
        t = am.AmTelemetry(state=am.AmDriveState.SWITCHED_ON, mode=am.AmMode.IMPEDANCE, q_rad=0.1, qd_rad_s=-0.5,
                           tau_est_Nm=1.5, iq_A=0.4, t_winding_C=31.0, vbus_V=48.0, ref_state=am.AmRefState.VALID,
                           t_ns=5000, q_motor_counts=99, warning_word=2)
        fb = C.drive_pack_cfd_fb(t, 9, True, True)
        u, echo = am.am_codec_unpack_cfd_fb(self.c, fb)
        self.assertEqual(echo, 9)
        self.assertEqual(u.state, am.AmDriveState.SWITCHED_ON)
        self.assertEqual(u.mode, am.AmMode.IMPEDANCE)
        self.assertEqual(u.ref_state, am.AmRefState.VALID)
        self.assertTrue(u.statusword & C.SW_REMOTE)
        self.assertAlmostEqual(u.t_winding_C, 31.0)
        self.assertEqual(u.warning_word, 2)

    def test_state_walk(self):
        sw = 0x0040
        seq = []
        for st in (0x0040, 0x0021, 0x0023, 0x0027):
            seq.append(am.am_codec_next_controlword(st, True))
        self.assertEqual(seq, [0x06, 0x07, 0x0F, 0x0F])
        self.assertEqual(am.am_codec_decode_state(sw), am.AmDriveState.SWITCH_ON_DISABLED)


def _script(s, j, mode, n=1500):
    """A fixed command sequence: impedance sine, then a torque step. Returns telemetry per cycle."""
    out = []
    am.am_set_mode(j, mode)
    am.am_enable(j, 2000)
    for k in range(n):
        t = k * 1e-3
        if mode == am.AmMode.IMPEDANCE:
            q = 0.4 * math.sin(2 * math.pi * 1.0 * t)
            am.am_set_impedance(j, am.AmImpedance(q_ref_rad=q, qd_ref_rad_s=0.4 * 2 * math.pi * math.cos(2 * math.pi * t),
                                                  kp_Nm_per_rad=120.0, kd_Nm_s_per_rad=5.0))
        elif mode == am.AmMode.CSP:
            am.am_set_position(j, 0.3 * (1 - math.cos(2 * math.pi * 0.5 * t)))
        else:
            am.am_set_torque(j, 8.0 if k < n // 2 else -8.0)
        am.am_cycle(s)
        out.append(am.am_read_telemetry(j))
    return out


def _session(backend=None, units="M", **cfg):
    c = am.AmSessionConfig(interface="sim:" + units, sim_options=[am.SimOptions(load_inertia_kgm2=0.1)], **cfg)
    if backend is not None:
        c.backend = backend
    s = am.am_init(c)
    j = am.am_open_joint(s, getattr(backend, "first", 0) if backend else 0)
    am.am_start(s)
    return s, j


class SimWire(unittest.TestCase):
    def test_wire_matches_direct_sim(self):
        for mode in (am.AmMode.IMPEDANCE, am.AmMode.CSP, am.AmMode.CST):
            s1, j1 = _session()
            a = _script(s1, j1, mode)
            s2, j2 = _session(TransportBackend(SimWireTransport(["M"], [am.SimOptions(load_inertia_kgm2=0.1)])))
            b = _script(s2, j2, mode)
            for k, (x, y) in enumerate(zip(a, b)):
                self.assertEqual(x.state, y.state, "%s cycle %d" % (mode.name, k))
                self.assertAlmostEqual(x.q_rad, y.q_rad, delta=2e-4, msg="%s cycle %d" % (mode.name, k))
                self.assertAlmostEqual(x.tau_est_Nm, y.tau_est_Nm, delta=0.2, msg="%s cycle %d" % (mode.name, k))

    def test_identity_limits_gains_over_the_wire(self):
        s0 = am.am_init(am.AmSessionConfig(interface="sim:H-F"))
        jd = am.am_open_joint(s0, 0)
        s, j = _session(TransportBackend(SimWireTransport(["H-F"])), units="H-F")
        self.assertEqual([d.product_code for d in am.am_scan(s)], [4])
        a, b = am.am_get_identity(jd), am.am_get_identity(j)
        self.assertEqual((a.sku, a.ratio_num, a.ratio_den, a.serial, a.od_version, a.uid, a.device_name),
                         (b.sku, b.ratio_num, b.ratio_den, b.serial, b.od_version, b.uid, b.device_name))
        ta, tb = am.am_get_transparency(jd), am.am_get_transparency(j)
        for f in ("rated_torque_Nm", "peak_torque_2s_Nm", "no_load_speed_48V_rad_s", "backdrive_torque_Nm"):
            self.assertAlmostEqual(getattr(ta, f), getattr(tb, f), places=3)
        self.assertAlmostEqual(ta.reflected_inertia_kgm2, tb.reflected_inertia_kgm2, places=6)
        self.assertEqual(tb.basis, am.AmDataBasis.DESIGN_TARGET)
        eff, ceil = am.am_get_limits(j)
        self.assertAlmostEqual(ceil.torque_pos_Nm, ta.peak_torque_2s_Nm, places=3)
        bad = am.AmLimits(**{**eff.__dict__, "torque_pos_Nm": ceil.torque_pos_Nm * 1.02})
        with self.assertRaises(am.AmError) as cm:
            am.am_set_limits(j, bad)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_LIMIT)
        ok = am.AmLimits(**{**eff.__dict__, "torque_pos_Nm": 20.0, "torque_neg_Nm": 15.0, "pos_min_rad": -1.0,
                            "pos_max_rad": 1.0})
        am.am_set_limits(j, ok)
        e2 = am.am_get_limits(j)[0]
        self.assertAlmostEqual(e2.torque_pos_Nm, 20.0, delta=ta.rated_torque_Nm * 1e-3)
        self.assertAlmostEqual(e2.pos_max_rad, 1.0, places=6)
        g = am.am_get_loop_gains(j)
        g.pos_kp_1_per_s = 20.0
        am.am_set_loop_gains(j, g)
        self.assertAlmostEqual(am.am_get_loop_gains(j).pos_kp_1_per_s, 20.0, places=5)
        g.vel_filter_hz = 5.0
        with self.assertRaises(am.AmError) as cm:
            am.am_set_loop_gains(j, g)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_LIMIT)
        am.am_set_user_label(j, "left knee, joint 4")         # 18 bytes: segmented transfer on CAN
        self.assertEqual(am.am_get_identity(j).user_label, "left knee, joint 4")
        self.assertIn('"data_basis": "design_target"', am.am_read_calibration(j))
        with self.assertRaises(am.AmError):
            am.am_sdo_write(j, 0x6064, 0, b"\0\0\0\0")
        self.assertEqual(am.am_last_abort_code(j), 0x06010002)

    def test_faults_sto_and_events_over_the_wire(self):
        events = []
        tp = SimWireTransport(["M"], [am.SimOptions(load_inertia_kgm2=1e4)])
        s, j = _session(TransportBackend(tp), event_cb=events.append)
        am.am_set_mode(j, am.AmMode.CST)
        am.am_enable(j, 1000)
        sim = tp.nodes[0].joint
        for _ in range(5):
            am.am_set_torque(j, 20.0)
            am.am_cycle(s)
        self.assertAlmostEqual(sim.torque_applied, 20.0, delta=1e-3)
        sim.set_sto(True)
        am.am_cycle(s)
        self.assertEqual(sim.torque_applied, 0.0)
        self.assertTrue(am.am_read_telemetry(j).sto_active)
        with self.assertRaises(am.AmError) as cm:
            am.am_enable(j, 50)
        self.assertEqual(cm.exception.status, am.AmStatus.AM_ERR_STO)
        sim.set_sto(False)
        am.am_cycle(s)
        am.am_enable(j, 1000)
        sim.inject_fault(am.FAULT_HOST_STALE)
        for _ in range(3):
            am.am_cycle(s)
        t = am.am_read_telemetry(j)
        self.assertEqual(t.state, am.AmDriveState.FAULT)
        info = am.am_get_faults(j)
        self.assertIn(am.FAULT_HOST_STALE, info.codes)
        self.assertEqual(am.am_get_fault_history(j)[0].code, am.FAULT_HOST_STALE)
        sim.clear_fault_cause()
        am.am_clear_faults(j, 500)
        am.am_enable(j, 1000)
        kinds = {e.kind for e in events}
        for k in (am.AmEventKind.STO_CHANGED, am.AmEventKind.FAULT, am.AmEventKind.FAULT_CLEARED,
                  am.AmEventKind.STATE_CHANGED):
            self.assertIn(k, kinds)
        am.am_stop(s)
        self.assertEqual(sim.state, am.AmDriveState.SWITCH_ON_DISABLED)

    def test_csp_holds_until_first_command(self):
        tp = SimWireTransport(["M"], [am.SimOptions(load_inertia_kgm2=0.05, q0_rad=1.0)])
        s, j = _session(TransportBackend(tp))
        am.am_set_mode(j, am.AmMode.CSP)
        am.am_enable(j, 1000)
        for _ in range(20):
            am.am_cycle(s)
        self.assertLess(abs(tp.nodes[0].joint.torque_applied), 1e-3)
        self.assertAlmostEqual(am.am_read_telemetry(j).q_rad, 1.0, delta=1e-5)


@unittest.skipUnless(has("pysoem"), "pysoem not installed")
class EtherCat(unittest.TestCase):
    def test_ethercat_transport_on_fake_pysoem(self):
        import fake_pysoem
        from am_sdk.backends.ethercat import EtherCatTransport, list_nics
        from am_sdk.backends.simwire import SimNode
        cfg0 = am.AmSessionConfig(bus=am.AmBus.ETHERCAT)
        fake_pysoem.Master.nodes = [SimNode("M", 0, cfg0, am.SimOptions(load_inertia_kgm2=0.1)),
                                    SimNode("S-S", 1, cfg0)]
        self.assertEqual(list_nics(fake_pysoem)[0][1], "fake NIC")
        be = TransportBackend(EtherCatTransport("fake0", pysoem_module=fake_pysoem))
        be.transport.realtime = False
        s = am.am_init(am.AmSessionConfig(bus=am.AmBus.ETHERCAT, backend=be))
        self.assertEqual([d.product_code for d in am.am_scan(s)], [3, 2])
        j = am.am_open_joint(s, 0)
        am.am_start(s)
        ref = _session()
        a = _script(*ref, am.AmMode.IMPEDANCE, n=600)
        b = _script(s, j, am.AmMode.IMPEDANCE, n=600)
        for x, y in zip(a, b):
            self.assertAlmostEqual(x.q_rad, y.q_rad, delta=2e-4)
        self.assertTrue(b[-1].extended_valid)
        am.am_deinit(s)


@unittest.skipUnless(has("can"), "python-can not installed")
class CanFd(unittest.TestCase):
    def _run(self, compact, channel):
        from can_node import CanSimBus
        from am_sdk.backends.canfd import CanFdTransport
        node = CanSimBus(["M", "H-F"], channel=channel, options=[am.SimOptions(load_inertia_kgm2=0.1), None])
        try:
            tp = CanFdTransport("virtual", channel, rx_timeout_s=0.5)
            tp.realtime = False
            s = am.am_init(am.AmSessionConfig(bus=am.AmBus.CANFD, canfd_compact=compact, backend=TransportBackend(tp)))
            devs = am.am_scan(s)
            self.assertEqual([(d.address, d.product_code) for d in devs], [(1, 3), (2, 4)])
            self.assertTrue(all(d.has_control_authority for d in devs))
            j = am.am_open_joint(s, 1)
            am.am_open_joint(s, 2)
            am.am_set_user_label(j, "right hip pitch")
            self.assertEqual(am.am_get_identity(j).user_label, "right hip pitch")
            am.am_start(s)
            ref = _session()
            a = _script(*ref, am.AmMode.IMPEDANCE, n=400)
            b = _script(s, j, am.AmMode.IMPEDANCE, n=400)
            for x, y in zip(a, b):
                self.assertEqual(x.state, y.state)
                self.assertAlmostEqual(x.q_rad, y.q_rad, delta=2e-3 if compact else 2e-4)
            am.am_deinit(s)
        finally:
            node.close()

    def test_canfd_profile0(self):
        self._run(False, "am_p0")

    def test_canfd_compact(self):
        self._run(True, "am_cp")


if __name__ == "__main__":
    unittest.main(verbosity=2)
