# SPDX-License-Identifier: Apache-2.0
"""Sim backend: one plant plus one drive model per joint, in virtual time.

Per joint:
- plant: reflected output inertia (armature + optional load), Coulomb backdrive friction,
  optional external torque (gravity, contact) or an external physics engine (MuJoCo).
- drive: CiA 402 state machine, control modes (CSP, CSV, CST, impedance, current), torque
  clamp from the published continuous and peak torque-speed curves, a 2 s peak budget, a
  first-order thermal model (ThermalState from the unit's published model file) with
  derate and over-temperature trip, customer limits (lower-only), STO, fault latching.
- sensing: motor encoder 19 bit per motor turn, output encoder 14 bit per output turn,
  velocity from the motor encoder, current in 1 mA steps.

One am_cycle() = one drive cycle (default 1 kHz). The drive computes its command from the
feedback sample of the end of the previous cycle (one-cycle delay, as with a bus master),
then the plant runs for the cycle with that torque held.

Everything here is a model of design targets, not measured hardware.
"""
from __future__ import annotations

import copy
import math
import random
import struct
import zlib
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple, Union

from .. import units as U
from ..codec import (CW_FAULT_RESET, SW_INTERNAL_LIMIT, SW_REF_VALID, SW_REMOTE, SW_STO, SW_VOLTAGE,
                     SW_WARNING, STATE_TO_SW)
from ..am_types import (AM_CAP_ETHERCAT, AM_CAP_CANFD_CONTROL, AM_CAP_MODE_CURRENT, AM_CAP_OUTPUT_ABS_ENCODER,
                     AM_CAP_STO, AM_CAP_WINDING_SENSOR, AM_SIO_ENABLE_READBACK, AM_SIO_STO1, AM_SIO_STO2,
                     AM_SIO_GATE_SUPPLY_PRESENT, AM_SYNC_DC_LOCKED, AM_SYNC_TIMESTAMP_VALID, AM_SYNC_CAN_LOCKED,
                     AM_WARN_PEAK_BUDGET_EMPTY, AM_WARN_SPEED_LIMIT, AM_WARN_THERMAL_DERATE,
                     FAULT_COMMS_LOST, FAULT_MOTOR_OVERTEMP, FAULT_VBUS, OD_MAJOR_SUPPORTED, AmBus, AmCommand,
                     AmDataBasis, AmDeviceInfo, AmDriveState, AmError, AmEventKind, AmFaultInfo, AmFaultRecord,
                     AmIdentity, AmLimits, AmLoopGains, AmMode, AmRefState, AmSessionConfig, AmStatus,
                     AmTelemetry, AmTransparency, am_version_pack)
from .base import Backend, JointBackend

TWO_PI = 2.0 * math.pi
MOTOR_BITS = 19
OUTPUT_BITS = 14
PLANT_DT_S = 1.0e-4
BOOT_CYCLES = 3
REF_QUALIFY_CYCLES = 6        # about 6 cycles from reset to a valid reference
T_TRIP_C = 130.0              # winding trip (sim model value)
T_DERATE_C = 105.0            # derate starts where x = 1 at 25 C ambient
TRIP_HYST_K = 5.0
QUICK_STOP_DONE_RAD_S = 0.02
SANITY_TORQUE_NM = 1.0e4
SANITY_RAD = 1.0e6
ENVELOPE_HZ = 60.0            # sim impedance envelope: natural frequency of K on the armature
HOLD_ZERO = object()


@dataclass
class TruthPlant:
    """Opt-in richer plant for torque-estimator studies (Python simulator only; default None keeps
    the published model that the C simulator matches). With it the joint has a rigid gear with
    forward/reverse efficiency, Stribeck + viscous friction with temperature terms, output bearing
    friction, Kt drift with magnet temperature and saturation, torque and friction ripple, and
    current-sensor gain/offset/noise. Every value is an assumption for simulation, not a unit rating.
    The backdrive friction and friction_scale of SimOptions are not used when this is set."""
    kt_scale: float = 1.0                   # true Kt / unit-data Kt at kt_ref_C
    kt_ref_C: float = 60.0
    kt_temp_coeff_per_K: float = -0.00094
    kt_sat_frac: float = 0.0
    kt_sat_current_A: float = 0.0           # 0 = no saturation
    coulomb_Nm: float = 0.0                 # output-referred, rotor side of the gear
    breakaway_Nm: float = 0.0
    stribeck_vel_rad_s: float = 0.05
    viscous_Nm_s_per_rad: float = 0.0
    friction_ref_temp_C: float = 40.0
    coulomb_temp_coeff_per_K: float = 0.004
    viscous_temp_exp_per_K: float = 0.0225
    eta_forward: float = 1.0
    eta_reverse: float = 1.0
    bearing_Nm: float = 0.0                 # output side of the gear
    friction_ripple: float = 0.0            # fraction, once per motor revolution
    torque_ripple: float = 0.0              # fraction, six per motor revolution
    i_gain_err: float = 0.0                 # current sensor
    i_offset_A: float = 0.0
    i_noise_A: float = 0.0
    temp_sensor_bias_C: float = 0.0         # added to the reported winding and magnet temperatures
    seed: int = 1


@dataclass
class SimTruth:
    """Ground truth of the last cycle (means over the plant sub-steps). Simulator only."""
    tau_out_Nm: float = 0.0       # torque delivered at the output flange to the load
    tau_ext_Nm: float = 0.0       # external torque on the output (your external_torque function)
    tau_motor_Nm: float = 0.0     # motor torque before losses (truth plant) or drive torque (default plant)
    qdd_rad_s2: float = 0.0       # mean output acceleration over the cycle


@dataclass
class SimOptions:
    """Per-joint sim settings."""
    ambient_C: float = 25.0
    thermal_x0: float = 0.0           # initial normalised winding heat (1.0 = continuous limit)
    peak_budget_s: float = U.PEAK_BUDGET_S
    load_inertia_kgm2: float = 0.0    # added to the reflected armature inertia
    q0_rad: float = 0.0               # initial output angle
    bus_V: float = 48.0               # 48 or 36 selects the published curve set
    external_torque: Union[None, float, Callable[[float, float, float], float]] = None  # f(t, q, qd) in N*m
    external_plant: bool = False      # state comes from outside (MuJoCo coupling)
    friction_scale: float = 1.0
    authority_bus: Optional[AmBus] = None   # which bus the unit obeys; default = the session bus
    truth_plant: Optional[TruthPlant] = None   # Python only: richer plant for estimator studies


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class SimJoint(JointBackend):
    def __init__(self, unit: str, address: int, cfg: AmSessionConfig, opts: Optional[SimOptions] = None):
        self.ud = U.load_unit(unit)
        self.address = address
        self.opts = opts or SimOptions()
        self.dt = cfg.cycle_us * 1e-6
        self.n_sub = min(20, max(1, int(round(self.dt / PLANT_DT_S))))
        self.cfg_bus = cfg.bus
        self.extended = cfg.telemetry_extended
        self.can_sync = cfg.bus == AmBus.CANFD
        self._authority = self.opts.authority_bus if self.opts.authority_bus is not None else cfg.bus
        self._authority_cfg = self._authority
        self.label = ""
        self._ext_torque = self.opts.external_torque
        self.last_abort = 0
        self.history: List[AmFaultRecord] = []
        self.pending_events: List[Tuple[int, int, int]] = []
        self.sto_counter = 0
        self.sto_active = False
        self.faults: dict = {}          # code -> cause active (bool)
        self.first_fault = 0
        self.error_code = 0
        self.current_mode_enabled = False   # 0x2A11:3
        self.truth = SimTruth()
        tp = self.opts.truth_plant
        self._truth_rng = random.Random(tp.seed if tp is not None else 1)
        self._defaults()
        self.power_cycle()

    # ------------------------------------------------------------------ setup
    def _defaults(self):
        ud = self.ud
        wn = TWO_PI * ENVELOPE_HZ
        self.ceiling = AmLimits(
            torque_pos_Nm=ud.peak_nm, torque_neg_Nm=ud.peak_nm,
            current_A=ud.peak_nm / (ud.kt_motor_nm_per_a * ud.ratio * ud.gear_eta), speed_rad_s=ud.no_load_rad_s["48"],
            pos_min_rad=-math.inf, pos_max_rad=math.inf,
            kp_max_Nm_per_rad=ud.armature_kgm2 * wn * wn,
            kd_max_Nm_s_per_rad=2.0 * ud.armature_kgm2 * wn, comms_loss_ms=1000)
        self.limits = copy.copy(self.ceiling)
        self.limits.comms_loss_ms = 100
        jt = ud.armature_kgm2
        self.gains = AmLoopGains(pos_kp_1_per_s=30.0, vel_kp_Nm_s_per_rad=jt * TWO_PI * 50.0,
                                 vel_ki_Nm_per_rad=jt * TWO_PI * 50.0 * TWO_PI * 8.0,
                                 vel_filter_hz=300.0, torque_cmd_filter_hz=0.0, notch_hz=0.0, notch_q=1.0)
        self.gain_ceiling = AmLoopGains(pos_kp_1_per_s=200.0, vel_kp_Nm_s_per_rad=jt * TWO_PI * 100.0,
                                        vel_ki_Nm_per_rad=jt * TWO_PI * 100.0 * TWO_PI * 20.0,
                                        vel_filter_hz=1000.0)
        self.min_filter_hz = 50.0

    def power_cycle(self):
        """Drive reset: state, faults, thermal state, reference. Plant position is kept."""
        o = self.opts
        self.state = AmDriveState.NOT_READY_TO_SWITCH_ON
        self.boot_count = 0
        self.ref_cnt = 0
        self.ref_state = AmRefState.INVALID
        self.thermal = U.new_thermal(self.ud, o.thermal_x0)
        self.ambient = o.ambient_C
        self.t_magnet = self.ambient + 0.6 * U.WINDING_RISE_AT_RATED_K * self.thermal.x
        self.budget = o.peak_budget_s
        self.lockout = False
        self.bus_nominal = 48.0 if o.bus_V > 42.0 else 36.0
        self.vbus_in = o.bus_V
        self.faults = {}
        self.first_fault = 0
        self.error_code = 0
        self.warn = 0
        self.prev_cw = 0
        self.integ = 0.0
        self.have_cmd = False
        self.cmd = AmCommand()
        self.hold_q = 0.0
        self.mode = AmMode.NONE
        self.tau_applied = 0.0
        self.tau_filt = 0.0
        self.limit_active = False
        self.t_ns = getattr(self, "t_ns", 0)
        self.sample_counter = getattr(self, "sample_counter", 0)
        self.authority_cfg_pending = self._authority_cfg
        self._authority = self._authority_cfg
        if not hasattr(self, "q"):
            self.q = float(o.q0_rad)
            self.qd = 0.0
        self.ext_q = self.q
        self.ext_qd = self.qd
        self._sample_init()
        self.qs_active = False

    @property
    def inertia(self) -> float:
        return self.ud.armature_kgm2 + self.opts.load_inertia_kgm2

    # ------------------------------------------------------------------ sensing
    def _sample_init(self):
        self._quantise_position(self.q if not self.opts.external_plant else self.ext_q)
        self.qd_f = 0.0
        self.q_prev = self.q_meas
        self.iq_meas = 0.0
        self.vbus = self.vbus_in

    def _quantise_position(self, q: float):
        ratio = self.ud.ratio
        self.counts = int(round(q * ratio / TWO_PI * (1 << MOTOR_BITS)))
        self.q_meas = self.counts * TWO_PI / (1 << MOTOR_BITS) / ratio
        turn = (q % TWO_PI) / TWO_PI
        self.enc_out = (int(turn * (1 << OUTPUT_BITS)) % (1 << OUTPUT_BITS)) * TWO_PI / (1 << OUTPUT_BITS)

    def _sample(self, q: float, qd_true: float):
        self.q_prev = self.q_meas
        self._quantise_position(q)
        raw = (self.q_meas - self.q_prev) / self.dt
        a = 1.0 - math.exp(-TWO_PI * max(self.gains.vel_filter_hz, 1.0) * self.dt)
        self.qd_f += a * (raw - self.qd_f)

    # ------------------------------------------------------------------ envelope
    @property
    def x_limit(self) -> float:
        return max((T_DERATE_C - self.ambient) / U.WINDING_RISE_AT_RATED_K, 1e-3)

    @property
    def derate(self) -> float:
        x, xl = self.thermal.x, self.x_limit
        return 1.0 if x <= xl else 1.0 / math.sqrt(x / xl)

    @property
    def peak_headroom_x(self) -> float:
        """Heat a full peak event adds (normalised): (peak/rated)^2 over the budget time."""
        ud = self.ud
        return (ud.peak_nm / ud.rated_nm) ** 2 * (1.0 - math.exp(-self.opts.peak_budget_s / ud.thermal_tau_s))

    @property
    def peak_available(self) -> bool:
        # a peak event is released only while it cannot push the winding past the derate threshold
        return self.budget > 0.0 and not self.lockout and self.thermal.x <= self.x_limit - self.peak_headroom_x

    def _curve(self, w: float, kind: str) -> float:
        v = self.ud.model.torque_limit(abs(w), '48' if self.bus_nominal > 42.0 else '36', kind)
        return 0.0 if v is None else float(v)

    def envelope_at(self, w: float, peak: Optional[bool] = None) -> Tuple[float, float]:
        """(positive limit, negative limit magnitude) of the drive envelope at output speed w,
        before customer limits. Torque with the sign of the speed (motoring) follows the published
        curve; torque against the speed is capped at the stall value."""
        use_peak = self.peak_available if peak is None else peak
        kind = "peak" if use_peak else "continuous"
        motoring = self._curve(w, kind) * self.derate
        braking = self._curve(0.0, kind) * self.derate
        if abs(w) < 1e-6:
            return braking, braking
        return (motoring, braking) if w > 0 else (braking, motoring)

    def _clamp_torque(self, tau: float, w: float, q: float) -> float:
        pos, neg = self.envelope_at(w)
        lim = self.limits
        kt_tot = self.ud.kt_motor_nm_per_a * self.ud.ratio * self.ud.gear_eta
        icap = lim.current_A * kt_tot
        pos = min(pos, lim.torque_pos_Nm, icap)
        neg = min(neg, lim.torque_neg_Nm, icap)
        out = _clamp(tau, -neg, pos)
        self.warn &= ~(AM_WARN_SPEED_LIMIT)
        if abs(w) > lim.speed_rad_s and out * w > 0.0:
            out = 0.0
            self.warn |= AM_WARN_SPEED_LIMIT
        if (q <= lim.pos_min_rad and out < 0.0) or (q >= lim.pos_max_rad and out > 0.0):
            out = 0.0
        return out

    # ------------------------------------------------------------------ faults / events
    def _event(self, kind: AmEventKind, code: int = 0):
        self.pending_events.append((int(kind), code, self.t_ns))

    def raise_fault(self, code: int, cause_active: bool = True):
        if code in self.faults:
            self.faults[code] = cause_active or self.faults[code]
            return
        self.faults[code] = cause_active
        if not self.first_fault:
            self.first_fault = code
        self.error_code = code
        self.history.insert(0, AmFaultRecord(code=code, drive_state=int(self.state), t_ns=self.t_ns,
                                             vbus_V=self.vbus, iq_A=self.iq_meas,
                                             t_winding_C=self.t_winding, t_power_stage_C=self.t_ps))
        del self.history[32:]
        self._event(AmEventKind.FAULT, code)

    def inject_fault(self, code: int):
        """Test hook: latch a fault code as a persistent cause (clear with clear_fault_cause)."""
        self.raise_fault(code, True)

    def clear_fault_cause(self, code: Optional[int] = None):
        for c in ([code] if code is not None else list(self.faults)):
            if c in self.faults:
                self.faults[c] = False

    def set_sto(self, active: bool):
        """Test hook: the safety controller de-energises (True) or restores (False) both STO inputs."""
        if active != self.sto_active:
            self.sto_active = active
            if active:
                self.sto_counter += 1
            self._event(AmEventKind.STO_CHANGED, 1 if active else 0)

    def set_ambient(self, ambient_C: float):
        self.ambient = float(ambient_C)

    def set_bus_voltage(self, volts: float):
        self.vbus_in = float(volts)

    def set_external_torque(self, f):
        self._ext_torque = f

    def set_load_inertia(self, j: float):
        self.opts.load_inertia_kgm2 = float(j)

    def set_external_state(self, q: float, qd: float):
        """MuJoCo coupling: the plant state comes from outside; feedback is sampled here."""
        self.ext_q, self.ext_qd = q, qd
        self.q, self.qd = q, qd
        self._sample(q, qd)

    @property
    def t_winding(self) -> float:
        return self.ambient + U.WINDING_RISE_AT_RATED_K * self.thermal.x

    @property
    def t_ps(self) -> float:
        return self.ambient + 20.0 * self.thermal.x

    # ------------------------------------------------------------------ state machine
    def _statusword(self) -> int:
        sw = STATE_TO_SW[self.state]
        if self.state in (AmDriveState.READY_TO_SWITCH_ON, AmDriveState.SWITCHED_ON,
                          AmDriveState.OPERATION_ENABLED, AmDriveState.QUICK_STOP_ACTIVE):
            sw |= SW_VOLTAGE
        if self.warn:
            sw |= SW_WARNING
        if self._authority == self.cfg_bus:
            sw |= SW_REMOTE
        if self.limit_active:
            sw |= SW_INTERNAL_LIMIT
        if self.ref_state == AmRefState.VALID:
            sw |= SW_REF_VALID
        if self.sto_active:
            sw |= SW_STO
        return sw

    def _set_state(self, st: AmDriveState):
        if st != self.state:
            if self.state == AmDriveState.OPERATION_ENABLED:
                self.tau_applied = 0.0
            self.state = st
            if st == AmDriveState.OPERATION_ENABLED:
                self.have_cmd = False
                self.integ = 0.0
                self.hold_q = self.q_meas
            self._event(AmEventKind.STATE_CHANGED, int(st))

    def _state_machine(self, cw: int):
        st = self.state
        rising_reset = bool(cw & CW_FAULT_RESET) and not (self.prev_cw & CW_FAULT_RESET)
        self.prev_cw = cw
        if st == AmDriveState.NOT_READY_TO_SWITCH_ON:
            self.boot_count += 1
            if self.boot_count >= BOOT_CYCLES:
                self._set_state(AmDriveState.SWITCH_ON_DISABLED)
            return
        # faults latch the drive
        if self.faults and st not in (AmDriveState.FAULT, AmDriveState.FAULT_REACTION_ACTIVE):
            self._set_state(AmDriveState.FAULT_REACTION_ACTIVE)
            return
        if st == AmDriveState.FAULT_REACTION_ACTIVE:
            self._set_state(AmDriveState.FAULT)
            return
        if st == AmDriveState.FAULT:
            if rising_reset and not any(self.faults.values()):
                self.faults = {}
                self.first_fault = 0
                self.error_code = 0
                self.ref_cnt = 0
                self.ref_state = AmRefState.INVALID
                self.integ = 0.0
                self._event(AmEventKind.FAULT_CLEARED, 0)
                self._set_state(AmDriveState.SWITCH_ON_DISABLED)
            return
        # STO overrides everything: no torque, and no restart without a new enable walk
        if self.sto_active:
            if st != AmDriveState.SWITCH_ON_DISABLED:
                self._set_state(AmDriveState.SWITCH_ON_DISABLED)
            self.qs_active = False
            return
        # reference qualification: a few cycles after reset or fault clear
        if self.ref_state != AmRefState.VALID:
            self.ref_cnt += 1
            self.ref_state = AmRefState.VALID if self.ref_cnt >= REF_QUALIFY_CYCLES else AmRefState.CANDIDATE
        if st == AmDriveState.QUICK_STOP_ACTIVE:
            if abs(self.qd_f) < QUICK_STOP_DONE_RAD_S:
                self.qs_active = False
                self._set_state(AmDriveState.SWITCH_ON_DISABLED)
            return
        disable_voltage = (cw & 0x82) == 0x00
        quick_stop = (cw & 0x06) == 0x02
        shutdown = (cw & 0x87) == 0x06
        switch_on = (cw & 0x8F) == 0x07
        enable_op = (cw & 0x8F) == 0x0F
        if disable_voltage:
            if st != AmDriveState.SWITCH_ON_DISABLED:
                self._set_state(AmDriveState.SWITCH_ON_DISABLED)
            return
        if quick_stop and st in (AmDriveState.OPERATION_ENABLED, AmDriveState.SWITCHED_ON,
                                 AmDriveState.READY_TO_SWITCH_ON):
            if st == AmDriveState.OPERATION_ENABLED:
                self.qs_active = True
                self._set_state(AmDriveState.QUICK_STOP_ACTIVE)
            else:
                self._set_state(AmDriveState.SWITCH_ON_DISABLED)
            return
        if st == AmDriveState.SWITCH_ON_DISABLED:
            if shutdown:
                self._set_state(AmDriveState.READY_TO_SWITCH_ON)
        elif st == AmDriveState.READY_TO_SWITCH_ON:
            if switch_on or enable_op:
                self._set_state(AmDriveState.SWITCHED_ON)
        elif st == AmDriveState.SWITCHED_ON:
            if shutdown:
                self._set_state(AmDriveState.READY_TO_SWITCH_ON)
            elif enable_op and self.ref_state == AmRefState.VALID:
                self._set_state(AmDriveState.OPERATION_ENABLED)
        elif st == AmDriveState.OPERATION_ENABLED:
            if shutdown or switch_on:      # disable operation lands in Ready to switch on
                self._set_state(AmDriveState.READY_TO_SWITCH_ON)

    # ------------------------------------------------------------------ control laws
    def _control(self, mode: AmMode, fresh_cmd: Optional[AmCommand]) -> float:
        if self.state == AmDriveState.QUICK_STOP_ACTIVE:
            g = self.gains
            return g.vel_kp_Nm_s_per_rad * (0.0 - self.qd_f)
        if self.state != AmDriveState.OPERATION_ENABLED:
            return 0.0
        if fresh_cmd is not None:
            self.cmd = fresh_cmd
            self.have_cmd = True
        c = self.cmd
        q, qd, g, dt = self.q_meas, self.qd_f, self.gains, self.dt
        if mode == AmMode.CST:
            return c.tau_ff_Nm if self.have_cmd else 0.0
        if mode == AmMode.CURRENT:
            kt = self.ud.kt_motor_nm_per_a * self.ud.ratio * self.ud.gear_eta
            return c.iq_ref_A * kt if self.have_cmd else 0.0
        if mode == AmMode.IMPEDANCE:
            if not self.have_cmd:
                return 0.0
            return c.kp_Nm_per_rad * (c.q_ref_rad - q) + c.kd_Nm_s_per_rad * (c.qd_ref_rad_s - qd) + c.tau_ff_Nm
        if mode in (AmMode.CSP, AmMode.CSV):
            if mode == AmMode.CSP:
                q_ref = c.q_ref_rad if self.have_cmd else self.hold_q
                qd_cmd = g.pos_kp_1_per_s * (q_ref - q) + (c.qd_ref_rad_s if self.have_cmd else 0.0)
            else:
                qd_cmd = c.qd_ref_rad_s if self.have_cmd else 0.0
            qd_cmd = _clamp(qd_cmd, -self.limits.speed_rad_s, self.limits.speed_rad_s)
            err = qd_cmd - qd
            self._last_err = err
            self.integ += err * dt
            tau_ff = c.tau_ff_Nm if self.have_cmd else 0.0
            return g.vel_kp_Nm_s_per_rad * err + g.vel_ki_Nm_per_rad * self.integ + tau_ff
        return 0.0

    # ------------------------------------------------------------------ plant
    def _external_torque(self, t: float) -> float:
        f = self._ext_torque
        if f is None:
            return 0.0
        return float(f(t, self.q, self.qd)) if callable(f) else float(f)

    def _integrate(self, tau_motor: float):
        if self.opts.truth_plant is not None:
            self._integrate_truth(tau_motor)
            return
        J = self.inertia
        F = self.ud.backdrive_nm * self.opts.friction_scale
        h = self.dt / self.n_sub
        qd0 = self.qd
        acc_ext = 0.0
        for k in range(self.n_sub):
            t = (self.t_ns * 1e-9) + k * h
            ext = self._external_torque(t)
            acc_ext += ext
            tau_t = tau_motor + ext
            v = self.qd
            if v == 0.0 and abs(tau_t) <= F:
                continue
            s = (1.0 if v > 0 else -1.0) if v != 0.0 else (1.0 if tau_t > 0 else -1.0)
            v_new = v + h * (tau_t - F * s) / J
            if v != 0.0 and v * v_new < 0.0 and abs(tau_t) <= F:
                v_new = 0.0
            self.qd = v_new
            self.q += v_new * h
        ext_m = acc_ext / self.n_sub
        a = (self.qd - qd0) / self.dt
        self.truth = SimTruth(tau_out_Nm=self.opts.load_inertia_kgm2 * a - ext_m, tau_ext_Nm=ext_m,
                              tau_motor_Nm=tau_motor, qdd_rad_s2=a)

    def _integrate_truth(self, tau_cmd: float):
        """Rigid-gear plant of TruthPlant. Rotor side: J_arm w' = tau_m - f(w) - g * tau_t; output side:
        J_load w' = tau_t + tau_ext - f_bearing; g = 1/eta_f when the gear drives the load, eta_r when
        back-driven. Stick at w = 0 while tau_m stays inside the static band."""
        tp, ud = self.opts.truth_plant, self.ud
        N = ud.ratio
        iq = tau_cmd / (ud.kt_motor_nm_per_a * N * ud.gear_eta)
        Ja, JL = ud.armature_kgm2, self.opts.load_inertia_kgm2
        T = self.t_magnet
        kt = ud.kt_motor_nm_per_a * tp.kt_scale * (1.0 + tp.kt_temp_coeff_per_K * (T - tp.kt_ref_C))
        if tp.kt_sat_current_A > 0.0:
            kt *= 1.0 - tp.kt_sat_frac * (iq / tp.kt_sat_current_A) ** 2
        dT = tp.friction_ref_temp_C - T
        ct = 1.0 + tp.coulomb_temp_coeff_per_K * dT
        tc, ts = tp.coulomb_Nm * ct, tp.breakaway_Nm * ct
        b = tp.viscous_Nm_s_per_rad * math.exp(tp.viscous_temp_exp_per_K * dT)
        ef, er = tp.eta_forward, tp.eta_reverse
        h = self.dt / self.n_sub
        qd0 = self.qd
        acc_out = acc_ext = acc_m = 0.0
        for k in range(self.n_sub):
            t = (self.t_ns * 1e-9) + k * h
            ext = self._external_torque(t)
            th_m = N * self.q
            tau_m = N * kt * iq * (1.0 + tp.torque_ripple * math.sin(6.0 * th_m))
            fr = 1.0 + tp.friction_ripple * math.sin(th_m)
            v = self.qd
            acc_ext += ext
            acc_m += tau_m
            if v == 0.0:
                t0 = -ext
                lo, hi = sorted((er * t0, t0 / ef))
                band = ts * fr
                if lo - band <= tau_m <= hi + band:
                    acc_out += t0
                    continue
                s = 1.0 if tau_m > hi + band else -1.0
            else:
                s = 1.0 if v > 0.0 else -1.0
            x = v / tp.stribeck_vel_rad_s
            f = ((tc + (ts - tc) * math.exp(-x * x)) * s + b * v) * fr
            fb = tp.bearing_Nm * s
            g = 1.0 / ef
            a = (tau_m - f - g * (fb - ext)) / (Ja + g * JL)
            tau_t = JL * a - ext + fb
            if tau_t * s < 0.0:
                g = er
                a = (tau_m - f - g * (fb - ext)) / (Ja + g * JL)
                tau_t = JL * a - ext + fb
                if tau_t * s > 0.0:
                    g = 0.5 * (1.0 / ef + er)
                    a = (tau_m - f - g * (fb - ext)) / (Ja + g * JL)
                    tau_t = JL * a - ext + fb
            v_new = v + h * a
            if v != 0.0 and v * v_new < 0.0:
                v_new = 0.0
            self.qd = v_new
            self.q += v_new * h
            acc_out += tau_t - fb
        n = self.n_sub
        self.truth = SimTruth(tau_out_Nm=acc_out / n, tau_ext_Nm=acc_ext / n, tau_motor_Nm=acc_m / n,
                              qdd_rad_s2=(self.qd - qd0) / self.dt)

    # ------------------------------------------------------------------ one cycle
    def exchange(self, controlword: int, mode: AmMode, cmd: AmCommand, cmd_fresh: bool) -> AmTelemetry:
        self.t_ns += int(round(self.dt * 1e9))
        self.sample_counter = (self.sample_counter + 1) & 0xFFFF
        if mode != self.mode:
            self.integ = 0.0
            self.have_cmd = False
            self.hold_q = self.q_meas
        self.mode = mode
        was_op = self.state == AmDriveState.OPERATION_ENABLED
        authority_ok = self._authority == self.cfg_bus
        self._state_machine(controlword)
        fresh = copy.copy(cmd) if (cmd_fresh and authority_ok) else None
        tau_raw = self._control(mode, fresh)
        if self.state in (AmDriveState.OPERATION_ENABLED, AmDriveState.QUICK_STOP_ACTIVE):
            fc = self.gains.torque_cmd_filter_hz
            if fc > 0.0:
                a = 1.0 - math.exp(-TWO_PI * fc * self.dt)
                self.tau_filt += a * (tau_raw - self.tau_filt)
                tau_raw = self.tau_filt
            else:
                self.tau_filt = tau_raw
            tau = self._clamp_torque(tau_raw, self.qd_f, self.q_meas)
            self.limit_active = abs(tau_raw) > abs(tau) + 1e-9 or self.derate < 1.0
            if abs(tau_raw) > abs(tau) + 1e-9 and self.state == AmDriveState.OPERATION_ENABLED:
                # anti-windup: do not integrate into saturation
                if mode in (AmMode.CSP, AmMode.CSV) and getattr(self, "_last_err", 0.0) * tau_raw > 0:
                    self.integ -= self._last_err * self.dt
        else:
            tau = 0.0
            self.tau_filt = 0.0
            self.limit_active = False
            self.integ = 0.0
        self.tau_applied = tau
        self._budget_update(tau)
        self._thermal_and_faults(tau)
        if self.state != AmDriveState.OPERATION_ENABLED and self.state != AmDriveState.QUICK_STOP_ACTIVE:
            self.tau_applied = 0.0
            tau = 0.0
        if not self.opts.external_plant:
            self._integrate(tau)
            self._sample(self.q, self.qd)
        return self._telemetry()

    def _budget_update(self, tau: float):
        w = self.qd_f
        pos, neg = self.envelope_at(w, peak=False)
        cont = pos if tau >= 0 else neg
        if abs(tau) > cont * 1.0001 and abs(tau) > 1e-9:
            self.budget = max(0.0, self.budget - self.dt)
            if self.budget <= 0.0:
                self.lockout = True
        else:
            self.budget = min(self.opts.peak_budget_s, self.budget + U.PEAK_RECHARGE_S_PER_S * self.dt)
        if self.lockout and self.budget >= U.PEAK_REARM_S:
            self.lockout = False
        if self.lockout and self.state == AmDriveState.OPERATION_ENABLED:
            self.warn |= AM_WARN_PEAK_BUDGET_EMPTY
        else:
            self.warn &= ~AM_WARN_PEAK_BUDGET_EMPTY

    def _thermal_and_faults(self, tau: float):
        was = self.warn & AM_WARN_THERMAL_DERATE
        self.thermal.step(tau, self.dt)
        self.t_magnet += (self.ambient + 0.6 * (self.t_winding - self.ambient) - self.t_magnet) * self.dt / 600.0
        if self.thermal.x > self.x_limit:
            self.warn |= AM_WARN_THERMAL_DERATE
            if not was:
                self._event(AmEventKind.WARNING, FAULT_MOTOR_OVERTEMP)
        else:
            self.warn &= ~AM_WARN_THERMAL_DERATE
        if self.t_winding >= T_TRIP_C:
            self.raise_fault(FAULT_MOTOR_OVERTEMP, True)
        elif FAULT_MOTOR_OVERTEMP in self.faults and self.t_winding < T_TRIP_C - TRIP_HYST_K:
            self.faults[FAULT_MOTOR_OVERTEMP] = False
        # bus voltage
        kt = self.ud.kt_motor_nm_per_a * self.ud.ratio * self.ud.gear_eta
        self.iq_meas = round(tau / kt, 3)
        tp = self.opts.truth_plant
        if tp is not None:
            i = tau / kt * (1.0 + tp.i_gain_err) + tp.i_offset_A
            if tp.i_noise_A > 0.0:
                i += self._truth_rng.gauss(0.0, tp.i_noise_A)
            self.iq_meas = round(i, 3)
        ibus = abs(tau * self.qd_f) / max(self.vbus_in, 1.0)
        self.vbus = self.vbus_in - U.BUS_SOURCE_R_OHM * ibus
        self.ibus = ibus
        if not (0.75 * self.bus_nominal <= self.vbus <= 1.25 * self.bus_nominal):
            self.raise_fault(FAULT_VBUS, True)
        elif FAULT_VBUS in self.faults:
            self.faults[FAULT_VBUS] = False

    def _telemetry(self) -> AmTelemetry:
        ud = self.ud
        kt = ud.kt_motor_nm_per_a
        wm = self.qd_f * ud.ratio
        iq = self.iq_meas
        fw = 0
        for c in self.faults:
            if 0x1000 <= c < 0x1020:
                fw |= 1 << (c - 0x1000)
        sio = 0
        if not self.sto_active:
            sio |= AM_SIO_STO1 | AM_SIO_STO2 | AM_SIO_GATE_SUPPLY_PRESENT
            if self.state == AmDriveState.OPERATION_ENABLED:
                sio |= AM_SIO_ENABLE_READBACK
        t = AmTelemetry(
            t_ns=self.t_ns, sample_counter=self.sample_counter,
            sync_flags=(AM_SYNC_CAN_LOCKED if self.can_sync else AM_SYNC_DC_LOCKED) | AM_SYNC_TIMESTAMP_VALID,
            state=self.state, mode=self.mode, q_rad=self.q_meas, qd_rad_s=self.qd_f,
            q_motor_counts=self.counts, qd_motor_rad_s=wm, q_out_encoder_rad=self.enc_out, twist_rad=0.0,
            id_A=0.0, iq_A=iq, id_ref_A=0.0, iq_ref_A=self.tau_applied / (kt * ud.ratio * ud.gear_eta),
            vd_V=0.0, vq_V=kt * wm + 0.05 * iq, vbus_V=self.vbus, ibus_est_A=getattr(self, "ibus", 0.0),
            tau_est_Nm=iq * kt * ud.ratio * ud.gear_eta,
            t_winding_C=self.t_winding + self._temp_bias, t_magnet_C=self.t_magnet + self._temp_bias,
            t_power_stage_C=self.t_ps,
            t_board_C=self.ambient + 8.0 * self.thermal.x,
            fault_word=fw, error_code=self.error_code, warning_word=self.warn, safety_io=sio,
            ref_state=self.ref_state, sto_active=self.sto_active, internal_limit=self.limit_active,
            extended_valid=self.extended, statusword=self._statusword())
        if not self.extended:
            t.id_A = t.vd_V = t.vq_V = t.ibus_est_A = t.twist_rad = 0.0
            t.t_magnet_C = t.t_power_stage_C = t.t_board_C = 0.0
            t.q_motor_counts, t.qd_motor_rad_s = 0, 0.0
        return t

    @property
    def _temp_bias(self) -> float:
        tp = self.opts.truth_plant
        return tp.temp_sensor_bias_C if tp is not None else 0.0

    # ------------------------------------------------------------------ JointBackend
    def identity(self) -> AmIdentity:
        ud = self.ud
        n, d = U.ratio_fraction(ud.ratio)
        return AmIdentity(
            vendor_id=0, sku=ud.sku, revision=(1 << 16) | OD_MAJOR_SUPPORTED, serial=1000 + self.address,
            uid=("SIM-%s-%02d" % (ud.unit.replace("-", ""), self.address)).encode()[:12].ljust(12, b"\0"),
            od_version=am_version_pack(0, 1, 0),
            capabilities=(AM_CAP_OUTPUT_ABS_ENCODER | AM_CAP_WINDING_SENSOR | AM_CAP_ETHERCAT |
                          AM_CAP_CANFD_CONTROL | AM_CAP_STO | AM_CAP_MODE_CURRENT),
            cal_record_version=0, cal_record_crc32=zlib.crc32(("SIM-CAL-" + ud.unit).encode()) & 0xFFFFFFFF,
            ratio_num=n, ratio_den=d, fw_main=am_version_pack(0, 0, 0), fw_motor=am_version_pack(0, 0, 0),
            fw_boot=am_version_pack(0, 0, 0), device_name="AM %s (sim)" % ud.unit, user_label=self.label)

    def transparency(self) -> AmTransparency:
        ud = self.ud
        w0 = ud.no_load_rad_s["48"]
        stiff = backlash = math.nan
        if ud.unit == "S-S":
            stiff, backlash = 1005.0, 7260e-6
        return AmTransparency(
            rated_torque_Nm=ud.rated_nm, peak_torque_2s_Nm=ud.peak_nm, rated_speed_rad_s=ud.rated_speed_rad_s,
            no_load_speed_48V_rad_s=w0, reflected_inertia_kgm2=ud.armature_kgm2,
            backdrive_torque_Nm=ud.backdrive_nm,
            kinetic_energy_no_load_J=0.5 * ud.armature_kgm2 * w0 * w0,
            stiffness_Nm_per_rad=stiff, backlash_rad=backlash, sto_reaction_time_s=2.0e-3,
            collision_latency_s=math.nan, basis=AmDataBasis.DESIGN_TARGET)

    def fault_info(self) -> AmFaultInfo:
        fw = 0
        for c in self.faults:
            if 0x1000 <= c < 0x1020:
                fw |= 1 << (c - 0x1000)
        sio = 0 if self.sto_active else (AM_SIO_STO1 | AM_SIO_STO2 | AM_SIO_GATE_SUPPLY_PRESENT)
        return AmFaultInfo(error_code=self.error_code, fault_word=fw, first_fault=self.first_fault,
                           warning_word=self.warn, boot_block=0, safety_io=sio,
                           latched=bool(self.faults), codes=list(self.faults))

    def fault_history(self) -> List[AmFaultRecord]:
        return list(self.history)

    def get_limits(self):
        return copy.copy(self.limits), copy.copy(self.ceiling)

    def set_limits(self, lim: AmLimits) -> None:
        c = self.ceiling
        checks = [(lim.torque_pos_Nm, c.torque_pos_Nm), (lim.torque_neg_Nm, c.torque_neg_Nm),
                  (lim.current_A, c.current_A), (lim.speed_rad_s, c.speed_rad_s),
                  (lim.kp_max_Nm_per_rad, c.kp_max_Nm_per_rad), (lim.kd_max_Nm_s_per_rad, c.kd_max_Nm_s_per_rad),
                  (lim.comms_loss_ms, c.comms_loss_ms)]
        for v, cap in checks:
            if math.isnan(v) or v < 0 or v > cap * (1 + 1e-9):
                self.last_abort = 0x06090031
                raise AmError(AmStatus.AM_ERR_LIMIT, "value %g above ceiling %g" % (v, cap))
        if lim.pos_min_rad > lim.pos_max_rad:
            raise AmError(AmStatus.AM_ERR_PARAM, "pos_min above pos_max")
        self.limits = copy.copy(lim)

    def get_loop_gains(self) -> AmLoopGains:
        return copy.copy(self.gains)

    def set_loop_gains(self, g: AmLoopGains) -> None:
        c = self.gain_ceiling
        for v, cap in ((g.pos_kp_1_per_s, c.pos_kp_1_per_s), (g.vel_kp_Nm_s_per_rad, c.vel_kp_Nm_s_per_rad),
                       (g.vel_ki_Nm_per_rad, c.vel_ki_Nm_per_rad)):
            if math.isnan(v) or v < 0 or v > cap:
                raise AmError(AmStatus.AM_ERR_LIMIT, "gain %g outside envelope (max %g)" % (v, cap))
        if g.vel_filter_hz < self.min_filter_hz or g.vel_filter_hz > c.vel_filter_hz:
            raise AmError(AmStatus.AM_ERR_LIMIT, "velocity filter outside envelope")
        if g.torque_cmd_filter_hz != 0.0 and g.torque_cmd_filter_hz < self.min_filter_hz:
            raise AmError(AmStatus.AM_ERR_LIMIT, "torque filter below the minimum cut-off")
        self.gains = copy.copy(g)

    # ------------------------------------------------------------------ object dictionary (subset)
    # Entry: (struct format or "str"/"raw", getter, setter or None). Objects with a setter are
    # customer-writable; writes are refused while Operation enabled (abort 0x08000022) unless in _RW_ANY.
    def _od(self):
        ud = self.ud
        ident = self.identity()
        tr = self.transparency()

        def u32na(v, scale):
            return 0xFFFFFFFF if (isinstance(v, float) and math.isnan(v)) else int(round(v * scale))

        def pm(v, base):
            return int(round(1000.0 * v / base))

        def lim_set(field, conv, tol=0.0):
            # The drive accepts the wire representation of its ceiling: a written value within half an
            # LSB (tol) above the ceiling is stored as the ceiling itself.
            def f(raw):
                lim = copy.copy(self.limits)
                v = conv(raw)
                cap = getattr(self.ceiling, field)
                if isinstance(v, float) and cap < v <= cap + tol:
                    v = cap
                setattr(lim, field, v)
                self.set_limits(lim)
            return f

        def gain_set(field):
            def f(raw):
                g = copy.copy(self.gains)
                setattr(g, field, float(raw))
                self.set_loop_gains(g)
            return f

        def pos_get(v):
            if v == -math.inf:
                return -2 ** 31
            if v == math.inf:
                return 2 ** 31 - 1
            return int(round(v * 1e6))

        def pos_conv(raw):
            if raw <= -2 ** 31:
                return -math.inf
            if raw >= 2 ** 31 - 1:
                return math.inf
            return raw * 1e-6

        g, gc = self.gains, self.gain_ceiling
        lim, c = self.limits, self.ceiling
        rated_a = self.rated_current_A
        T = {
            (0x1008, 0): ("str", lambda: ident.device_name, None),
            (0x1010, 1): ("<I", lambda: 1, self._store_cmd),
            (0x1011, 1): ("<I", lambda: 1, self._restore_cmd),
            (0x1018, 1): ("<I", lambda: ident.vendor_id, None),
            (0x1018, 2): ("<I", lambda: int(ident.sku), None),
            (0x1018, 3): ("<I", lambda: ident.revision, None),
            (0x1018, 4): ("<I", lambda: ident.serial, None),
            (0x2000, 2): ("<H", lambda: int(ident.sku), None),
            (0x2000, 5): ("raw", lambda: ident.uid, None),
            (0x2000, 6): ("<I", lambda: ident.od_version, None),
            (0x2000, 8): ("<I", lambda: ident.capabilities, None),
            (0x2000, 9): ("raw", lambda: struct.pack("<II", ident.cal_record_version, ident.cal_record_crc32), None),
            (0x2100, 1): ("<I", lambda: ident.fw_main, None),
            (0x2100, 2): ("<I", lambda: ident.fw_motor, None),
            (0x2100, 3): ("<I", lambda: ident.fw_boot, None),
            (0x2300, 1): ("<f", lambda: c.current_A, None),
            (0x2300, 4): ("<f", lambda: lim.speed_rad_s, lim_set("speed_rad_s", float, 1e-3)),
            (0x2300, 20): ("<I", lambda: lim.comms_loss_ms, lim_set("comms_loss_ms", int)),
            (0x2401, 1): ("<h", lambda: int(round(self.t_winding * 10)), None),
            (0x2600, 1): ("<I", self._fault_word, None),
            (0x2600, 2): ("<I", lambda: self.first_fault, None),
            (0x2601, 0): ("<B", lambda: len(self.history), None),
            (0x2602, 1): ("<I", lambda: self.warn, None),
            (0x2603, 1): ("<I", lambda: 0, None),
            (0x2620, 1): ("<B", self._safety_io, None),
            (0x2620, 3): ("<I", lambda: self.sto_counter, None),
            (0x2901, 1): ("<B", lambda: int(self._authority_cfg), self.set_control_bus),
            (0x2901, 3): ("<B", lambda: int(self._authority), None),
            (0x2902, 2): ("<I", lambda: 0x1C0 + self.address // 4, None),
            (0x2902, 3): ("<B", lambda: self.address % 4, None),
            (0x2A00, 4): ("<I", lambda: int(round(lim.kp_max_Nm_per_rad * 1e3)),
                          lim_set("kp_max_Nm_per_rad", lambda r: r * 1e-3, 1e-3)),
            (0x2A00, 5): ("<I", lambda: int(round(lim.kd_max_Nm_s_per_rad * 1e3)),
                          lim_set("kd_max_Nm_s_per_rad", lambda r: r * 1e-3, 1e-3)),
            (0x2A11, 3): ("<B", lambda: 1 if self.current_mode_enabled else 0, self._set_current_mode),
            (0x2A20, 1): ("<H", lambda: 20, None),
            (0x2A20, 2): ("<H", lambda: 1, None),
            (0x2A30, 1): ("<f", lambda: g.pos_kp_1_per_s, gain_set("pos_kp_1_per_s")),
            (0x2A30, 2): ("<f", lambda: g.vel_kp_Nm_s_per_rad, gain_set("vel_kp_Nm_s_per_rad")),
            (0x2A30, 3): ("<f", lambda: g.vel_ki_Nm_per_rad, gain_set("vel_ki_Nm_per_rad")),
            (0x2A40, 1): ("<f", lambda: g.vel_filter_hz, gain_set("vel_filter_hz")),
            (0x2A40, 2): ("<f", lambda: g.torque_cmd_filter_hz, gain_set("torque_cmd_filter_hz")),
            (0x2A40, 3): ("<f", lambda: g.notch_hz, gain_set("notch_hz")),
            (0x2A40, 4): ("<f", lambda: g.notch_q, gain_set("notch_q")),
            (0x2A50, 1): ("<I", lambda: int(round(c.kp_max_Nm_per_rad * 1e3)), None),
            (0x2A50, 2): ("<I", lambda: int(round(c.kd_max_Nm_s_per_rad * 1e3)), None),
            (0x2A50, 3): ("raw", lambda: struct.pack("<ff", 0.0, gc.pos_kp_1_per_s), None),
            (0x2A50, 4): ("<f", lambda: gc.vel_kp_Nm_s_per_rad, None),
            (0x2A50, 5): ("<f", lambda: gc.vel_ki_Nm_per_rad, None),
            (0x2A50, 6): ("<f", lambda: self.min_filter_hz, None),
            (0x2B00, 1): ("<I", lambda: ident.cal_record_crc32, None),
            (0x2D00, 1): ("<H", lambda: 1, None),
            (0x2D00, 2): ("<I", lambda: u32na(tr.peak_torque_2s_Nm, 1e3), None),
            (0x2D00, 3): ("<I", lambda: u32na(tr.rated_speed_rad_s, 1e3), None),
            (0x2D00, 4): ("<I", lambda: u32na(tr.no_load_speed_48V_rad_s, 1e3), None),
            (0x2D00, 5): ("<f", lambda: tr.reflected_inertia_kgm2, None),
            (0x2D00, 6): ("<I", lambda: u32na(tr.backdrive_torque_Nm, 1e3), None),
            (0x2D00, 7): ("<f", lambda: tr.kinetic_energy_no_load_J, None),
            (0x2D00, 8): ("<f", lambda: tr.stiffness_Nm_per_rad, None),
            (0x2D00, 9): ("<I", lambda: u32na(tr.backlash_rad, 1e6), None),
            (0x2D00, 10): ("<I", lambda: u32na(tr.sto_reaction_time_s, 1e6), None),
            (0x2D00, 11): ("<I", lambda: u32na(tr.collision_latency_s, 1e6), None),
            (0x2D00, 12): ("<B", lambda: int(tr.basis), None),
            (0x2E00, 1): ("str", lambda: self.label, self._set_label),
            (0x603F, 0): ("<H", lambda: self.error_code, None),
            (0x6041, 0): ("<H", self._statusword, None),
            (0x6061, 0): ("<b", lambda: int(self.mode), None),
            (0x6064, 0): ("<i", lambda: int(round(self.q_meas * 1e6)), None),
            (0x606C, 0): ("<i", lambda: int(round(self.qd_f * 1e3)), None),
            (0x6073, 0): ("<H", lambda: pm(lim.current_A, rated_a),
                          lim_set("current_A", lambda r: r * rated_a / 1000.0, rated_a / 1000.0)),
            (0x6075, 0): ("<I", lambda: int(round(rated_a * 1e3)), None),
            (0x6076, 0): ("<I", lambda: int(round(ud.rated_nm * 1e3)), None),
            (0x6077, 0): ("<h", lambda: pm(self.tau_applied, ud.rated_nm), None),
            (0x607D, 1): ("<i", lambda: pos_get(lim.pos_min_rad), lim_set("pos_min_rad", pos_conv)),
            (0x607D, 2): ("<i", lambda: pos_get(lim.pos_max_rad), lim_set("pos_max_rad", pos_conv)),
            (0x6091, 1): ("<I", lambda: ident.ratio_num, None),
            (0x6091, 2): ("<I", lambda: ident.ratio_den, None),
            (0x60E0, 0): ("<H", lambda: pm(lim.torque_pos_Nm, ud.rated_nm),
                          lim_set("torque_pos_Nm", lambda r: r * ud.rated_nm / 1000.0, ud.rated_nm / 1000.0)),
            (0x60E1, 0): ("<H", lambda: pm(lim.torque_neg_Nm, ud.rated_nm),
                          lim_set("torque_neg_Nm", lambda r: r * ud.rated_nm / 1000.0, ud.rated_nm / 1000.0)),
        }
        for i, rec in enumerate(self.history[:32]):
            T[(0x2601, i + 1)] = ("raw", (lambda r: lambda: struct.pack(
                "<HBxQiihh", r.code, r.drive_state, r.t_ns, int(round(r.vbus_V * 1e3)), int(round(r.iq_A * 1e3)),
                int(round(r.t_winding_C * 10)), int(round(r.t_power_stage_C * 10))))(rec), None)
        return T

    _FACTORY = {0x2200, 0x2300, 0x2A50, 0x2B00}
    _RW_ANY = {(0x60E0, 0), (0x60E1, 0)}      # RW in any state (the SDK still refuses limits while enabled)

    def _fault_word(self) -> int:
        fw = 0
        for c in self.faults:
            if 0x1000 <= c < 0x1020:
                fw |= 1 << (c - 0x1000)
        return fw

    def _safety_io(self) -> int:
        if self.sto_active:
            return 0
        sio = AM_SIO_STO1 | AM_SIO_STO2 | AM_SIO_GATE_SUPPLY_PRESENT
        if self.state == AmDriveState.OPERATION_ENABLED:
            sio |= AM_SIO_ENABLE_READBACK
        return sio

    @property
    def rated_current_A(self) -> float:
        ud = self.ud
        return ud.rated_nm / (ud.kt_motor_nm_per_a * ud.ratio * ud.gear_eta)

    def _set_label(self, raw: bytes):
        if len(raw) > 31:
            raise AmError(AmStatus.AM_ERR_PARAM, "label longer than 31 characters")
        self.label = raw.decode(errors="replace")

    def _set_current_mode(self, raw):
        self.current_mode_enabled = bool(raw)

    def _store_cmd(self, raw):
        if raw != 0x65766173:
            self._abort(0x08000020, text="(write the signature 0x65766173)")
        self.store_parameters()

    def _restore_cmd(self, raw):
        if raw != 0x64616F6C:
            self._abort(0x08000020, text="(write the signature 0x64616F6C)")
        self.restore_defaults()

    def _abort(self, code: int, status: AmStatus = AmStatus.AM_ERR_SDO_ABORT, text: str = ""):
        self.last_abort = code
        raise AmError(status, "abort 0x%08X%s" % (code, (" " + text) if text else ""))

    def sdo_read(self, index: int, sub: int) -> bytes:
        T = self._od()
        key = (index, sub)
        if key not in T:
            self._abort(0x06020000 if all(k[0] != index for k in T) else 0x06090011)
        fmt, get, _ = T[key]
        v = get()
        if fmt == "str":
            return v.encode()
        if fmt == "raw":
            return bytes(v)
        return struct.pack(fmt, v)

    def sdo_write(self, index: int, sub: int, data: bytes) -> None:
        key = (index, sub)
        T = self._od()
        if key not in T:
            if index in self._FACTORY:
                self._abort(0x08000020, text="(factory object)")
            self._abort(0x06020000 if all(k[0] != index for k in T) else 0x06090011)
        fmt, _, setter = T[key]
        if setter is None:
            self._abort(0x08000020 if index in self._FACTORY else 0x06010002)
        if self.state == AmDriveState.OPERATION_ENABLED and key not in self._RW_ANY:
            self.raise_fault_warning()
            self._abort(0x08000022, text="(not allowed in this state)")
        if fmt in ("str", "raw"):
            raw = bytes(data)
        else:
            if len(data) != struct.calcsize(fmt):
                self._abort(0x06070010, text="(length does not match)")
            raw = struct.unpack(fmt, bytes(data))[0]
            if fmt == "<f" and not math.isfinite(raw):
                self._abort(0x06090030, AmStatus.AM_ERR_PARAM, "(value range)")
        try:
            setter(raw)
        except AmError as e:
            if e.status == AmStatus.AM_ERR_LIMIT:
                self.last_abort = 0x06090031
            elif e.status == AmStatus.AM_ERR_PARAM:
                self.last_abort = 0x06090030
            elif e.status == AmStatus.AM_ERR_STATE:
                self.last_abort = 0x08000022
            raise

    def raise_fault_warning(self):
        self._event(AmEventKind.WARNING, 0x4000)

    def store_parameters(self) -> None:
        self._stored = (copy.copy(self.limits), copy.copy(self.gains), self.label)

    def restore_defaults(self) -> None:
        if self.state == AmDriveState.OPERATION_ENABLED:
            raise AmError(AmStatus.AM_ERR_STATE, "not while Operation enabled")
        lbl = self.label
        self._defaults()
        self.label = ""
        self.current_mode_enabled = False
        self._stored = None
        del lbl

    def set_user_label(self, label: str) -> None:
        self.sdo_write(0x2E00, 1, label.encode())

    def get_control_bus(self):
        return int(self._authority_cfg), int(self._authority)

    def set_control_bus(self, bus: int) -> None:
        if bus not in (int(AmBus.ETHERCAT), int(AmBus.CANFD), int(AmBus.NONE)):
            raise AmError(AmStatus.AM_ERR_PARAM, "bus must be NONE, ETHERCAT or CANFD")
        if self.state not in (AmDriveState.NOT_READY_TO_SWITCH_ON, AmDriveState.SWITCH_ON_DISABLED):
            raise AmError(AmStatus.AM_ERR_STATE, "control bus can be written in Switch on disabled or lower")
        self._authority_cfg = AmBus(bus)       # takes effect at the next drive reset only

    def read_calibration(self) -> str:
        import json
        ud = self.ud
        return json.dumps({
            "schema": "am_cal_sim_v0", "unit": ud.unit, "data_basis": "design_target",
            "gear_ratio": ud.ratio, "reflected_inertia_kgm2": ud.armature_kgm2,
            "backdrive_torque_Nm": ud.backdrive_nm,
            "kt_motor_Nm_per_A": ud.kt_motor_nm_per_a, "gear_efficiency": ud.gear_eta,
            "thermal_tau_s": ud.thermal_tau_s, "thermal_tau_published": ud.thermal_tau_published,
            "motor_encoder_bits": MOTOR_BITS, "output_encoder_bits": OUTPUT_BITS}, indent=1)

    def pop_events(self):
        ev, self.pending_events = self.pending_events, []
        return ev

    def comms_lost(self) -> None:
        if self.state == AmDriveState.OPERATION_ENABLED or self.state == AmDriveState.QUICK_STOP_ACTIVE:
            self.tau_applied = 0.0
            self._set_state(AmDriveState.SWITCH_ON_DISABLED)
        self._event(AmEventKind.LINK_LOST, FAULT_COMMS_LOST)

    def last_abort_code(self) -> int:
        return self.last_abort

    # test conveniences
    @property
    def torque_applied(self) -> float:
        return self.tau_applied


class SimBackend(Backend):
    name = "sim"

    def __init__(self, units: Optional[List[str]] = None, options: Optional[List[Union[SimOptions, dict]]] = None):
        self._units = units
        self._options = options
        self.joints: List[SimJoint] = []
        self.cfg: Optional[AmSessionConfig] = None
        self.running = False

    def open(self, cfg: AmSessionConfig) -> None:
        self.cfg = cfg
        units = self._units or cfg.sim_units
        if not units:
            spec = cfg.interface.split(":", 1)[1] if ":" in cfg.interface else ""
            units = [u for u in spec.replace(";", ",").split(",") if u.strip()] or ["M"]
        opts = self._options or cfg.sim_options or []
        self.joints = []
        for i, u in enumerate(units):
            o = opts[i] if i < len(opts) else SimOptions()
            if isinstance(o, dict):
                o = SimOptions(**o)
            self.joints.append(SimJoint(u, i, cfg, o))

    def scan(self) -> List[AmDeviceInfo]:
        out = []
        for j in self.joints:
            ident = j.identity()
            out.append(AmDeviceInfo(bus=self.cfg.bus, address=j.address, vendor_id=ident.vendor_id,
                                    product_code=int(ident.sku), revision=ident.revision, serial=ident.serial,
                                    is_am_unit=True, od_compatible=True,
                                    has_control_authority=j._authority == self.cfg.bus))
        return out

    def open_joint(self, address: int) -> SimJoint:
        for j in self.joints:
            if j.address == address:
                return j
        raise AmError(AmStatus.AM_ERR_NO_DEVICE, "no unit at address %d" % address)

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False
        for j in self.joints:
            j.comms_lost()

