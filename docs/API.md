# Artificial Motion joint API, version 0.1.0

This document specifies the host API for Artificial Motion joints: the wire level (CiA 402 object dictionary over EtherCAT and CAN-FD), the C library (`c/include/am_api.h`) and the Python package (`python/am_sdk`). The object dictionary is in [`object_dictionary.csv`](object_dictionary.csv).

The joints are pre-hardware. Everything below runs today against the simulator in this repository. Values that depend on a unit (ratings, inertia, envelopes) are read from the unit; the simulator reports design values with data basis 1.

---

## 1. Layers

```
 Your robot controller (RT thread @ 1 kHz)              Your tools / Python / notebooks
          |                                                         |
 L2 session API  am_init / am_scan / am_open_joint / am_cycle ...   Python package am_sdk (same calls)
          |                 \
 L1 codec  am_codec_*        transport (your master, or the built-in simulator)
 (pure: pack/unpack PDOs, SI <-> OD units, CiA 402 state walk)
          |
 L0 wire   CiA 402 object dictionary over EtherCAT CoE (primary) or CANopen FD on the CAN-FD pins (selectable)
          |
 Joint firmware (Artificial Motion, closed)
```

- **L0 is the contract.** Any CiA 402 master (TwinCAT, CODESYS, acontis, SOEM or IgH based stacks) can drive a joint with the ESI file, without this SDK. The SDK is a convenience, not a gate.
- **L1 has no I/O.** If you already own an EtherCAT master, call `am_codec_pack_rx1601()` and `am_codec_unpack_tx1a00()` / `am_codec_unpack_tx1a01()` inside your own cyclic task.
- **L2 owns a bus** through `am_transport_ops` (your master or CAN driver) or runs the built-in simulator.
- **Python** has the same calls and the same simulator, plus EtherCAT (pysoem) and CAN-FD (python-can) transports. Use it for development, commissioning and identification; run 1 kHz control on hardware from C.

No customer code runs on the joint in v1. Your program is the stream of commands and parameters you send over the bus.

---

## 2. Wire level

### 2.1 Conventions

- Standard objects keep their CiA 402 type and unit. Torque objects are INT16/UINT16 in per mille of 0x6076. Position is INT32 in µrad at the output. Velocity is INT32 in mrad/s at the output.
- High-resolution and AM-specific values are in AM objects: torque INT32 in mNm (0x2A10:1, 0x2805:1), current INT32 in mA (0x2A11, 0x2811).
- All values are output-referred SI-derived integers; temperatures in 0.1 °C.
- One object dictionary for all six joint types. A type changes values, array lengths and capability bits only. A missing feature reads a defined "not available" value.
- PDOs are little-endian.

### 2.2 Access classes

| Code | Meaning |
|---|---|
| RO | read only |
| RW | read/write in any state; cyclic command objects |
| RW_U | read/write only while the drive is not armed (SDO abort 0x08000022 otherwise) |
| RO_F | written at the factory; read only for you |
| WO_CMD | edge-triggered command |

### 2.3 Control modes

| Mode | 0x6060 | Commands | Feedback | Notes |
|---|---|---|---|---|
| CSP | 8 | 0x607A + 0x60B1/0x60B2 (or 0x2A10:1) | 0x6064 | position loop in the drive; gains 0x2A30, inside 0x2A50 |
| CSV | 9 | 0x60FF + 0x60B2 | 0x606C | velocity loop in the drive |
| CST | 10 | 0x6071 (per mille) or 0x2A10:1 (mNm), + 0x60B2 | 0x6077, 0x2805:1 | open-loop torque through kt(T), ratio and efficiency; no torque sensor |
| Impedance (AM) | −1 | q_ref 0x607A, qd_ref 0x60FF, τ_ff 0x2A10:1, K 0x2A00:1, D 0x2A00:2 | as CSP | τ = K(q*−q) + D(q̇*−q̇) + τ_ff, saturated by the limit chain |
| Cyclic current (AM) | −2 | iq 0x2A11:1 (PDO), id 0x2A11:2 (SDO only) | 0x2811:1/2 | off until 0x2A11:3 = 1; every drive clamp still applies |

Mode changes while enabled: CSP, CSV, CST and impedance switch bumplessly. Entry into −2 requires a state below Operation enabled; `am_set_mode()` returns `AM_ERR_STATE` otherwise.

Limit chain: the effective torque limit is the minimum of the joint ceiling, the thermal and magnet ceilings, 0x6072, 0x60E0/0x60E1 and the input-current limiter. Statusword bit 11 reports that a limit is active.

### 2.4 PDO maps

Fixed content in v1 (no dynamic mapping). EtherCAT assignment: 0x1C12 = {0x1600} or {0x1601}; 0x1C13 = {0x1A00} or {0x1A00, 0x1A01}. The SDK uses 0x1601 and 0x1A00 (+ 0x1A01 when `telemetry_extended` is set).

**RxPDO 0x1600, CiA 402 generic (20 B)**

| Offset | Object | Type | Unit |
|---|---|---|---|
| 0 | 0x6040 Controlword | UINT16 | |
| 2 | 0x6060 Modes of operation | INT8 | |
| 3 | pad | | |
| 4 | 0x607A Target position | INT32 | µrad |
| 8 | 0x60FF Target velocity | INT32 | mrad/s |
| 12 | 0x6071 Target torque | INT16 | ‰ of 0x6076 |
| 14 | 0x60B2 Torque offset | INT16 | ‰ |
| 16 | 0x60B1 Velocity offset | INT32 | mrad/s |

`am_codec_pack_rx1600()` fills it for CSP (0x607A, velocity feed-forward in 0x60B1, torque feed-forward in 0x60B2), CSV (0x60FF, 0x60B2) and CST (0x6071). Impedance and current need 0x1601.

**RxPDO 0x1601, AM high-resolution / impedance / current (28 B)**

| Offset | Object | Type | Unit |
|---|---|---|---|
| 0 | 0x6040, 0x6060, pad | | |
| 4 | 0x607A | INT32 | µrad |
| 8 | 0x60FF | INT32 | mrad/s (CSV target; velocity reference in CSP and impedance) |
| 12 | 0x2A10:1 torque target / feed-forward | INT32 | mNm |
| 16 | 0x2A00:1 stiffness K | UINT32 | mNm/rad |
| 20 | 0x2A00:2 damping D | UINT32 | mNm·s/rad |
| 24 | 0x2A11:1 iq_ref (mode −2 only) | INT32 | mA |

**TxPDO 0x1A00, status (24 B)**

| Offset | Object | Type | Unit |
|---|---|---|---|
| 0 | 0x6041 Statusword | UINT16 | bit 14 reference valid, bit 15 STO active |
| 2 | 0x6061 Mode display | INT8 | |
| 3 | pad | | |
| 4 | 0x6064 Position actual | INT32 | µrad, absolute |
| 8 | 0x606C Velocity actual | INT32 | mrad/s |
| 12 | 0x6077 Torque actual (estimate) | INT16 | ‰ |
| 14 | 0x603F Error code | UINT16 | |
| 16 | 0x2600:1 Fault word | UINT32 | bitfield |
| 20 | 0x2620:1 Safety I/O status | UINT8 | bitfield |
| 21 | 0x2500:1 Reference state | UINT8 | enum |
| 22 | 0x2401:1 Winding temperature | INT16 | 0.1 °C |

**TxPDO 0x1A01, estimator telemetry (60 B)**

| Offset | Object | Type | Unit |
|---|---|---|---|
| 0 | 0x2800:1 Sample time | UINT64 | ns (DC system time; CAN-FD: drive time base) |
| 8 | 0x2800:2 Sample counter | UINT16 | |
| 10 | 0x2800:3 Sync status | UINT16 | bitfield |
| 12 | 0x2801:1 Motor position (multi-turn) | INT32 | 2^19 counts per motor turn |
| 16 | 0x2801:2 Motor velocity | INT32 | mrad/s (motor) |
| 20 | 0x2802:1 Output angle | UINT16 | 2^16 per turn |
| 22 | 0x2802:2 Output encoder status, reference state | UINT8 ×2 | |
| 24 | 0x2802:3 Twist residual | INT32 | µrad |
| 28 | 0x2803:1..4 id, iq, id_ref, iq_ref | INT16 ×4 | 10 mA |
| 36 | 0x2803:5..6 vd, vq | INT16 ×2 | 10 mV |
| 40 | 0x2804:1 Bus voltage | UINT16 | mV |
| 42 | 0x2804:2 Bus current estimate | INT16 | 10 mA |
| 44 | 0x2804:3..6 winding, magnet, power stage, board temperature | INT16 ×4 | 0.1 °C |
| 52 | 0x2805:1 Torque estimate | INT32 | mNm |
| 56 | 0x2805:2 Warning word | UINT16 | bitfield |
| 58 | pad | 2 B | |

Codec rule: the pack functions round half away from zero and return `AM_ERR_PARAM` when a value is NaN or does not fit its field. Nothing is clamped silently.

### 2.5 Manufacturer object index (summary)

| Range | Group | Content | Your access |
|---|---|---|---|
| 0x2000 | Identity | family, product code, UID, OD version (:6), capabilities (:8), calibration record version + CRC (:9), nominal ratio (:10) | RO |
| 0x2100 | Firmware | firmware versions | RO |
| 0x2200–0x2230 | Calibration | kt, R, L, encoder, transmission and friction models (factory); host estimator model handle 0x2230 | RO; 0x2230 RW_U |
| 0x2300–0x2302 | Limits | ceilings (factory), speed limit, lowering, comms-loss time | lower-only RW_U |
| 0x2400–0x2401 | Thermal | sensor availability, ambient reference, temperatures | RO; ambient reference RW_U |
| 0x2500–0x2504 | Reference | reference state, output and motor angles, cross-check thresholds | RO |
| 0x2600–0x2603 | Faults | fault word, first fault, clear, history, warnings, boot block | RO; clear |
| 0x2620–0x2621 | STO | safety I/O status (PDO), discrepancy timer, event counter; STO settings (factory) | RO |
| 0x2700 | Supervisor | state, epochs | RO |
| 0x2800–0x2806 | Telemetry | estimator telemetry, processing descriptor | RO; sample mode RW_U |
| 0x2810–0x2811 | Trace / current | 20 kHz trace buffer; high-resolution iq/id | RW_U / RO |
| 0x2900–0x2902 | Communication | CAN-FD node and bit timing, EtherCAT alias, control authority 0x2901, CAN-FD cyclic profile 0x2902 | RW_U |
| 0x2A00–0x2A50 | Control | impedance K/D and your caps, torque and current targets, compact scales, CSP/CSV gains, filters, AM envelope | RW / RW_U inside the envelope |
| 0x2B00 | Configuration store | CRC, record directory, commit | RO |
| 0x2C00 | Brake | reserved; not fitted in v1 | RO |
| 0x2D00 | Transparency | peak torque, rated and no-load speed, reflected inertia, backdrive torque, kinetic energy, stiffness, backlash, STO reaction time, collision latency, data basis | RO |
| 0x2E00 | User objects | label, tag, 64 B data | RW_U |
| 0x3000–0x5FFF | reserved | | |

### 2.6 Telemetry

All values are sampled in the drive once per cycle (1 kHz baseline). On EtherCAT every sample carries DC system time. On CAN-FD there is no DC; section 3.7 gives the replacement. The full 60 B estimator record at 1 kHz is practical on EtherCAT; CAN-FD carries the 16 B compact or 32 B estimator-lite feedback (3.4).

`am_telemetry.extended_valid` is false when only the status part arrived; the motor, voltage, temperature and estimator fields are then held from the last full sample.

### 2.7 Identity and transparency

0x2D00 publishes the joint's transparency data in the drive itself, with a basis field: 1 = design target, 2 = measured on the type, 3 = measured on this unit. The SDK returns it as `am_transparency` (NaN where a value is not available), so your logs show which data your model used.

### 2.8 STO status

- Statusword bit 15 = STO active. 0x2620:1 (STO1, STO2, gate-driver supply present, enable readback, discrepancy) is in TxPDO 0x1A00; on the CAN-FD compact profile it is in the status word.
- No bus can release, mask or delay STO.
- After STO release you walk Shutdown, Switch on, Enable operation again. The drive never restarts on its own.

### 2.9 State machine

CiA 402 with three AM rules:
- Disable operation lands in Ready to switch on.
- Fault reset re-qualifies the drive from reset and re-establishes the output reference from the output encoder (no homing).
- One request per drive cycle; a few cycles from reset to a valid reference.

`am_enable()` and `am_codec_next_controlword()` implement the walk.

---

## 3. CAN-FD as a selectable control bus

### 3.1 Physical layer

- Isolated CAN-FD on the joint connector. Recommended bit rates: 1 Mbit/s nominal, 5 Mbit/s data (default); 1/2 Mbit/s for long or stub-heavy harnesses; 500 kbit/s / 2 Mbit/s also accepted. Sample points 80 % nominal, 75 % data, transmitter delay compensation on at 2 and 5 Mbit/s.
- Line topology with short stubs, terminated at both ends in the harness.

### 3.2 Framing

CANopen FD for everything (NMT, SDO, EMCY, heartbeat) plus an AM compact cyclic profile. Select with 0x2902:1:

- **Profile 0, CiA PDOs:** RxPDO 0x1601 on 0x200 + node (28 B in a 32 B frame) and SYNC; TxPDO 0x1A00 on 0x180 + node and, when assigned, 0x1A01 on 0x280 + node (60 B in a 64 B frame). Any CANopen FD master can use it.
- **Profile 1/2, AM compact:** one 64 B group command frame shared by up to 4 joints, and a 16 B (profile 1) or 32 B (profile 2) feedback frame per joint (3.4). The same state machine runs behind it; there is no second one.

Object access: the SDK's CAN-FD transport (`am_sdk.backends.canfd`) uses the CiA 301 SDO protocol, expedited and segmented, on 0x600 + node (request) and 0x580 + node (response) in classic 8-byte frames.

### 3.3 Identifiers

| Frame | COB-ID | Notes |
|---|---|---|
| NMT | 0x000 | host |
| SYNC | 0x080 | profile 0 only |
| EMCY | 0x080 + node | fault and warning events |
| Group command (compact) | 0x1C0 + group, group 0..3 (0x2902:2) | uses the TPDO1 IDs of node ids 64..67, so joint node ids are 1..63 |
| Feedback / TPDO1 | 0x180 + node | |
| TPDO2 (0x1A01, profile 0) | 0x280 + node | |
| RxPDO (profile 0) | 0x200 + node | |
| SDO | 0x600 + node / 0x580 + node | |
| Heartbeat | 0x700 + node | host heartbeat consumed through 0x1016 |

### 3.4 AM compact cyclic profile (little-endian)

**Group command frame, 64 B = 4 slots × 16 B** (object 0x2A22; each drive reads its slot 0x2902:3)

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | q_ref | INT32 | µrad |
| 4 | qd_ref | INT16 | 10 mrad/s |
| 6 | τ_ff, or iq_ref in mode −2 | INT16 | 10 mNm or 10 mA |
| 8 | K | UINT16 | LSB 0x2A20:1 (default 20 mNm/rad) |
| 10 | D | UINT16 | LSB 0x2A20:2 (default 1 mNm·s/rad) |
| 12 | control | UINT8 | b0 enable (rising edge starts the walk; level holds), b1 quick stop, b2 fault reset (edge), b3 zero torque, b4–7 mode (0 keep, 1 CST, 2 CSV, 3 CSP, 4 impedance, 5 current) |
| 13 | counter | UINT8 | echoed in the feedback |
| 14 | reserved | INT16 | 0 |

- `enable` is edge-triggered after any STO, fault or comms loss.
- In CST, τ_ff is the torque target. K and D are used in impedance mode only.

**Compact feedback, 16 B** (object 0x2A23, 0x180 + node)

| Offset | Field | Type | Unit |
|---|---|---|---|
| 0 | q (0x6064) | INT32 | µrad |
| 4 | qd | INT16 | 10 mrad/s |
| 6 | τ_est | INT16 | 10 mNm |
| 8 | iq measured | INT16 | 10 mA |
| 10 | status | UINT16 | b0–2 CiA 402 state (`am_drive_state`), b3 warning, b4 STO active, b5 reference valid, b6 internal limit, b7 control authority, b8–11 mode (compact code), b12 sample time valid, b13 fault |
| 12 | echoed counter | UINT8 | |
| 13 | winding temperature | UINT8 | °C + 40 |
| 14 | bus voltage | UINT16 | 10 mV |

**Estimator-lite feedback, 32 B** (object 0x2A24): the 16 B above, then

| Offset | Field | Type | Unit |
|---|---|---|---|
| 16 | sample time | UINT32 | drive time base, ns, low 32 bits |
| 20 | motor position | INT32 | 2^19 counts per turn |
| 24 | output encoder angle | UINT16 | 2^16 per turn |
| 26 | id measured | INT16 | 10 mA |
| 28 | power-stage temperature | INT16 | 0.1 °C |
| 30 | warning word | UINT16 | |

Fault code and fault word travel by EMCY on change; on the compact profile read them with `am_get_faults()`.

### 3.5 Joints per bus

Worst-case bit stuffing, the cyclic schedule using at most 70 % of the cycle (the rest is for SDO, EMCY, heartbeat, one retransmission and arbitration jitter). `tools/canfd_budget.py` computes it.

| Scheme | 1/5 Mbit/s @1 kHz | @500 Hz | 1/2 Mbit/s @1 kHz | @500 Hz |
|---|---|---|---|---|
| Profile 0 (0x1601 + 0x1A00 + SYNC) | 3 | 6 | 1 | 3 |
| Profile 0 with 0x1A01 | 1 | 3 | 0 | 1 |
| Compact, 16 B feedback | 5 | 12 | 3 | 6 |
| Compact, 32 B feedback | 4 | 9 | 2 | 4 |

At 1 kHz a CAN-FD bus carries a limb. A full-size humanoid on CAN-FD uses one bus per limb plus torso; on EtherCAT it is one or two lines.

### 3.6 Bus selection and single command authority

1. One authority object: 0x2901:1 configured (0 none, 1 EtherCAT, 2 CAN-FD); 0x2901:3 effective.
2. A write is accepted in Switch on disabled or lower, from either bus, and takes effect at the next drive reset. There is no runtime hand-over.
3. The other bus can read and diagnose. Its command PDOs are ignored, statusword bit 9 (remote) reads 0 there, and command-object writes abort with 0x08000022. The SDK returns `AM_ERR_AUTHORITY` for commands on a bus without authority.
4. No automatic fail-over in v1. Loss of the authority bus: watchdog, zero torque, then disarm after the comms-loss time 0x2300:20.
5. STO is independent of the bus selection.

### 3.7 Time base on CAN-FD

Every drive on a bus receives the group command frame at the same instant and time-stamps it; that frame is the common sync point. The drive reports sample time in its own time base (0x2800:3 bit 2 = CAN sync locked). Send the group frame first in each cycle, before any SDO traffic. For multi-limb estimator fusion with tight time alignment, use EtherCAT.

---

## 4. Host SDK

### 4.1 Session and handle model

- `am_session` is one bus connection. `am_init(cfg)` creates it and changes no drive state. Several sessions can coexist, for example one per CAN-FD limb bus.
- `am_scan()` reads identity only.
- `am_joint` is one joint in a session. `am_open_joint(address)` checks the OD major and reads identity, transparency and codec scaling. Open joints before `am_start()`; joints are freed by `am_deinit()`.
- `cfg.transport` = your `am_transport_ops`: the SDK packs, unpacks and runs the state walk; your master moves the bytes. `cfg.interface = "sim:M,H-F"` with no transport runs the built-in simulator.

**Process image per joint**, concatenated in `am_open_joint()` order:

| Bus | Out | In |
|---|---|---|
| EtherCAT, CAN-FD profile 0 | RxPDO 0x1601 (28 B) | TxPDO 0x1A00 (24 B) + 0x1A01 (60 B) with `telemetry_extended` |
| CAN-FD compact | one group-frame slot (16 B) | 16 B or 32 B feedback |

`start()` (optional) receives the opened addresses and the per-joint sizes before cyclic exchange; `stop()` (optional) is called by `am_stop()`. A joint whose input is zero-filled after `pdo_exchange()` did not answer; its telemetry turns stale after `stale_limit_cycles`.

**Commands on a transport.** Until you call a setter after `am_enable()` or `am_set_mode()`, the SDK sends a neutral command: the latched position in CSP, zero velocity in CSV, zero torque in CST and impedance. After that it sends your latest command every cycle.

### 4.2 Function list

| Group | Functions |
|---|---|
| Lifecycle | `am_api_version`, `am_init`, `am_deinit`, `am_scan`, `am_open_joint`, `am_start`, `am_stop` |
| Cyclic [RT] | `am_cycle`, `am_set_command`, `am_set_impedance`, `am_set_position`, `am_set_velocity`, `am_set_torque`, `am_set_current`, `am_read_telemetry`, `am_disable`, `am_quick_stop` |
| State | `am_set_mode`, `am_enable` (walk with timeout) |
| Identity | `am_get_identity`, `am_get_transparency`, `am_read_calibration` |
| Faults | `am_get_faults`, `am_clear_faults`, `am_get_fault_history`, `am_fault_name`, `am_last_abort_code`, event callback |
| Configuration | `am_set_limits` / `am_get_limits` (lower-only), `am_set_loop_gains` / `am_get_loop_gains`, `am_store_parameters`, `am_restore_defaults`, `am_set_user_label`, `am_sdo_read` / `am_sdo_write` |
| Bus | `am_get_control_bus`, `am_set_control_bus` |
| Diagnostics | `am_trace_arm`, `am_trace_read` (not available in 0.1.0) |
| L1 codec | `am_codec_init`, `am_codec_pack_rx1600/1601`, `am_codec_unpack_tx1a00/1a01`, `am_codec_pack_cfd_slot`, `am_codec_unpack_cfd_fb`, `am_codec_compact_mode`, `am_codec_decode_state`, `am_codec_next_controlword` |
| Simulator (`am_sim.h`) | `am_sim_of`, `am_sim_configure`, `am_sim_set_external_torque`, `am_sim_set_sto`, `am_sim_inject_fault`, `am_sim_clear_fault_cause`, `am_sim_set_ambient`, `am_sim_set_bus_voltage`, `am_sim_set_load_inertia`, `am_sim_power_cycle`, `am_sim_get_plant`, `am_sim_set_plant`, `am_sim_set_external_plant`, `am_sim_transport_open/joint/close` |

SDK units are SI floats (positions as double). Conversion to OD integers happens only in the codec.

**Limits and their ceilings.** `am_get_limits()` returns your effective limits and the AM ceilings. Ceilings on a bus: torque from 0x2D00:2, current from 0x2300:1, speed from 0x2D00:4, K and D from 0x2A50:1/2, comms-loss time 1000 ms. A value above its ceiling returns `AM_ERR_LIMIT` and nothing is written. The limits you read back can always be written again (the check allows one LSB of the wire unit; the drive has the last word).

### 4.3 Real-time and threading rules

- [RT] functions allocate nothing, make no blocking system call and take bounded time.
- One thread per session calls `am_cycle()` once per cycle.
- The 0.1.0 reference implementation is single-threaded: `am_enable()` and `am_clear_faults()` run session cycles themselves while they wait, and the event callback runs at the end of `am_cycle()` on the cycle thread. Call the [NRT] functions from the cycle thread, or serialise them with it.
- On EtherCAT, mailbox traffic shares the frame with process data as your master schedules it. On CAN-FD, SDO traffic uses the leftover part of each cycle.
- Recommended host: PREEMPT_RT Linux, an isolated core, locked memory, a dedicated NIC or CAN adapter.
- Python runs the cycle on the host clock. A 1 kHz control loop on hardware belongs in C.

### 4.4 Fault handling

| Event | SDK behaviour |
|---|---|
| New fault | `AM_EV_FAULT` with the family code (`am_fault_name()`); setters for that joint return `AM_ERR_FAULT` until it is cleared |
| Clear | `am_clear_faults()`: controlword bit 7 edge, waits for re-qualification. The output reference comes back from the output encoder; no homing. Walk `am_enable()` again |
| STO | `AM_EV_STO_CHANGED`; `am_enable()` returns `AM_ERR_STO` while active; no automatic re-enable after release |
| Link loss | `AM_EV_LINK_LOST`; drives go to zero torque and then disarm on their own |
| Command beyond limits | the drive clamps and sets statusword bit 11; telemetry flag `internal_limit` |
| Parameter beyond the AM envelope | `AM_ERR_LIMIT` (SDO abort 0x06090031/32), nothing written |

Fault history record (0x2601:1..32, newest first, 24 B): code UINT16, drive state UINT8, pad, time UINT64 ns, bus voltage INT32 mV, iq INT32 mA, winding temperature INT16 0.1 °C, power-stage temperature INT16 0.1 °C. 0x2601:0 is the count.

### 4.5 Status codes

`am_status`: 0 = OK; −1…−17 specific errors; −99 internal. `am_status_str()` gives the text. SDO aborts keep their CiA 301 code (`am_last_abort_code()`). Python raises `AmError` with the same code in `.status`.

### 4.6 Error semantics

- No function silently changes a value you passed.
- Clamping happens only in the drive, and it is always visible (statusword bit 11).

### 4.7 Versioning

| Item | Rule |
|---|---|
| SDK API | SemVer. MAJOR = a function removed or its signature or meaning changed. MINOR = functions, enum values or macros added. PATCH = fixes. Released structs are never changed: a new field means a new struct and a new function (`..._v2`). 0.y.z carries no compatibility promise. |
| SDK ABI | Stable within a MAJOR on each published platform (Linux x86-64 and aarch64, Windows x64). |
| Object dictionary | Additive = minor; remove, retype or renumber = major; indices and fault codes are never reused. SDK MAJOR N supports OD majors N and N−1; `am_open_joint()` refuses others with `AM_ERR_VERSION`. |
| PDO maps | 0x1600, 0x1601, 0x1A00 and 0x1A01 are frozen per OD major. New maps get new indices. |
| Deprecation | A deprecated function or object stays for at least one MAJOR and 12 months, whichever is longer. |
| 1.0.0 | API and OD 1.0.0 are frozen with the first evaluation units. Until then, 0.x with change notes in CHANGELOG.md. |

### 4.8 Python package

`am_sdk` has the same function names. A function returns its output instead of filling a pointer, and raises `AmError` where C returns a negative status. Backends:

| `interface` | Backend |
|---|---|
| `sim`, `sim:M,H-F` | built-in simulator |
| `ecat:<nic>` | EtherCAT through pysoem (`pip install "am-sdk[ecat]"`) |
| `can:<python-can interface>:<channel>[@nominal/data]` | CAN-FD through python-can (`pip install "am-sdk[can]"`) |
| `AmSessionConfig(backend=TransportBackend(your_transport))` | your own `am_sdk.Transport` |

`am_sdk.SimWireTransport` is the simulator behind the transport interface. `am_sdk.mujoco_coupling.MujocoCoupling` makes a MuJoCo model the plant for the same calls (`pip install "am-sdk[mujoco]"`).

### 4.9 The simulator

Per joint, in virtual time, one drive cycle per `am_cycle()`:

- plant: reflected output inertia plus your load, Coulomb backdrive friction, optional external torque or an external physics engine;
- drive: CiA 402 state machine, the five modes, torque clamp from the published continuous and peak torque-speed curves at 48 V or 36 V, a 2 s peak budget, a first-order thermal model with derate and trip, lower-only limits, STO, fault latching;
- sensing: 19-bit motor encoder, 14-bit output encoder, current in 1 mA steps; the drive acts on the feedback of the previous cycle, as with a bus master.

The C and Python simulators are the same model. `python/tests/test_cross_check_c.py` runs the same command scripts through both and requires the same telemetry.

The simulator does not model gearbox compliance, backlash, cogging, encoder noise, bus latency or jitter. Use it to write and test controllers, not for safety design.

---

## 5. What you can change

| Area | You can change (within the AM envelope) | Readable, set by AM | Not exposed |
|---|---|---|---|
| Control law | mode; impedance K, D, τ_ff and targets each cycle; CSP/CSV gains 0x2A30; your K/D caps 0x2A00:4/5 | envelope 0x2A50 | current-loop internals |
| Limits | torque ±, current, speed, software position limits, quick-stop slope, comms-loss time: lower-only | ceilings, thermal and magnet curves, input-current limiter | protection thresholds |
| Filters and sampling | velocity feedback low-pass, torque command low-pass, notch (0x2A40, bounded); telemetry sample mode 0x2800:4; ambient reference 0x2400:3 | processing descriptor 0x2806 | current sampling |
| Bus | control authority, CAN-FD node id, bit rate, cyclic profile, group and slot, watchdog times, PDO assignment (fixed maps) | | bus stack internals |
| Calibration | host estimator handle 0x2230; read the whole calibration record | kt, R, L, encoder and transmission models | |
| User data | label, tag, 64 B data (0x2E00) | | |
| Safety | read STO status, events, counters | STO discrepancy time and test-pulse filter | the STO trip path (hardware) |

Every gain or filter write is checked against 0x2A50. A rejected write changes nothing. Your settings can only make the joint softer, slower or weaker than the AM envelope.
