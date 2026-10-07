# am-sdk

The host API for Artificial Motion joints. Same calls in C and Python.

Write your controller today against the simulator. Moving to hardware changes one line: the interface.

Evaluation units ship Q1 2027.

## What you get

- **C library** (`c/`). C99, no dependencies, builds anywhere CMake runs. A pure codec for people who run their own EtherCAT master, and a session API on top of it.
- **Python package** (`python/`). The same calls, the same simulator, plus EtherCAT and CAN-FD backends and a MuJoCo coupling.
- **Simulator**. Every joint in the family: reflected inertia, backdrive friction, the published torque-speed curves, peak budget, thermal model, CiA 402 state machine, STO and faults. The C and Python simulators are one model, and the tests hold them to the same output.
- **Spec** (`docs/`). The API, the PDO byte layouts and the full object dictionary.
- **ROS 2** (`ros2/`). A `ros2_control` hardware plugin, URDF/xacro macros for every unit, a demo and a Python bridge node, on the simulator. See [docs/ROS2.md](docs/ROS2.md).

## Quickstart: C

```sh
cmake -S c -B c/build
cmake --build c/build
ctest --test-dir c/build
./c/build/impedance_sine
```

```c
#include "am_api.h"

am_session_config cfg = {0};
cfg.interface = "sim:M";              /* one M joint in the simulator */
cfg.cycle_us = 1000;

am_session *s;
am_joint *knee;
am_init(&cfg, &s);
am_open_joint(s, 0, &knee);
am_start(s);
am_set_mode(knee, AM_MODE_IMPEDANCE);
am_enable(knee, 1000);

for (;;) {                            /* your RT thread */
    am_impedance imp = {0};
    imp.q_ref_rad = 0.3;
    imp.kp_Nm_per_rad = 150.0f;
    imp.kd_Nm_s_per_rad = 6.0f;
    am_set_impedance(knee, &imp);
    am_cycle(s, NULL);

    am_telemetry t;
    am_read_telemetry(knee, &t);      /* q, qd, iq, torque, temperatures, status */
}
```

On hardware you keep this code and pass your EtherCAT master or CAN driver in `cfg.transport`. If you run your own master already, use the codec alone: `am_codec_pack_rx1601()` and `am_codec_unpack_tx1a00()` in your cyclic task.

## Quickstart: Python

```sh
pip install -e "python[dev]"
python python/examples/01_impedance_sine.py
```

```python
import am_sdk as am

s = am.am_init(am.AmSessionConfig(interface="sim:M"))
knee = am.am_open_joint(s, 0)
am.am_start(s)
am.am_set_mode(knee, am.AmMode.IMPEDANCE)
am.am_enable(knee, timeout_ms=1000)

for k in range(3000):
    am.am_set_impedance(knee, am.AmImpedance(q_ref_rad=0.3, kp_Nm_per_rad=150.0, kd_Nm_s_per_rad=6.0))
    am.am_cycle(s)
    t = am.am_read_telemetry(knee)
```

On hardware: `interface="ecat:eth0"` with `bus=am.AmBus.ETHERCAT`, or `interface="can:socketcan:can0"` with `bus=am.AmBus.CANFD`. Nothing else changes.

## Torque estimation and collision detection

The joints have no output torque sensor. The SDK estimates joint torque from current, position and temperature with a calibrated model of the motor, friction and gear. It also detects unexpected contact with a momentum observer. Both are open, in C (`am_est.h`) and Python (`am_sdk.estimation`), with the same numbers in both.

Every value is labelled as an estimate, with an uncertainty bound, a confidence value and a quality flag (stick, ambiguous, no temperature). You can add a UKF and a learned residual, and keep them only where they beat the baseline on your data. The collision detector is not a safety function.

```sh
python python/examples/05_torque_estimation_collision.py
```

All of this is simulated today. Accuracy targets are in [docs/ESTIMATION.md](docs/ESTIMATION.md), to be confirmed on hardware.

## Layers

| Layer | Where | What it does |
|---|---|---|
| Codec | `am_codec_*` | Pure functions. SI units in, PDO bytes out, and back. CiA 402 state walk. No I/O, no allocation. |
| Session | `am_*` | Sessions and joints, modes, enable walk, limits, faults, events. Runs over your transport or the simulator. |
| Python | `am_sdk` | The same calls in Python, with the simulator, EtherCAT, CAN-FD and MuJoCo. |

The wire contract is the CiA 402 object dictionary. Any CiA 402 master drives the joints without this SDK. The SDK makes it faster.

## Buses

- **EtherCAT**, CoE with CiA 402. The main bus. Fixed PDO maps, distributed clocks.
- **CAN-FD**, CANopen FD on the same object dictionary. Standard PDOs, or a compact profile that runs four joints per command frame.

Every joint has both. You pick which one holds control authority.

## Repository

```
c/          C library, simulator, tests, examples (CMake)
python/     Python package am_sdk, tests, examples
docs/       API.md, ESTIMATION.md, object_dictionary.csv, ROS2.md
ros2/       ROS 2 packages: am_ros2_control, am_description, am_ros2_demo, am_ros2_bridge
tools/      unit data and URDF generators, CAN-FD bus budget
```

Run everything:

```sh
cmake -S c -B c/build && cmake --build c/build && ctest --test-dir c/build
cd python && python -m pytest
```

The Python tests include the C cross-check when `c/build` exists.

## Open SDK, closed firmware

The SDK is open source under Apache-2.0. Fork it, ship it, build on it. The joint firmware is not open.

Firmware update and service tools come with the evaluation units.

## Status

Version 0.2.0 in development (0.1.0 released). The API is stable in shape and can still change before 1.0, which freezes with the first evaluation units. Changes go in [CHANGELOG.md](CHANGELOG.md).

[artificialmotion.com](https://artificialmotion.com)
