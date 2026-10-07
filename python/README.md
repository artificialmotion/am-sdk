# am-sdk for Python

The Python side of the Artificial Motion host SDK. Same calls as the C library, same simulator.

```
pip install -e ".[dev]"            # from this folder
pip install -e ".[can]"            # CAN-FD through python-can
pip install -e ".[ecat]"           # EtherCAT through pysoem
pip install -e ".[mujoco]"         # MuJoCo coupling
pip install -e ".[batch]"          # numpy, for the batched actuator model
```

```python
import math
import am_sdk as am

s = am.am_init(am.AmSessionConfig(interface="sim:M"))
joint = am.am_open_joint(s, 0)
am.am_start(s)
am.am_set_mode(joint, am.AmMode.IMPEDANCE)
am.am_enable(joint, timeout_ms=1000)

for k in range(3000):
    q_ref = 0.5 * math.sin(2 * math.pi * 0.5 * k * 1e-3)
    am.am_set_impedance(joint, am.AmImpedance(q_ref_rad=q_ref, kp_Nm_per_rad=150.0, kd_Nm_s_per_rad=6.0))
    am.am_cycle(s)
    t = am.am_read_telemetry(joint)

am.am_deinit(s)
```

A negative C status becomes an `AmError` with the status in `.status`.

Interfaces:

| `interface` | Backend |
|---|---|
| `sim`, `sim:M,H-F` | built-in simulator, one joint per unit |
| `ecat:<nic>` | EtherCAT (pysoem); set `bus=AmBus.ETHERCAT` |
| `can:<python-can interface>:<channel>[@nominal/data]` | CAN-FD (python-can); set `bus=AmBus.CANFD` |

Your own master: implement `am_sdk.Transport` (process-data exchange plus object read and write) and pass
`AmSessionConfig(backend=am_sdk.TransportBackend(your_transport))`. `am_sdk.SimWireTransport` is the
simulator behind that same interface, byte for byte.

Examples are in `examples/`, tests in `tests/` (`python -m pytest`). The tests include a cross-check
against the C library when it is built (see the top-level README).

Python runs the cycle with the host clock. Use it for development, commissioning and identification.
Run 1 kHz control on hardware from C.

Apache-2.0. See the top-level README for the full picture.

Actuator model for any actuator (MuJoCo, Isaac Lab, your own simulator): `am_sdk.actuator_model`,
documented in `../docs/ACTUATOR_MODEL.md`.
