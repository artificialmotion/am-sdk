# Changelog

All notable changes to am-sdk. The project follows semantic versioning; 0.x releases carry no compatibility promise.

## 0.1.0

First public release.

### C library (`c/`)
- `am_api.h`: L1 codec (`am_codec_*`) and L2 session API (`am_*`), C99, compiles warning-free as C and C++.
- Reference implementation: codec for RxPDO 0x1600/0x1601, TxPDO 0x1A00/0x1A01 and the CAN-FD compact profile; session layer over a pluggable `am_transport_ops` or the built-in simulator.
- `am_sim.h`: simulator controls (load, external torque, STO, faults, ambient, bus voltage) and a byte-level simulator transport.
- CMake build, CTest unit tests, examples `impedance_sine` and `leg_two_joints`.

### Python package (`python/`, `am_sdk`)
- Control API with the same calls as C.
- Simulator backend, MuJoCo coupling (`[mujoco]`), EtherCAT transport on pysoem (`[ecat]`), CAN-FD transport on python-can (`[can]`), and a `Transport` interface for your own master.
- Unit data for S-S, S-D, M, H-F, H-T and H-X.
- Tests, including a cross-check that runs the same command scripts on the C and Python simulators.

### Docs
- `docs/API.md`: layers, PDO byte layouts, CAN-FD profile, session model, error and versioning rules.
- `docs/object_dictionary.csv`: the object dictionary.

### Not in 0.1.0
- 20 kHz trace buffer access (`am_trace_*` return `AM_ERR_NOT_SUPPORTED`).
- Firmware update and service tools. They come with the evaluation units.
