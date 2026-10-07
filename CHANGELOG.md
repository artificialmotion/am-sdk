# Changelog

All notable changes to am-sdk. The project follows semantic versioning; 0.x releases carry no compatibility promise.

## 0.2.0-dev (unreleased)

Torque estimation and collision detection without an output torque sensor. Simulated, pre-hardware: no accuracy is claimed; targets are in `docs/ESTIMATION.md`, to be confirmed on hardware.

### C library
- `am_est.h` / `src/am_est.c`: baseline current-model estimator (temperature-corrected Kt with saturation, Stribeck + viscous friction or a friction map, forward/reverse gear efficiency or a load x temperature grid, output bearing), quality flags, uncertainty bound and confidence, optional bounded dither, momentum-observer collision detector with threshold, stick scaling and hysteresis. Fixed-size structs, no allocation.
- `am_est_calibration`: calibration record am_est_cal_v1.
- Tests: `test_est`; `am_est_xcheck` runner for the C/Python estimator cross-check. `test_cpp` includes `am_est.h`.
- API version 0.2.0 (`am_api_version()`).

### Python package
- `am_sdk.estimation`: the same estimator and detector, `EstimatorCalibration` with a JSON schema, `UkfTorqueEstimator`, learned-residual hooks (`LinearFeatureResidual`, `adoption_check`), all standard library only.
- `am_sdk.estimation.pinn.PinnResidual`: optional physics-informed MLP residual (extra `[pinn]`, numpy).
- Simulator: optional `SimOptions.truth_plant` (`TruthPlant`) and per-cycle ground truth `SimJoint.truth` (`SimTruth`). The default plant is unchanged.
- `am_sdk.estimation.simharness`: runs the estimators against the simulator and logs truth, estimates and collision events.
- Example `05_torque_estimation_collision.py`; tests `test_estimation.py`, `test_cross_check_est.py`.

### Docs and CI
- `docs/ESTIMATION.md`: method, assumptions, limitations, targets.
- CI runs the estimator cross-check and example 05; the Python 3.12 job installs `[pinn]`.

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
