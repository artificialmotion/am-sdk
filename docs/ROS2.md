# ROS 2

Drop AM joints into a `ros2_control` stack today, against the simulator. When evaluation units ship (Q1 2027) the hardware transport comes with them, and your URDF changes in one place: the `backend` parameter.

Everything on this page is **simulated**. The joints are the am-sdk simulator: reflected inertia, backdrive friction, the published torque-speed curves, peak budget, thermal model, CiA 402 state machine, STO and faults. Values are design targets.

ROS 2 Humble and Jazzy. Status: the plugin core is built and unit-tested on every push without ROS. The ROS packages build and test in the `ros2` CI workflow (ros:humble and ros:jazzy containers); see [Status](#status).

## Packages (`ros2/`)

| Package | What it is |
|---|---|
| `am_ros2_control` | `ros2_control` SystemInterface plugin `am_ros2_control/AmSystemInterface`, C++ on the am-sdk C library. |
| `am_description` | xacro macros for every unit (S-S, S-D, M, H-F, H-T, H-X): the joint with its limits, and the `ros2_control` block. |
| `am_ros2_demo` | One joint, `controllers.yaml`, a launch file and a command script. |
| `am_ros2_bridge` | Python-only node (rclpy) on the Python simulator, for stacks without `ros2_control`. |

## Build

```sh
source /opt/ros/humble/setup.bash            # or jazzy
git clone https://github.com/artificialmotion/am-sdk.git
cd am-sdk
rosdep install --from-paths ros2 --ignore-src -y
colcon build --base-paths ros2 --symlink-install
source install/setup.bash
```

`--base-paths ros2` keeps colcon to the ROS packages. The plugin compiles the C library from `c/` in this repository, so build from a clone of the whole repository, not from a copy of `ros2/` alone (or set `-DAM_SDK_SOURCE_DIR=/path/to/am-sdk/c`).

`am_ros2_bridge` also needs the am-sdk Python package:

```sh
pip install ./python          # Jazzy on Ubuntu 24.04: add --break-system-packages, or use a venv with --system-site-packages
```

## Run the demo

```sh
ros2 launch am_ros2_demo demo.launch.py                       # M joint, forward_position_controller
ros2 run am_ros2_demo demo_commands.py                         # 0.5 rad sine on /forward_position_controller/commands
```

Other units and controllers:

```sh
ros2 launch am_ros2_demo demo.launch.py unit:=H-X controller:=joint_trajectory_controller
ros2 run am_ros2_demo demo_commands.py --controller joint_trajectory_controller

ros2 launch am_ros2_demo demo.launch.py controller:=impedance_controller
ros2 run am_ros2_demo demo_commands.py --controller impedance_controller --stiffness 150 --damping 6
```

Or by hand:

```sh
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.4]}"
ros2 topic echo /joint_states
ros2 control list_hardware_interfaces
```

`/joint_states` carries position (rad), velocity (rad/s) and effort (the joint's estimated output torque, N m). The winding temperature is the `temperature` state interface.

## Your own robot

```xml
<xacro:include filename="$(find am_description)/urdf/am_units.urdf.xacro"/>
<xacro:include filename="$(find am_description)/urdf/am_ros2_control.xacro"/>

<xacro:am_joint unit="H-X" name="hip_pitch" parent="pelvis" child="thigh" lower="-2.0" upper="1.0" axis="0 1 0">
  <origin xyz="0 0.1 0" rpy="0 0 0"/>
</xacro:am_joint>
<xacro:am_joint unit="M" name="knee" parent="thigh" child="shank" lower="0" upper="2.4" axis="0 1 0">
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
</xacro:am_joint>

<xacro:am_ros2_control name="am_system" backend="sim">
  <joints>
    <xacro:am_ros2_control_joint name="hip_pitch" unit="H-X" sim_load_inertia="0.3"/>
    <xacro:am_ros2_control_joint name="knee" unit="M" sim_load_inertia="0.1"/>
  </joints>
</xacro:am_ros2_control>
```

`am_joint` writes a revolute joint with the unit's limits: `effort` = 2 s peak torque, `velocity` = 48 V no-load speed, `friction` = backdrive torque. `lower`/`upper` are your range of motion; the actuator has none of its own (default -pi..pi). The macros also define per-unit properties (`am_M_peak_torque`, `am_M_rated_torque`, `am_M_mass`, `am_M_armature`, ...). Actuator mass is not added to any link; add `am_<unit>_mass` to the link that carries it.

`urdf/am_units.urdf.xacro` is generated from the SDK unit data by `tools/gen_ros2_xacro.py`; CI fails if it is out of date.

## The plugin

### Interfaces

| State | Unit | Source |
|---|---|---|
| `position` | rad | output position (0x6064) |
| `velocity` | rad/s | output velocity (0x606C) |
| `effort` | N m | estimated output torque, open loop kt(T)·iq·ratio·eta (0x2805:1) |
| `temperature` | degC | winding temperature (0x2804:3) |

| Command | Unit | |
|---|---|---|
| `position` | rad | |
| `velocity` | rad/s | |
| `effort` | N m | |
| `stiffness` | N m/rad | impedance K, clamped to the drive cap |
| `damping` | N m s/rad | impedance D, clamped to the drive cap |

### Mode follows what the controllers claim

| Claimed command interfaces | Drive mode |
|---|---|
| `stiffness` or `damping` (with any others) | AM impedance: tau = K(q* - q) + D(qd* - qd) + tau_ff. q* from `position` if claimed, else the position at the switch; qd* from `velocity`; tau_ff from `effort` |
| `position` (+ `velocity`, `effort` as feed-forward) | CSP |
| `velocity` (+ `effort` as feed-forward) | CSV |
| `effort` | CST |
| nothing | CSP holding the position at the switch |

Switches are bumpless while the drive is enabled. A newly claimed interface starts at "hold here" (current position, zero velocity and effort), so a controller that has not written yet does not move the joint. A NaN command holds too.

Unclaimed `stiffness`/`damping` use the joint parameters `default_stiffness`/`default_damping`; unset, they are 5 % of the unit's K cap and 25 % of its D cap (M: 213 N m/rad, 5.7 N m s/rad).

### Lifecycle

| ros2_control | am-sdk | CiA 402 state after |
|---|---|---|
| `on_init` | parameters checked, no I/O | - |
| `on_configure` | `am_init`, `am_open_joint`, `am_start` | Switch on disabled |
| `on_activate` | `am_set_mode`, `am_enable` | Operation enabled |
| `on_deactivate` | `am_disable` | Ready to switch on |
| `on_cleanup` / `on_shutdown` | `am_deinit` | - |
| `on_error` | `am_quick_stop`, `am_deinit` | - |

`read()` returns an error when a drive leaves Operation enabled (fault, STO, quick stop), with the reason in the log; ros2_control then runs `on_error`.

### Parameters

Hardware:

| Name | Default | |
|---|---|---|
| `backend` | `sim` | `sim` (units from the joints), `sim:M,H-X` (explicit, addresses 0..n-1). `ethercat:<nic>` and `canfd:<if>` are placeholders: they fail at load with "available with evaluation units". |
| `cycle_us` | 1000 | one `write()` runs one am-sdk cycle of `cycle_us`. Keep it at 1e6 / `update_rate`; the plugin warns once if the controller period differs. |
| `enable_timeout_ms` | 1000 | CiA 402 enable walk |

Joint: `unit`, `address`, `default_stiffness`, `default_damping`, `sim_load_inertia` (kg m², the link inertia the simulated joint carries), `sim_initial_position` (rad).

The simulator has no gravity. A joint carries the inertia you give it in `sim_load_inertia` and nothing else. For contact and gravity, couple the Python simulator to MuJoCo (`am_sdk.mujoco_coupling`).

## Python bridge (no ros2_control)

```sh
ros2 launch am_ros2_bridge bridge.launch.py backend:=sim:M
ros2 topic pub --once /am_bridge/position_commands std_msgs/msg/Float64MultiArray "{data: [0.4]}"
ros2 topic pub --once /am_bridge/impedance_commands std_msgs/msg/Float64MultiArray "{data: [0.2, 150.0, 6.0]}"
ros2 service call /am_bridge/disable std_srvs/srv/Trigger
```

Publishes `joint_states` and `~/temperatures`; takes `~/position_commands`, `~/velocity_commands`, `~/effort_commands` (one value per joint) and `~/impedance_commands` ([q*, K, D] per joint); services `~/enable`, `~/disable`, `~/hold`. Parameters: `backend`, `joint_names`, `cycle_us`, `publish_rate` (100 Hz; each tick runs enough simulator cycles to keep simulated time with wall time), `auto_enable`, `sim_load_inertia`.

## Tests without ROS

```sh
cmake -S ros2/am_ros2_control -B build-ros2 -DAM_ROS2_STANDALONE=ON
cmake --build build-ros2
ctest --test-dir build-ros2 --output-on-failure      # core on the simulator + plugin on mock ROS headers

pip install -e "python[dev]" xacro
python -m pytest ros2/am_ros2_bridge/test ros2/am_description/test
```

The standalone build compiles the plugin source against small mock headers in `ros2/am_ros2_control/test/mock_ros` and runs its lifecycle. That tests the logic, not the build against real ROS headers; the `ros2` workflow does that.

## Status

| Part | Checked where |
|---|---|
| Plugin core (backend parsing, lifecycle to CiA 402, claims to mode, read/write) | unit tests on the simulator, every push: Linux gcc, and locally on MSVC and MinGW gcc |
| Plugin source on mock ROS headers | same |
| xacro macros and demo URDF | expanded with the `xacro` Python package and checked against the SDK unit data |
| Bridge core | pytest on the Python simulator |
| Plugin against real ROS 2 headers, pluginlib loading, colcon build of all four packages | `ros2` workflow, ros:humble and ros:jazzy containers (marked non-blocking until it has run green) |
| `ros2 launch` of the demo, controller spawning, the rclpy node | not run in CI; manual |

Not included yet: EtherCAT and CAN-FD transports in the plugin (they come with evaluation units), gravity in the simulator, a Gazebo plugin.
