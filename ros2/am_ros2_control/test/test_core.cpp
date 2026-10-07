// SPDX-License-Identifier: Apache-2.0
// Unit tests of the ROS-free core against the am-sdk simulator. No ROS needed.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "am_ros2_control/am_system_core.hpp"
#include "am_sim.h"

using am_ros2_control::AmSystemCore;
using am_ros2_control::BackendKind;
using am_ros2_control::BackendSpec;
using am_ros2_control::Claims;
using am_ros2_control::ControlMode;
using am_ros2_control::JointConfig;

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { \
    ++g_checks; \
    if (!(c)) {++g_fail; std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);} \
} while (0)
#define CHECK_NEAR(a, b, tol) do { \
    ++g_checks; \
    double a_ = (a), b_ = (b); \
    if (!(std::fabs(a_ - b_) <= (tol))) { \
      ++g_fail; \
      std::printf("FAIL %s:%d: %s = %.6g, expected %.6g +- %.3g\n", __FILE__, __LINE__, #a, a_, b_, (double)(tol)); \
    } \
} while (0)

static bool contains(const std::string & s, const char * sub) {return s.find(sub) != std::string::npos;}

static bool run(AmSystemCore & c, int cycles)
{
  std::string err;
  for (int k = 0; k < cycles; ++k) {
    if (!c.write(&err) || !c.read(&err)) {
      std::printf("cycle %d: %s\n", k, err.c_str());
      return false;
    }
  }
  return true;
}

static bool switch_to(AmSystemCore & c, const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::vector<Claims> planned;
  std::string err;
  if (!c.plan_switch(start, stop, &planned, &err)) {
    std::printf("plan_switch: %s\n", err.c_str());
    return false;
  }
  if (!c.apply_switch(planned, &err)) {
    std::printf("apply_switch: %s\n", err.c_str());
    return false;
  }
  return true;
}

static JointConfig joint(const char * name, const char * unit, int address = -1, double load = 0.05)
{
  JointConfig j;
  j.name = name;
  j.unit = unit;
  j.address = address;
  j.sim_load_inertia = load;
  return j;
}

static void test_parse_backend()
{
  BackendSpec b;
  std::string err;
  CHECK(am_ros2_control::parse_backend("sim", &b, &err));
  CHECK(b.kind == BackendKind::kSim && b.sim_units.empty());
  CHECK(am_ros2_control::parse_backend(" sim: m , h_f ", &b, &err));
  CHECK(b.sim_units.size() == 2 && b.sim_units[0] == "M" && b.sim_units[1] == "H-F");
  CHECK(!am_ros2_control::parse_backend("sim:Q", &b, &err) && contains(err, "unknown unit"));
  CHECK(!am_ros2_control::parse_backend("sim:", &b, &err));
  CHECK(am_ros2_control::parse_backend("ethercat:eth0", &b, &err));
  CHECK(b.kind == BackendKind::kEtherCAT && b.interface_name == "eth0");
  CHECK(am_ros2_control::parse_backend("canfd:can0", &b, &err) && b.kind == BackendKind::kCanFd);
  CHECK(!am_ros2_control::parse_backend("serial:/dev/ttyUSB0", &b, &err) && contains(err, "not recognised"));
  CHECK(am_ros2_control::normalize_unit("s_d") == "S-D");
  CHECK(am_ros2_control::normalize_unit("X").empty());
}

static void test_mode_mapping()
{
  Claims c;
  CHECK(am_ros2_control::mode_from_claims(c) == ControlMode::kHold);
  c.effort = true;
  CHECK(am_ros2_control::mode_from_claims(c) == ControlMode::kEffort);
  c.velocity = true;
  CHECK(am_ros2_control::mode_from_claims(c) == ControlMode::kVelocity);
  c.position = true;
  CHECK(am_ros2_control::mode_from_claims(c) == ControlMode::kPosition);
  c.damping = true;
  CHECK(am_ros2_control::mode_from_claims(c) == ControlMode::kImpedance);
  CHECK(am_ros2_control::sdk_mode(ControlMode::kHold) == AM_MODE_CSP);
  CHECK(am_ros2_control::sdk_mode(ControlMode::kVelocity) == AM_MODE_CSV);
  CHECK(am_ros2_control::sdk_mode(ControlMode::kEffort) == AM_MODE_CST);
  CHECK(am_ros2_control::sdk_mode(ControlMode::kImpedance) == AM_MODE_IMPEDANCE);
}

static void test_init_errors()
{
  AmSystemCore c;
  std::string err;
  std::vector<JointConfig> j1 = {joint("j", "M")};
  CHECK(!c.init("ethercat:eth0", 1000, 1000, j1, &err));
  CHECK(contains(err, "available with evaluation units"));
  CHECK(!c.init("canfd:can0", 1000, 1000, j1, &err));
  CHECK(contains(err, "available with evaluation units") && contains(err, "CAN-FD"));
  CHECK(!c.init("sim", 50, 1000, j1, &err) && contains(err, "cycle_us"));
  CHECK(!c.init("sim", 1000, 1000, {joint("j", "")}, &err) && contains(err, "needs a joint parameter 'unit'"));
  CHECK(!c.init("sim:M", 1000, 1000, {joint("j", "M", 1)}, &err) && contains(err, "address 1"));
  CHECK(!c.init("sim:M", 1000, 1000, {joint("j", "H-X")}, &err) && contains(err, "simulates M"));
  CHECK(!c.init("sim", 1000, 1000, {joint("a", "M", 0), joint("b", "M", 0)}, &err) && contains(err, "twice"));
  CHECK(!c.init("sim", 1000, 1000, {joint("a", "M", 1)}, &err));
  CHECK(!c.configure(&err));  // init failed
  CHECK(c.init("sim:M", 1000, 1000, {joint("j", "")}, &err));
  CHECK(c.init("sim", 1000, 1000, j1, &err));
}

static void test_lifecycle_and_modes()
{
  AmSystemCore c;
  std::string err;
  CHECK(c.init("sim:M", 1000, 1000, {joint("knee", "M")}, &err));
  CHECK(c.configure(&err));
  CHECK(c.lifecycle() == am_ros2_control::Lifecycle::kInactive);
  CHECK(c.drive_state(0) != AM_DS_OPERATION_ENABLED);
  // Limits come from the SDK unit data: M peak 80 N*m, no-load 12.63 rad/s at 48 V.
  CHECK_NEAR(c.limits(0).effort_max, 80.0, 1e-3);
  CHECK_NEAR(c.limits(0).velocity_max, 12.6292, 1e-3);
  CHECK(c.limits(0).stiffness_max > 1000.0 && c.limits(0).damping_max > 10.0);
  CHECK_NEAR(c.state(0).temperature, 25.0, 1.0);

  CHECK(c.activate(&err));
  CHECK(c.drive_state(0) == AM_DS_OPERATION_ENABLED);
  CHECK(c.mode(0) == ControlMode::kHold);
  CHECK(run(c, 300));
  CHECK_NEAR(c.state(0).position, 0.0, 0.01);

  // position (CSP), as joint_trajectory_controller / forward_command_controller would claim it
  CHECK(switch_to(c, {"knee/position"}, {}));
  CHECK(c.mode(0) == ControlMode::kPosition);
  CHECK_NEAR(c.command(0).position, c.state(0).position, 1e-9);  // a new claim starts at "hold here"
  c.command(0).position = 0.3;
  CHECK(run(c, 2000));
  CHECK_NEAR(c.state(0).position, 0.3, 0.01);
  CHECK_NEAR(c.state(0).velocity, 0.0, 0.05);

  // NaN command (a controller that has not written yet) holds instead of faulting
  c.command(0).position = std::nan("");
  CHECK(run(c, 100));
  CHECK(c.drive_state(0) == AM_DS_OPERATION_ENABLED);

  // velocity (CSV)
  CHECK(switch_to(c, {"knee/velocity"}, {"knee/position"}));
  CHECK(c.mode(0) == ControlMode::kVelocity);
  c.command(0).velocity = 1.0;
  CHECK(run(c, 1000));
  CHECK_NEAR(c.state(0).velocity, 1.0, 0.1);

  // effort (CST): 5 N*m on armature 0.03 + load 0.05 kg*m^2, minus 1.4 N*m backdrive friction
  CHECK(switch_to(c, {"knee/effort"}, {"knee/velocity"}));
  CHECK(c.mode(0) == ControlMode::kEffort);
  c.command(0).effort = 0.0;
  CHECK(run(c, 1500));  // coast down
  double v0 = c.state(0).velocity;
  c.command(0).effort = 5.0;
  CHECK(run(c, 100));
  CHECK(c.state(0).velocity > v0 + 1.0);
  CHECK_NEAR(c.state(0).effort, 5.0, 1.0);  // estimated output torque
  c.command(0).effort = -5.0;
  CHECK(run(c, 200));

  // impedance: position + stiffness + damping
  CHECK(switch_to(c, {"knee/position", "knee/stiffness", "knee/damping"}, {"knee/effort"}));
  CHECK(c.mode(0) == ControlMode::kImpedance);
  CHECK_NEAR(c.command(0).stiffness, 0.05 * c.limits(0).stiffness_max, 1e-9);  // automatic default K
  c.command(0).position = 0.2;
  c.command(0).stiffness = 300.0;
  c.command(0).damping = 10.0;
  CHECK(run(c, 3000));
  CHECK_NEAR(c.state(0).position, 0.2, 0.02);
  // K above the envelope cap is clamped and counted, not an error
  c.command(0).stiffness = 1.0e9;
  CHECK(run(c, 10));
  CHECK(c.clamp_count() == 10);
  c.command(0).stiffness = 300.0;

  // impedance with stiffness only: q* is the hold position
  CHECK(switch_to(c, {}, {"knee/position", "knee/damping"}));
  CHECK(c.mode(0) == ControlMode::kImpedance && c.claims(0).stiffness && !c.claims(0).position);

  // unknown interface
  std::vector<Claims> planned;
  CHECK(!c.plan_switch({"knee/current"}, {}, &planned, &err) && contains(err, "unknown command interface"));
  // other hardware's joints are ignored
  CHECK(c.plan_switch({"other_joint/position"}, {}, &planned, &err));

  // release everything: hold
  CHECK(switch_to(c, {}, {"knee/stiffness"}));
  CHECK(c.mode(0) == ControlMode::kHold);
  CHECK(run(c, 50));

  CHECK(c.deactivate(&err));
  CHECK(c.drive_state(0) == AM_DS_READY_TO_SWITCH_ON);
  CHECK(c.lifecycle() == am_ros2_control::Lifecycle::kInactive);
  CHECK(c.write(&err) && c.read(&err));  // inactive cycling keeps the exchange alive
  CHECK(c.activate(&err));
  CHECK(c.drive_state(0) == AM_DS_OPERATION_ENABLED);
  c.cleanup();
  CHECK(c.lifecycle() == am_ros2_control::Lifecycle::kUnconfigured);
  CHECK(!c.write(&err));
  // configure again after cleanup
  CHECK(c.configure(&err) && c.activate(&err));
  CHECK(run(c, 10));
}

static void test_fault_and_sto()
{
  AmSystemCore c;
  std::string err;
  CHECK(c.init("sim:H-F", 1000, 1000, {joint("hip", "H-F")}, &err));
  CHECK(c.configure(&err) && c.activate(&err));
  CHECK(run(c, 10));
  am_sim_joint * sj = am_sim_of(c.sdk_joint(0));
  CHECK(sj != nullptr);
  CHECK(am_sim_inject_fault(sj, 0x1001) == AM_OK);
  bool failed = false;
  for (int k = 0; k < 20 && !failed; ++k) {
    failed = !c.write(&err) || !c.read(&err);
  }
  CHECK(failed && contains(err, "left Operation enabled") && contains(err, "OVERCURRENT"));
  c.emergency_stop();
  CHECK(c.lifecycle() == am_ros2_control::Lifecycle::kUnconfigured);

  CHECK(c.configure(&err) && c.activate(&err));
  sj = am_sim_of(c.sdk_joint(0));
  CHECK(am_sim_set_sto(sj, true) == AM_OK);
  failed = false;
  for (int k = 0; k < 20 && !failed; ++k) {
    failed = !c.write(&err) || !c.read(&err);
  }
  CHECK(failed && contains(err, "STO"));
  c.cleanup();
}

static void test_multi_joint()
{
  AmSystemCore c;
  std::string err;
  // backend "sim" builds the simulator from the joints' units, in address order
  CHECK(c.init("sim", 1000, 1000, {joint("knee", "M", 1), joint("hip", "H-X", 0)}, &err));
  CHECK(c.configure(&err));
  am_identity id;
  CHECK(am_get_identity(c.sdk_joint(0), &id) == AM_OK && id.sku == AM_SKU_M);
  CHECK(am_get_identity(c.sdk_joint(1), &id) == AM_OK && id.sku == AM_SKU_H_X);
  CHECK(c.limits(1).effort_max > c.limits(0).effort_max);
  CHECK(c.activate(&err));
  CHECK(switch_to(c, {"knee/position", "hip/velocity"}, {}));
  c.command(0).position = -0.2;
  c.command(1).velocity = 0.5;
  CHECK(run(c, 1500));
  CHECK_NEAR(c.state(0).position, -0.2, 0.01);
  CHECK_NEAR(c.state(1).velocity, 0.5, 0.05);
  CHECK(c.deactivate(&err));
}

int main()
{
  test_parse_backend();
  test_mode_mapping();
  test_init_errors();
  test_lifecycle_and_modes();
  test_fault_and_sto();
  test_multi_joint();
  std::printf("test_core: %d checks, %d failed\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
