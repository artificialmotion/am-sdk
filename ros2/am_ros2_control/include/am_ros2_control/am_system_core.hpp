// SPDX-License-Identifier: Apache-2.0
//
// am_system_core.hpp - the ROS-free core of the am_ros2_control hardware plugin.
//
// Everything the ros2_control SystemInterface does that does not need ROS lives here: backend
// parsing, the lifecycle -> CiA 402 mapping (through the am-sdk C API), command-interface claims
// -> drive mode, and the cyclic read/write. The plugin (am_system_interface.hpp) is a thin
// adapter that exports pointers into the state and command storage of this class.
//
// It compiles as C++14 against am_api.h only, so it is built and unit-tested without ROS.
//
// Pre-hardware release: the only working backend is the am-sdk simulator ("sim:<unit>").
// "ethercat" and "canfd" are accepted by the parser and refused by init() until evaluation units ship.
#ifndef AM_ROS2_CONTROL__AM_SYSTEM_CORE_HPP_
#define AM_ROS2_CONTROL__AM_SYSTEM_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "am_api.h"

namespace am_ros2_control
{

// ros2_control interface names. position/velocity/effort match hardware_interface::HW_IF_*.
extern const char * const kIfPosition;     // "position"     rad
extern const char * const kIfVelocity;     // "velocity"     rad/s
extern const char * const kIfEffort;       // "effort"       N*m (state: estimated output torque)
extern const char * const kIfTemperature;  // "temperature"  degC, winding (state only)
extern const char * const kIfStiffness;    // "stiffness"    N*m/rad   impedance K (command only)
extern const char * const kIfDamping;      // "damping"      N*m*s/rad impedance D (command only)

enum class BackendKind { kSim, kEtherCAT, kCanFd };

struct BackendSpec
{
  BackendKind kind = BackendKind::kSim;
  std::vector<std::string> sim_units;  // "sim:M,H-F" -> {"M", "H-F"}; empty for plain "sim"
  std::string interface_name;          // "ethercat:eth0" -> "eth0" (placeholder backends)
};

// Parse "sim", "sim:<unit>[,<unit>...]", "ethercat[:<nic>]", "canfd[:<if>]".
// Returns false and sets *error on a malformed string or an unknown unit.
bool parse_backend(const std::string & text, BackendSpec * out, std::string * error);

// "m", "h_f", " H-x " -> "M", "H-F", "H-X". Empty string when the unit is unknown.
std::string normalize_unit(const std::string & unit);

// The SKU code (0x1018:2) of a normalised unit name; AM_SKU_UNKNOWN when unknown.
am_sku unit_sku(const std::string & unit);

// Which drive mode runs a joint, chosen from the command interfaces that controllers claim.
enum class ControlMode
{
  kHold,       // nothing claimed: CSP holding the position captured at the switch
  kPosition,   // position claimed (velocity/effort, if claimed too, are feed-forward): CSP
  kVelocity,   // velocity claimed, no position (effort, if claimed, is feed-forward): CSV
  kEffort,     // effort only: CST
  kImpedance   // stiffness or damping claimed: AM impedance mode, tau = K(q*-q) + D(qd*-qd) + tau_ff
};
const char * control_mode_name(ControlMode m);
am_mode sdk_mode(ControlMode m);

struct Claims
{
  bool position = false, velocity = false, effort = false, stiffness = false, damping = false;
  bool any() const {return position || velocity || effort || stiffness || damping;}
};
ControlMode mode_from_claims(const Claims & c);

struct JointConfig
{
  static constexpr double kAuto = std::numeric_limits<double>::quiet_NaN();
  std::string name;
  std::string unit;                  // required with backend "sim", optional check with "sim:<units>"
  int address = -1;                  // -1: the joint's index in the list
  // Impedance K (N*m/rad) and D (N*m*s/rad) used when "stiffness"/"damping" are not claimed.
  // NaN = automatic: 5 % of the unit's K cap and 25 % of its D cap (resolved at configure).
  double default_stiffness = kAuto;
  double default_damping = kAuto;
  double sim_load_inertia = 0.0;     // simulator only: link inertia on the output, kg*m^2
  double sim_initial_position = 0.0; // simulator only: initial output angle, rad
};

struct JointState
{
  double position = 0.0, velocity = 0.0, effort = 0.0, temperature = 0.0;
};

struct JointCommand
{
  double position = 0.0, velocity = 0.0, effort = 0.0, stiffness = 0.0, damping = 0.0;
};

// Effective limits read from the drive after configure (am_get_limits).
struct JointLimitsInfo
{
  double effort_max = 0.0;     // N*m   (min of the positive and negative torque limit)
  double velocity_max = 0.0;   // rad/s
  double stiffness_max = 0.0;  // N*m/rad
  double damping_max = 0.0;    // N*m*s/rad
};

enum class Lifecycle { kUnconfigured, kInactive, kActive };

class AmSystemCore
{
public:
  AmSystemCore() = default;
  ~AmSystemCore();
  AmSystemCore(const AmSystemCore &) = delete;
  AmSystemCore & operator=(const AmSystemCore &) = delete;

  // on_init: validate the configuration. Opens nothing.
  bool init(
    const std::string & backend, uint32_t cycle_us, uint32_t enable_timeout_ms,
    const std::vector<JointConfig> & joints, std::string * error);

  // on_configure: open the session, attach the joints, start cyclic exchange.
  // Drives end in Switch on disabled (CiA 402); torque off.
  bool configure(std::string * error);

  // on_activate: select each joint's mode and walk Shutdown -> Switch on -> Enable operation.
  // Commands are reset to "hold the current position".
  bool activate(std::string * error);

  // on_deactivate: Disable operation (torque off, joint coasts); drives land in Ready to switch on.
  bool deactivate(std::string * error);

  // on_cleanup / on_shutdown: disable, stop the exchange and free the session.
  void cleanup();

  // on_error: quick stop every joint, then cleanup.
  void emergency_stop();

  // prepare_command_mode_switch: merge start/stop interface names ("joint/interface") into
  // per-joint claims. Interfaces of other hardware are ignored. False on an unknown interface.
  bool plan_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop,
    std::vector<Claims> * planned, std::string * error) const;

  // perform_command_mode_switch: apply planned claims (from plan_switch).
  bool apply_switch(const std::vector<Claims> & planned, std::string * error);

  // write(): send the commands of every joint, then run one am_cycle().
  bool write(std::string * error);
  // read(): copy the latest telemetry into the state storage. False on stale data or a drive fault.
  bool read(std::string * error);

  std::size_t size() const {return joints_.size();}
  const std::string & joint_name(std::size_t j) const {return joints_[j].cfg.name;}
  JointState & state(std::size_t j) {return joints_[j].state;}
  JointCommand & command(std::size_t j) {return joints_[j].cmd;}
  const Claims & claims(std::size_t j) const {return joints_[j].claims;}
  ControlMode mode(std::size_t j) const {return joints_[j].mode;}
  const JointLimitsInfo & limits(std::size_t j) const {return joints_[j].limits;}
  am_drive_state drive_state(std::size_t j) const {return joints_[j].drive_state;}
  Lifecycle lifecycle() const {return lifecycle_;}
  const BackendSpec & backend() const {return backend_;}
  uint32_t cycle_us() const {return cycle_us_;}
  // Commands clamped to the K/D caps since configure (the SDK refuses K/D above the cap).
  uint64_t clamp_count() const {return clamp_count_;}

  // Raw SDK handles, for tests and advanced use (simulator controls through am_sim.h).
  am_session * session() {return session_;}
  am_joint * sdk_joint(std::size_t j) {return joints_[j].handle;}

private:
  struct Joint
  {
    JointConfig cfg;
    uint16_t address = 0;
    am_joint * handle = nullptr;
    JointState state;
    JointCommand cmd;
    Claims claims;
    ControlMode mode = ControlMode::kHold;
    double hold_position = 0.0;
    JointLimitsInfo limits;
    am_drive_state drive_state = AM_DS_NOT_READY_TO_SWITCH_ON;
  };

  bool set_mode(Joint & jt, ControlMode mode, std::string * error);
  bool send_command(Joint & jt, std::string * error);
  bool refresh(Joint & jt, std::string * error);
  void reset_commands(Joint & jt);
  bool run_cycles_until_disabled(std::string * error);

  BackendSpec backend_;
  std::string interface_string_;
  uint32_t cycle_us_ = 1000;
  uint32_t enable_timeout_ms_ = 1000;
  std::vector<Joint> joints_;
  am_session * session_ = nullptr;
  Lifecycle lifecycle_ = Lifecycle::kUnconfigured;
  bool initialized_ = false;
  uint64_t clamp_count_ = 0;
};

}  // namespace am_ros2_control

#endif  // AM_ROS2_CONTROL__AM_SYSTEM_CORE_HPP_
