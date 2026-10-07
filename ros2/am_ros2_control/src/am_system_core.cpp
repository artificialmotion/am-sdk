// SPDX-License-Identifier: Apache-2.0
#include "am_ros2_control/am_system_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "am_sim.h"

namespace am_ros2_control
{

const char * const kIfPosition = "position";
const char * const kIfVelocity = "velocity";
const char * const kIfEffort = "effort";
const char * const kIfTemperature = "temperature";
const char * const kIfStiffness = "stiffness";
const char * const kIfDamping = "damping";

namespace
{

// SDK sanity ranges (am_session.c): values beyond them are refused with AM_ERR_PARAM.
const double kSanityTorque = 1.0e4;
const double kSanityRate = 1.0e4;
const double kSanityQ = 1.0e6;
// Disable takes effect in the next cycle; allow a few for the drive to report it.
const int kDisableCycles = 10;

std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, f);
  int n = std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  if (n < 0) {return std::string(f);}
  return std::string(buf);
}

void set_error(std::string * error, const std::string & msg)
{
  if (error) {*error = msg;}
}

bool sdk_fail(std::string * error, const char * what, const std::string & joint, am_status st)
{
  set_error(
    error, fmt(
      "%s failed on joint '%s': %s (%d)", what, joint.c_str(), am_status_str(st),
      static_cast<int>(st)));
  return false;
}

std::string trim(const std::string & s)
{
  std::size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) {++a;}
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) {--b;}
  return s.substr(a, b - a);
}

std::string lower(std::string s)
{
  for (std::size_t i = 0; i < s.size(); ++i) {
    s[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(s[i])));
  }
  return s;
}

double clamp(double v, double lo, double hi) {return std::min(hi, std::max(lo, v));}

const char * drive_state_name(am_drive_state s)
{
  switch (s) {
    case AM_DS_NOT_READY_TO_SWITCH_ON: return "Not ready to switch on";
    case AM_DS_SWITCH_ON_DISABLED: return "Switch on disabled";
    case AM_DS_READY_TO_SWITCH_ON: return "Ready to switch on";
    case AM_DS_SWITCHED_ON: return "Switched on";
    case AM_DS_OPERATION_ENABLED: return "Operation enabled";
    case AM_DS_QUICK_STOP_ACTIVE: return "Quick stop active";
    case AM_DS_FAULT_REACTION_ACTIVE: return "Fault reaction active";
    case AM_DS_FAULT: return "Fault";
  }
  return "unknown";
}

const char * const kUnits[] = {"S-S", "S-D", "M", "H-F", "H-T", "H-X"};
const am_sku kSkus[] = {AM_SKU_S_S, AM_SKU_S_D, AM_SKU_M, AM_SKU_H_F, AM_SKU_H_T, AM_SKU_H_X};

}  // namespace

// ------------------------------------------------------------------------------------ helpers

std::string normalize_unit(const std::string & unit)
{
  std::string u = trim(unit);
  for (std::size_t i = 0; i < u.size(); ++i) {
    char c = static_cast<char>(std::toupper(static_cast<unsigned char>(u[i])));
    u[i] = (c == '_') ? '-' : c;
  }
  for (const char * k : kUnits) {
    if (u == k) {return u;}
  }
  return std::string();
}

am_sku unit_sku(const std::string & unit)
{
  for (std::size_t i = 0; i < sizeof kUnits / sizeof kUnits[0]; ++i) {
    if (unit == kUnits[i]) {return kSkus[i];}
  }
  return AM_SKU_UNKNOWN;
}

bool parse_backend(const std::string & text, BackendSpec * out, std::string * error)
{
  BackendSpec spec;
  std::string t = trim(text);
  std::size_t colon = t.find(':');
  std::string kind = lower(trim(t.substr(0, colon)));
  std::string rest = colon == std::string::npos ? std::string() : trim(t.substr(colon + 1));
  if (kind == "sim") {
    spec.kind = BackendKind::kSim;
    if (colon != std::string::npos) {
      std::size_t pos = 0;
      while (true) {
        std::size_t comma = rest.find(',', pos);
        std::string item = rest.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
        std::string u = normalize_unit(item);
        if (u.empty()) {
          set_error(
            error, fmt(
              "backend '%s': unknown unit '%s'; units are S-S, S-D, M, H-F, H-T, H-X",
              text.c_str(), trim(item).c_str()));
          return false;
        }
        spec.sim_units.push_back(u);
        if (comma == std::string::npos) {break;}
        pos = comma + 1;
      }
    }
  } else if (kind == "ethercat" || kind == "ecat") {
    spec.kind = BackendKind::kEtherCAT;
    spec.interface_name = rest;
  } else if (kind == "canfd" || kind == "can") {
    spec.kind = BackendKind::kCanFd;
    spec.interface_name = rest;
  } else {
    set_error(
      error, fmt(
        "backend '%s' not recognised; use sim:<unit> (e.g. sim:M), ethercat:<nic> or canfd:<if>",
        text.c_str()));
    return false;
  }
  if (out) {*out = spec;}
  return true;
}

const char * control_mode_name(ControlMode m)
{
  switch (m) {
    case ControlMode::kHold: return "hold (CSP)";
    case ControlMode::kPosition: return "position (CSP)";
    case ControlMode::kVelocity: return "velocity (CSV)";
    case ControlMode::kEffort: return "effort (CST)";
    case ControlMode::kImpedance: return "impedance";
  }
  return "unknown";
}

am_mode sdk_mode(ControlMode m)
{
  switch (m) {
    case ControlMode::kHold: return AM_MODE_CSP;
    case ControlMode::kPosition: return AM_MODE_CSP;
    case ControlMode::kVelocity: return AM_MODE_CSV;
    case ControlMode::kEffort: return AM_MODE_CST;
    case ControlMode::kImpedance: return AM_MODE_IMPEDANCE;
  }
  return AM_MODE_CSP;
}

ControlMode mode_from_claims(const Claims & c)
{
  if (c.stiffness || c.damping) {return ControlMode::kImpedance;}
  if (c.position) {return ControlMode::kPosition;}
  if (c.velocity) {return ControlMode::kVelocity;}
  if (c.effort) {return ControlMode::kEffort;}
  return ControlMode::kHold;
}

// ------------------------------------------------------------------------------------ lifecycle

AmSystemCore::~AmSystemCore() {cleanup();}

bool AmSystemCore::init(
  const std::string & backend, uint32_t cycle_us, uint32_t enable_timeout_ms,
  const std::vector<JointConfig> & joints, std::string * error)
{
  initialized_ = false;
  if (!parse_backend(backend, &backend_, error)) {return false;}
  if (backend_.kind != BackendKind::kSim) {
    const char * bus = backend_.kind == BackendKind::kEtherCAT ? "EtherCAT" : "CAN-FD";
    set_error(
      error, fmt(
        "backend '%s': the %s transport is available with evaluation units (Q1 2027). "
        "This release runs against the simulator: set the hardware parameter backend to sim:<unit>, "
        "for example sim:M.", backend.c_str(), bus));
    return false;
  }
  if (cycle_us < 100 || cycle_us > 100000) {
    set_error(error, fmt("cycle_us %u outside 100..100000", static_cast<unsigned>(cycle_us)));
    return false;
  }
  if (joints.empty()) {
    set_error(error, "no joints in the ros2_control block");
    return false;
  }
  cycle_us_ = cycle_us;
  enable_timeout_ms_ = enable_timeout_ms ? enable_timeout_ms : 1000;

  // Resolve addresses and the simulated unit at each address.
  const std::size_t n = joints.size();
  std::size_t n_addr = backend_.sim_units.empty() ? n : backend_.sim_units.size();
  std::vector<std::string> unit_at(n_addr);
  std::vector<bool> used(n_addr, false);
  joints_.clear();
  for (std::size_t i = 0; i < n; ++i) {
    Joint jt;
    jt.cfg = joints[i];
    if (jt.cfg.name.empty()) {
      set_error(error, fmt("joint %u has no name", static_cast<unsigned>(i)));
      return false;
    }
    int addr = jt.cfg.address < 0 ? static_cast<int>(i) : jt.cfg.address;
    if (addr >= static_cast<int>(n_addr)) {
      set_error(
        error, fmt(
          "joint '%s': address %d but the simulator has %u joint(s) (addresses 0..%u)",
          jt.cfg.name.c_str(), addr, static_cast<unsigned>(n_addr), static_cast<unsigned>(n_addr - 1)));
      return false;
    }
    if (used[addr]) {
      set_error(error, fmt("joint '%s': address %d used twice", jt.cfg.name.c_str(), addr));
      return false;
    }
    used[addr] = true;
    jt.address = static_cast<uint16_t>(addr);
    std::string u;
    if (!jt.cfg.unit.empty()) {
      u = normalize_unit(jt.cfg.unit);
      if (u.empty()) {
        set_error(
          error, fmt(
            "joint '%s': unknown unit '%s'; units are S-S, S-D, M, H-F, H-T, H-X",
            jt.cfg.name.c_str(), jt.cfg.unit.c_str()));
        return false;
      }
      jt.cfg.unit = u;
    }
    if (backend_.sim_units.empty()) {
      if (u.empty()) {
        set_error(
          error, fmt(
            "joint '%s': backend 'sim' needs a joint parameter 'unit' (S-S, S-D, M, H-F, H-T, H-X), "
            "or name the units in the backend, e.g. sim:M", jt.cfg.name.c_str()));
        return false;
      }
      unit_at[addr] = u;
    } else {
      if (!u.empty() && u != backend_.sim_units[addr]) {
        set_error(
          error, fmt(
            "joint '%s': unit %s but the backend simulates %s at address %d", jt.cfg.name.c_str(),
            u.c_str(), backend_.sim_units[addr].c_str(), addr));
        return false;
      }
      jt.cfg.unit = backend_.sim_units[addr];
    }
    const double ks = jt.cfg.default_stiffness, kd = jt.cfg.default_damping;
    if ((!std::isnan(ks) && !(ks >= 0.0 && std::isfinite(ks))) ||
      (!std::isnan(kd) && !(kd >= 0.0 && std::isfinite(kd))))
    {
      set_error(error, fmt("joint '%s': default stiffness/damping must be finite and >= 0 (or unset for automatic)", jt.cfg.name.c_str()));
      return false;
    }
    joints_.push_back(jt);
  }
  if (backend_.sim_units.empty()) {
    for (std::size_t a = 0; a < n_addr; ++a) {
      if (!used[a]) {
        set_error(error, fmt("backend 'sim': addresses must be 0..%u; %u is unused",
          static_cast<unsigned>(n_addr - 1), static_cast<unsigned>(a)));
        return false;
      }
    }
    interface_string_ = "sim:";
    for (std::size_t a = 0; a < n_addr; ++a) {
      interface_string_ += (a ? "," : "") + unit_at[a];
    }
  } else {
    interface_string_ = "sim:";
    for (std::size_t a = 0; a < n_addr; ++a) {
      interface_string_ += (a ? "," : "") + backend_.sim_units[a];
    }
  }
  initialized_ = true;
  lifecycle_ = Lifecycle::kUnconfigured;
  return true;
}

bool AmSystemCore::configure(std::string * error)
{
  if (!initialized_) {
    set_error(error, "configure before a successful init");
    return false;
  }
  cleanup();
  am_session_config cfg = {};
  cfg.bus = AM_BUS_ETHERCAT;
  cfg.interface = interface_string_.c_str();
  cfg.cycle_us = cycle_us_;
  cfg.telemetry_extended = true;  // estimated torque and temperatures
  am_status st = am_init(&cfg, &session_);
  if (st != AM_OK) {
    session_ = nullptr;
    set_error(error, fmt("am_init('%s') failed: %s", interface_string_.c_str(), am_status_str(st)));
    return false;
  }
  for (Joint & jt : joints_) {
    st = am_open_joint(session_, jt.address, &jt.handle);
    if (st != AM_OK) {
      sdk_fail(error, "am_open_joint", jt.cfg.name, st);
      cleanup();
      return false;
    }
    am_identity id;
    if (am_get_identity(jt.handle, &id) == AM_OK && id.sku != unit_sku(jt.cfg.unit)) {
      set_error(error, fmt("joint '%s': device at address %u is not a %s", jt.cfg.name.c_str(),
        static_cast<unsigned>(jt.address), jt.cfg.unit.c_str()));
      cleanup();
      return false;
    }
    am_sim_joint * sj = am_sim_of(jt.handle);
    if (sj) {
      am_sim_options o;
      am_sim_options_default(&o);
      o.load_inertia_kgm2 = jt.cfg.sim_load_inertia;
      o.q0_rad = jt.cfg.sim_initial_position;
      st = am_sim_configure(sj, &o);
      if (st != AM_OK) {
        sdk_fail(error, "am_sim_configure", jt.cfg.name, st);
        cleanup();
        return false;
      }
    }
    am_limits eff;
    st = am_get_limits(jt.handle, &eff, nullptr);
    if (st != AM_OK) {
      sdk_fail(error, "am_get_limits", jt.cfg.name, st);
      cleanup();
      return false;
    }
    jt.limits.effort_max = std::min(eff.torque_pos_Nm, eff.torque_neg_Nm);
    jt.limits.velocity_max = eff.speed_rad_s;
    jt.limits.stiffness_max = eff.kp_max_Nm_per_rad;
    jt.limits.damping_max = eff.kd_max_Nm_s_per_rad;
    if (std::isnan(jt.cfg.default_stiffness)) {jt.cfg.default_stiffness = 0.05 * jt.limits.stiffness_max;}
    if (std::isnan(jt.cfg.default_damping)) {jt.cfg.default_damping = 0.25 * jt.limits.damping_max;}
  }
  st = am_start(session_);
  if (st != AM_OK) {
    set_error(error, fmt("am_start failed: %s", am_status_str(st)));
    cleanup();
    return false;
  }
  for (Joint & jt : joints_) {
    if (!refresh(jt, error)) {
      cleanup();
      return false;
    }
    reset_commands(jt);
  }
  clamp_count_ = 0;
  lifecycle_ = Lifecycle::kInactive;
  return true;
}

bool AmSystemCore::activate(std::string * error)
{
  if (lifecycle_ != Lifecycle::kInactive) {
    set_error(error, "activate needs the configured (inactive) state");
    return false;
  }
  for (Joint & jt : joints_) {
    if (!refresh(jt, error)) {return false;}
    jt.hold_position = jt.state.position;
    ControlMode m = mode_from_claims(jt.claims);
    am_status st = am_set_mode(jt.handle, sdk_mode(m));
    if (st != AM_OK) {return sdk_fail(error, "am_set_mode", jt.cfg.name, st);}
    jt.mode = m;
    reset_commands(jt);
    if (!send_command(jt, error)) {return false;}
    st = am_enable(jt.handle, enable_timeout_ms_);
    if (st != AM_OK) {
      am_fault_info fi;
      std::string extra;
      if (am_get_faults(jt.handle, &fi) == AM_OK && fi.fault_word) {
        for (int b = 0; b < 32; ++b) {
          if (fi.fault_word & (1u << b)) {
            extra = fmt(" (fault %s)", am_fault_name(static_cast<uint16_t>(0x1000 + b)));
            break;
          }
        }
      }
      set_error(error, fmt("am_enable failed on joint '%s': %s%s", jt.cfg.name.c_str(),
        am_status_str(st), extra.c_str()));
      return false;
    }
  }
  for (Joint & jt : joints_) {
    if (!refresh(jt, error)) {return false;}
    jt.hold_position = jt.state.position;
    reset_commands(jt);
  }
  lifecycle_ = Lifecycle::kActive;
  return true;
}

bool AmSystemCore::run_cycles_until_disabled(std::string * error)
{
  for (int k = 0; k < kDisableCycles; ++k) {
    am_status st = am_cycle(session_, nullptr);
    if (st != AM_OK) {
      set_error(error, fmt("am_cycle failed: %s", am_status_str(st)));
      return false;
    }
    bool all = true;
    for (Joint & jt : joints_) {
      am_telemetry t;
      if (am_read_telemetry(jt.handle, &t) == AM_OK) {jt.drive_state = t.state;}
      if (jt.drive_state == AM_DS_OPERATION_ENABLED) {all = false;}
    }
    if (all) {return true;}
  }
  set_error(error, "a drive is still in Operation enabled after Disable operation");
  return false;
}

bool AmSystemCore::deactivate(std::string * error)
{
  if (lifecycle_ != Lifecycle::kActive) {return true;}
  bool ok = true;
  for (Joint & jt : joints_) {
    am_status st = am_disable(jt.handle);
    if (st != AM_OK) {ok = sdk_fail(error, "am_disable", jt.cfg.name, st);}
  }
  if (ok) {ok = run_cycles_until_disabled(error);}
  for (Joint & jt : joints_) {refresh(jt, nullptr);}
  lifecycle_ = Lifecycle::kInactive;
  return ok;
}

void AmSystemCore::emergency_stop()
{
  if (session_) {
    for (Joint & jt : joints_) {
      if (jt.handle) {am_quick_stop(jt.handle);}
    }
    for (int k = 0; k < kDisableCycles; ++k) {am_cycle(session_, nullptr);}
  }
  cleanup();
}

void AmSystemCore::cleanup()
{
  if (session_) {
    am_deinit(session_);  // disables every joint, stops the exchange, frees the session
    session_ = nullptr;
  }
  for (Joint & jt : joints_) {
    jt.handle = nullptr;
    jt.drive_state = AM_DS_NOT_READY_TO_SWITCH_ON;
  }
  if (lifecycle_ != Lifecycle::kUnconfigured) {lifecycle_ = Lifecycle::kUnconfigured;}
}

// ------------------------------------------------------------------------------------ mode switch

bool AmSystemCore::plan_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop,
  std::vector<Claims> * planned, std::string * error) const
{
  std::vector<Claims> c(joints_.size());
  for (std::size_t j = 0; j < joints_.size(); ++j) {c[j] = joints_[j].claims;}
  for (int pass = 0; pass < 2; ++pass) {
    const std::vector<std::string> & names = pass == 0 ? stop : start;
    const bool value = pass == 1;
    for (const std::string & full : names) {
      std::size_t slash = full.rfind('/');
      if (slash == std::string::npos) {continue;}
      std::string jn = full.substr(0, slash), itf = full.substr(slash + 1);
      std::size_t j = 0;
      while (j < joints_.size() && joints_[j].cfg.name != jn) {++j;}
      if (j == joints_.size()) {continue;}  // another hardware component's joint
      Claims & cl = c[j];
      if (itf == kIfPosition) {cl.position = value;} else if (itf == kIfVelocity) {
        cl.velocity = value;
      } else if (itf == kIfEffort) {cl.effort = value;} else if (itf == kIfStiffness) {
        cl.stiffness = value;
      } else if (itf == kIfDamping) {cl.damping = value;} else {
        set_error(error, fmt("unknown command interface '%s'", full.c_str()));
        return false;
      }
    }
  }
  if (planned) {*planned = c;}
  return true;
}

bool AmSystemCore::apply_switch(const std::vector<Claims> & planned, std::string * error)
{
  if (planned.size() != joints_.size()) {
    set_error(error, "planned claims do not match the joint count");
    return false;
  }
  for (std::size_t j = 0; j < joints_.size(); ++j) {
    Joint & jt = joints_[j];
    const Claims old = jt.claims;
    const Claims & nw = planned[j];
    // Newly claimed interfaces start from "hold here" values, so a controller that has not
    // written yet does not move the joint.
    if (nw.position && !old.position) {jt.cmd.position = jt.state.position;}
    if (nw.velocity && !old.velocity) {jt.cmd.velocity = 0.0;}
    if (nw.effort && !old.effort) {jt.cmd.effort = 0.0;}
    if (nw.stiffness && !old.stiffness) {jt.cmd.stiffness = jt.cfg.default_stiffness;}
    if (nw.damping && !old.damping) {jt.cmd.damping = jt.cfg.default_damping;}
    jt.claims = nw;
    ControlMode m = mode_from_claims(nw);
    if (lifecycle_ == Lifecycle::kActive && m != jt.mode) {
      if (!set_mode(jt, m, error)) {return false;}
    } else if (lifecycle_ != Lifecycle::kActive) {
      jt.mode = m;  // selected at activate
    }
  }
  return true;
}

bool AmSystemCore::set_mode(Joint & jt, ControlMode mode, std::string * error)
{
  // CSP, CSV, CST and impedance switch bumplessly while Operation enabled (am_api.h).
  jt.hold_position = jt.state.position;
  am_status st = am_set_mode(jt.handle, sdk_mode(mode));
  if (st != AM_OK) {return sdk_fail(error, "am_set_mode", jt.cfg.name, st);}
  jt.mode = mode;
  return true;
}

// ------------------------------------------------------------------------------------ cyclic

void AmSystemCore::reset_commands(Joint & jt)
{
  jt.cmd.position = jt.state.position;
  jt.cmd.velocity = 0.0;
  jt.cmd.effort = 0.0;
  jt.cmd.stiffness = jt.cfg.default_stiffness;
  jt.cmd.damping = jt.cfg.default_damping;
}

bool AmSystemCore::send_command(Joint & jt, std::string * error)
{
  const JointCommand & c = jt.cmd;
  const Claims & cl = jt.claims;
  auto pick = [](bool claimed, double v, double fallback) {
      return (claimed && std::isfinite(v)) ? v : fallback;
    };
  double q = clamp(pick(cl.position, c.position, jt.hold_position), -kSanityQ, kSanityQ);
  double qd = clamp(pick(cl.velocity, c.velocity, 0.0), -kSanityRate, kSanityRate);
  double tau = clamp(pick(cl.effort, c.effort, 0.0), -kSanityTorque, kSanityTorque);
  am_status st = AM_OK;
  switch (jt.mode) {
    case ControlMode::kHold:
      st = am_set_position(jt.handle, jt.hold_position, 0.0f, 0.0f);
      break;
    case ControlMode::kPosition:
      st = am_set_position(jt.handle, q, static_cast<float>(qd), static_cast<float>(tau));
      break;
    case ControlMode::kVelocity:
      st = am_set_velocity(jt.handle, static_cast<float>(qd), static_cast<float>(tau));
      break;
    case ControlMode::kEffort:
      st = am_set_torque(jt.handle, static_cast<float>(tau));
      break;
    case ControlMode::kImpedance: {
        double k = pick(cl.stiffness, c.stiffness, jt.cfg.default_stiffness);
        double d = pick(cl.damping, c.damping, jt.cfg.default_damping);
        // The SDK refuses K/D above the envelope cap (AM_ERR_LIMIT, no silent clamp); a
        // controller can ask for anything, so clamp here and count it.
        double kc = clamp(k, 0.0, jt.limits.stiffness_max);
        double dc = clamp(d, 0.0, jt.limits.damping_max);
        if (kc != k || dc != d) {++clamp_count_;}
        am_impedance imp;
        imp.q_ref_rad = q;
        imp.qd_ref_rad_s = static_cast<float>(qd);
        // float rounding must not push the value above the float cap
        imp.kp_Nm_per_rad = std::min(static_cast<float>(kc), static_cast<float>(jt.limits.stiffness_max));
        imp.kd_Nm_s_per_rad = std::min(static_cast<float>(dc), static_cast<float>(jt.limits.damping_max));
        imp.tau_ff_Nm = static_cast<float>(tau);
        st = am_set_impedance(jt.handle, &imp);
        break;
      }
  }
  if (st != AM_OK) {return sdk_fail(error, "set command", jt.cfg.name, st);}
  return true;
}

bool AmSystemCore::write(std::string * error)
{
  if (!session_) {
    set_error(error, "write without a configured session");
    return false;
  }
  if (lifecycle_ == Lifecycle::kActive) {
    for (Joint & jt : joints_) {
      if (!send_command(jt, error)) {return false;}
    }
  }
  am_status st = am_cycle(session_, nullptr);
  if (st != AM_OK) {
    set_error(error, fmt("am_cycle failed: %s", am_status_str(st)));
    return false;
  }
  return true;
}

bool AmSystemCore::refresh(Joint & jt, std::string * error)
{
  am_telemetry t;
  am_status st = am_read_telemetry(jt.handle, &t);
  if (st != AM_OK) {return sdk_fail(error, "am_read_telemetry", jt.cfg.name, st);}
  jt.state.position = t.q_rad;
  jt.state.velocity = t.qd_rad_s;
  jt.state.effort = t.tau_est_Nm;
  jt.state.temperature = t.t_winding_C;
  jt.drive_state = t.state;
  return true;
}

bool AmSystemCore::read(std::string * error)
{
  if (!session_) {
    set_error(error, "read without a configured session");
    return false;
  }
  for (Joint & jt : joints_) {
    if (!refresh(jt, error)) {return false;}
    if (lifecycle_ == Lifecycle::kActive && jt.drive_state != AM_DS_OPERATION_ENABLED) {
      am_telemetry t;
      am_read_telemetry(jt.handle, &t);
      std::string why;
      if (t.sto_active) {why = ", STO active";}
      for (int b = 0; b < 32 && why.empty(); ++b) {
        if (t.fault_word & (1u << b)) {
          why = fmt(", fault %s", am_fault_name(static_cast<uint16_t>(0x1000 + b)));
        }
      }
      if (why.empty() && t.error_code) {
        why = fmt(", error code 0x%04X", static_cast<unsigned>(t.error_code));
      }
      set_error(error, fmt("joint '%s' left Operation enabled: %s%s", jt.cfg.name.c_str(),
        drive_state_name(jt.drive_state), why.c_str()));
      return false;
    }
  }
  return true;
}

}  // namespace am_ros2_control
