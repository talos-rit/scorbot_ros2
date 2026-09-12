#include "scorbot_esp_sim/controller_sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace scorbot_esp_sim
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

template <std::size_t N>
void copyString(char (&dst)[N], const std::string& src)
{
  std::strncpy(dst, src.c_str(), N - 1);
  dst[N - 1] = '\0';
}
}  // namespace

JointCalibration defaultCalibration(std::size_t index)
{
  JointCalibration c = scorbot_v1_JointCalibration_init_zero;
  c.index = static_cast<uint32_t>(index);
  c.counts_per_motor_rev = 80;
  const double d = kPi / 180.0;
  switch (index)
  {
    case 0: c.gear_ratio = 635.5f; c.soft_limit_min_rad = -155 * d; c.soft_limit_max_rad = 155 * d; c.max_velocity_rad_s = 0.35f; c.max_accel_rad_s2 = 1.0f; break;
    case 1: c.gear_ratio = 457.56f; c.soft_limit_min_rad = -35 * d; c.soft_limit_max_rad = 130 * d; c.max_velocity_rad_s = 0.45f; c.max_accel_rad_s2 = 1.0f; break;
    case 2: c.gear_ratio = 457.56f; c.soft_limit_min_rad = -130 * d; c.soft_limit_max_rad = 130 * d; c.max_velocity_rad_s = 0.45f; c.max_accel_rad_s2 = 1.0f; break;
    case 3: c.gear_ratio = 125.54f; c.soft_limit_min_rad = -130 * d; c.soft_limit_max_rad = 130 * d; c.max_velocity_rad_s = 1.4f; c.max_accel_rad_s2 = 3.0f; break;
    default: c.gear_ratio = 125.54f; c.soft_limit_min_rad = -570 * d; c.soft_limit_max_rad = 570 * d; c.max_velocity_rad_s = 1.8f; c.max_accel_rad_s2 = 3.0f; break;
  }
  c.home_offset_rad = 0.0f;
  c.home_direction = -1;
  c.current_limit_a = 2.0f;
  c.invert = false;
  c.encoder_invert = false;
  return c;
}

ControllerSim::ControllerSim(SimOptions options)
  : options_(std::move(options)), robot_type_(options_.robot_type), watchdog_ms_(options_.watchdog_ms)
{
  const std::size_t n = std::min(options_.joint_names.size(), kMaxJoints);
  joints_.resize(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    JointSim& j = joints_[i];
    j.cal = i < options_.calibration.size() ? options_.calibration[i] : defaultCalibration(i);
    j.cal.index = static_cast<uint32_t>(i);
    j.encoder_reversed = i < options_.encoder_reversed.size() && options_.encoder_reversed[i];
    const double away = -0.4 * (j.cal.home_direction >= 0 ? 1.0 : -1.0);
    j.true_position = i < options_.initial_positions.size() ? options_.initial_positions[i]
                                                             : j.cal.home_offset_rad + away;
    j.encoder_zero = j.true_position;  // reads 0 at boot
    j.limit_switch = j.switchPressed(j.true_position);
  }
  state_ = scorbot_v1_SystemState_SYSTEM_STATE_BOOT;
}

// ---- outbound helpers --------------------------------------------------------------------

Envelope ControllerSim::newEnvelope() { return scorbot_protocol::makeEnvelope(++seq_, now_us_); }

void ControllerSim::emitEvent(const scorbot_v1_Event& event)
{
  Envelope e = newEnvelope();
  scorbot_protocol::setEvent(e) = event;
  outbound_.push_back(e);
}

void ControllerSim::emitJointState()
{
  Envelope e = newEnvelope();
  auto& s = scorbot_protocol::setJointState(e, joints_.size());
  s.state = state_;
  s.fault = fault_;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const JointSim& j = joints_[i];
    const double sign = j.cal.invert ? -1.0 : 1.0;
    s.position[i] = static_cast<float>(sign * j.reportedPosition());
    s.velocity[i] = static_cast<float>(sign * j.reportedVelocity());
    s.current[i] = static_cast<float>(sign * j.current);
    s.flags[i].homed = j.homed;
    s.flags[i].limit_switch = j.limit_switch;
    s.flags[i].at_soft_limit = j.at_soft_limit;
    s.flags[i].fault = j.fault;
  }
  s.commands_received = commands_received_;
  s.commands_ignored = commands_ignored_;
  s.control_loop_period_us = static_cast<uint32_t>(last_dt_s_ * 1e6);
  s.boot_count = boot_count_;
  outbound_.push_back(e);
}

bool ControllerSim::popOutbound(Envelope& out)
{
  if (outbound_.empty())
    return false;
  out = outbound_.front();
  outbound_.pop_front();
  return true;
}

// ---- state machine -------------------------------------------------------------------------

bool ControllerSim::drivesEnabled() const
{
  return state_ == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE || state_ == scorbot_v1_SystemState_SYSTEM_STATE_HOMING;
}

bool ControllerSim::positionsTrusted() const
{
  for (const auto& j : joints_)
    if (!j.homed)
      return false;
  return true;
}

void ControllerSim::enterState(SystemState next)
{
  if (next == state_)
    return;
  state_ = next;
  scorbot_v1_Event ev = scorbot_v1_Event_init_zero;
  ev.which_body = scorbot_v1_Event_state_changed_tag;
  ev.body.state_changed = next;
  emitEvent(ev);
  if (next != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE)
    unhomed_active_ = unhomed_active_ && next == scorbot_v1_SystemState_SYSTEM_STATE_FAULT;
  if (next != scorbot_v1_SystemState_SYSTEM_STATE_HOMING)
    for (auto& j : joints_)
      j.homing = false;
  if (!drivesEnabled())
    for (auto& j : joints_)
    {
      j.mode = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
      j.velocity = 0.0;
    }
}

void ControllerSim::raiseFault(FaultCode code, int joint)
{
  if (state_ == scorbot_v1_SystemState_SYSTEM_STATE_FAULT)
    return;
  fault_ = code;
  if (joint >= 0 && static_cast<std::size_t>(joint) < joints_.size())
    joints_[static_cast<std::size_t>(joint)].fault = true;
  scorbot_v1_Event ev = scorbot_v1_Event_init_zero;
  ev.which_body = scorbot_v1_Event_fault_raised_tag;
  ev.body.fault_raised = code;
  emitEvent(ev);
  enterState(scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
}

void ControllerSim::injectFault(FaultCode code, int joint) { raiseFault(code, joint); }

void ControllerSim::reboot()
{
  ++boot_count_;
  outbound_.clear();
  for (auto& j : joints_)
  {
    j.encoder_zero = j.true_position;
    j.homed = false;
    j.fault = false;
    j.at_soft_limit = false;
    j.mode = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    j.velocity = 0.0;
    j.homing = false;
  }
  fault_ = scorbot_v1_FaultCode_FAULT_NONE;
  unhomed_active_ = false;
  watchdog_ms_ = options_.watchdog_ms;  // calibration and robot type persist (NVS); watchdog does not
  commands_received_ = commands_ignored_ = 0;
  boot_elapsed_s_ = 0.0;
  state_ = scorbot_v1_SystemState_SYSTEM_STATE_BOOT;
}

// ---- inbound ---------------------------------------------------------------------------------

void ControllerSim::handle(const Envelope& in)
{
  switch (in.which_payload)
  {
    case scorbot_v1_Envelope_joint_command_tag: onJointCommand(in.payload.joint_command); break;
    case scorbot_v1_Envelope_request_tag: onRequest(in.payload.request); break;
    default: break;  // states, responses and events from a peer are ignored
  }
}

void ControllerSim::onJointCommand(const scorbot_protocol::JointCommand& cmd)
{
  if (state_ != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE)
  {
    ++commands_ignored_;
    return;
  }
  ++commands_received_;
  since_command_s_ = 0.0;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    JointSim& j = joints_[i];
    const double sign = j.cal.invert ? -1.0 : 1.0;
    j.mode = i < cmd.mode_count ? cmd.mode[i] : scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    j.setpoint_position = i < cmd.position_count ? sign * cmd.position[i] : j.reportedPosition();
    j.setpoint_velocity = i < cmd.velocity_count ? sign * cmd.velocity[i] : 0.0;
  }
}

void ControllerSim::onRequest(const Request& req)
{
  // The response envelope gets its seq only once the request has been processed, so the
  // events a request causes (state_changed on Enable, for example) are numbered and
  // sent before the response that acknowledges it. The peer then sees no seq gaps.
  Response resp = scorbot_v1_Response_init_zero;
  std::string msg;
  Result result = scorbot_v1_Result_RESULT_OK;

  switch (req.which_body)
  {
    case scorbot_v1_Request_ping_tag:
      resp.which_body = scorbot_v1_Response_pong_tag;
      resp.body.pong = req.body.ping.nonce;
      break;

    case scorbot_v1_Request_get_info_tag:
    {
      resp.which_body = scorbot_v1_Response_get_info_tag;
      auto& info = resp.body.get_info;
      info = scorbot_v1_GetInfoResponse_init_zero;
      info.protocol_major = scorbot_protocol::kProtocolMajor;
      info.protocol_minor = scorbot_protocol::kProtocolMinor;
      copyString(info.firmware_version, options_.firmware_version);
      copyString(info.robot_type, robot_type_);
      info.joint_names_count = static_cast<pb_size_t>(joints_.size());
      for (std::size_t i = 0; i < joints_.size(); ++i)
        copyString(info.joint_names[i], options_.joint_names[i]);
      info.boot_count = boot_count_;
      info.watchdog_ms = watchdog_ms_;
      info.calibration_count = static_cast<pb_size_t>(joints_.size());
      for (std::size_t i = 0; i < joints_.size(); ++i)
        info.calibration[i] = joints_[i].cal;
      break;
    }

    case scorbot_v1_Request_set_calibration_tag:
      if (!req.body.set_calibration.has_joint)
      {
        result = scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
        msg = "missing joint calibration";
      }
      else
        result = doSetCalibration(req.body.set_calibration.joint, msg);
      break;

    case scorbot_v1_Request_set_robot_type_tag:
      robot_type_ = req.body.set_robot_type.robot_type;
      break;

    case scorbot_v1_Request_set_watchdog_tag:
      watchdog_ms_ = req.body.set_watchdog.watchdog_ms;
      break;

    case scorbot_v1_Request_home_tag:
      result = doHome(req.body.home.joint_mask, req.body.home.timeout_ms, msg);
      break;

    case scorbot_v1_Request_enable_tag:
      result = doEnable(req.body.enable.allow_unhomed, msg);
      break;

    case scorbot_v1_Request_disable_tag:
      result = doDisable();
      break;

    case scorbot_v1_Request_clear_fault_tag:
      result = doClearFault();
      break;

    default:
      result = scorbot_v1_Result_RESULT_UNSUPPORTED;
      msg = "unknown request";
      break;
  }

  Envelope e = newEnvelope();
  Response& out = scorbot_protocol::setResponse(e, req.request_id, result);
  out.which_body = resp.which_body;
  out.body = resp.body;
  copyString(out.message, msg);
  outbound_.push_back(e);
}

Result ControllerSim::doSetCalibration(const JointCalibration& cal, std::string& msg)
{
  if (cal.index >= joints_.size())
  {
    msg = "joint index out of range";
    return scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
  }
  if (state_ == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE || state_ == scorbot_v1_SystemState_SYSTEM_STATE_HOMING)
  {
    msg = "cannot change calibration while moving";
    return scorbot_v1_Result_RESULT_INVALID_STATE;
  }
  if (cal.counts_per_motor_rev <= 0 || cal.gear_ratio <= 0.0f || cal.max_velocity_rad_s <= 0.0f ||
      cal.max_accel_rad_s2 <= 0.0f || cal.soft_limit_min_rad >= cal.soft_limit_max_rad ||
      (cal.home_direction != 1 && cal.home_direction != -1))
  {
    msg = "invalid calibration values";
    return scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
  }
  JointSim& j = joints_[cal.index];
  j.cal = cal;
  j.cal.index = cal.index;
  return scorbot_v1_Result_RESULT_OK;
}

Result ControllerSim::doHome(uint32_t mask, uint32_t timeout_ms, std::string& msg)
{
  switch (state_)
  {
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING: msg = "homing in progress"; return scorbot_v1_Result_RESULT_BUSY;
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE: msg = "disable before homing"; return scorbot_v1_Result_RESULT_INVALID_STATE;
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT: msg = "clear the fault first"; return scorbot_v1_Result_RESULT_INVALID_STATE;
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT: msg = "booting"; return scorbot_v1_Result_RESULT_BUSY;
    default: break;
  }
  const uint32_t all = (1u << joints_.size()) - 1u;
  if (mask == 0)
    mask = all;
  if (mask & ~all)
  {
    msg = "joint mask names a joint that does not exist";
    return scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
  }
  homing_mask_ = mask;
  homed_progress_mask_ = 0;
  homing_elapsed_s_ = 0.0;
  homing_timeout_s_ = (timeout_ms ? timeout_ms : options_.default_home_timeout_ms) / 1000.0;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    JointSim& j = joints_[i];
    if (!(mask & (1u << i)))
      continue;
    j.homing = true;
    j.homed = false;
    j.homing_direction = j.cal.home_direction >= 0 ? 1 : -1;
    j.homing_reversed = false;
    j.homing_passed = false;
    // Already sitting on the switch (parked there by the last homing): back off until it
    // releases and come back, so the encoder is latched on the edge, not wherever we are.
    j.homing_backoff = j.limit_switch;
    j.homing_released = false;
    if (j.homing_backoff)
      j.homing_direction = -j.homing_direction;
  }
  enterState(scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
  return scorbot_v1_Result_RESULT_OK;
}

Result ControllerSim::doEnable(bool allow_unhomed, std::string& msg)
{
  switch (state_)
  {
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE: return scorbot_v1_Result_RESULT_OK;
    case scorbot_v1_SystemState_SYSTEM_STATE_READY:
      unhomed_active_ = false;
      break;
    case scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED:
      if (!allow_unhomed)
      {
        msg = "not homed";
        return scorbot_v1_Result_RESULT_INVALID_STATE;
      }
      unhomed_active_ = true;
      break;
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING: msg = "homing in progress"; return scorbot_v1_Result_RESULT_BUSY;
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT: msg = "clear the fault first"; return scorbot_v1_Result_RESULT_INVALID_STATE;
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT: msg = "booting"; return scorbot_v1_Result_RESULT_BUSY;
  }
  for (auto& j : joints_)
  {
    j.mode = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    j.setpoint_position = j.reportedPosition();
    j.setpoint_velocity = 0.0;
  }
  since_command_s_ = 0.0;
  enterState(scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  return scorbot_v1_Result_RESULT_OK;
}

Result ControllerSim::doDisable()
{
  switch (state_)
  {
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE:
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING:
      enterState(positionsTrusted() ? scorbot_v1_SystemState_SYSTEM_STATE_READY
                                    : scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
      break;
    default: break;  // already stopped
  }
  return scorbot_v1_Result_RESULT_OK;
}

Result ControllerSim::doClearFault()
{
  if (state_ != scorbot_v1_SystemState_SYSTEM_STATE_FAULT)
    return scorbot_v1_Result_RESULT_OK;
  bool trusted = positionsTrusted();
  switch (fault_)
  {
    case scorbot_v1_FaultCode_FAULT_ENCODER:
    case scorbot_v1_FaultCode_FAULT_DRIVER:
    case scorbot_v1_FaultCode_FAULT_STALL:
    case scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT:
    case scorbot_v1_FaultCode_FAULT_INTERNAL:
      trusted = false;
      break;
    default: break;
  }
  if (!trusted)
    for (auto& j : joints_)
      j.homed = false;
  for (auto& j : joints_)
    j.fault = false;
  fault_ = scorbot_v1_FaultCode_FAULT_NONE;
  unhomed_active_ = false;
  enterState(trusted ? scorbot_v1_SystemState_SYSTEM_STATE_READY : scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
  return scorbot_v1_Result_RESULT_OK;
}

// ---- time ----------------------------------------------------------------------------------

void ControllerSim::tick(double dt)
{
  if (dt <= 0.0)
    return;
  last_dt_s_ = dt;
  now_us_ += static_cast<uint64_t>(dt * 1e6);

  if (state_ == scorbot_v1_SystemState_SYSTEM_STATE_BOOT)
  {
    boot_elapsed_s_ += dt;
    if (boot_elapsed_s_ >= options_.boot_delay_s)
      enterState(scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
  }

  if (state_ == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE && watchdog_ms_ > 0)
  {
    since_command_s_ += dt;
    if (since_command_s_ * 1000.0 > watchdog_ms_)
      raiseFault(scorbot_v1_FaultCode_FAULT_WATCHDOG, -1);
  }

  if (state_ == scorbot_v1_SystemState_SYSTEM_STATE_HOMING)
    stepHoming(dt);

  stepJoints(dt);

  since_state_s_ += dt;
  if (since_state_s_ + 1e-9 >= options_.state_period_s)
  {
    since_state_s_ = 0.0;
    emitJointState();
  }
}

void ControllerSim::stepHoming(double dt)
{
  homing_elapsed_s_ += dt;
  bool all_done = true;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    JointSim& j = joints_[i];
    if (!j.homing)
      continue;
    all_done = false;
    // Drive toward the switch at homing speed.
    j.mode = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
    j.setpoint_velocity = j.homing_direction * options_.homing_speed_fraction * j.cal.max_velocity_rad_s;

    const int home_dir = j.cal.home_direction >= 0 ? 1 : -1;
    if (j.homing_backoff)
    {
      if (!j.homing_released)
      {
        if (j.limit_switch)
          continue;  // still pressed: keep backing off
        j.homing_released = true;
        j.homing_release_pos = j.true_position;
      }
      // Keep going a definite distance past the release point so the re-approach
      // happens at homing speed, then turn around for the final approach onto the edge.
      if (std::fabs(j.true_position - j.homing_release_pos) < j.backoff_margin)
        continue;
      j.homing_backoff = false;
      j.homing_direction = home_dir;
      continue;
    }
    if (j.limit_switch && j.homing_direction != home_dir)
    {
      // Reversed pass: we are crossing the switch from its far side. Drive through it and
      // re-approach from the calibrated direction so the edge is the same one the
      // calibration was measured on.
      j.homing_passed = true;
      continue;
    }
    if (j.homing_passed && !j.limit_switch && j.homing_direction != home_dir)
    {
      j.homing_direction = home_dir;  // final approach
      j.homing_passed = false;
      continue;
    }
    if (j.limit_switch)
    {
      // Found the edge from the calibrated side. The firmware latches the encoder count
      // on the switch interrupt, so the edge itself (not the position a tick later)
      // reads home_offset_rad. In the simulator the truth frame is the calibrated
      // frame, so the edge sits at home_offset_rad and the encoder zero becomes 0.
      j.encoder_zero = j.cal.home_offset_rad - j.cal.home_offset_rad;
      j.homed = true;
      j.homing = false;
      j.mode = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
      j.velocity = 0.0;
      homed_progress_mask_ |= (1u << i);
      scorbot_v1_Event ev = scorbot_v1_Event_init_zero;
      ev.which_body = scorbot_v1_Event_homing_progress_mask_tag;
      ev.body.homing_progress_mask = homed_progress_mask_;
      emitEvent(ev);
      continue;
    }
    // Ran into the mechanical end without finding the switch: the firmware detects
    // this from the current spike and searches the other way, once.
    const bool at_end = (j.homing_direction > 0 && j.true_position >= j.hardMax() - 1e-9) ||
                        (j.homing_direction < 0 && j.true_position <= j.hardMin() + 1e-9);
    if (at_end)
    {
      if (j.homing_reversed)
      {
        raiseFault(scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT, static_cast<int>(i));
        return;
      }
      j.homing_reversed = true;
      j.homing_direction = -j.homing_direction;
    }
  }
  if (all_done)
  {
    scorbot_v1_Event ev = scorbot_v1_Event_init_zero;
    ev.which_body = scorbot_v1_Event_homing_complete_mask_tag;
    ev.body.homing_complete_mask = homed_progress_mask_;
    emitEvent(ev);
    enterState(positionsTrusted() ? scorbot_v1_SystemState_SYSTEM_STATE_READY
                                  : scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    return;
  }
  if (homing_elapsed_s_ > homing_timeout_s_)
    raiseFault(scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT, -1);
}

void ControllerSim::stepJoints(double dt)
{
  const bool enabled = drivesEnabled();
  const bool enforce_soft = enabled && !unhomed_active_ && state_ != scorbot_v1_SystemState_SYSTEM_STATE_HOMING;
  uint32_t switch_edges = 0;

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    JointSim& j = joints_[i];
    const double vmax = j.cal.max_velocity_rad_s;
    const double amax = j.cal.max_accel_rad_s2;
    const double prev_v = j.velocity;
    j.at_soft_limit = false;

    double v_des = 0.0;
    if (enabled)
    {
      switch (j.mode)
      {
        case scorbot_v1_ControlMode_CONTROL_MODE_POSITION:
        {
          double target = j.setpoint_position;
          if (enforce_soft)
          {
            const double clamped = clampd(target, j.softMin(), j.softMax());
            j.at_soft_limit = clamped != target;
            target = clamped;
          }
          v_des = clampd(10.0 * (target - j.reportedPosition()), -vmax, vmax);
          break;
        }
        case scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY:
          v_des = clampd(j.setpoint_velocity, -vmax, vmax);
          break;
        default: v_des = 0.0; break;
      }
      if (enforce_soft)
      {
        // Ramp to zero approaching a soft limit: never exceed the speed that can stop in time.
        const double room_up = std::max(0.0, j.softMax() - j.reportedPosition());
        const double room_dn = std::max(0.0, j.reportedPosition() - j.softMin());
        const double cap_up = std::sqrt(2.0 * amax * room_up);
        const double cap_dn = std::sqrt(2.0 * amax * room_dn);
        if (v_des > cap_up) { v_des = cap_up; j.at_soft_limit = true; }
        if (v_des < -cap_dn) { v_des = -cap_dn; j.at_soft_limit = true; }
      }
    }

    // Acceleration limit, integrate.
    const double dv = clampd(v_des - j.velocity, -amax * dt, amax * dt);
    j.velocity += dv;
    j.true_position += j.velocity * dt;

    // Mechanical end stops: the motor stalls against them.
    bool stalled = false;
    if (j.true_position > j.hardMax()) { j.true_position = j.hardMax(); stalled = j.velocity > 0 || v_des > 0; j.velocity = 0.0; }
    if (j.true_position < j.hardMin()) { j.true_position = j.hardMin(); stalled = j.velocity < 0 || v_des < 0; j.velocity = 0.0; }
    j.accel = (j.velocity - prev_v) / dt;

    // Current model: bias + speed + acceleration, stall pins it above the limit.
    const double imax = j.cal.current_limit_a;
    j.current = enabled ? 0.15 + 0.5 * imax * std::fabs(j.velocity) / vmax + 0.4 * imax * std::fabs(j.accel) / amax : 0.0;
    if (stalled)
      j.current = 1.5 * imax;

    const bool pressed = j.switchPressed(j.true_position);
    if (pressed != j.limit_switch)
      switch_edges |= (1u << i);
    j.limit_switch = pressed;

    if (state_ == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE)
    {
      if (j.current > imax)
        raiseFault(scorbot_v1_FaultCode_FAULT_OVERCURRENT, static_cast<int>(i));
      else if (options_.limit_switch_faults_when_active && pressed)
        raiseFault(scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH, static_cast<int>(i));
    }
  }

  if (switch_edges)
  {
    scorbot_v1_Event ev = scorbot_v1_Event_init_zero;
    ev.which_body = scorbot_v1_Event_limit_switch_mask_tag;
    ev.body.limit_switch_mask = switch_edges;
    emitEvent(ev);
  }
}

}  // namespace scorbot_esp_sim
