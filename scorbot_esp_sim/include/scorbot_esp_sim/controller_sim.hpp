// ControllerSim: a faithful model of what the ESP32 firmware must do, driven by a
// deterministic clock and fed Envelopes. It is the executable form of
// scorbot_protocol/docs/protocol.md: the hardware interface is tested against it, and
// the firmware must behave the same way.
//
// No transport, no threads, no ROS: tick(dt) advances time, handle() consumes an
// inbound Envelope, popOutbound() yields responses, events and the periodic
// JointState. main.cpp wires it to a pty; tests wire it to a TransportPair.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "scorbot_protocol/messages.hpp"

namespace scorbot_esp_sim
{

using scorbot_protocol::ControlMode;
using scorbot_protocol::Envelope;
using scorbot_protocol::FaultCode;
using scorbot_protocol::JointCalibration;
using scorbot_protocol::Request;
using scorbot_protocol::Response;
using scorbot_protocol::Result;
using scorbot_protocol::SystemState;

constexpr std::size_t kMaxJoints = scorbot_protocol::kMaxJoints;

/// One simulated axis: true mechanics plus what the controller can observe.
struct JointSim
{
  JointCalibration cal{};

  // Mechanical truth (never visible on the wire directly).
  bool encoder_reversed{false}; ///< physical: the encoder phases are wired so counts fall when the motor drives positive
  double true_position{0.0};   ///< rad, absolute
  double velocity{0.0};        ///< rad/s
  double accel{0.0};           ///< rad/s^2, last step
  double hard_margin{0.15};    ///< mechanical end stop this far beyond each soft limit
  double switch_width{0.02};   ///< limit switch stays pressed this far past its edge

  // Controller-side knowledge.
  double encoder_zero{0.0};    ///< true position that reads as 0 before homing
  bool homed{false};
  bool limit_switch{false};
  bool at_soft_limit{false};
  bool fault{false};
  double current{0.0};         ///< A
  ControlMode mode{scorbot_v1_ControlMode_CONTROL_MODE_NONE};
  double setpoint_position{0.0};
  double setpoint_velocity{0.0};

  // Homing bookkeeping.
  bool homing{false};
  int homing_direction{0};
  bool homing_reversed{false}; ///< hit the hard stop once and turned around
  bool homing_passed{false};   ///< on the reversed pass: drove through the switch, now re-approaching
  bool homing_backoff{false};  ///< started on the switch: backing off past the release point
  bool homing_released{false}; ///< the switch released during the back-off
  double homing_release_pos{0.0};  ///< where it released; back off backoff_margin beyond
  double backoff_margin{0.02};     ///< rad past the release point before re-approaching

  /// Switch model: the edge that trips when approached from `home_direction` is exactly
  /// `home_offset_rad`; the switch stays pressed for `switch_width` beyond it (mid-range
  /// switches on the Scorbots, not end-of-travel).
  bool switchPressed(double pos) const
  {
    const double s = (cal.home_direction >= 0 ? 1.0 : -1.0) * (pos - cal.home_offset_rad);
    return s >= 0.0 && s < switch_width;
  }

  /// Sign the firmware's corrected count has relative to true motion: +1 when the
  /// calibration's encoder_invert matches the wiring, -1 when it does not (runaway).
  double encoderSign() const { return (cal.encoder_invert != encoder_reversed) ? -1.0 : 1.0; }
  /// Position as the controller believes it (corrected counts converted with the calibration).
  double reportedPosition() const { return encoderSign() * (true_position - encoder_zero); }
  /// Velocity as the controller derives it from the counts.
  double reportedVelocity() const { return encoderSign() * velocity; }
  double softMin() const { return cal.soft_limit_min_rad; }
  double softMax() const { return cal.soft_limit_max_rad; }
  double hardMin() const { return cal.soft_limit_min_rad - hard_margin; }
  double hardMax() const { return cal.soft_limit_max_rad + hard_margin; }
};

struct SimOptions
{
  std::string robot_type{"er_4pc"};
  std::string firmware_version{"sim-0.1.0"};
  std::vector<std::string> joint_names{"base_joint", "shoulder_joint", "elbow_joint",
                                       "wrist_pitch_joint", "wrist_roll_joint"};
  uint32_t watchdog_ms{200};
  double state_period_s{0.010};      ///< JointState emission period
  double control_period_s{0.001};    ///< reported inner loop period
  double homing_speed_fraction{0.25}; ///< of max_velocity_rad_s
  uint32_t default_home_timeout_ms{30000};
  double boot_delay_s{0.05};         ///< BOOT -> UNHOMED
  /// Fault when a switch is pressed while ACTIVE. Off for Scorbots (mid-range switches).
  bool limit_switch_faults_when_active{false};
  /// Optional initial true positions (rad); default puts every joint 0.4 rad away from
  /// its switch, against the homing direction, so homing has to move.
  std::vector<double> initial_positions;
  /// Optional per-joint calibration; default is a plausible 5-joint Scorbot.
  std::vector<JointCalibration> calibration;
  /// Optional per-joint physical encoder phase order (true = counts fall on positive drive).
  std::vector<bool> encoder_reversed;
};

/// Default Scorbot-like calibration for joint `index`.
JointCalibration defaultCalibration(std::size_t index);

class ControllerSim
{
public:
  explicit ControllerSim(SimOptions options = SimOptions{});

  // ---- time and I/O ----------------------------------------------------------------
  void tick(double dt_s);
  void handle(const Envelope& in);
  bool popOutbound(Envelope& out);
  std::size_t outboundSize() const { return outbound_.size(); }
  uint64_t nowUs() const { return now_us_; }

  // ---- injection (bench and tests) -------------------------------------------------
  void injectFault(FaultCode code, int joint = -1);
  void reboot();

  // ---- introspection ---------------------------------------------------------------
  SystemState state() const { return state_; }
  FaultCode fault() const { return fault_; }
  std::size_t jointCount() const { return joints_.size(); }
  const JointSim& joint(std::size_t i) const { return joints_[i]; }
  JointSim& joint(std::size_t i) { return joints_[i]; }
  uint32_t bootCount() const { return boot_count_; }
  uint32_t watchdogMs() const { return watchdog_ms_; }
  const std::string& robotType() const { return robot_type_; }
  uint32_t commandsReceived() const { return commands_received_; }
  uint32_t commandsIgnored() const { return commands_ignored_; }
  bool unhomedActive() const { return unhomed_active_; }
  const SimOptions& options() const { return options_; }

private:
  void enterState(SystemState next);
  void raiseFault(FaultCode code, int joint);
  void onJointCommand(const scorbot_protocol::JointCommand& cmd);
  void onRequest(const Request& req);
  Result doSetCalibration(const JointCalibration& cal, std::string& msg);
  Result doHome(uint32_t mask, uint32_t timeout_ms, std::string& msg);
  Result doEnable(bool allow_unhomed, std::string& msg);
  Result doDisable();
  Result doClearFault();
  void stepJoints(double dt);
  void stepHoming(double dt);
  void emitJointState();
  void emitEvent(const scorbot_v1_Event& event);
  Envelope newEnvelope();
  bool drivesEnabled() const;
  bool positionsTrusted() const;

  SimOptions options_;
  std::vector<JointSim> joints_;
  std::string robot_type_;
  uint32_t watchdog_ms_;
  SystemState state_{scorbot_v1_SystemState_SYSTEM_STATE_BOOT};
  FaultCode fault_{scorbot_v1_FaultCode_FAULT_NONE};
  bool unhomed_active_{false};
  uint32_t boot_count_{1};
  uint64_t now_us_{0};
  double boot_elapsed_s_{0.0};
  double since_state_s_{0.0};
  double since_command_s_{0.0};
  double homing_elapsed_s_{0.0};
  double homing_timeout_s_{0.0};
  uint32_t homing_mask_{0};
  uint32_t homed_progress_mask_{0};
  uint32_t commands_received_{0};
  uint32_t commands_ignored_{0};
  uint32_t seq_{0};
  double last_dt_s_{0.001};
  std::deque<Envelope> outbound_;
};

}  // namespace scorbot_esp_sim
