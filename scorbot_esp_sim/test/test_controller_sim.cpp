// State machine and physics conformance of the simulated controller. These are the
// behaviours the firmware must reproduce (scorbot_protocol/docs/protocol.md).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "scorbot_esp_sim/controller_sim.hpp"

using namespace scorbot_esp_sim;
using scorbot_protocol::Envelope;

namespace
{

struct Harness
{
  ControllerSim sim;
  uint32_t next_id{1};
  std::vector<Envelope> out;

  explicit Harness(SimOptions o = SimOptions{}) : sim(std::move(o)) {}

  void run(double seconds, double dt = 0.001)
  {
    const int steps = static_cast<int>(std::lround(seconds / dt));
    for (int i = 0; i < steps; ++i)
    {
      sim.tick(dt);
      drain();
    }
  }
  void drain()
  {
    Envelope e;
    while (sim.popOutbound(e))
      out.push_back(e);
  }
  Response request(void (*fill)(Request&), uint32_t* id_out = nullptr)
  {
    Envelope e = scorbot_protocol::makeEnvelope(1, sim.nowUs());
    Request& r = scorbot_protocol::setRequest(e, next_id++);
    fill(r);
    sim.handle(e);
    drain();
    if (id_out)
      *id_out = r.request_id;
    for (auto it = out.rbegin(); it != out.rend(); ++it)
      if (it->which_payload == scorbot_v1_Envelope_response_tag && it->payload.response.request_id == r.request_id)
        return it->payload.response;
    ADD_FAILURE() << "no response";
    return scorbot_v1_Response_init_zero;
  }
  void boot() { run(0.1); }
  Response enable(bool allow_unhomed = false)
  {
    return allow_unhomed ? request([](Request& r) { scorbot_protocol::requestEnable(r, true); })
                         : request([](Request& r) { scorbot_protocol::requestEnable(r, false); });
  }
  Response home() { return request([](Request& r) { scorbot_protocol::requestHome(r, 0, 0); }); }
  Response disable() { return request([](Request& r) { scorbot_protocol::requestDisable(r); }); }
  Response clearFault() { return request([](Request& r) { scorbot_protocol::requestClearFault(r); }); }
  void command(std::size_t joint, ControlMode mode, float value)
  {
    Envelope e = scorbot_protocol::makeEnvelope(1, sim.nowUs());
    auto& cmd = scorbot_protocol::setJointCommand(e, sim.jointCount());
    for (std::size_t i = 0; i < sim.jointCount(); ++i)
      cmd.mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    cmd.mode[joint] = mode;
    if (mode == scorbot_v1_ControlMode_CONTROL_MODE_POSITION)
      cmd.position[joint] = value;
    else
      cmd.velocity[joint] = value;
    sim.handle(e);
  }
  /// Keep the watchdog fed while running.
  void runCommanding(double seconds, std::size_t joint, ControlMode mode, float value)
  {
    const int steps = static_cast<int>(seconds / 0.01);
    for (int i = 0; i < steps; ++i)
    {
      command(joint, mode, value);
      run(0.01);
    }
  }
  std::size_t count(pb_size_t payload_tag) const
  {
    std::size_t n = 0;
    for (const auto& e : out)
      n += (e.which_payload == payload_tag);
    return n;
  }
  std::vector<scorbot_v1_Event> events() const
  {
    std::vector<scorbot_v1_Event> v;
    for (const auto& e : out)
      if (e.which_payload == scorbot_v1_Envelope_event_tag)
        v.push_back(e.payload.event);
    return v;
  }
  const scorbot_v1_JointState* lastState() const
  {
    for (auto it = out.rbegin(); it != out.rend(); ++it)
      if (it->which_payload == scorbot_v1_Envelope_joint_state_tag)
        return &it->payload.joint_state;
    return nullptr;
  }
};

}  // namespace

TEST(Sim, BootsIntoUnhomedAndStreamsStateAt100Hz)
{
  Harness h;
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_BOOT);
  h.run(1.0);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
  EXPECT_NEAR(static_cast<double>(h.count(scorbot_v1_Envelope_joint_state_tag)), 100.0, 2.0);
  const auto* s = h.lastState();
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->position_count, 5);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FLOAT_EQ(s->position[i], 0.0f) << "reads zero before homing";
    EXPECT_FALSE(s->flags[i].homed);
  }
  // A state_changed event announced BOOT -> UNHOMED.
  bool announced = false;
  for (const auto& ev : h.events())
    announced |= ev.which_body == scorbot_v1_Event_state_changed_tag &&
                 ev.body.state_changed == scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED;
  EXPECT_TRUE(announced);
}

TEST(Sim, PingAndInfo)
{
  Harness h;
  h.boot();
  Response r = h.request([](Request& q) { scorbot_protocol::requestPing(q, 1234); });
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_OK);
  ASSERT_EQ(r.which_body, scorbot_v1_Response_pong_tag);
  EXPECT_EQ(r.body.pong, 1234u);

  r = h.request([](Request& q) { scorbot_protocol::requestGetInfo(q); });
  ASSERT_EQ(r.which_body, scorbot_v1_Response_get_info_tag);
  const auto& info = r.body.get_info;
  EXPECT_EQ(info.protocol_major, scorbot_protocol::kProtocolMajor);
  EXPECT_STREQ(info.robot_type, "er_4pc");
  EXPECT_EQ(info.joint_names_count, 5);
  EXPECT_STREQ(info.joint_names[4], "wrist_roll_joint");
  EXPECT_EQ(info.boot_count, 1u);
  EXPECT_EQ(info.watchdog_ms, 200u);
  EXPECT_EQ(info.calibration_count, 5);
  EXPECT_FLOAT_EQ(info.calibration[0].gear_ratio, 635.5f);
}

TEST(Sim, EnableRequiresHomingUnlessOverridden)
{
  Harness h;
  h.boot();
  Response r = h.enable(false);
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_INVALID_STATE);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);

  r = h.enable(true);
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_OK);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  EXPECT_TRUE(h.sim.unhomedActive());

  r = h.disable();
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_OK);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED) << "still not homed";
}

TEST(Sim, CommandsIgnoredUnlessActive)
{
  Harness h;
  h.boot();
  h.command(0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
  h.run(0.5);
  EXPECT_EQ(h.sim.commandsIgnored(), 1u);
  EXPECT_EQ(h.sim.commandsReceived(), 0u);
  EXPECT_FLOAT_EQ(static_cast<float>(h.sim.joint(0).velocity), 0.0f);
}

TEST(Sim, VelocityModeIntegratesWithAccelerationLimit)
{
  Harness h;
  h.boot();
  h.enable(true);
  const double vmax = h.sim.joint(0).cal.max_velocity_rad_s;
  h.runCommanding(0.05, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 10.0f);  // asks for far more than vmax
  EXPECT_LT(h.sim.joint(0).velocity, vmax);  // still accelerating
  EXPECT_GT(h.sim.joint(0).velocity, 0.0);
  h.runCommanding(2.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 10.0f);
  EXPECT_NEAR(h.sim.joint(0).velocity, vmax, 1e-6) << "clamped to max_velocity";
  EXPECT_GT(h.sim.joint(0).reportedPosition(), 0.3);
  const auto* s = h.lastState();
  ASSERT_NE(s, nullptr);
  EXPECT_NEAR(s->velocity[0], vmax, 1e-5);
  EXPECT_GT(s->current[0], 0.15f);
}

TEST(Sim, WatchdogFaultsWhenCommandsStop)
{
  Harness h;
  h.boot();
  h.enable(true);
  h.runCommanding(0.2, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  h.run(0.15);  // < 200 ms since the last command
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  h.run(0.1);   // now > 200 ms
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
  EXPECT_EQ(h.sim.fault(), scorbot_v1_FaultCode_FAULT_WATCHDOG);
  EXPECT_FLOAT_EQ(static_cast<float>(h.sim.joint(0).velocity), 0.0f) << "braked";
  bool raised = false;
  for (const auto& ev : h.events())
    raised |= ev.which_body == scorbot_v1_Event_fault_raised_tag && ev.body.fault_raised == scorbot_v1_FaultCode_FAULT_WATCHDOG;
  EXPECT_TRUE(raised);

  // Commands are ignored in FAULT; enable is refused; clear_fault after an unhomed
  // session returns to UNHOMED (positions were never trusted).
  h.command(0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
  EXPECT_EQ(h.sim.commandsIgnored(), 1u);
  EXPECT_EQ(h.enable(true).result, scorbot_v1_Result_RESULT_INVALID_STATE);
  EXPECT_EQ(h.clearFault().result, scorbot_v1_Result_RESULT_OK);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

TEST(Sim, WatchdogCanBeDisabledForBench)
{
  Harness h;
  h.boot();
  h.request([](Request& q) { scorbot_protocol::requestSetWatchdog(q, 0); });
  h.enable(true);
  h.run(2.0);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

TEST(Sim, HomingFindsSwitchesAndZeroesEncoders)
{
  Harness h;
  h.boot();
  for (std::size_t i = 0; i < h.sim.jointCount(); ++i)
    EXPECT_NEAR(h.sim.joint(i).true_position, 0.4, 1e-9) << "starts 0.4 rad past the switch";

  Response r = h.home();
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_OK) << "acknowledged immediately";
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
  EXPECT_EQ(h.home().result, scorbot_v1_Result_RESULT_BUSY);
  EXPECT_EQ(h.enable().result, scorbot_v1_Result_RESULT_BUSY);

  h.run(15.0);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  for (std::size_t i = 0; i < h.sim.jointCount(); ++i)
  {
    const JointSim& j = h.sim.joint(i);
    EXPECT_TRUE(j.homed) << i;
    EXPECT_NEAR(j.reportedPosition(), j.true_position, 1e-9) << "encoder zero now matches truth";
    EXPECT_NEAR(j.true_position, j.cal.home_offset_rad, j.switch_width + 1e-6) << "parked on the switch";
  }
  bool complete = false;
  for (const auto& ev : h.events())
    complete |= ev.which_body == scorbot_v1_Event_homing_complete_mask_tag && ev.body.homing_complete_mask == 0x1F;
  EXPECT_TRUE(complete);

  // Now normal Enable works, and the parked switch does not fault (mid-range switches).
  EXPECT_EQ(h.enable(false).result, scorbot_v1_Result_RESULT_OK);
  EXPECT_FALSE(h.sim.unhomedActive());
  h.runCommanding(0.5, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

TEST(Sim, HomingReversesAtTheHardStopWhenStartedOnTheWrongSide)
{
  SimOptions o;
  o.initial_positions = {-0.4, -0.4, -0.4, -0.4, -0.4};  // home_direction is -1: switch is behind us
  for (std::size_t i = 0; i < 5; ++i)
  {
    // Narrow travel so the trip to the hard stop and back fits the default 30 s timeout
    // (the real ±570° wrist roll would take minutes at homing speed).
    JointCalibration c = defaultCalibration(i);
    c.soft_limit_min_rad = -0.6f;
    c.soft_limit_max_rad = 0.6f;
    o.calibration.push_back(c);
  }
  Harness h(o);
  h.boot();
  h.home();
  h.run(25.0);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  for (std::size_t i = 0; i < h.sim.jointCount(); ++i)
  {
    const JointSim& j = h.sim.joint(i);
    EXPECT_TRUE(j.homed) << i;
    EXPECT_TRUE(j.homing_reversed) << i;
    // Re-approached from the calibrated side: parked just past the same edge as a
    // normal homing run, and the encoder is absolute again.
    EXPECT_NEAR(j.true_position, j.cal.home_offset_rad, j.switch_width + 1e-6) << i;
    EXPECT_NEAR(j.reportedPosition(), j.true_position, 1e-9) << i;
  }
}

TEST(Sim, HomingAgainFromTheSwitchBacksOffAndLatchesTheEdge)
{
  Harness h;
  h.boot();
  h.home();
  h.run(15.0);
  ASSERT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  for (std::size_t i = 0; i < h.sim.jointCount(); ++i)
    ASSERT_TRUE(h.sim.joint(i).limit_switch) << "parked on the switch";

  // Second homing: must not "find" the switch where the arm already sits.
  const std::size_t events_before = h.events().size();
  h.home();
  h.run(0.02);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_HOMING) << "not done in one tick";
  bool early_progress = false;
  for (std::size_t k = events_before; k < h.events().size(); ++k)
    early_progress |= h.events()[k].which_body == scorbot_v1_Event_homing_progress_mask_tag;
  EXPECT_FALSE(early_progress);
  bool released = false;
  double farthest = 0.0;
  int steps = 20;
  for (; steps < 5000 && h.sim.state() == scorbot_v1_SystemState_SYSTEM_STATE_HOMING; ++steps)
  {
    h.run(0.001);
    released |= !h.sim.joint(0).limit_switch;
    farthest = std::max(farthest, std::fabs(h.sim.joint(0).true_position - h.sim.joint(0).cal.home_offset_rad));
  }
  EXPECT_TRUE(released) << "backed off the switch before re-approaching";
  EXPECT_GT(farthest, h.sim.joint(0).backoff_margin) << "backed off a definite distance, not just until release";
  EXPECT_GT(steps, 200) << "re-homing takes real time (base joint at homing speed)";
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  for (std::size_t i = 0; i < h.sim.jointCount(); ++i)
  {
    const JointSim& j = h.sim.joint(i);
    EXPECT_TRUE(j.homed) << i;
    EXPECT_TRUE(j.limit_switch) << i;
    EXPECT_NEAR(j.reportedPosition(), j.true_position, 1e-9) << "edge latched again";
  }
}

TEST(Sim, HomingTimesOut)
{
  SimOptions o;
  o.default_home_timeout_ms = 500;
  Harness h(o);
  h.boot();
  h.home();
  h.run(1.0);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
  EXPECT_EQ(h.sim.fault(), scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT);
  EXPECT_EQ(h.clearFault().result, scorbot_v1_Result_RESULT_OK);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

TEST(Sim, PositionModeTracksAndSoftLimitsClamp)
{
  Harness h;
  h.boot();
  h.home();
  h.run(15.0);
  ASSERT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  h.enable();
  h.runCommanding(4.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_POSITION, 0.5f);
  EXPECT_NEAR(h.sim.joint(0).reportedPosition(), 0.5, 0.01);
  EXPECT_FALSE(h.sim.joint(0).at_soft_limit);

  // Ask for far beyond the +155 deg base limit: clamped, flagged, no fault.
  h.runCommanding(20.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_POSITION, 10.0f);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  EXPECT_NEAR(h.sim.joint(0).reportedPosition(), h.sim.joint(0).softMax(), 0.01);
  const auto* s = h.lastState();
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->flags[0].at_soft_limit);
  EXPECT_LE(s->position[0], static_cast<float>(h.sim.joint(0).softMax()) + 1e-4f);
}

TEST(Sim, VelocityIntoSoftLimitRampsToZero)
{
  Harness h;
  h.boot();
  h.home();
  h.run(15.0);
  h.enable();
  h.runCommanding(30.0, 1, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 5.0f);  // shoulder up, hard
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  EXPECT_NEAR(h.sim.joint(1).reportedPosition(), h.sim.joint(1).softMax(), 0.02);
  EXPECT_NEAR(h.sim.joint(1).velocity, 0.0, 0.05);
  EXPECT_LT(h.sim.joint(1).current, h.sim.joint(1).cal.current_limit_a) << "never stalled against the hard stop";
}

TEST(Sim, UnhomedActiveHasNoSoftLimitsAndStallsIntoOvercurrent)
{
  Harness h;
  h.boot();
  h.request([](Request& q) { scorbot_protocol::requestSetWatchdog(q, 0); });
  h.enable(true);
  // Drive the shoulder toward its mechanical stop with no soft limit protection.
  h.runCommanding(60.0, 1, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 1.0f);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
  EXPECT_EQ(h.sim.fault(), scorbot_v1_FaultCode_FAULT_OVERCURRENT);
  const auto* s = h.lastState();
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->flags[1].fault);
  EXPECT_FALSE(s->flags[0].fault);
}

TEST(Sim, ClearFaultKeepsHomingWhenPositionsAreTrusted)
{
  Harness h;
  h.boot();
  h.home();
  h.run(15.0);
  h.enable();
  h.sim.injectFault(scorbot_v1_FaultCode_FAULT_OVERCURRENT, 2);
  h.drain();
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
  EXPECT_EQ(h.clearFault().result, scorbot_v1_Result_RESULT_OK);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  EXPECT_TRUE(h.sim.joint(2).homed);
  EXPECT_FALSE(h.sim.joint(2).fault);

  h.enable();
  h.sim.injectFault(scorbot_v1_FaultCode_FAULT_ENCODER, 2);
  h.drain();
  h.clearFault();
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED) << "encoder faults lose position";
  EXPECT_FALSE(h.sim.joint(0).homed);
}

TEST(Sim, RebootBumpsBootCountAndForgetsHoming)
{
  Harness h;
  h.boot();
  h.request([](Request& q) { scorbot_protocol::requestSetWatchdog(q, 50); });
  h.request([](Request& q) { scorbot_protocol::requestSetRobotType(q, "er_v"); });
  h.home();
  h.run(15.0);
  ASSERT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_READY);
  h.sim.reboot();
  h.run(0.1);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
  Response r = h.request([](Request& q) { scorbot_protocol::requestGetInfo(q); });
  EXPECT_EQ(r.body.get_info.boot_count, 2u);
  EXPECT_STREQ(r.body.get_info.robot_type, "er_v") << "persisted";
  EXPECT_EQ(r.body.get_info.watchdog_ms, 200u) << "back to default";
  EXPECT_FALSE(h.sim.joint(0).homed);
  EXPECT_FLOAT_EQ(static_cast<float>(h.sim.joint(0).reportedPosition()), 0.0f) << "counts restart at zero";
  // The stream carries the same counter, so the Pi notices without asking.
  uint32_t streamed = 0;
  for (auto it = h.out.rbegin(); it != h.out.rend(); ++it)
    if (it->which_payload == scorbot_v1_Envelope_joint_state_tag)
    {
      streamed = it->payload.joint_state.boot_count;
      break;
    }
  EXPECT_EQ(streamed, 2u);
}

TEST(Sim, CalibrationIsValidatedAndApplied)
{
  Harness h;
  h.boot();
  JointCalibration cal = defaultCalibration(3);
  cal.soft_limit_max_rad = 0.3f;
  cal.max_velocity_rad_s = 0.2f;
  static JointCalibration shared;
  shared = cal;
  Response r = h.request([](Request& q) { scorbot_protocol::requestSetCalibration(q, shared); });
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_OK);
  EXPECT_FLOAT_EQ(h.sim.joint(3).cal.soft_limit_max_rad, 0.3f);

  shared.index = 9;
  r = h.request([](Request& q) { scorbot_protocol::requestSetCalibration(q, shared); });
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_INVALID_ARGUMENT);

  shared.index = 3;
  shared.gear_ratio = -1.0f;
  r = h.request([](Request& q) { scorbot_protocol::requestSetCalibration(q, shared); });
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_INVALID_ARGUMENT);

  shared = cal;
  h.enable(true);
  r = h.request([](Request& q) { scorbot_protocol::requestSetCalibration(q, shared); });
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_INVALID_STATE) << "not while active";
}

TEST(Sim, UnknownRequestIsUnsupported)
{
  Harness h;
  h.boot();
  Response r = h.request([](Request& q) { q.which_body = 0; });
  EXPECT_EQ(r.result, scorbot_v1_Result_RESULT_UNSUPPORTED);
}

TEST(Sim, InvertFlipsReportedSign)
{
  SimOptions o;
  o.calibration.push_back(defaultCalibration(0));
  o.calibration[0].invert = true;
  Harness h(o);
  h.boot();
  h.request([](Request& q) { scorbot_protocol::requestSetWatchdog(q, 0); });
  h.enable(true);
  h.runCommanding(2.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
  const auto* s = h.lastState();
  ASSERT_NE(s, nullptr);
  EXPECT_GT(s->velocity[0], 0.1f) << "the commanded (joint-space) sign is what comes back";
  EXPECT_LT(h.sim.joint(0).velocity, 0.0) << "the motor turns the other way";
}

TEST(Sim, SwappedEncoderPhasesAreCorrectedByEncoderInvert)
{
  // The ER-4u and ER-Vplus manuals disagree on the P0/P1 pad order: on one arm the
  // count falls when the motor drives positive. With encoder_invert set to match, the
  // controller behaves exactly as with straight wiring.
  SimOptions o;
  o.encoder_reversed = {true, false, false, false, false};
  o.calibration.push_back(defaultCalibration(0));
  o.calibration[0].encoder_invert = true;
  Harness h(o);
  h.boot();
  h.request([](Request& q) { scorbot_protocol::requestSetWatchdog(q, 0); });
  h.enable(true);
  h.runCommanding(2.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
  EXPECT_GT(h.sim.joint(0).velocity, 0.1) << "motor turns positive";
  EXPECT_GT(h.lastState()->velocity[0], 0.1f) << "and the corrected count follows it";
  EXPECT_GT(h.lastState()->position[0], 0.2f);
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

TEST(Sim, EncoderPhaseMismatchIsARunaway)
{
  // Same wiring, flag not set: the commissioning symptom (positive drive, falling count)
  // and the consequence under closed-loop position control (runs into the hard stop
  // and faults on overcurrent). This is why encoder_invert is its own flag.
  SimOptions o;
  o.encoder_reversed = {true, false, false, false, false};
  Harness h(o);
  h.boot();
  h.request([](Request& q) { scorbot_protocol::requestSetWatchdog(q, 0); });
  h.enable(true);
  h.runCommanding(1.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
  EXPECT_GT(h.sim.joint(0).velocity, 0.1) << "motor turns positive";
  EXPECT_LT(h.lastState()->velocity[0], -0.1f) << "but the count falls: the commissioning check catches this";

  h.runCommanding(12.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_POSITION, 0.1f);  // 2.4 rad to the hard stop at vmax
  EXPECT_EQ(h.sim.state(), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
  EXPECT_EQ(h.sim.fault(), scorbot_v1_FaultCode_FAULT_OVERCURRENT) << "ran away into the hard stop";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
