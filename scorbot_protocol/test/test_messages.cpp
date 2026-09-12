#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "scorbot_protocol/messages.hpp"

using namespace scorbot_protocol;

namespace
{
Envelope roundTrip(const Envelope& in, std::size_t* encoded_len = nullptr)
{
  uint8_t buf[kMaxPayload];
  std::size_t len = 0;
  EXPECT_TRUE(encodeEnvelope(in, buf, sizeof(buf), len));
  if (encoded_len)
    *encoded_len = len;
  Envelope out;
  EXPECT_TRUE(decodeEnvelope(buf, len, out));
  return out;
}
}  // namespace

TEST(Messages, EnvelopeMaxFitsFrame)
{
  EXPECT_LE(scorbot_v1_Envelope_size, kMaxPayload);
}

TEST(Messages, JointCommandRoundTrip)
{
  Envelope e = makeEnvelope(42, 123456789ULL);
  JointCommand& cmd = setJointCommand(e, 5);
  for (int i = 0; i < 5; ++i)
  {
    cmd.mode[i] = i % 2 ? scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY : scorbot_v1_ControlMode_CONTROL_MODE_POSITION;
    cmd.position[i] = 0.1f * i;
    cmd.velocity[i] = -0.2f * i;
  }
  std::size_t len = 0;
  const Envelope out = roundTrip(e, &len);
  EXPECT_EQ(out.seq, 42u);
  EXPECT_EQ(out.timestamp_us, 123456789ULL);
  ASSERT_EQ(out.which_payload, scorbot_v1_Envelope_joint_command_tag);
  EXPECT_EQ(out.payload.joint_command.mode_count, 5);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(out.payload.joint_command.mode[i], cmd.mode[i]);
    EXPECT_FLOAT_EQ(out.payload.joint_command.position[i], cmd.position[i]);
    EXPECT_FLOAT_EQ(out.payload.joint_command.velocity[i], cmd.velocity[i]);
  }
  EXPECT_LT(len, 80u) << "a 5-joint command should be small on the wire";
}

TEST(Messages, JointStateRoundTripAndSize)
{
  Envelope e = makeEnvelope(1, 2);
  JointState& s = setJointState(e, 5);
  s.state = scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE;
  s.fault = scorbot_v1_FaultCode_FAULT_NONE;
  for (int i = 0; i < 5; ++i)
  {
    s.position[i] = 1.0f + i;
    s.velocity[i] = 0.5f;
    s.current[i] = 0.25f * i;
    s.flags[i].homed = true;
    s.flags[i].limit_switch = (i == 2);
  }
  s.commands_received = 1000;
  s.control_loop_period_us = 1000;
  std::size_t len = 0;
  const Envelope out = roundTrip(e, &len);
  ASSERT_EQ(out.which_payload, scorbot_v1_Envelope_joint_state_tag);
  EXPECT_EQ(out.payload.joint_state.state, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  EXPECT_EQ(out.payload.joint_state.flags_count, 5);
  EXPECT_TRUE(out.payload.joint_state.flags[2].limit_switch);
  EXPECT_FALSE(out.payload.joint_state.flags[1].limit_switch);
  EXPECT_FLOAT_EQ(out.payload.joint_state.position[4], 5.0f);
  EXPECT_EQ(out.payload.joint_state.commands_received, 1000u);
  // Budget from the design doc: a 5-joint state well under 200 bytes, so a frame at
  // 100 Hz uses a small fraction of 921600 baud.
  EXPECT_LT(len, 160u);
}

TEST(Messages, EveryRequestRoundTrips)
{
  struct Case { const char* name; void (*fill)(Request&); pb_size_t tag; };
  const Case cases[] = {
    {"get_info", [](Request& r) { requestGetInfo(r); }, scorbot_v1_Request_get_info_tag},
    {"set_robot_type", [](Request& r) { requestSetRobotType(r, "er_4pc"); }, scorbot_v1_Request_set_robot_type_tag},
    {"set_watchdog", [](Request& r) { requestSetWatchdog(r, 200); }, scorbot_v1_Request_set_watchdog_tag},
    {"home", [](Request& r) { requestHome(r, 0x1F, 30000); }, scorbot_v1_Request_home_tag},
    {"enable", [](Request& r) { requestEnable(r, true); }, scorbot_v1_Request_enable_tag},
    {"disable", [](Request& r) { requestDisable(r); }, scorbot_v1_Request_disable_tag},
    {"clear_fault", [](Request& r) { requestClearFault(r); }, scorbot_v1_Request_clear_fault_tag},
    {"ping", [](Request& r) { requestPing(r, 0xDEADBEEF); }, scorbot_v1_Request_ping_tag},
  };
  for (const auto& c : cases)
  {
    Envelope e = makeEnvelope(9, 9);
    Request& r = setRequest(e, 77);
    c.fill(r);
    const Envelope out = roundTrip(e);
    ASSERT_EQ(out.which_payload, scorbot_v1_Envelope_request_tag) << c.name;
    EXPECT_EQ(out.payload.request.request_id, 77u) << c.name;
    EXPECT_EQ(out.payload.request.which_body, c.tag) << c.name;
    EXPECT_STREQ(requestName(out.payload.request), c.name);
  }
}

TEST(Messages, CalibrationRoundTrip)
{
  JointCalibration cal = scorbot_v1_JointCalibration_init_zero;
  cal.index = 3;
  cal.counts_per_motor_rev = 80;
  cal.gear_ratio = 125.54f;
  cal.home_offset_rad = -0.5f;
  cal.home_direction = -1;
  cal.soft_limit_min_rad = -2.2689f;
  cal.soft_limit_max_rad = 2.2689f;
  cal.max_velocity_rad_s = 1.4f;
  cal.max_accel_rad_s2 = 3.0f;
  cal.current_limit_a = 1.5f;
  cal.invert = true;
  cal.encoder_invert = true;

  Envelope e = makeEnvelope(1, 1);
  requestSetCalibration(setRequest(e, 5), cal);
  const Envelope out = roundTrip(e);
  ASSERT_EQ(out.payload.request.which_body, scorbot_v1_Request_set_calibration_tag);
  ASSERT_TRUE(out.payload.request.body.set_calibration.has_joint);
  const JointCalibration& c = out.payload.request.body.set_calibration.joint;
  EXPECT_EQ(c.index, 3u);
  EXPECT_EQ(c.counts_per_motor_rev, 80);
  EXPECT_FLOAT_EQ(c.gear_ratio, 125.54f);
  EXPECT_EQ(c.home_direction, -1);
  EXPECT_TRUE(c.invert);
  EXPECT_TRUE(c.encoder_invert);
}

TEST(Messages, GetInfoResponseRoundTrip)
{
  Envelope e = makeEnvelope(3, 3);
  Response& resp = setResponse(e, 12, scorbot_v1_Result_RESULT_OK);
  resp.which_body = scorbot_v1_Response_get_info_tag;
  GetInfoResponse& info = resp.body.get_info;
  info.protocol_major = kProtocolMajor;
  info.protocol_minor = kProtocolMinor;
  std::strcpy(info.firmware_version, "0.1.0+test");
  std::strcpy(info.robot_type, "er_v");
  const char* names[] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};
  info.joint_names_count = 5;
  for (int i = 0; i < 5; ++i)
    std::strcpy(info.joint_names[i], names[i]);
  info.boot_count = 17;
  info.watchdog_ms = 200;
  info.calibration_count = 5;
  for (int i = 0; i < 5; ++i)
  {
    info.calibration[i] = scorbot_v1_JointCalibration_init_zero;
    info.calibration[i].index = i;
    info.calibration[i].gear_ratio = 100.0f + i;
  }

  std::size_t len = 0;
  const Envelope out = roundTrip(e, &len);
  ASSERT_EQ(out.which_payload, scorbot_v1_Envelope_response_tag);
  EXPECT_EQ(out.payload.response.request_id, 12u);
  EXPECT_EQ(out.payload.response.result, scorbot_v1_Result_RESULT_OK);
  ASSERT_EQ(out.payload.response.which_body, scorbot_v1_Response_get_info_tag);
  const GetInfoResponse& i2 = out.payload.response.body.get_info;
  EXPECT_STREQ(i2.firmware_version, "0.1.0+test");
  EXPECT_STREQ(i2.robot_type, "er_v");
  EXPECT_EQ(i2.joint_names_count, 5);
  EXPECT_STREQ(i2.joint_names[3], "wrist_pitch_joint");
  EXPECT_EQ(i2.boot_count, 17u);
  EXPECT_FLOAT_EQ(i2.calibration[4].gear_ratio, 104.0f);
  EXPECT_LE(len, kMaxPayload);
}

TEST(Messages, ErrorResponseAndEvents)
{
  Envelope e = makeEnvelope(1, 1);
  Response& resp = setResponse(e, 4, scorbot_v1_Result_RESULT_INVALID_STATE);
  std::strcpy(resp.message, "not homed");
  Envelope out = roundTrip(e);
  EXPECT_EQ(out.payload.response.result, scorbot_v1_Result_RESULT_INVALID_STATE);
  EXPECT_STREQ(out.payload.response.message, "not homed");
  EXPECT_STREQ(toString(out.payload.response.result), "invalid_state");

  Envelope ev = makeEnvelope(2, 2);
  Event& event = setEvent(ev);
  event.which_body = scorbot_v1_Event_fault_raised_tag;
  event.body.fault_raised = scorbot_v1_FaultCode_FAULT_OVERCURRENT;
  out = roundTrip(ev);
  ASSERT_EQ(out.which_payload, scorbot_v1_Envelope_event_tag);
  EXPECT_EQ(out.payload.event.which_body, scorbot_v1_Event_fault_raised_tag);
  EXPECT_STREQ(toString(out.payload.event.body.fault_raised), "overcurrent");
  EXPECT_STREQ(payloadName(out), "event");
}

TEST(Messages, DecodeRejectsGarbage)
{
  const uint8_t junk[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  Envelope e;
  EXPECT_FALSE(decodeEnvelope(junk, sizeof(junk), e));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
