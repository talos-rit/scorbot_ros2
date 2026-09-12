#include "scorbot_protocol/messages.hpp"

#include <cstring>

#include "pb_decode.h"
#include "pb_encode.h"

namespace scorbot_protocol
{

bool encodeEnvelope(const Envelope& envelope, uint8_t* out, std::size_t cap, std::size_t& len)
{
  pb_ostream_t stream = pb_ostream_from_buffer(out, cap);
  if (!pb_encode(&stream, scorbot_v1_Envelope_fields, &envelope))
  {
    len = 0;
    return false;
  }
  len = stream.bytes_written;
  return true;
}

bool decodeEnvelope(const uint8_t* data, std::size_t len, Envelope& envelope)
{
  envelope = scorbot_v1_Envelope_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(data, len);
  return pb_decode(&stream, scorbot_v1_Envelope_fields, &envelope);
}

namespace
{
template <std::size_t N>
void copyString(char (&dst)[N], const std::string& src)
{
  std::strncpy(dst, src.c_str(), N - 1);
  dst[N - 1] = '\0';
}
}  // namespace

JointCommand& setJointCommand(Envelope& e, std::size_t joint_count)
{
  e.which_payload = scorbot_v1_Envelope_joint_command_tag;
  e.payload.joint_command = scorbot_v1_JointCommand_init_zero;
  const auto n = static_cast<pb_size_t>(joint_count > kMaxJoints ? kMaxJoints : joint_count);
  e.payload.joint_command.mode_count = n;
  e.payload.joint_command.position_count = n;
  e.payload.joint_command.velocity_count = n;
  return e.payload.joint_command;
}

JointState& setJointState(Envelope& e, std::size_t joint_count)
{
  e.which_payload = scorbot_v1_Envelope_joint_state_tag;
  e.payload.joint_state = scorbot_v1_JointState_init_zero;
  const auto n = static_cast<pb_size_t>(joint_count > kMaxJoints ? kMaxJoints : joint_count);
  e.payload.joint_state.position_count = n;
  e.payload.joint_state.velocity_count = n;
  e.payload.joint_state.current_count = n;
  e.payload.joint_state.flags_count = n;
  return e.payload.joint_state;
}

Request& setRequest(Envelope& e, uint32_t request_id)
{
  e.which_payload = scorbot_v1_Envelope_request_tag;
  e.payload.request = scorbot_v1_Request_init_zero;
  e.payload.request.request_id = request_id;
  return e.payload.request;
}

Response& setResponse(Envelope& e, uint32_t request_id, Result result)
{
  e.which_payload = scorbot_v1_Envelope_response_tag;
  e.payload.response = scorbot_v1_Response_init_zero;
  e.payload.response.request_id = request_id;
  e.payload.response.result = result;
  return e.payload.response;
}

Event& setEvent(Envelope& e)
{
  e.which_payload = scorbot_v1_Envelope_event_tag;
  e.payload.event = scorbot_v1_Event_init_zero;
  return e.payload.event;
}

void requestGetInfo(Request& r)
{
  r.which_body = scorbot_v1_Request_get_info_tag;
  r.body.get_info = scorbot_v1_GetInfoRequest_init_zero;
}

void requestSetCalibration(Request& r, const JointCalibration& joint)
{
  r.which_body = scorbot_v1_Request_set_calibration_tag;
  r.body.set_calibration.has_joint = true;
  r.body.set_calibration.joint = joint;
}

void requestSetRobotType(Request& r, const std::string& robot_type)
{
  r.which_body = scorbot_v1_Request_set_robot_type_tag;
  copyString(r.body.set_robot_type.robot_type, robot_type);
}

void requestSetWatchdog(Request& r, uint32_t watchdog_ms)
{
  r.which_body = scorbot_v1_Request_set_watchdog_tag;
  r.body.set_watchdog.watchdog_ms = watchdog_ms;
}

void requestHome(Request& r, uint32_t joint_mask, uint32_t timeout_ms)
{
  r.which_body = scorbot_v1_Request_home_tag;
  r.body.home.joint_mask = joint_mask;
  r.body.home.timeout_ms = timeout_ms;
}

void requestEnable(Request& r, bool allow_unhomed)
{
  r.which_body = scorbot_v1_Request_enable_tag;
  r.body.enable.allow_unhomed = allow_unhomed;
}

void requestDisable(Request& r)
{
  r.which_body = scorbot_v1_Request_disable_tag;
  r.body.disable = scorbot_v1_DisableRequest_init_zero;
}

void requestClearFault(Request& r)
{
  r.which_body = scorbot_v1_Request_clear_fault_tag;
  r.body.clear_fault = scorbot_v1_ClearFaultRequest_init_zero;
}

void requestPing(Request& r, uint32_t nonce)
{
  r.which_body = scorbot_v1_Request_ping_tag;
  r.body.ping.nonce = nonce;
}

const char* toString(ControlMode v)
{
  switch (v)
  {
    case scorbot_v1_ControlMode_CONTROL_MODE_NONE: return "none";
    case scorbot_v1_ControlMode_CONTROL_MODE_POSITION: return "position";
    case scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY: return "velocity";
  }
  return "?";
}

const char* toString(SystemState v)
{
  switch (v)
  {
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT: return "BOOT";
    case scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED: return "UNHOMED";
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING: return "HOMING";
    case scorbot_v1_SystemState_SYSTEM_STATE_READY: return "READY";
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE: return "ACTIVE";
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT: return "FAULT";
  }
  return "?";
}

const char* toString(FaultCode v)
{
  switch (v)
  {
    case scorbot_v1_FaultCode_FAULT_NONE: return "none";
    case scorbot_v1_FaultCode_FAULT_WATCHDOG: return "watchdog";
    case scorbot_v1_FaultCode_FAULT_OVERCURRENT: return "overcurrent";
    case scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH: return "limit_switch";
    case scorbot_v1_FaultCode_FAULT_SOFT_LIMIT: return "soft_limit";
    case scorbot_v1_FaultCode_FAULT_STALL: return "stall";
    case scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT: return "homing_timeout";
    case scorbot_v1_FaultCode_FAULT_ENCODER: return "encoder";
    case scorbot_v1_FaultCode_FAULT_DRIVER: return "driver";
    case scorbot_v1_FaultCode_FAULT_INTERNAL: return "internal";
  }
  return "?";
}

const char* toString(Result v)
{
  switch (v)
  {
    case scorbot_v1_Result_RESULT_OK: return "ok";
    case scorbot_v1_Result_RESULT_INVALID_STATE: return "invalid_state";
    case scorbot_v1_Result_RESULT_INVALID_ARGUMENT: return "invalid_argument";
    case scorbot_v1_Result_RESULT_BUSY: return "busy";
    case scorbot_v1_Result_RESULT_UNSUPPORTED: return "unsupported";
    case scorbot_v1_Result_RESULT_INTERNAL_ERROR: return "internal_error";
  }
  return "?";
}

const char* payloadName(const Envelope& e)
{
  switch (e.which_payload)
  {
    case scorbot_v1_Envelope_joint_command_tag: return "joint_command";
    case scorbot_v1_Envelope_joint_state_tag: return "joint_state";
    case scorbot_v1_Envelope_request_tag: return "request";
    case scorbot_v1_Envelope_response_tag: return "response";
    case scorbot_v1_Envelope_event_tag: return "event";
    default: return "empty";
  }
}

const char* requestName(const Request& r)
{
  switch (r.which_body)
  {
    case scorbot_v1_Request_get_info_tag: return "get_info";
    case scorbot_v1_Request_set_calibration_tag: return "set_calibration";
    case scorbot_v1_Request_set_robot_type_tag: return "set_robot_type";
    case scorbot_v1_Request_set_watchdog_tag: return "set_watchdog";
    case scorbot_v1_Request_home_tag: return "home";
    case scorbot_v1_Request_enable_tag: return "enable";
    case scorbot_v1_Request_disable_tag: return "disable";
    case scorbot_v1_Request_clear_fault_tag: return "clear_fault";
    case scorbot_v1_Request_ping_tag: return "ping";
    default: return "empty";
  }
}

}  // namespace scorbot_protocol
