// C++ conveniences over the nanopb-generated scorbot.v1 messages: encode/decode of an
// Envelope to bytes, typed builders for the common requests, and enum-to-string.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "scorbot/v1/scorbot.pb.h"
#include "scorbot_protocol/framing.hpp"

namespace scorbot_protocol
{

constexpr uint32_t kProtocolMajor = 1;
constexpr uint32_t kProtocolMinor = 0;
constexpr std::size_t kMaxJoints = 8;

using Envelope = scorbot_v1_Envelope;
using JointCommand = scorbot_v1_JointCommand;
using JointState = scorbot_v1_JointState;
using JointFlags = scorbot_v1_JointFlags;
using JointCalibration = scorbot_v1_JointCalibration;
using Request = scorbot_v1_Request;
using Response = scorbot_v1_Response;
using Event = scorbot_v1_Event;
using GetInfoResponse = scorbot_v1_GetInfoResponse;
using ControlMode = scorbot_v1_ControlMode;
using SystemState = scorbot_v1_SystemState;
using FaultCode = scorbot_v1_FaultCode;
using Result = scorbot_v1_Result;

static_assert(scorbot_v1_Envelope_size <= kMaxPayload,
              "kMaxPayload must hold the largest possible Envelope");

/// Serialize an Envelope. Returns false if it does not fit or is invalid.
bool encodeEnvelope(const Envelope& envelope, uint8_t* out, std::size_t cap, std::size_t& len);

/// Parse an Envelope. Returns false on malformed input.
bool decodeEnvelope(const uint8_t* data, std::size_t len, Envelope& envelope);

/// A zeroed Envelope with header fields set and no payload yet.
inline Envelope makeEnvelope(uint32_t seq, uint64_t timestamp_us)
{
  Envelope e = scorbot_v1_Envelope_init_zero;
  e.seq = seq;
  e.timestamp_us = timestamp_us;
  return e;
}

// ---- payload builders (each sets which_payload and returns the payload for editing) --

JointCommand& setJointCommand(Envelope& e, std::size_t joint_count);
JointState& setJointState(Envelope& e, std::size_t joint_count);
Request& setRequest(Envelope& e, uint32_t request_id);
Response& setResponse(Envelope& e, uint32_t request_id, Result result);
Event& setEvent(Envelope& e);

// ---- request builders (fill the Request body) -----------------------------------------

void requestGetInfo(Request& r);
void requestSetCalibration(Request& r, const JointCalibration& joint);
void requestSetRobotType(Request& r, const std::string& robot_type);
void requestSetWatchdog(Request& r, uint32_t watchdog_ms);
void requestHome(Request& r, uint32_t joint_mask, uint32_t timeout_ms);
void requestEnable(Request& r, bool allow_unhomed);
void requestDisable(Request& r);
void requestClearFault(Request& r);
void requestPing(Request& r, uint32_t nonce);

// ---- names -----------------------------------------------------------------------------

const char* toString(ControlMode v);
const char* toString(SystemState v);
const char* toString(FaultCode v);
const char* toString(Result v);
/// Name of the payload type carried by an Envelope ("joint_state", ...).
const char* payloadName(const Envelope& e);
/// Name of the request type ("ping", "home", ...).
const char* requestName(const Request& r);

}  // namespace scorbot_protocol
