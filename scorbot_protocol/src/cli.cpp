// scorbot_protocol_cli: talk to a Scorbot controller (or the simulator) from a shell.
//
//   scorbot_protocol_cli --port /dev/ttyUSB0 ping
//   scorbot_protocol_cli --port /dev/ttyUSB0 info
//   scorbot_protocol_cli --port /dev/ttyUSB0 monitor 5        # print JointState for 5 s
//   scorbot_protocol_cli --port /dev/ttyUSB0 enable [--allow-unhomed]
//   scorbot_protocol_cli --port /dev/ttyUSB0 home [mask] [timeout_ms]
//   scorbot_protocol_cli --port /dev/ttyUSB0 disable | clear-fault | watchdog <ms>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "scorbot_protocol/link.hpp"
#include "scorbot_protocol/transport.hpp"

using namespace scorbot_protocol;

namespace
{

void usage()
{
  std::fprintf(stderr,
               "usage: scorbot_protocol_cli --port <device> [--baud N] [--timeout ms] <command> [args]\n"
               "commands: ping | info | monitor [seconds] | enable [--allow-unhomed] | disable |\n"
               "          home [mask] [timeout_ms] | clear-fault | watchdog <ms> | stats [seconds]\n");
}

void printJointState(const Envelope& e)
{
  const JointState& s = e.payload.joint_state;
  std::printf("seq=%u t=%llu state=%s fault=%s rx=%u ign=%u loop=%uus\n", e.seq,
              static_cast<unsigned long long>(e.timestamp_us), toString(s.state), toString(s.fault),
              s.commands_received, s.commands_ignored, s.control_loop_period_us);
  for (pb_size_t i = 0; i < s.position_count; ++i)
  {
    const JointFlags f = i < s.flags_count ? s.flags[i] : JointFlags{};
    std::printf("  j%u pos=%+8.4f vel=%+8.4f cur=%+6.2f%s%s%s%s\n", i, s.position[i],
                i < s.velocity_count ? s.velocity[i] : 0.0f, i < s.current_count ? s.current[i] : 0.0f,
                f.homed ? " homed" : "", f.limit_switch ? " LIMIT" : "", f.at_soft_limit ? " softlim" : "",
                f.fault ? " FAULT" : "");
  }
}

void printEvent(const Envelope& e)
{
  const Event& ev = e.payload.event;
  switch (ev.which_body)
  {
    case scorbot_v1_Event_fault_raised_tag: std::printf("event: fault %s\n", toString(ev.body.fault_raised)); break;
    case scorbot_v1_Event_homing_progress_mask_tag: std::printf("event: homing progress 0x%02x\n", ev.body.homing_progress_mask); break;
    case scorbot_v1_Event_homing_complete_mask_tag: std::printf("event: homing complete 0x%02x\n", ev.body.homing_complete_mask); break;
    case scorbot_v1_Event_limit_switch_mask_tag: std::printf("event: limit switch 0x%02x\n", ev.body.limit_switch_mask); break;
    case scorbot_v1_Event_state_changed_tag: std::printf("event: state %s\n", toString(ev.body.state_changed)); break;
    default: std::printf("event: (empty)\n");
  }
}

int doRequest(Link& link, const Request& req, std::chrono::milliseconds timeout, Response& resp)
{
  if (!link.request(req, resp, timeout))
  {
    std::fprintf(stderr, "no response to %s within %lld ms (tx_err=%llu crc_err=%llu)\n", requestName(req),
                 static_cast<long long>(timeout.count()), static_cast<unsigned long long>(link.stats().tx_errors),
                 static_cast<unsigned long long>(link.stats().decoder.crc_errors));
    return 2;
  }
  if (resp.result != scorbot_v1_Result_RESULT_OK)
  {
    std::fprintf(stderr, "%s: %s %s\n", requestName(req), toString(resp.result), resp.message);
    return 1;
  }
  return 0;
}

void printInfo(const GetInfoResponse& i)
{
  std::printf("protocol %u.%u firmware %s robot_type %s boot_count %u watchdog_ms %u\n", i.protocol_major,
              i.protocol_minor, i.firmware_version, i.robot_type, i.boot_count, i.watchdog_ms);
  for (pb_size_t j = 0; j < i.joint_names_count; ++j)
  {
    std::printf("  [%u] %s", j, i.joint_names[j]);
    for (pb_size_t k = 0; k < i.calibration_count; ++k)
    {
      const JointCalibration& c = i.calibration[k];
      if (c.index != j)
        continue;
      std::printf("  cpr=%d ratio=%.2f home=%.3f dir=%d limits=[%.3f, %.3f] vmax=%.2f amax=%.2f imax=%.2f%s",
                  c.counts_per_motor_rev, c.gear_ratio, c.home_offset_rad, c.home_direction, c.soft_limit_min_rad,
                  c.soft_limit_max_rad, c.max_velocity_rad_s, c.max_accel_rad_s2, c.current_limit_a,
                  c.invert ? " inverted" : "");
      if (c.encoder_invert)
        std::printf(" encoder-inverted");
    }
    std::printf("\n");
  }
}

void monitor(Link& link, double seconds, bool states)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end)
  {
    link.transport().waitReadable(std::chrono::milliseconds(50));
    link.poll();
    Envelope e;
    while (link.receive(e))
    {
      if (e.which_payload == scorbot_v1_Envelope_joint_state_tag && states)
        printJointState(e);
      else if (e.which_payload == scorbot_v1_Envelope_event_tag)
        printEvent(e);
    }
  }
  const LinkStats& s = link.stats();
  std::printf("rx=%llu tx=%llu gaps=%llu crc_err=%llu cobs_err=%llu overflow=%llu decode_err=%llu dropped=%llu\n",
              static_cast<unsigned long long>(s.rx_envelopes), static_cast<unsigned long long>(s.tx_envelopes),
              static_cast<unsigned long long>(s.rx_seq_gaps), static_cast<unsigned long long>(s.decoder.crc_errors),
              static_cast<unsigned long long>(s.decoder.cobs_errors),
              static_cast<unsigned long long>(s.decoder.overflows),
              static_cast<unsigned long long>(s.rx_decode_errors), static_cast<unsigned long long>(s.rx_dropped));
}

}  // namespace

int main(int argc, char** argv)
{
  std::string port;
  int baud = 921600;
  int timeout_ms = 500;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc)
      port = argv[++i];
    else if (a == "--baud" && i + 1 < argc)
      baud = std::atoi(argv[++i]);
    else if (a == "--timeout" && i + 1 < argc)
      timeout_ms = std::atoi(argv[++i]);
    else if (a == "-h" || a == "--help")
    {
      usage();
      return 0;
    }
    else
      args.push_back(a);
  }
  if (port.empty() || args.empty())
  {
    usage();
    return 64;
  }

  SerialTransport serial;
  std::string error;
  if (!serial.open(port, baud, &error))
  {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 2;
  }
  Link link(serial);
  const auto timeout = std::chrono::milliseconds(timeout_ms);
  const std::string& cmd = args[0];
  Request req = scorbot_v1_Request_init_zero;
  Response resp = scorbot_v1_Response_init_zero;

  if (cmd == "ping")
  {
    const uint32_t nonce = static_cast<uint32_t>(link.now_us() & 0xFFFFFFFFu);
    requestPing(req, nonce);
    const auto t0 = std::chrono::steady_clock::now();
    if (const int rc = doRequest(link, req, timeout, resp))
      return rc;
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const bool ok = resp.which_body == scorbot_v1_Response_pong_tag && resp.body.pong == nonce;
    std::printf("pong %s in %.2f ms\n", ok ? "ok" : "WRONG NONCE", ms);
    return ok ? 0 : 1;
  }
  if (cmd == "info")
  {
    requestGetInfo(req);
    if (const int rc = doRequest(link, req, timeout, resp))
      return rc;
    if (resp.which_body != scorbot_v1_Response_get_info_tag)
    {
      std::fprintf(stderr, "response carried no info\n");
      return 1;
    }
    printInfo(resp.body.get_info);
    return 0;
  }
  if (cmd == "monitor" || cmd == "stats")
  {
    const double seconds = args.size() > 1 ? std::atof(args[1].c_str()) : 5.0;
    monitor(link, seconds, cmd == "monitor");
    return 0;
  }
  if (cmd == "enable")
  {
    const bool allow_unhomed = args.size() > 1 && args[1] == "--allow-unhomed";
    requestEnable(req, allow_unhomed);
  }
  else if (cmd == "disable")
    requestDisable(req);
  else if (cmd == "clear-fault")
    requestClearFault(req);
  else if (cmd == "home")
  {
    const uint32_t mask = args.size() > 1 ? static_cast<uint32_t>(std::strtoul(args[1].c_str(), nullptr, 0)) : 0;
    const uint32_t to = args.size() > 2 ? static_cast<uint32_t>(std::atoi(args[2].c_str())) : 0;
    requestHome(req, mask, to);
  }
  else if (cmd == "watchdog" && args.size() > 1)
    requestSetWatchdog(req, static_cast<uint32_t>(std::atoi(args[1].c_str())));
  else
  {
    usage();
    return 64;
  }
  if (const int rc = doRequest(link, req, timeout, resp))
    return rc;
  std::printf("%s: ok\n", requestName(req));
  return 0;
}
