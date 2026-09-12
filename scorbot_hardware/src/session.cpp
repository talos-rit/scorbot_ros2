#include "scorbot_hardware/session.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace scorbot_hardware
{

using namespace std::chrono_literals;
using scorbot_protocol::Envelope;
using scorbot_protocol::Request;
using scorbot_protocol::Response;
using scorbot_protocol::SystemState;

const char* const kGpioCommandNames[kGpioCommandCount] = {"home", "enable", "disable", "clear_fault"};
const char* const kGpioStateNames[kGpioStateCount] = {"state",       "fault_code",       "homed_mask",
                                                      "limit_mask",  "link_age_ms",      "firmware_version",
                                                      "boot_count",  "last_result",      "commands_ignored",
                                                      "request_count"};

double versionNumber(const std::string& version)
{
  int parts[3] = {0, 0, 0};
  int n = 0;
  std::size_t i = 0;
  while (i < version.size() && n < 3)
  {
    if (std::isdigit(static_cast<unsigned char>(version[i])))
    {
      int v = 0;
      while (i < version.size() && std::isdigit(static_cast<unsigned char>(version[i])))
        v = v * 10 + (version[i++] - '0');
      parts[n++] = v;
      if (i < version.size() && version[i] == '.')
        ++i;
      else
        break;
    }
    else
      ++i;
  }
  return parts[0] * 10000.0 + parts[1] * 100.0 + parts[2];
}

namespace
{

uint32_t homedMask(const scorbot_protocol::JointState& s)
{
  uint32_t m = 0;
  for (pb_size_t i = 0; i < s.flags_count && i < 32; ++i)
    if (s.flags[i].homed)
      m |= 1u << i;
  return m;
}

uint32_t limitMask(const scorbot_protocol::JointState& s)
{
  uint32_t m = 0;
  for (pb_size_t i = 0; i < s.flags_count && i < 32; ++i)
    if (s.flags[i].limit_switch)
      m |= 1u << i;
  return m;
}

}  // namespace

Session::Session(SessionConfig config) : config_(std::move(config))
{
  const std::size_t n = config_.joint_names.size();
  position_state.assign(n, 0.0);
  velocity_state.assign(n, 0.0);
  effort_state.assign(n, 0.0);
  position_command.assign(n, std::numeric_limits<double>::quiet_NaN());
  velocity_command.assign(n, std::numeric_limits<double>::quiet_NaN());
  modes_.assign(n, JointMode::kNone);
  gpio_state.fill(0.0);
  gpio_command.fill(0.0);
  for (std::size_t i = 0; i < config_.calibration.size(); ++i)
    config_.calibration[i].index = static_cast<uint32_t>(i);
  logger_ = [](int level, const std::string& m) {
    static const char* names[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    std::fprintf(stderr, "[scorbot_hardware %s] %s\n", names[level < 0 ? 0 : level > 3 ? 3 : level], m.c_str());
  };
}

Session::~Session() { cleanup(); }

void Session::log(int level, const std::string& message) const
{
  if (logger_)
    logger_(level, message);
}

// ---- lifecycle -------------------------------------------------------------------------

bool Session::requestOk(const Request& req, Response& resp, std::string& error)
{
  if (!client_ || !client_->request(req, resp, std::chrono::milliseconds(config_.request_timeout_ms), &error))
    return false;
  if (resp.result != scorbot_v1_Result_RESULT_OK)
  {
    error = std::string(scorbot_protocol::requestName(req)) + " refused: " + scorbot_protocol::toString(resp.result) +
            (resp.message[0] ? std::string(" (") + resp.message + ")" : "");
    return false;
  }
  return true;
}

bool Session::waitForState(std::function<bool(const scorbot_protocol::JointState&)> pred, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  const uint64_t fresh_us = static_cast<uint64_t>(config_.state_timeout_ms) * 1000;
  for (;;)
  {
    const StateSnapshot snap = client_->latestState();
    if (snap.valid && client_->stateAgeUs() <= fresh_us && pred(snap.state))
      return true;
    if (std::chrono::steady_clock::now() >= deadline)
      return false;
    std::this_thread::sleep_for(2ms);
  }
}

bool Session::configure(std::unique_ptr<scorbot_protocol::Transport> transport, std::string& error)
{
  if (client_)
  {
    error = "already configured";
    return false;
  }
  if (!transport || !transport->isOpen())
  {
    error = "transport is not open";
    return false;
  }
  if (config_.calibration.size() != config_.joint_names.size())
  {
    error = "calibration has " + std::to_string(config_.calibration.size()) + " joints, description has " +
            std::to_string(config_.joint_names.size());
    return false;
  }
  const std::string where = transport->describe();
  client_ = std::make_unique<ControllerClient>(std::move(transport));
  client_->start();
  auto fail = [&](const std::string& what) {
    error = what;
    log(kError, "configure failed on " + where + ": " + what);
    client_->stop();
    client_.reset();
    return false;
  };

  // GetInfo, retried while the controller may still be booting.
  Request req = scorbot_v1_Request_init_zero;
  Response resp = scorbot_v1_Response_init_zero;
  scorbot_protocol::requestGetInfo(req);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(config_.activate_timeout_s);
  std::string err;
  for (;;)
  {
    if (requestOk(req, resp, err) && resp.which_body == scorbot_v1_Response_get_info_tag)
      break;
    if (std::chrono::steady_clock::now() >= deadline)
      return fail("controller does not answer GetInfo: " + err);
    std::this_thread::sleep_for(50ms);
  }
  info_ = resp.body.get_info;

  if (info_.protocol_major != scorbot_protocol::kProtocolMajor)
    return fail("protocol major mismatch: controller speaks " + std::to_string(info_.protocol_major) + "." +
                std::to_string(info_.protocol_minor) + ", this interface speaks " +
                std::to_string(scorbot_protocol::kProtocolMajor) + "." + std::to_string(scorbot_protocol::kProtocolMinor));
  if (info_.protocol_minor != scorbot_protocol::kProtocolMinor)
    log(kWarn, "protocol minor mismatch: controller " + std::to_string(info_.protocol_major) + "." +
                   std::to_string(info_.protocol_minor) + ", interface " + std::to_string(scorbot_protocol::kProtocolMajor) +
                   "." + std::to_string(scorbot_protocol::kProtocolMinor));

  if (info_.joint_names_count != config_.joint_names.size())
    return fail("controller has " + std::to_string(info_.joint_names_count) + " joints, description has " +
                std::to_string(config_.joint_names.size()));
  for (std::size_t i = 0; i < config_.joint_names.size(); ++i)
  {
    const std::string expected = config_.joint_names[i].compare(0, config_.prefix.size(), config_.prefix) == 0
                                     ? config_.joint_names[i].substr(config_.prefix.size())
                                     : config_.joint_names[i];
    if (expected != info_.joint_names[i])
      return fail("joint " + std::to_string(i) + " is '" + info_.joint_names[i] + "' on the controller but '" +
                  expected + "' in the description");
  }

  const std::string ctrl_type = info_.robot_type;
  if (ctrl_type.empty() || ctrl_type == "unset")
  {
    scorbot_protocol::requestSetRobotType(req, config_.robot_type);
    if (!requestOk(req, resp, err))
      return fail(err);
    log(kInfo, "controller had no robot type; set to " + config_.robot_type);
  }
  else if (ctrl_type != config_.robot_type)
    return fail("controller says it is '" + ctrl_type + "', the description is '" + config_.robot_type + "'");

  for (std::size_t i = 0; i < config_.calibration.size(); ++i)
  {
    scorbot_protocol::requestSetCalibration(req, config_.calibration[i]);
    if (!requestOk(req, resp, err))
      return fail("calibration for '" + config_.joint_names[i] + "': " + err);
  }
  scorbot_protocol::requestSetWatchdog(req, config_.watchdog_ms);
  if (!requestOk(req, resp, err))
    return fail(err);

  boot_count_ = info_.boot_count;
  have_state_ = false;
  gpio_state[kGpioFirmwareVersion] = versionNumber(info_.firmware_version);
  gpio_state[kGpioBootCount] = boot_count_;
  last_result_.store(0.0);

  if (!waitForState([](const scorbot_protocol::JointState&) { return true; }, config_.activate_timeout_s))
    return fail("controller answers requests but streams no JointState");
  if (!waitForState([](const scorbot_protocol::JointState& s) { return s.state != scorbot_v1_SystemState_SYSTEM_STATE_BOOT; },
                    config_.activate_timeout_s))
    return fail("controller stays in BOOT (self-test not finishing)");

  worker_running_.store(true);
  worker_ = std::thread([this] { workerLoop(); });
  log(kInfo, "configured " + config_.robot_type + " on " + where + ": firmware " + info_.firmware_version +
                 ", boot " + std::to_string(boot_count_) + ", protocol " + std::to_string(info_.protocol_major) +
                 "." + std::to_string(info_.protocol_minor));
  return true;
}

bool Session::homeAndWait(std::string& error)
{
  Request req = scorbot_v1_Request_init_zero;
  Response resp;
  scorbot_protocol::requestHome(req, 0, static_cast<uint32_t>(config_.home_timeout_s * 1000.0));
  if (!requestOk(req, resp, error))
    return false;
  log(kInfo, "homing all joints");
  const bool done = waitForState(
      [](const scorbot_protocol::JointState& s) {
        return s.state == scorbot_v1_SystemState_SYSTEM_STATE_READY || s.state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT;
      },
      config_.home_timeout_s + 1.0);
  const StateSnapshot snap = client_->latestState();
  if (!done || snap.state.state != scorbot_v1_SystemState_SYSTEM_STATE_READY)
  {
    error = std::string("homing did not finish: state ") + scorbot_protocol::toString(snap.state.state) +
            ", fault " + scorbot_protocol::toString(snap.state.fault);
    return false;
  }
  return true;
}

bool Session::enableAndWait(bool allow_unhomed, std::string& error)
{
  Request req = scorbot_v1_Request_init_zero;
  Response resp;
  scorbot_protocol::requestEnable(req, allow_unhomed);
  if (!requestOk(req, resp, error))
    return false;
  if (!waitForState([](const scorbot_protocol::JointState& s) { return s.state == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE; },
                    1.0))
  {
    error = "controller accepted Enable but did not report ACTIVE";
    return false;
  }
  return true;
}

bool Session::activate(std::string& error)
{
  if (!client_)
  {
    error = "not configured";
    return false;
  }
  if (!waitForState([](const scorbot_protocol::JointState& s) { return s.state != scorbot_v1_SystemState_SYSTEM_STATE_BOOT; },
                    config_.activate_timeout_s))
  {
    error = "no fresh JointState from the controller, or it is stuck in BOOT";
    return false;
  }
  // Seed the state and command interfaces so the first write() cannot jump.
  if (read() != ReadStatus::kOk)
  {
    error = "telemetry not usable (link lost or controller rebooted)";
    return false;
  }
  for (std::size_t i = 0; i < jointCount(); ++i)
  {
    position_command[i] = position_state[i];
    velocity_command[i] = 0.0;
  }

  StateSnapshot snap = client_->latestState();
  const uint32_t all = jointCount() >= 32 ? 0xFFFFFFFFu : ((1u << jointCount()) - 1u);
  if (config_.auto_home_on_activate && snap.state.state != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE &&
      (homedMask(snap.state) & all) != all)
  {
    if (snap.state.state != scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED &&
        snap.state.state != scorbot_v1_SystemState_SYSTEM_STATE_READY)
    {
      error = std::string("cannot home from state ") + scorbot_protocol::toString(snap.state.state);
      return false;
    }
    if (!homeAndWait(error))
      return false;
    snap = client_->latestState();
  }

  switch (snap.state.state)
  {
    case scorbot_v1_SystemState_SYSTEM_STATE_READY:
      if (!enableAndWait(false, error))
        return false;
      log(kInfo, "drives enabled");
      break;
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE:
      break;
    case scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED:
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT:
      if (config_.allow_unhomed)
      {
        if (!enableAndWait(true, error))
          return false;
        log(kWarn, "drives enabled UNHOMED (allow_unhomed): no soft limits, positions are relative");
      }
      else
        log(kWarn, "controller is UNHOMED: commands are ignored until it is homed and enabled "
                   "(system controller ~/home then ~/enable, with the joint controllers stopped)");
      break;
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING:
      log(kWarn, "controller is still HOMING; enable it when it reaches READY");
      break;
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT:
      log(kWarn, std::string("controller is in FAULT (") + scorbot_protocol::toString(snap.state.fault) +
                     "): clear it through the system controller before commanding");
      break;
    default:
      break;
  }
  read();
  return true;
}

void Session::deactivate()
{
  if (!client_)
    return;
  Request req = scorbot_v1_Request_init_zero;
  Response resp;
  std::string err;
  scorbot_protocol::requestDisable(req);
  if (!requestOk(req, resp, err))
    log(kWarn, "Disable on deactivate: " + err);
  else
    log(kInfo, "drives disabled");
}

void Session::cleanup()
{
  if (worker_running_.exchange(false))
  {
    jobs_cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
  }
  if (client_)
  {
    if (client_->running() && !client_->transportFailed() && last_state_ == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE)
      deactivate();
    client_->stop();
    client_.reset();
  }
  have_state_ = false;
}

// ---- cyclic ------------------------------------------------------------------------------

ReadStatus Session::read()
{
  if (!client_)
    return ReadStatus::kLinkLost;
  const StateSnapshot snap = client_->latestState();
  const uint64_t age_us = client_->stateAgeUs();
  gpio_state[kGpioLinkAgeMs] = age_us == UINT64_MAX ? 1e9 : static_cast<double>(age_us) / 1000.0;
  gpio_state[kGpioLastResult] = last_result_.load();
  gpio_state[kGpioRequestCount] = request_count_;
  handleEvents();

  if (!snap.valid || client_->transportFailed() || age_us > static_cast<uint64_t>(config_.state_timeout_ms) * 1000)
  {
    if (have_state_)
      log(kError, client_->transportFailed() ? "link to the controller failed" :
                                               "no JointState for " + std::to_string(age_us / 1000) + " ms");
    have_state_ = false;
    return ReadStatus::kLinkLost;
  }

  const scorbot_protocol::JointState& s = snap.state;
  const std::size_t n = jointCount();
  for (std::size_t i = 0; i < n; ++i)
  {
    position_state[i] = i < s.position_count ? s.position[i] : 0.0;
    velocity_state[i] = i < s.velocity_count ? s.velocity[i] : 0.0;
    effort_state[i] = i < s.current_count ? s.current[i] : 0.0;
  }
  gpio_state[kGpioSystemState] = static_cast<double>(s.state);
  gpio_state[kGpioFaultCode] = static_cast<double>(s.fault);
  gpio_state[kGpioHomedMask] = static_cast<double>(homedMask(s));
  gpio_state[kGpioLimitMask] = static_cast<double>(limitMask(s));
  gpio_state[kGpioBootCount] = static_cast<double>(s.boot_count);
  gpio_state[kGpioCommandsIgnored] = static_cast<double>(s.commands_ignored);

  if (s.boot_count != boot_count_)
  {
    if (have_state_ || s.boot_count != 0)
    {
      log(kError, "controller rebooted (boot " + std::to_string(boot_count_) + " -> " + std::to_string(s.boot_count) +
                      "): calibration and homing are gone; reconfigure the hardware");
      last_state_ = s.state;
      have_state_ = true;
      return ReadStatus::kRebooted;
    }
  }
  if (!have_state_ || s.state != last_state_)
    log(s.state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT ? kError : kInfo,
        std::string("controller state ") + scorbot_protocol::toString(s.state) +
            (s.state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT ? std::string(", fault ") + scorbot_protocol::toString(s.fault) : ""));
  last_state_ = s.state;
  have_state_ = true;
  return ReadStatus::kOk;
}

void Session::write()
{
  if (!client_)
    return;
  handleGpioCommands();
  gpio_state[kGpioLastResult] = last_result_.load();  // a refusal is visible this cycle
  gpio_state[kGpioRequestCount] = request_count_;
  if (!have_state_ || last_state_ != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE)
    return;  // the controller would ignore it; keep commands_ignored meaningful

  Envelope e = client_->newEnvelope();
  scorbot_protocol::JointCommand& cmd = scorbot_protocol::setJointCommand(e, jointCount());
  for (std::size_t i = 0; i < jointCount() && i < scorbot_protocol::kMaxJoints; ++i)
  {
    cmd.mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    cmd.position[i] = 0.0f;
    cmd.velocity[i] = 0.0f;
    switch (modes_[i])
    {
      case JointMode::kPosition:
        if (!std::isnan(position_command[i]))
        {
          cmd.mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_POSITION;
          cmd.position[i] = static_cast<float>(position_command[i]);
        }
        break;
      case JointMode::kVelocity:
        if (!std::isnan(velocity_command[i]))
        {
          cmd.mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
          cmd.velocity[i] = static_cast<float>(velocity_command[i]);
        }
        break;
      default:
        break;
    }
  }
  if (client_->send(e))
    ++commands_sent_;
}

void Session::handleEvents()
{
  for (const scorbot_protocol::Event& ev : client_->takeEvents())
  {
    switch (ev.which_body)
    {
      case scorbot_v1_Event_fault_raised_tag:
        log(kError, std::string("controller fault raised: ") + scorbot_protocol::toString(ev.body.fault_raised));
        break;
      case scorbot_v1_Event_homing_progress_mask_tag:
        log(kInfo, "homing progress: joints homed mask 0x" + [&] {
          char buf[16];
          std::snprintf(buf, sizeof(buf), "%x", ev.body.homing_progress_mask);
          return std::string(buf);
        }());
        break;
      case scorbot_v1_Event_homing_complete_mask_tag:
        log(kInfo, "homing complete");
        break;
      case scorbot_v1_Event_limit_switch_mask_tag:
        log(kDebug, "limit switch edge, mask " + std::to_string(ev.body.limit_switch_mask));
        break;
      case scorbot_v1_Event_state_changed_tag:
        break;  // reported from the JointState stream, which is authoritative
      default:
        break;
    }
  }
}

// ---- GPIO commands -> worker -----------------------------------------------------------

void Session::handleGpioCommands()
{
  for (std::size_t k = 0; k < kGpioCommandCount; ++k)
  {
    const double v = gpio_command[k];
    if (std::isnan(v) || v == 0.0)
      continue;
    gpio_command[k] = 0.0;  // consumed: a pulse
    ++request_count_;
    Job job;
    job.request = scorbot_v1_Request_init_zero;
    job.name = kGpioCommandNames[k];
    switch (k)
    {
      case kGpioHome:
      {
        if (anyJointClaimed())
        {
          last_result_.store(static_cast<double>(scorbot_v1_Result_RESULT_INVALID_STATE));
          log(kWarn, "home refused: joint controllers are active; deactivate them first");
          continue;
        }
        const uint32_t mask = v < 0.0 ? 0u : static_cast<uint32_t>(v);
        scorbot_protocol::requestHome(job.request, mask, static_cast<uint32_t>(config_.home_timeout_s * 1000.0));
        break;
      }
      case kGpioEnable:
      {
        // Enabling hands control to whatever the joint controllers are writing right now.
        // Refuse if that would move the arm: a stale hold point after homing, for example.
        for (std::size_t i = 0; i < jointCount(); ++i)
        {
          const bool jumps = (modes_[i] == JointMode::kPosition && !std::isnan(position_command[i]) &&
                              std::fabs(position_command[i] - position_state[i]) > config_.enable_position_tolerance_rad) ||
                             (modes_[i] == JointMode::kVelocity && !std::isnan(velocity_command[i]) &&
                              velocity_command[i] != 0.0);
          if (jumps)
          {
            last_result_.store(static_cast<double>(scorbot_v1_Result_RESULT_INVALID_STATE));
            log(kWarn, "enable refused: '" + config_.joint_names[i] +
                           "' is commanded away from its current position; stop the joint controllers first");
            job.name.clear();
            break;
          }
        }
        if (job.name.empty())
          continue;
        scorbot_protocol::requestEnable(job.request, v >= 2.0);
        break;
      }
      case kGpioDisable:
        scorbot_protocol::requestDisable(job.request);
        break;
      case kGpioClearFault:
        scorbot_protocol::requestClearFault(job.request);
        break;
      default:
        continue;
    }
    enqueue(std::move(job));
  }
}

void Session::enqueue(Job job)
{
  std::lock_guard<std::mutex> lock(jobs_mutex_);
  if (job_active_ || !jobs_.empty())
  {
    last_result_.store(static_cast<double>(scorbot_v1_Result_RESULT_BUSY));
    log(kWarn, job.name + " dropped: another request is in flight");
    return;
  }
  last_result_.store(kResultPending);
  jobs_.push_back(std::move(job));
  jobs_cv_.notify_all();
}

void Session::workerLoop()
{
  while (worker_running_.load())
  {
    Job job;
    {
      std::unique_lock<std::mutex> lock(jobs_mutex_);
      jobs_cv_.wait(lock, [&] { return !jobs_.empty() || !worker_running_.load(); });
      if (!worker_running_.load())
        return;
      job = std::move(jobs_.front());
      jobs_.pop_front();
      job_active_ = true;
    }
    Response resp = scorbot_v1_Response_init_zero;
    std::string err;
    double result;
    if (client_->request(job.request, resp, std::chrono::milliseconds(config_.request_timeout_ms), &err))
    {
      result = static_cast<double>(resp.result);
      if (resp.result == scorbot_v1_Result_RESULT_OK)
        log(kInfo, job.name + ": ok");
      else
        log(kWarn, job.name + " refused: " + scorbot_protocol::toString(resp.result) +
                       (resp.message[0] ? std::string(" (") + resp.message + ")" : ""));
    }
    else
    {
      result = static_cast<double>(scorbot_v1_Result_RESULT_INTERNAL_ERROR);
      log(kError, job.name + ": " + err);
    }
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      last_result_.store(result);
      job_active_ = false;
      jobs_cv_.notify_all();
    }
  }
}

void Session::waitIdle()
{
  std::unique_lock<std::mutex> lock(jobs_mutex_);
  jobs_cv_.wait(lock, [&] { return (!job_active_ && jobs_.empty()) || !worker_running_.load(); });
}

// ---- command modes -----------------------------------------------------------------------

int Session::jointIndex(const std::string& interface_name, std::string& interface) const
{
  const std::size_t slash = interface_name.rfind('/');
  if (slash == std::string::npos)
    return -1;
  const std::string joint = interface_name.substr(0, slash);
  interface = interface_name.substr(slash + 1);
  for (std::size_t i = 0; i < config_.joint_names.size(); ++i)
    if (config_.joint_names[i] == joint)
      return static_cast<int>(i);
  return -1;
}

std::vector<JointMode> Session::modesAfter(const std::vector<std::string>& start,
                                           const std::vector<std::string>& stop) const
{
  // Per joint: which command interfaces are claimed after the switch.
  std::vector<bool> pos(jointCount()), vel(jointCount());
  for (std::size_t i = 0; i < jointCount(); ++i)
  {
    pos[i] = modes_[i] == JointMode::kPosition;
    vel[i] = modes_[i] == JointMode::kVelocity;
  }
  std::string iface;
  for (const std::string& name : stop)
  {
    const int j = jointIndex(name, iface);
    if (j < 0)
      continue;
    if (iface == "position")
      pos[j] = false;
    else if (iface == "velocity")
      vel[j] = false;
  }
  for (const std::string& name : start)
  {
    const int j = jointIndex(name, iface);
    if (j < 0)
      continue;
    if (iface == "position")
      pos[j] = true;
    else if (iface == "velocity")
      vel[j] = true;
  }
  std::vector<JointMode> out(jointCount(), JointMode::kNone);
  for (std::size_t i = 0; i < jointCount(); ++i)
  {
    if (pos[i] && vel[i])
      out[i] = static_cast<JointMode>(-1);  // conflict marker
    else if (pos[i])
      out[i] = JointMode::kPosition;
    else if (vel[i])
      out[i] = JointMode::kVelocity;
  }
  return out;
}

bool Session::prepareModeSwitch(const std::vector<std::string>& start, const std::vector<std::string>& stop,
                                std::string& error) const
{
  std::string iface;
  for (const std::string& name : start)
  {
    const int j = jointIndex(name, iface);
    if (j < 0)
      continue;  // GPIO interfaces carry no mode
    if (iface != "position" && iface != "velocity")
    {
      error = "unknown command interface '" + name + "'";
      return false;
    }
  }
  const std::vector<JointMode> after = modesAfter(start, stop);
  for (std::size_t i = 0; i < after.size(); ++i)
    if (after[i] == static_cast<JointMode>(-1))
    {
      error = "'" + config_.joint_names[i] + "' would have both position and velocity claimed";
      return false;
    }
  return true;
}

void Session::performModeSwitch(const std::vector<std::string>& start, const std::vector<std::string>& stop)
{
  const std::vector<JointMode> after = modesAfter(start, stop);
  for (std::size_t i = 0; i < after.size(); ++i)
  {
    if (after[i] == static_cast<JointMode>(-1))
      continue;  // prepare would have refused; keep the old mode
    if (after[i] != modes_[i])
    {
      if (after[i] == JointMode::kPosition)
        position_command[i] = position_state[i];
      if (after[i] == JointMode::kVelocity)
        velocity_command[i] = 0.0;
      modes_[i] = after[i];
    }
  }
}

bool Session::anyJointClaimed() const
{
  return std::any_of(modes_.begin(), modes_.end(), [](JointMode m) { return m != JointMode::kNone; });
}

}  // namespace scorbot_hardware
