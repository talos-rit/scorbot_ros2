#include "scorbot_system_controller/scorbot_system_controller.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "scorbot_protocol/messages.hpp"

namespace scorbot_system_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;
using namespace std::chrono_literals;

namespace
{
const char* const kCmdNames[] = {"home", "enable", "disable", "clear_fault"};
const char* const kStNames[] = {"state",      "fault_code",       "homed_mask", "limit_mask",  "link_age_ms",
                                "firmware_version", "boot_count", "last_result", "commands_ignored",
                                "request_count"};

std::string resultName(double v)
{
  if (v < 0.0)
    return "pending";
  return scorbot_protocol::toString(static_cast<scorbot_protocol::Result>(static_cast<int>(v)));
}

std::string versionString(double v)
{
  const int n = static_cast<int>(v);
  return std::to_string(n / 10000) + "." + std::to_string((n / 100) % 100) + "." + std::to_string(n % 100);
}
}  // namespace

CallbackReturn ScorbotSystemController::on_init()
{
  try
  {
    gpio_name_ = auto_declare<std::string>("gpio_name", "system");
    joints_ = auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    home_timeout_s_ = auto_declare<double>("home_timeout_s", 90.0);
    request_timeout_s_ = auto_declare<double>("request_timeout_s", 2.0);
    const double rate = auto_declare<double>("status_rate_hz", 10.0);
    status_period_s_ = rate > 0.0 ? 1.0 / rate : 0.0;
    link_timeout_ms_ = auto_declare<double>("link_timeout_ms", 100.0);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter error: %s", e.what());
    return CallbackReturn::ERROR;
  }
  for (auto& p : pending_)
    p.store(0.0);
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ScorbotSystemController::command_interface_configuration() const
{
  InterfaceConfiguration c;
  c.type = interface_configuration_type::INDIVIDUAL;
  for (const char* n : kCmdNames)
    c.names.push_back(gpio_name_ + "/" + n);
  return c;
}

InterfaceConfiguration ScorbotSystemController::state_interface_configuration() const
{
  InterfaceConfiguration c;
  c.type = interface_configuration_type::INDIVIDUAL;
  for (const char* n : kStNames)
    c.names.push_back(gpio_name_ + "/" + n);
  return c;
}

CallbackReturn ScorbotSystemController::on_configure(const rclcpp_lifecycle::State&)
{
  auto node = get_node();
  // Services block while the hardware works (homing takes seconds); give them their own
  // reentrant group so the controller manager's other callbacks keep running.
  service_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  home_srv_ = node->create_service<scorbot_msgs::srv::Home>(
      "~/home",
      [this](const std::shared_ptr<scorbot_msgs::srv::Home::Request> req,
             std::shared_ptr<scorbot_msgs::srv::Home::Response> resp) { onHome(req, resp); },
      rclcpp::ServicesQoS(), service_group_);
  enable_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/enable",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) { onEnable(req, resp, false); },
      rclcpp::ServicesQoS(), service_group_);
  enable_unhomed_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/enable_unhomed",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) { onEnable(req, resp, true); },
      rclcpp::ServicesQoS(), service_group_);
  disable_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/disable",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) { onDisable(req, resp); },
      rclcpp::ServicesQoS(), service_group_);
  clear_fault_srv_ = node->create_service<std_srvs::srv::Trigger>(
      "~/clear_fault",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) { onClearFault(req, resp); },
      rclcpp::ServicesQoS(), service_group_);
  status_pub_ = node->create_publisher<scorbot_msgs::msg::SystemStatus>("~/status", rclcpp::SystemDefaultsQoS());
  rt_status_pub_ = std::make_unique<realtime_tools::RealtimePublisher<scorbot_msgs::msg::SystemStatus>>(status_pub_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystemController::on_activate(const rclcpp_lifecycle::State&)
{
  if (command_interfaces_.size() != kCmdCount || state_interfaces_.size() != kStCount)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "expected %zu command and %zu state interfaces on gpio '%s', got %zu and %zu",
                 static_cast<std::size_t>(kCmdCount), static_cast<std::size_t>(kStCount), gpio_name_.c_str(),
                 command_interfaces_.size(), state_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  for (auto& p : pending_)
    p.store(0.0);
  active_.store(true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystemController::on_deactivate(const rclcpp_lifecycle::State&)
{
  active_.store(false);
  return CallbackReturn::SUCCESS;
}

return_type ScorbotSystemController::update(const rclcpp::Time& time, const rclcpp::Duration&)
{
  // Pulses requested by the services: written once, the hardware resets them to 0.
  for (std::size_t k = 0; k < kCmdCount; ++k)
  {
    const double v = pending_[k].exchange(0.0);
    if (v != 0.0 && !command_interfaces_[k].set_value(v))
      pending_[k].store(v);  // handle busy: try again next cycle
  }
  Snapshot s = snapshot();  // keep the last good value where a read fails
  for (std::size_t k = 0; k < kStCount; ++k)
    if (const auto v = state_interfaces_[k].get_optional())
      s.v[k] = *v;
  for (std::size_t k = 0; k < kCmdCount; ++k)
    if (const auto v = command_interfaces_[k].get_optional())
      s.cmd[k] = *v;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_ = s;
  }
  const int64_t now_ns = time.nanoseconds();
  if (status_period_s_ > 0.0 && static_cast<double>(now_ns - last_status_ns_) * 1e-9 >= status_period_s_)
  {
    last_status_ns_ = now_ns;
    publishStatus(time);
  }
  return return_type::OK;
}

void ScorbotSystemController::publishStatus(const rclcpp::Time& time)
{
  if (!rt_status_pub_ || !rt_status_pub_->trylock())
    return;
  const Snapshot s = snapshot();
  auto& m = rt_status_pub_->msg_;
  m.header.stamp = time;
  m.state_code = static_cast<uint8_t>(s.v[kState]);
  m.state = scorbot_protocol::toString(static_cast<scorbot_protocol::SystemState>(m.state_code));
  m.fault_code = static_cast<uint8_t>(s.v[kFaultCode]);
  m.fault = scorbot_protocol::toString(static_cast<scorbot_protocol::FaultCode>(m.fault_code));
  m.homed_mask = static_cast<uint32_t>(s.v[kHomedMask]);
  m.limit_mask = static_cast<uint32_t>(s.v[kLimitMask]);
  m.joints = joints_;
  m.homed.resize(joints_.size());
  m.limit_switch.resize(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    m.homed[i] = (m.homed_mask >> i) & 1u;
    m.limit_switch[i] = (m.limit_mask >> i) & 1u;
  }
  m.link_age_ms = s.v[kLinkAgeMs];
  m.link_ok = s.v[kLinkAgeMs] <= link_timeout_ms_;
  m.boot_count = static_cast<uint32_t>(s.v[kBootCount]);
  m.firmware_version = versionString(s.v[kFirmwareVersion]);
  m.last_result_code = static_cast<int32_t>(s.v[kLastResult]);
  m.last_result = resultName(s.v[kLastResult]);
  m.commands_ignored = static_cast<uint32_t>(s.v[kCommandsIgnored]);
  rt_status_pub_->unlockAndPublish();
}

ScorbotSystemController::Snapshot ScorbotSystemController::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

bool ScorbotSystemController::waitFor(const std::function<bool(const Snapshot&)>& pred, double timeout_s,
                                      std::string& message)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  for (;;)
  {
    if (!active_.load())
    {
      message = "system controller is not active";
      return false;
    }
    const Snapshot s = snapshot();
    if (s.v[kLinkAgeMs] > link_timeout_ms_ * 10.0)
    {
      message = "link to the controller is down";
      return false;
    }
    if (pred(s))
      return true;
    if (std::chrono::steady_clock::now() >= deadline)
    {
      message = "timed out after " + std::to_string(timeout_s) + " s in state " +
                scorbot_protocol::toString(static_cast<scorbot_protocol::SystemState>(static_cast<int>(s.v[kState])));
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
}

bool ScorbotSystemController::pulse(Cmd cmd, double value, std::string& message)
{
  if (!active_.load())
  {
    message = "system controller is not active";
    return false;
  }
  const double count_before = snapshot().v[kRequestCount];
  pending_[cmd].store(value);
  // Handshake with the hardware interface: update() writes the pulse, the hardware's
  // write() consumes it (request_count increments) and sets last_result to pending,
  // or straight to a refusal; its worker then stores the controller's Result.
  const bool done = waitFor(
      [&](const Snapshot& s) { return s.v[kRequestCount] != count_before && s.v[kLastResult] != kResultPending; },
      request_timeout_s_, message);
  if (!done)
    return false;
  const Snapshot s = snapshot();
  if (s.v[kLastResult] != 0.0)
  {
    message = std::string(kCmdNames[cmd]) + " refused: " + resultName(s.v[kLastResult]) +
              " (the hardware interface log says why)";
    return false;
  }
  return true;
}

uint32_t ScorbotSystemController::maskFor(const std::vector<std::string>& joints, std::string& error) const
{
  if (joints.empty())
    return 0;  // all
  uint32_t mask = 0;
  for (const std::string& name : joints)
  {
    bool found = false;
    for (std::size_t i = 0; i < joints_.size(); ++i)
      if (joints_[i] == name)
      {
        mask |= 1u << i;
        found = true;
      }
    if (!found)
    {
      error = "unknown joint '" + name + "'";
      return 0;
    }
  }
  return mask;
}

void ScorbotSystemController::onHome(const std::shared_ptr<scorbot_msgs::srv::Home::Request> req,
                                     std::shared_ptr<scorbot_msgs::srv::Home::Response> resp)
{
  std::lock_guard<std::mutex> lock(service_mutex_);
  std::string err;
  const uint32_t mask = maskFor(req->joints, err);
  if (!err.empty())
  {
    resp->success = false;
    resp->message = err;
    return;
  }
  const double timeout = req->timeout_s > 0.0f ? req->timeout_s : home_timeout_s_;
  const Snapshot before = snapshot();
  const auto st = static_cast<int>(before.v[kState]);
  if (st != scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED && st != scorbot_v1_SystemState_SYSTEM_STATE_READY)
  {
    resp->success = false;
    resp->message = std::string("cannot home from state ") +
                    scorbot_protocol::toString(static_cast<scorbot_protocol::SystemState>(st)) +
                    (st == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE ? " (disable first)" : "");
    return;
  }
  // 0 means "all" on the wire, but a nonzero pulse is needed to be noticed.
  const uint32_t all = joints_.empty() ? 0x1Fu : ((1u << joints_.size()) - 1u);
  if (!pulse(kHome, static_cast<double>(mask == 0 ? all : mask), resp->message))
  {
    resp->success = false;
    return;
  }
  const bool done = waitFor(
      [](const Snapshot& s) {
        const int v = static_cast<int>(s.v[kState]);
        return v == scorbot_v1_SystemState_SYSTEM_STATE_READY || v == scorbot_v1_SystemState_SYSTEM_STATE_FAULT ||
               v == scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED;
      },
      timeout, resp->message);
  const Snapshot after = snapshot();
  resp->homed_mask = static_cast<uint32_t>(after.v[kHomedMask]);
  const int v = static_cast<int>(after.v[kState]);
  if (!done)
  {
    resp->success = false;
    return;
  }
  if (v == scorbot_v1_SystemState_SYSTEM_STATE_FAULT)
  {
    resp->success = false;
    resp->message = std::string("homing faulted: ") +
                    scorbot_protocol::toString(static_cast<scorbot_protocol::FaultCode>(static_cast<int>(after.v[kFaultCode])));
    return;
  }
  resp->success = v == scorbot_v1_SystemState_SYSTEM_STATE_READY;
  resp->message = resp->success ? "homed" : "homing ended UNHOMED";
}

void ScorbotSystemController::onEnable(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> resp, bool allow_unhomed)
{
  std::lock_guard<std::mutex> lock(service_mutex_);
  if (!pulse(kEnable, allow_unhomed ? 2.0 : 1.0, resp->message))
  {
    resp->success = false;
    return;
  }
  resp->success = waitFor(
      [](const Snapshot& s) { return static_cast<int>(s.v[kState]) == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE; },
      request_timeout_s_, resp->message);
  if (resp->success)
    resp->message = allow_unhomed ? "drives enabled UNHOMED: no soft limits" : "drives enabled";
}

void ScorbotSystemController::onDisable(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  std::lock_guard<std::mutex> lock(service_mutex_);
  if (!pulse(kDisable, 1.0, resp->message))
  {
    resp->success = false;
    return;
  }
  resp->success = waitFor(
      [](const Snapshot& s) { return static_cast<int>(s.v[kState]) != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE; },
      request_timeout_s_, resp->message);
  if (resp->success)
    resp->message = "drives disabled";
}

void ScorbotSystemController::onClearFault(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  std::lock_guard<std::mutex> lock(service_mutex_);
  if (!pulse(kClearFault, 1.0, resp->message))
  {
    resp->success = false;
    return;
  }
  resp->success = waitFor(
      [](const Snapshot& s) { return static_cast<int>(s.v[kState]) != scorbot_v1_SystemState_SYSTEM_STATE_FAULT; },
      request_timeout_s_, resp->message);
  if (resp->success)
  {
    const Snapshot s = snapshot();
    resp->message = std::string("fault cleared, state ") +
                    scorbot_protocol::toString(static_cast<scorbot_protocol::SystemState>(static_cast<int>(s.v[kState])));
  }
}

}  // namespace scorbot_system_controller

PLUGINLIB_EXPORT_CLASS(scorbot_system_controller::ScorbotSystemController, controller_interface::ControllerInterface)
