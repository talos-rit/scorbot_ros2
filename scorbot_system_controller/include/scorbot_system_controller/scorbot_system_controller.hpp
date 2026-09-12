// ScorbotSystemController: the non-realtime service surface of the ESP32 controller,
// multiplexed through scorbot_hardware's "system" GPIO interfaces (design D8, 6.5).
//
//   ~/home         scorbot_msgs/srv/Home   home joints, wait for READY
//   ~/enable       std_srvs/srv/Trigger    Enable, wait for ACTIVE
//   ~/enable_unhomed  std_srvs/srv/Trigger Enable(allow_unhomed): bench only
//   ~/disable      std_srvs/srv/Trigger    Disable, wait for READY/UNHOMED
//   ~/clear_fault  std_srvs/srv/Trigger    ClearFault, wait for the fault to clear
//   ~/status       scorbot_msgs/msg/SystemStatus at status_rate_hz
//
// The hardware refuses home while joint controllers are active and enable while a
// joint controller commands a position away from the current one; the services
// report those refusals in their message.

#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "scorbot_msgs/msg/system_status.hpp"
#include "scorbot_msgs/srv/home.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace scorbot_system_controller
{

class ScorbotSystemController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  // Indices into command_interfaces_ / state_interfaces_ follow the order in the
  // configuration functions, which follow scorbot_hardware's kGpio*Names.
  enum Cmd : std::size_t { kHome = 0, kEnable, kDisable, kClearFault, kCmdCount };
  enum St : std::size_t
  {
    kState = 0, kFaultCode, kHomedMask, kLimitMask, kLinkAgeMs, kFirmwareVersion, kBootCount, kLastResult,
    kCommandsIgnored, kRequestCount, kStCount
  };
  static constexpr double kResultPending = -1.0;

  /// Snapshot of the state and command interfaces, written by update(), read by the
  /// services. A command value back at 0 means the hardware consumed the pulse.
  struct Snapshot
  {
    std::array<double, kStCount> v{};
    std::array<double, kCmdCount> cmd{};
  };

  void onHome(const std::shared_ptr<scorbot_msgs::srv::Home::Request> req,
              std::shared_ptr<scorbot_msgs::srv::Home::Response> resp);
  void onEnable(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> resp,
                bool allow_unhomed);
  void onDisable(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void onClearFault(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  /// Pulse one GPIO command and wait for the hardware's result. Returns true on RESULT_OK.
  bool pulse(Cmd cmd, double value, std::string& message);
  /// Wait until `pred(snapshot)` or `timeout_s`; false on timeout or link loss.
  bool waitFor(const std::function<bool(const Snapshot&)>& pred, double timeout_s, std::string& message);
  Snapshot snapshot() const;
  uint32_t maskFor(const std::vector<std::string>& joints, std::string& error) const;
  void publishStatus(const rclcpp::Time& time);

  std::string gpio_name_{"system"};
  std::vector<std::string> joints_;
  double home_timeout_s_{90.0};
  double request_timeout_s_{2.0};
  double status_period_s_{0.1};
  double link_timeout_ms_{100.0};

  std::array<std::atomic<double>, kCmdCount> pending_{};
  std::atomic<bool> active_{false};
  mutable std::mutex snapshot_mutex_;
  Snapshot snapshot_;
  std::mutex service_mutex_;  ///< one operation at a time

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Service<scorbot_msgs::srv::Home>::SharedPtr home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_, enable_unhomed_srv_, disable_srv_, clear_fault_srv_;
  rclcpp::Publisher<scorbot_msgs::msg::SystemStatus>::SharedPtr status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<scorbot_msgs::msg::SystemStatus>> rt_status_pub_;
  int64_t last_status_ns_{0};
};

}  // namespace scorbot_system_controller
