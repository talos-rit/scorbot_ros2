// ScorbotSystem: ros2_control SystemInterface for the ESP32 Scorbot controller. A thin
// adapter over scorbot_hardware::Session (which holds all behavior and is tested
// without ROS). Design: project_documentation/technical/ros2/ros2_architecture.md, 5.5 and 6.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "scorbot_hardware/session.hpp"

namespace scorbot_hardware
{

class ScorbotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ScorbotSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::string param(const std::string& key, const std::string& fallback) const;
  bool paramBool(const std::string& key, bool fallback) const;
  double paramDouble(const std::string& key, double fallback) const;
  bool openTransport(std::unique_ptr<scorbot_protocol::Transport>& transport, std::string& error) const;

  rclcpp::Logger logger_{rclcpp::get_logger("scorbot_hardware")};
  std::unique_ptr<Session> session_;
  std::string gpio_name_;
  std::vector<std::string> joint_names_;
  // Full interface names, precomputed so read()/write() do no string building.
  std::vector<std::string> if_position_state_, if_velocity_state_, if_effort_state_;
  std::vector<std::string> if_position_cmd_, if_velocity_cmd_;
  std::vector<std::string> if_gpio_state_, if_gpio_cmd_;
};

}  // namespace scorbot_hardware
