#include "scorbot_hardware/scorbot_system.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include "scorbot_hardware/calibration.hpp"
#include "scorbot_protocol/transport.hpp"

namespace scorbot_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
bool hasInterface(const std::vector<hardware_interface::InterfaceInfo>& list, const std::string& name)
{
  return std::any_of(list.begin(), list.end(),
                     [&](const hardware_interface::InterfaceInfo& i) { return i.name == name; });
}
}  // namespace

std::string ScorbotSystem::param(const std::string& key, const std::string& fallback) const
{
  const auto it = info_.hardware_parameters.find(key);
  return it == info_.hardware_parameters.end() || it->second.empty() ? fallback : it->second;
}

bool ScorbotSystem::paramBool(const std::string& key, bool fallback) const
{
  const std::string v = param(key, fallback ? "true" : "false");
  return v == "true" || v == "True" || v == "1";
}

double ScorbotSystem::paramDouble(const std::string& key, double fallback) const
{
  const std::string v = param(key, "");
  if (v.empty())
    return fallback;
  try
  {
    return std::stod(v);
  }
  catch (const std::exception&)
  {
    RCLCPP_WARN(logger_, "parameter '%s' = '%s' is not a number; using %g", key.c_str(), v.c_str(), fallback);
    return fallback;
  }
}

CallbackReturn ScorbotSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // ---- joints: exactly the interfaces of the description's ros2_control block ----------
  joint_names_.clear();
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    const bool ok = hasInterface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) &&
                    hasInterface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) &&
                    hasInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) &&
                    hasInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY) &&
                    hasInterface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT);
    if (!ok || joint.command_interfaces.size() != 2 || joint.state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(logger_,
                   "joint '%s' must have command interfaces position, velocity and state interfaces "
                   "position, velocity, effort",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }
  if (joint_names_.empty() || joint_names_.size() > scorbot_protocol::kMaxJoints)
  {
    RCLCPP_ERROR(logger_, "need between 1 and %zu joints, got %zu", scorbot_protocol::kMaxJoints, joint_names_.size());
    return CallbackReturn::ERROR;
  }

  // ---- the one GPIO component carrying the service surface ----------------------------
  if (info_.gpios.size() != 1)
  {
    RCLCPP_ERROR(logger_, "expected exactly one <gpio> (the system services), got %zu", info_.gpios.size());
    return CallbackReturn::ERROR;
  }
  const hardware_interface::ComponentInfo& gpio = info_.gpios.front();
  gpio_name_ = gpio.name;
  for (std::size_t k = 0; k < kGpioCommandCount; ++k)
    if (!hasInterface(gpio.command_interfaces, kGpioCommandNames[k]))
    {
      RCLCPP_ERROR(logger_, "gpio '%s' is missing command interface '%s'", gpio_name_.c_str(), kGpioCommandNames[k]);
      return CallbackReturn::ERROR;
    }
  for (std::size_t k = 0; k < kGpioStateCount; ++k)
    if (!hasInterface(gpio.state_interfaces, kGpioStateNames[k]))
    {
      RCLCPP_ERROR(logger_, "gpio '%s' is missing state interface '%s'", gpio_name_.c_str(), kGpioStateNames[k]);
      return CallbackReturn::ERROR;
    }

  // ---- session configuration from the hardware parameters ------------------------------
  SessionConfig cfg;
  cfg.robot_type = param("robot_type", "");
  cfg.prefix = param("prefix", "");
  cfg.joint_names = joint_names_;
  cfg.watchdog_ms = static_cast<uint32_t>(paramDouble("command_watchdog_ms", 200));
  cfg.state_timeout_ms = static_cast<uint32_t>(paramDouble("state_timeout_ms", 100));
  cfg.request_timeout_ms = static_cast<uint32_t>(paramDouble("request_timeout_ms", 200));
  cfg.auto_home_on_activate = paramBool("auto_home_on_activate", false);
  cfg.allow_unhomed = paramBool("allow_unhomed", false);
  cfg.home_timeout_s = paramDouble("home_timeout_s", 90.0);
  cfg.activate_timeout_s = paramDouble("activate_timeout_s", 3.0);
  if (cfg.robot_type.empty())
  {
    RCLCPP_ERROR(logger_, "hardware parameter 'robot_type' is required (er_4pc | er_v)");
    return CallbackReturn::ERROR;
  }
  const std::string calibration_file = param("calibration_file", "");
  if (calibration_file.empty())
  {
    RCLCPP_ERROR(logger_, "hardware parameter 'calibration_file' is required");
    return CallbackReturn::ERROR;
  }
  CalibrationFile file;
  std::string error;
  if (!loadCalibrationFile(calibration_file, file, error) ||
      !orderCalibration(file, joint_names_, cfg.prefix, cfg.calibration, error))
  {
    RCLCPP_ERROR(logger_, "%s", error.c_str());
    return CallbackReturn::ERROR;
  }
  if (!file.robot_type.empty() && file.robot_type != cfg.robot_type)
  {
    RCLCPP_ERROR(logger_, "calibration file is for '%s' but robot_type is '%s'", file.robot_type.c_str(),
                 cfg.robot_type.c_str());
    return CallbackReturn::ERROR;
  }

  session_ = std::make_unique<Session>(cfg);
  session_->setLogger([this](int level, const std::string& m) {
    switch (level)
    {
      case Session::kDebug: RCLCPP_DEBUG(logger_, "%s", m.c_str()); break;
      case Session::kInfo: RCLCPP_INFO(logger_, "%s", m.c_str()); break;
      case Session::kWarn: RCLCPP_WARN(logger_, "%s", m.c_str()); break;
      default: RCLCPP_ERROR(logger_, "%s", m.c_str()); break;
    }
  });

  // Precompute interface names.
  if_position_state_.clear();
  for (const std::string& j : joint_names_)
  {
    if_position_state_.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    if_velocity_state_.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
    if_effort_state_.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
    if_position_cmd_.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    if_velocity_cmd_.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (std::size_t k = 0; k < kGpioStateCount; ++k)
    if_gpio_state_.push_back(gpio_name_ + "/" + kGpioStateNames[k]);
  for (std::size_t k = 0; k < kGpioCommandCount; ++k)
    if_gpio_cmd_.push_back(gpio_name_ + "/" + kGpioCommandNames[k]);

  RCLCPP_INFO(logger_, "%s: %zu joints, %s, calibration %s", info_.name.c_str(), joint_names_.size(),
              cfg.robot_type.c_str(), calibration_file.c_str());
  return CallbackReturn::SUCCESS;
}

bool ScorbotSystem::openTransport(std::unique_ptr<scorbot_protocol::Transport>& transport, std::string& error) const
{
  const std::string kind = param("transport", "serial");
  if (kind != "serial")
  {
    error = "transport '" + kind + "' is not supported yet (serial only)";
    return false;
  }
  const std::string port = param("serial_port", "/dev/ttyUSB0");
  const int baud = static_cast<int>(paramDouble("baud_rate", 921600));
  auto serial = std::make_unique<scorbot_protocol::SerialTransport>();
  if (!serial->open(port, baud, &error))
    return false;
  transport = std::move(serial);
  return true;
}

CallbackReturn ScorbotSystem::on_configure(const rclcpp_lifecycle::State&)
{
  std::unique_ptr<scorbot_protocol::Transport> transport;
  std::string error;
  if (!openTransport(transport, error))
  {
    RCLCPP_ERROR(logger_, "cannot open the controller link: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  if (!session_->configure(std::move(transport), error))
  {
    RCLCPP_ERROR(logger_, "configure failed: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  session_->cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystem::on_activate(const rclcpp_lifecycle::State&)
{
  std::string error;
  if (!session_->activate(error))
  {
    RCLCPP_ERROR(logger_, "activate failed: %s", error.c_str());
    return CallbackReturn::FAILURE;
  }
  // Publish the seeded values so controllers activating next read a consistent state.
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    set_state(if_position_state_[i], session_->position_state[i]);
    set_state(if_velocity_state_[i], session_->velocity_state[i]);
    set_state(if_effort_state_[i], session_->effort_state[i]);
    set_command(if_position_cmd_[i], session_->position_command[i]);
    set_command(if_velocity_cmd_[i], session_->velocity_command[i]);
  }
  for (std::size_t k = 0; k < kGpioStateCount; ++k)
    set_state(if_gpio_state_[k], session_->gpio_state[k]);
  for (std::size_t k = 0; k < kGpioCommandCount; ++k)
    set_command(if_gpio_cmd_[k], 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  session_->deactivate();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  session_->cleanup();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScorbotSystem::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_ERROR(logger_, "entering error state: link closed; reconfigure to reconnect");
  session_->cleanup();
  return CallbackReturn::SUCCESS;
}

return_type ScorbotSystem::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                       const std::vector<std::string>& stop_interfaces)
{
  std::string error;
  if (!session_->prepareModeSwitch(start_interfaces, stop_interfaces, error))
  {
    RCLCPP_ERROR(logger_, "command mode switch refused: %s", error.c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type ScorbotSystem::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                       const std::vector<std::string>& stop_interfaces)
{
  session_->performModeSwitch(start_interfaces, stop_interfaces);
  // Seeds made by the switch (hold current position, zero velocity) go to the handles.
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    set_command(if_position_cmd_[i], session_->position_command[i]);
    set_command(if_velocity_cmd_[i], session_->velocity_command[i]);
  }
  return return_type::OK;
}

return_type ScorbotSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  const ReadStatus status = session_->read();
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    set_state(if_position_state_[i], session_->position_state[i]);
    set_state(if_velocity_state_[i], session_->velocity_state[i]);
    set_state(if_effort_state_[i], session_->effort_state[i]);
  }
  for (std::size_t k = 0; k < kGpioStateCount; ++k)
    set_state(if_gpio_state_[k], session_->gpio_state[k]);
  // A controller FAULT is reported through the gpio state and cleared through the
  // system controller; only a dead link or a rebooted controller is a read error.
  return status == ReadStatus::kOk ? return_type::OK : return_type::ERROR;
}

return_type ScorbotSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    session_->position_command[i] = get_command(if_position_cmd_[i]);
    session_->velocity_command[i] = get_command(if_velocity_cmd_[i]);
  }
  for (std::size_t k = 0; k < kGpioCommandCount; ++k)
    session_->gpio_command[k] = get_command(if_gpio_cmd_[k]);
  session_->write();
  // Pulses are consumed by the session; reflect that so a controller sees 0 again.
  for (std::size_t k = 0; k < kGpioCommandCount; ++k)
    set_command(if_gpio_cmd_[k], session_->gpio_command[k]);
  set_state(if_gpio_state_[kGpioLastResult], session_->gpio_state[kGpioLastResult]);
  set_state(if_gpio_state_[kGpioRequestCount], session_->gpio_state[kGpioRequestCount]);
  return return_type::OK;
}

}  // namespace scorbot_hardware

PLUGINLIB_EXPORT_CLASS(scorbot_hardware::ScorbotSystem, hardware_interface::SystemInterface)
