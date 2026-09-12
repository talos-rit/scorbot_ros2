// The plugin loaded through hardware_interface::ResourceManager and driven against the
// scorbot_esp_sim executable over a pty: the same path robot.launch.py mock:=false
// takes, minus the controller manager. Needs a ROS 2 Jazzy install (colcon test).

#include <gtest/gtest.h>

#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "scorbot_hardware/session.hpp"

using namespace std::chrono_literals;

namespace
{

/// Runs `scorbot_esp_sim --link <path>` for the duration of a test.
class SimProcess
{
public:
  explicit SimProcess(const std::string& link_path, double speed) : link_(link_path)
  {
    const std::string exe = ament_index_cpp::get_package_prefix("scorbot_esp_sim") + "/lib/scorbot_esp_sim/scorbot_esp_sim";
    ::unlink(link_.c_str());
    pid_ = ::fork();
    if (pid_ == 0)
    {
      const std::string speed_s = std::to_string(speed);
      ::execl(exe.c_str(), exe.c_str(), "--link", link_.c_str(), "--speed", speed_s.c_str(), "--quiet",
              static_cast<char*>(nullptr));
      ::_exit(127);
    }
    for (int i = 0; i < 100; ++i)
    {
      struct stat st{};
      if (::lstat(link_.c_str(), &st) == 0)
        return;
      std::this_thread::sleep_for(50ms);
    }
  }
  ~SimProcess()
  {
    if (pid_ > 0)
    {
      ::kill(pid_, SIGTERM);
      int status = 0;
      ::waitpid(pid_, &status, 0);
    }
    ::unlink(link_.c_str());
  }
  bool alive() const
  {
    int status = 0;
    return pid_ > 0 && ::waitpid(pid_, &status, WNOHANG) == 0;
  }

private:
  std::string link_;
  pid_t pid_{-1};
};

std::string urdf(const std::string& serial_port, const std::string& calibration_file)
{
  const char* joints[] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};
  std::string s = R"(<?xml version="1.0"?>
<robot name="scorbot">
  <link name="base_link"/>)";
  std::string prev = "base_link";
  for (int i = 0; i < 5; ++i)
  {
    const std::string link = "link" + std::to_string(i + 1);
    s += "\n  <link name=\"" + link + "\"/>\n  <joint name=\"" + joints[i] + "\" type=\"revolute\">"
         "<parent link=\"" + prev + "\"/><child link=\"" + link + "\"/><axis xyz=\"0 0 1\"/>"
         "<limit lower=\"-3\" upper=\"3\" effort=\"10\" velocity=\"2\"/></joint>";
    prev = link;
  }
  s += R"(
  <ros2_control name="scorbot" type="system">
    <hardware>
      <plugin>scorbot_hardware/ScorbotSystem</plugin>
      <param name="transport">serial</param>
      <param name="serial_port">)" + serial_port + R"(</param>
      <param name="baud_rate">921600</param>
      <param name="robot_type">er_4pc</param>
      <param name="calibration_file">)" + calibration_file + R"(</param>
      <param name="state_timeout_ms">100</param>
      <param name="command_watchdog_ms">500</param>
      <param name="auto_home_on_activate">false</param>
    </hardware>)";
  for (const char* j : joints)
    s += std::string("\n    <joint name=\"") + j + "\">"
         "<command_interface name=\"position\"/><command_interface name=\"velocity\"/>"
         "<state_interface name=\"position\"/><state_interface name=\"velocity\"/><state_interface name=\"effort\"/>"
         "</joint>";
  s += "\n    <gpio name=\"system\">";
  for (std::size_t k = 0; k < scorbot_hardware::kGpioCommandCount; ++k)
    s += std::string("<command_interface name=\"") + scorbot_hardware::kGpioCommandNames[k] + "\"/>";
  for (std::size_t k = 0; k < scorbot_hardware::kGpioStateCount; ++k)
    s += std::string("<state_interface name=\"") + scorbot_hardware::kGpioStateNames[k] + "\"/>";
  s += "</gpio>\n  </ros2_control>\n</robot>\n";
  return s;
}

const char* kCalibration = R"(
robot_type: er_4pc
joints:
  base_joint:        {counts_per_motor_rev: 80, gear_ratio: 635.5, home_offset_rad: 0.0, home_direction: -1, soft_limit_min_rad: -2.7, soft_limit_max_rad: 2.7, max_velocity_rad_s: 0.35, max_accel_rad_s2: 1.0, current_limit_a: 2.0, invert: false, encoder_invert: false}
  shoulder_joint:    {counts_per_motor_rev: 80, gear_ratio: 457.56, home_offset_rad: 0.0, home_direction: -1, soft_limit_min_rad: -0.61, soft_limit_max_rad: 2.27, max_velocity_rad_s: 0.45, max_accel_rad_s2: 1.0, current_limit_a: 2.0, invert: false, encoder_invert: false}
  elbow_joint:       {counts_per_motor_rev: 80, gear_ratio: 457.56, home_offset_rad: 0.0, home_direction: -1, soft_limit_min_rad: -2.27, soft_limit_max_rad: 2.27, max_velocity_rad_s: 0.45, max_accel_rad_s2: 1.0, current_limit_a: 2.0, invert: false, encoder_invert: false}
  wrist_pitch_joint: {counts_per_motor_rev: 80, gear_ratio: 125.54, home_offset_rad: 0.0, home_direction: -1, soft_limit_min_rad: -2.27, soft_limit_max_rad: 2.27, max_velocity_rad_s: 1.4, max_accel_rad_s2: 3.0, current_limit_a: 1.5, invert: false, encoder_invert: false}
  wrist_roll_joint:  {counts_per_motor_rev: 80, gear_ratio: 125.54, home_offset_rad: 0.0, home_direction: -1, soft_limit_min_rad: -9.9, soft_limit_max_rad: 9.9, max_velocity_rad_s: 1.8, max_accel_rad_s2: 3.0, current_limit_a: 1.5, invert: false, encoder_invert: false}
)";

rclcpp_lifecycle::State state(uint8_t id, const char* label) { return rclcpp_lifecycle::State(id, label); }

}  // namespace

class ScorbotSystemTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string dir = std::string(::getenv("TMPDIR") ? ::getenv("TMPDIR") : "/tmp");
    link_ = dir + "/scorbot_hw_test_" + std::to_string(::getpid());
    calibration_ = link_ + "_cal.yaml";
    {
      std::FILE* f = std::fopen(calibration_.c_str(), "w");
      ASSERT_NE(f, nullptr);
      std::fputs(kCalibration, f);
      std::fclose(f);
    }
    sim_ = std::make_unique<SimProcess>(link_, 4.0);  // simulated time 4x: homing takes ~1.5 s
    ASSERT_TRUE(sim_->alive()) << "scorbot_esp_sim did not start";
    node_ = std::make_shared<rclcpp::Node>("scorbot_hardware_test");
  }
  void TearDown() override
  {
    rm_.reset();
    sim_.reset();
    ::unlink(calibration_.c_str());
  }

  void load()
  {
    rm_ = std::make_unique<hardware_interface::ResourceManager>(
        urdf(link_, calibration_), node_->get_node_clock_interface(), node_->get_node_logging_interface(), false, 100);
  }
  void setState(uint8_t id, const char* label)
  {
    rclcpp_lifecycle::State target = state(id, label);  // taken by non-const reference
    rm_->set_component_state("scorbot", target);
    ASSERT_EQ(rm_->get_components_status().at("scorbot").state.label(), label);
  }
  void cycle()
  {
    const rclcpp::Time t = node_->now();
    rm_->read(t, rclcpp::Duration(10ms));
    rm_->write(t, rclcpp::Duration(10ms));
  }
  bool cycleUntil(hardware_interface::LoanedStateInterface& iface, double value, double seconds)
  {
    for (int i = 0; i < static_cast<int>(seconds * 100); ++i)
    {
      cycle();
      if (iface.get_optional().value_or(-1.0) == value)
        return true;
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::string link_, calibration_;
  std::unique_ptr<SimProcess> sim_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;
};

TEST_F(ScorbotSystemTest, LoadsExportsAndConfiguresAgainstTheSimulator)
{
  ASSERT_NO_THROW(load());
  EXPECT_TRUE(rm_->state_interface_exists("base_joint/position"));
  EXPECT_TRUE(rm_->state_interface_exists("wrist_roll_joint/effort"));
  EXPECT_TRUE(rm_->command_interface_exists("elbow_joint/velocity"));
  EXPECT_TRUE(rm_->command_interface_exists("system/home"));
  EXPECT_TRUE(rm_->state_interface_exists("system/last_result"));

  setState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, hardware_interface::lifecycle_state_names::INACTIVE);
  setState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);
}

TEST_F(ScorbotSystemTest, HomesEnablesAndFollowsPositionCommands)
{
  load();
  setState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, hardware_interface::lifecycle_state_names::INACTIVE);
  setState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);

  auto sys_state = rm_->claim_state_interface("system/state");
  auto homed = rm_->claim_state_interface("system/homed_mask");
  auto last_result = rm_->claim_state_interface("system/last_result");
  auto elbow_pos = rm_->claim_state_interface("elbow_joint/position");
  auto home = rm_->claim_command_interface("system/home");
  auto enable = rm_->claim_command_interface("system/enable");

  cycle();
  EXPECT_EQ(sys_state.get_optional().value(), static_cast<double>(scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED));

  ASSERT_TRUE(home.set_value(31.0));
  ASSERT_TRUE(cycleUntil(sys_state, static_cast<double>(scorbot_v1_SystemState_SYSTEM_STATE_READY), 30.0));
  EXPECT_EQ(homed.get_optional().value(), 31.0);
  EXPECT_EQ(home.get_optional().value(), 0.0) << "pulse consumed";
  EXPECT_EQ(last_result.get_optional().value(), 0.0);

  ASSERT_TRUE(enable.set_value(1.0));
  ASSERT_TRUE(cycleUntil(sys_state, static_cast<double>(scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE), 5.0));

  // A controller claiming elbow position: mode switch, then a setpoint.
  ASSERT_EQ(rm_->prepare_command_mode_switch({"elbow_joint/position"}, {}), true);
  ASSERT_EQ(rm_->perform_command_mode_switch({"elbow_joint/position"}, {}), true);
  auto elbow_cmd = rm_->claim_command_interface("elbow_joint/position");
  cycle();
  EXPECT_NEAR(elbow_cmd.get_optional().value(), elbow_pos.get_optional().value(), 1e-6) << "seeded with the current position";
  ASSERT_TRUE(elbow_cmd.set_value(0.3));
  for (int i = 0; i < 150; ++i)
  {
    cycle();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_NEAR(elbow_pos.get_optional().value(), 0.3, 0.05);
  EXPECT_EQ(sys_state.get_optional().value(), static_cast<double>(scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE));

  // Both modes on one joint is refused.
  EXPECT_EQ(rm_->prepare_command_mode_switch({"elbow_joint/velocity"}, {}), false);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
