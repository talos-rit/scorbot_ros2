// The hardware interface's behavior, driven the way ros2_control drives ScorbotSystem
// (configure / activate / read / write / mode switches / GPIO pulses), against the
// simulated controller. These are the tests the design doc's section 11 asks for.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "scorbot_hardware/session.hpp"
#include "sim_server.hpp"

using namespace scorbot_hardware;
using namespace std::chrono_literals;
using scorbot_hardware_test::SimServer;

namespace
{

const std::vector<std::string> kJoints = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
                                          "wrist_roll_joint"};

SessionConfig baseConfig()
{
  SessionConfig c;
  c.robot_type = "er_4pc";
  c.joint_names = kJoints;
  for (std::size_t i = 0; i < kJoints.size(); ++i)
    c.calibration.push_back(scorbot_esp_sim::defaultCalibration(i));
  c.watchdog_ms = 2000;  // tests drive write() by hand; the watchdog test overrides
  c.state_timeout_ms = 100;
  c.request_timeout_ms = 300;
  c.home_timeout_s = 20.0;
  return c;
}

/// Simulator that starts close to its switches so homing takes well under a second.
scorbot_esp_sim::SimOptions quickSim()
{
  scorbot_esp_sim::SimOptions o;
  o.initial_positions.assign(5, 0.05);  // home_direction -1: switch is 0.05 rad below
  return o;
}

struct Fixture
{
  std::vector<std::string> log;  // before session: its destructor still logs
  SimServer server;
  Session session;

  explicit Fixture(scorbot_esp_sim::SimOptions o = quickSim(), SessionConfig c = baseConfig(), double speed = 2.0)
    : server(std::move(o), speed), session(std::move(c))
  {
    session.setLogger([this](int level, const std::string& m) {
      log.push_back(std::to_string(level) + ":" + m);
      if (level >= 2)
        std::fprintf(stderr, "    [log %d] %s\n", level, m.c_str());
    });
  }
  bool configure(std::string& err) { return session.configure(server.clientTransport(), err); }

  /// One ros2_control cycle.
  ReadStatus cycle()
  {
    const ReadStatus r = session.read();
    session.write();
    return r;
  }
  /// Cycle at 100 Hz for `seconds` of wall time.
  void run(double seconds)
  {
    const int n = static_cast<int>(seconds * 100);
    for (int i = 0; i < n; ++i)
    {
      cycle();
      std::this_thread::sleep_for(10ms);
    }
  }
  bool waitState(scorbot_v1_SystemState s, double seconds)
  {
    for (int i = 0; i < static_cast<int>(seconds * 100); ++i)
    {
      cycle();
      if (session.systemState() == s)
        return true;
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }
  bool hasLog(const std::string& needle) const
  {
    for (const auto& l : log)
      if (l.find(needle) != std::string::npos)
        return true;
    return false;
  }
};

}  // namespace

TEST(Session, ConfigureVerifiesTheControllerAndPushesCalibration)
{
  Fixture f;
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  EXPECT_TRUE(f.session.configured());
  EXPECT_EQ(f.session.info().joint_names_count, 5);
  EXPECT_EQ(f.session.info().boot_count, 1u);
  EXPECT_EQ(f.session.gpio_state[kGpioFirmwareVersion], versionNumber("sim-0.1.0"));
  EXPECT_EQ(versionNumber("1.2.3+abc"), 10203.0);
  EXPECT_EQ(f.server.withSim([](auto& sim) { return sim.watchdogMs(); }), 2000u);
  // The calibration on the controller is the one we pushed, index by index.
  for (std::size_t i = 0; i < 5; ++i)
  {
    const auto cal = f.server.withSim([&](auto& sim) { return sim.joint(i).cal; });
    EXPECT_EQ(cal.index, i);
    EXPECT_FLOAT_EQ(cal.gear_ratio, scorbot_esp_sim::defaultCalibration(i).gear_ratio);
  }
  EXPECT_TRUE(f.hasLog("configured er_4pc"));
  EXPECT_FALSE(f.configure(err)) << "second configure refused";
}

TEST(Session, ConfigureRefusesTheWrongRobot)
{
  scorbot_esp_sim::SimOptions o = quickSim();
  o.robot_type = "er_v";
  Fixture f(o);
  std::string err;
  EXPECT_FALSE(f.configure(err));
  EXPECT_NE(err.find("er_v"), std::string::npos) << err;
  EXPECT_FALSE(f.session.configured()) << "can be retried";
}

TEST(Session, ConfigureSetsAnUnsetRobotTypeAndRefusesWrongJoints)
{
  {
    scorbot_esp_sim::SimOptions o = quickSim();
    o.robot_type = "unset";
    Fixture f(o);
    std::string err;
    ASSERT_TRUE(f.configure(err)) << err;
    EXPECT_EQ(f.server.withSim([](auto& sim) { return sim.robotType(); }), "er_4pc");
  }
  {
    scorbot_esp_sim::SimOptions o = quickSim();
    o.joint_names = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "gripper"};
    Fixture f(o);
    std::string err;
    EXPECT_FALSE(f.configure(err));
    EXPECT_NE(err.find("gripper"), std::string::npos) << err;
  }
  {
    // Prefixed description names are matched against the controller's plain names.
    SessionConfig c = baseConfig();
    c.prefix = "bluey_";
    for (auto& j : c.joint_names)
      j = "bluey_" + j;
    Fixture f(quickSim(), c);
    std::string err;
    EXPECT_TRUE(f.configure(err)) << err;
  }
}

TEST(Session, ConfigureFailsWithoutAController)
{
  Fixture f;
  f.server.pause(true);
  SessionConfig c = baseConfig();
  std::string err;
  EXPECT_FALSE(f.configure(err));
  EXPECT_NE(err.find("GetInfo"), std::string::npos) << err;
}

TEST(Session, ActivateUnhomedWarnsAndSendsNoCommands)
{
  Fixture f;
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;
  EXPECT_TRUE(f.hasLog("UNHOMED"));
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
  f.session.performModeSwitch({"base_joint/position"}, {});
  f.run(0.2);
  EXPECT_EQ(f.session.commandsSent(), 0u) << "nothing is streamed while the controller ignores commands";
  EXPECT_EQ(f.server.withSim([](auto& sim) { return sim.commandsIgnored(); }), 0u);
  EXPECT_EQ(f.session.gpio_state[kGpioSystemState], 1.0);
  EXPECT_LT(f.session.gpio_state[kGpioLinkAgeMs], 100.0);
}

TEST(Session, GpioHomeEnableThenPositionAndVelocityCommands)
{
  Fixture f;
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;

  // Home through the GPIO pulse (as the system controller does).
  f.session.gpio_command[kGpioHome] = 0x1F;
  f.cycle();
  EXPECT_EQ(f.session.gpio_command[kGpioHome], 0.0) << "pulse consumed";
  EXPECT_TRUE(f.session.gpio_state[kGpioLastResult] == kResultPending || f.session.gpio_state[kGpioLastResult] == 0.0);
  f.session.waitIdle();
  f.cycle();
  EXPECT_EQ(f.session.gpio_state[kGpioLastResult], 0.0) << "RESULT_OK";
  ASSERT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_READY, 10.0));
  EXPECT_EQ(f.session.gpio_state[kGpioHomedMask], 31.0);
  EXPECT_TRUE(f.hasLog("homing complete"));

  // Enable, then claim position on the elbow and command a move.
  f.session.gpio_command[kGpioEnable] = 1;
  f.cycle();
  f.session.waitIdle();
  ASSERT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, 2.0));

  EXPECT_TRUE(f.session.prepareModeSwitch({"elbow_joint/position"}, {}, err)) << err;
  f.session.performModeSwitch({"elbow_joint/position"}, {});
  EXPECT_EQ(f.session.mode(2), JointMode::kPosition);
  EXPECT_NEAR(f.session.position_command[2], f.session.position_state[2], 1e-6) << "seeded";
  f.session.position_command[2] = 0.3;
  f.run(1.5);
  EXPECT_GT(f.session.commandsSent(), 100u);
  EXPECT_NEAR(f.session.position_state[2], 0.3, 0.05);
  EXPECT_EQ(f.session.gpio_state[kGpioCommandsIgnored], 0.0);
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);

  // Switch the same joint to velocity: the seed is 0, then a velocity moves it.
  EXPECT_TRUE(f.session.prepareModeSwitch({"elbow_joint/velocity"}, {"elbow_joint/position"}, err)) << err;
  f.session.performModeSwitch({"elbow_joint/velocity"}, {"elbow_joint/position"});
  EXPECT_EQ(f.session.mode(2), JointMode::kVelocity);
  EXPECT_EQ(f.session.velocity_command[2], 0.0);
  const double before = f.session.position_state[2];
  f.session.velocity_command[2] = -0.2;
  f.run(0.6);
  EXPECT_LT(f.session.position_state[2], before - 0.05);
  EXPECT_LT(f.session.velocity_state[2], -0.1);
  EXPECT_GT(std::fabs(f.session.effort_state[2]), 0.0) << "current reported as effort";

  // Releasing the claim stops the joint.
  f.session.performModeSwitch({}, {"elbow_joint/velocity"});
  EXPECT_EQ(f.session.mode(2), JointMode::kNone);
  f.run(0.5);
  EXPECT_NEAR(f.session.velocity_state[2], 0.0, 1e-3);
  EXPECT_EQ(f.session.client()->stats().rx_seq_gaps, 0u);
}

TEST(Session, HomeIsRefusedWhileJointsAreClaimedAndEnableRefusedIfItWouldJump)
{
  Fixture f;
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;
  f.session.performModeSwitch({"base_joint/position"}, {});

  f.session.gpio_command[kGpioHome] = 0x1F;
  f.cycle();
  f.session.waitIdle();
  EXPECT_EQ(f.session.gpio_state[kGpioLastResult], static_cast<double>(scorbot_v1_Result_RESULT_INVALID_STATE));
  EXPECT_TRUE(f.hasLog("home refused"));
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);

  f.session.performModeSwitch({}, {"base_joint/position"});
  f.session.gpio_command[kGpioHome] = 0;  // 0 = all joints too
  f.session.gpio_command[kGpioHome] = 31;
  f.cycle();
  f.session.waitIdle();
  ASSERT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_READY, 10.0));

  // A controller holding a stale point must not be handed the drives.
  f.session.performModeSwitch({"base_joint/position"}, {});
  f.session.position_command[0] = f.session.position_state[0] + 0.5;
  f.session.gpio_command[kGpioEnable] = 1;
  f.cycle();
  f.session.waitIdle();
  EXPECT_EQ(f.session.gpio_state[kGpioLastResult], static_cast<double>(scorbot_v1_Result_RESULT_INVALID_STATE));
  EXPECT_TRUE(f.hasLog("enable refused"));
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_READY);

  // Holding the current position is fine.
  f.session.position_command[0] = f.session.position_state[0];
  f.session.gpio_command[kGpioEnable] = 1;
  f.cycle();
  f.session.waitIdle();
  EXPECT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, 2.0));
  EXPECT_EQ(f.session.gpio_state[kGpioLastResult], 0.0);
}

TEST(Session, AutoHomeOnActivateEndsActive)
{
  SessionConfig c = baseConfig();
  c.auto_home_on_activate = true;
  Fixture f(quickSim(), c);
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  EXPECT_EQ(f.session.gpio_state[kGpioHomedMask], 31.0);
  f.session.deactivate();
  ASSERT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_READY, 2.0));
  ASSERT_TRUE(f.session.activate(err)) << err << " (re-activation from READY enables again)";
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

TEST(Session, AllowUnhomedEnablesForTheBench)
{
  SessionConfig c = baseConfig();
  c.allow_unhomed = true;
  Fixture f(quickSim(), c);
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
  EXPECT_TRUE(f.server.withSim([](auto& sim) { return sim.unhomedActive(); }));
  EXPECT_TRUE(f.hasLog("allow_unhomed"));
}

TEST(Session, WatchdogFaultIsReportedNotFatalAndClearable)
{
  SessionConfig c = baseConfig();
  c.auto_home_on_activate = true;
  c.watchdog_ms = 100;
  Fixture f(quickSim(), c);
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;
  f.session.performModeSwitch({"wrist_roll_joint/velocity"}, {});
  f.run(0.2);
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);

  // Stop calling write() for longer than the watchdog (simulated time runs 2x).
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(f.session.read(), ReadStatus::kOk) << "a fault is reported, not a read error";
  EXPECT_EQ(f.session.systemState(), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
  EXPECT_EQ(f.session.gpio_state[kGpioFaultCode], static_cast<double>(scorbot_v1_FaultCode_FAULT_WATCHDOG));
  EXPECT_TRUE(f.hasLog("fault raised: watchdog") || f.hasLog("FAULT"));

  f.session.gpio_command[kGpioClearFault] = 1;
  f.cycle();
  f.session.waitIdle();
  ASSERT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_READY, 2.0)) << "positions still trusted after a watchdog fault";
  f.session.gpio_command[kGpioEnable] = 1;  // velocity command is 0: allowed while claimed
  f.cycle();
  f.session.waitIdle();
  ASSERT_TRUE(f.waitState(scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, 2.0));
  EXPECT_EQ(f.session.gpio_state[kGpioFaultCode], 0.0);
}

TEST(Session, LinkLossAndRebootAreReadErrors)
{
  Fixture f;
  std::string err;
  ASSERT_TRUE(f.configure(err)) << err;
  ASSERT_TRUE(f.session.activate(err)) << err;
  EXPECT_EQ(f.session.read(), ReadStatus::kOk);

  f.server.pause(true);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(f.session.read(), ReadStatus::kLinkLost);
  EXPECT_GT(f.session.gpio_state[kGpioLinkAgeMs], 100.0);
  EXPECT_TRUE(f.hasLog("no JointState"));
  f.server.pause(false);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(f.session.read(), ReadStatus::kOk) << "recovers when frames return";

  f.server.withSim([](auto& sim) { sim.reboot(); return 0; });
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(f.session.read(), ReadStatus::kRebooted);
  EXPECT_EQ(f.session.gpio_state[kGpioBootCount], 2.0);
  EXPECT_TRUE(f.hasLog("rebooted"));
}

TEST(Session, ModeSwitchRules)
{
  SessionConfig c = baseConfig();
  Session s(c);
  std::string err;
  EXPECT_TRUE(s.prepareModeSwitch({"base_joint/position", "elbow_joint/velocity", "system/home"}, {}, err)) << err;
  EXPECT_FALSE(s.prepareModeSwitch({"base_joint/position", "base_joint/velocity"}, {}, err));
  EXPECT_NE(err.find("base_joint"), std::string::npos);
  EXPECT_FALSE(s.prepareModeSwitch({"base_joint/effort"}, {}, err));
  s.performModeSwitch({"base_joint/position"}, {});
  EXPECT_FALSE(s.prepareModeSwitch({"base_joint/velocity"}, {}, err)) << "already in position";
  EXPECT_TRUE(s.prepareModeSwitch({"base_joint/velocity"}, {"base_joint/position"}, err)) << err;
  EXPECT_TRUE(s.anyJointClaimed());
  s.performModeSwitch({}, {"base_joint/position"});
  EXPECT_FALSE(s.anyJointClaimed());
  EXPECT_TRUE(s.prepareModeSwitch({}, {}, err));
  EXPECT_EQ(s.read(), ReadStatus::kLinkLost) << "not configured";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
