// Session: everything the ros2_control plugin does, without ros2_control. Owns the
// interface storage (the doubles the plugin exports), the ControllerClient, the
// per-joint command mode, and a worker thread for the non-realtime requests
// triggered through the GPIO command interfaces. ScorbotSystem is a thin adapter
// over this class; the tests drive it directly against scorbot_esp_sim.

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "scorbot_hardware/controller_client.hpp"
#include "scorbot_protocol/messages.hpp"

namespace scorbot_hardware
{

struct SessionConfig
{
  std::string robot_type;                 ///< "er_4pc" | "er_v"; must match the controller
  std::string prefix;                     ///< joint-name prefix used by the description
  std::vector<std::string> joint_names;   ///< ros2_control order (with prefix)
  std::vector<scorbot_protocol::JointCalibration> calibration;  ///< same order; index set here
  uint32_t watchdog_ms{200};              ///< pushed with SetWatchdog
  uint32_t state_timeout_ms{100};         ///< read() reports the link lost past this age
  uint32_t request_timeout_ms{200};       ///< per request/response
  bool auto_home_on_activate{false};
  double home_timeout_s{60.0};
  bool allow_unhomed{false};              ///< bench: Enable(allow_unhomed) when UNHOMED
  double activate_timeout_s{2.0};         ///< waiting for telemetry / ACTIVE
  double enable_position_tolerance_rad{0.02};  ///< see Session::write, enable rule
};

/// GPIO command interfaces (order = index into Session::gpio_command).
enum GpioCommand : std::size_t
{
  kGpioHome = 0,       ///< pulse: joint bitmask to home, 0x1F or 0 = all
  kGpioEnable,         ///< pulse: 1 = Enable, 2 = Enable(allow_unhomed)
  kGpioDisable,        ///< pulse: nonzero = Disable
  kGpioClearFault,     ///< pulse: nonzero = ClearFault
  kGpioCommandCount
};
/// GPIO state interfaces (order = index into Session::gpio_state).
enum GpioState : std::size_t
{
  kGpioSystemState = 0,  ///< SystemState enum value
  kGpioFaultCode,        ///< FaultCode enum value
  kGpioHomedMask,
  kGpioLimitMask,
  kGpioLinkAgeMs,
  kGpioFirmwareVersion,  ///< major*10000 + minor*100 + patch parsed from the version string
  kGpioBootCount,
  kGpioLastResult,       ///< Result of the last GPIO request; -1 while one is in flight
  kGpioCommandsIgnored,  ///< controller's rolling count of JointCommands it dropped
  kGpioRequestCount,     ///< pulses consumed so far (accepted or refused): service handshake
  kGpioStateCount
};
extern const char* const kGpioCommandNames[kGpioCommandCount];
extern const char* const kGpioStateNames[kGpioStateCount];

/// Value of `last_result` while a request is in flight.
constexpr double kResultPending = -1.0;

enum class JointMode
{
  kNone,
  kPosition,
  kVelocity
};

enum class ReadStatus
{
  kOk,
  kLinkLost,   ///< no JointState within state_timeout_ms, or the transport failed
  kRebooted    ///< boot_count changed: calibration and homing must be redone (reconfigure)
};

/// Numeric form of a version string: digits groups major.minor.patch -> major*10000+minor*100+patch.
double versionNumber(const std::string& version);

class Session
{
public:
  using Logger = std::function<void(int level, const std::string& message)>;  // level: 0 debug 1 info 2 warn 3 error
  enum LogLevel
  {
    kDebug = 0,
    kInfo = 1,
    kWarn = 2,
    kError = 3
  };

  explicit Session(SessionConfig config);
  ~Session();
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;

  void setLogger(Logger logger) { logger_ = std::move(logger); }
  const SessionConfig& config() const { return config_; }
  std::size_t jointCount() const { return config_.joint_names.size(); }

  // ---- interface storage (exported by the plugin) -------------------------------------
  std::vector<double> position_state;
  std::vector<double> velocity_state;
  std::vector<double> effort_state;      ///< motor current, A
  std::vector<double> position_command;
  std::vector<double> velocity_command;
  std::array<double, kGpioStateCount> gpio_state{};
  std::array<double, kGpioCommandCount> gpio_command{};

  // ---- lifecycle ---------------------------------------------------------------------
  /// Take the open transport, start reading, verify the controller (protocol, robot type,
  /// joint names), push calibration and the watchdog, wait for telemetry.
  bool configure(std::unique_ptr<scorbot_protocol::Transport> transport, std::string& error);
  /// Seed commands from the current state; home if configured; Enable when READY.
  bool activate(std::string& error);
  /// Disable the drives; the link stays up.
  void deactivate();
  /// Disable best-effort, stop threads, close the transport.
  void cleanup();
  bool configured() const { return client_ != nullptr; }

  // ---- cyclic --------------------------------------------------------------------------
  ReadStatus read();
  void write();

  // ---- command mode switching (ros2_control prepare/perform) ---------------------------
  /// `start`/`stop` are full interface names "<joint>/position" etc. GPIO names are ignored.
  bool prepareModeSwitch(const std::vector<std::string>& start, const std::vector<std::string>& stop,
                         std::string& error) const;
  void performModeSwitch(const std::vector<std::string>& start, const std::vector<std::string>& stop);
  JointMode mode(std::size_t joint) const { return modes_[joint]; }
  bool anyJointClaimed() const;

  // ---- introspection ------------------------------------------------------------------
  const scorbot_protocol::GetInfoResponse& info() const { return info_; }
  scorbot_protocol::SystemState systemState() const { return last_state_; }
  ControllerClient* client() { return client_.get(); }
  /// Blocks until no GPIO job is queued or running (tests).
  void waitIdle();
  std::size_t commandsSent() const { return commands_sent_; }

private:
  struct Job
  {
    scorbot_protocol::Request request;
    std::string name;
  };
  void log(int level, const std::string& message) const;
  bool requestOk(const scorbot_protocol::Request& req, scorbot_protocol::Response& resp, std::string& error);
  bool waitForState(std::function<bool(const scorbot_protocol::JointState&)> pred, double timeout_s);
  bool homeAndWait(std::string& error);
  bool enableAndWait(bool allow_unhomed, std::string& error);
  void enqueue(Job job);
  void workerLoop();
  void handleGpioCommands();
  void handleEvents();
  std::vector<JointMode> modesAfter(const std::vector<std::string>& start, const std::vector<std::string>& stop) const;
  int jointIndex(const std::string& interface_name, std::string& interface) const;

  SessionConfig config_;
  Logger logger_;
  std::unique_ptr<ControllerClient> client_;
  scorbot_protocol::GetInfoResponse info_{};
  std::vector<JointMode> modes_;
  scorbot_protocol::SystemState last_state_{scorbot_v1_SystemState_SYSTEM_STATE_BOOT};
  bool have_state_{false};
  uint32_t boot_count_{0};
  std::size_t commands_sent_{0};

  std::atomic<double> last_result_{0.0};
  uint32_t request_count_{0};
  std::thread worker_;
  std::atomic<bool> worker_running_{false};
  std::mutex jobs_mutex_;
  std::condition_variable jobs_cv_;
  std::deque<Job> jobs_;
  bool job_active_{false};
};

}  // namespace scorbot_hardware
