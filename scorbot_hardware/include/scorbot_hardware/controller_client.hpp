// ControllerClient: the Pi side of the ESP32 link with a reader thread. Wraps a
// scorbot_protocol::Link so that the newest JointState is always available without
// blocking, requests get their Response matched by id, and Events are queued.
// No ROS dependencies: the hardware interface's tests drive it against
// scorbot_esp_sim::ControllerSim in-process.

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "scorbot_protocol/link.hpp"

namespace scorbot_hardware
{

/// The newest JointState and when it arrived (ControllerClient::nowUs clock).
struct StateSnapshot
{
  scorbot_protocol::JointState state{};
  uint32_t seq{0};
  uint64_t rx_time_us{0};
  bool valid{false};
};

class ControllerClient
{
public:
  /// Takes ownership of an open transport. Call start() to begin reading.
  explicit ControllerClient(std::unique_ptr<scorbot_protocol::Transport> transport,
                            std::size_t max_events = 64);
  ~ControllerClient();
  ControllerClient(const ControllerClient&) = delete;
  ControllerClient& operator=(const ControllerClient&) = delete;

  void start();
  void stop();
  bool running() const { return running_.load(); }

  /// Send one Envelope now (thread-safe, non-blocking). False if it could not be written in full.
  bool send(const scorbot_protocol::Envelope& envelope);
  /// A new outbound Envelope with the next sequence number.
  scorbot_protocol::Envelope newEnvelope();

  /// Send `req` and wait up to `timeout` for the Response with the same id. One request
  /// in flight at a time (serialised here). False on send failure or timeout; `error`
  /// gets a human-readable reason.
  bool request(const scorbot_protocol::Request& req, scorbot_protocol::Response& resp,
               std::chrono::milliseconds timeout, std::string* error = nullptr);

  /// Copy of the newest JointState (valid == false before the first one arrives).
  StateSnapshot latestState() const;
  /// Microseconds since the newest JointState arrived; UINT64_MAX if none yet.
  uint64_t stateAgeUs() const;
  std::size_t statesReceived() const;

  /// Drain the queued Events (oldest first). At most `max_events` are kept.
  std::vector<scorbot_protocol::Event> takeEvents();

  scorbot_protocol::LinkStats stats() const;
  /// True once the transport reported a closed peer or an I/O error.
  bool transportFailed() const { return transport_failed_.load(); }
  scorbot_protocol::Transport& transport() { return *transport_; }

  /// Monotonic microseconds (same clock as the Link and StateSnapshot::rx_time_us).
  static uint64_t nowUs() { return scorbot_protocol::Link::defaultClock(); }

private:
  void readerLoop();
  void dispatch(const scorbot_protocol::Envelope& e);

  std::unique_ptr<scorbot_protocol::Transport> transport_;
  scorbot_protocol::Link link_;
  mutable std::mutex link_mutex_;  ///< guards link_ (send and poll share its buffers/stats)

  std::thread reader_;
  std::atomic<bool> running_{false};
  std::atomic<bool> transport_failed_{false};

  mutable std::mutex state_mutex_;
  StateSnapshot latest_{};
  std::size_t states_received_{0};

  std::mutex request_mutex_;  ///< one request in flight
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  uint32_t pending_id_{0};
  bool have_response_{false};
  scorbot_protocol::Response pending_response_{};
  uint32_t next_request_id_{1};

  std::mutex events_mutex_;
  std::deque<scorbot_protocol::Event> events_;
  std::size_t max_events_;
};

}  // namespace scorbot_hardware
