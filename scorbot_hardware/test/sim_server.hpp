// Test helper: scorbot_esp_sim::ControllerSim served on its own thread over one end of
// a socketpair, in wall-clock time (optionally sped up), so a Session on the other end
// sees exactly what it would see from an ESP32.

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "scorbot_esp_sim/controller_sim.hpp"
#include "scorbot_protocol/link.hpp"
#include "scorbot_protocol/transport.hpp"

namespace scorbot_hardware_test
{

class SimServer
{
public:
  explicit SimServer(scorbot_esp_sim::SimOptions options = {}, double speed = 1.0)
    : sim(std::move(options)), speed_(speed)
  {
    if (!scorbot_protocol::TransportPair::create(pair_))
      throw std::runtime_error("socketpair");
    link_ = std::make_unique<scorbot_protocol::Link>(pair_.a);
    thread_ = std::thread([this] { loop(); });
  }
  ~SimServer()
  {
    running_.store(false);
    thread_.join();
  }

  /// The Session's end of the link (ownership passes to the caller).
  std::unique_ptr<scorbot_protocol::Transport> clientTransport()
  {
    return std::make_unique<scorbot_protocol::FdTransport>(std::move(pair_.b));
  }

  /// Stop serving (link appears dead) or resume.
  void pause(bool paused) { paused_.store(paused); }

  /// Run `fn` with the simulator locked (no ticks in between).
  template <typename Fn>
  auto withSim(Fn fn)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fn(sim);
  }

  scorbot_esp_sim::ControllerSim sim;

private:
  void loop()
  {
    using namespace std::chrono;
    auto last = steady_clock::now();
    while (running_.load())
    {
      std::this_thread::sleep_for(500us);
      const auto now = steady_clock::now();
      double dt = duration<double>(now - last).count() * speed_;
      last = now;
      if (paused_.load())
        continue;
      std::lock_guard<std::mutex> lock(mutex_);
      link_->poll();
      scorbot_protocol::Envelope e;
      while (link_->receive(e))
        sim.handle(e);
      sim.tick(std::min(dt, 0.05));
      while (sim.popOutbound(e))
        link_->send(e);
    }
  }

  double speed_;
  scorbot_protocol::TransportPair pair_;
  std::unique_ptr<scorbot_protocol::Link> link_;
  std::mutex mutex_;
  std::atomic<bool> running_{true};
  std::atomic<bool> paused_{false};
  std::thread thread_;
};

}  // namespace scorbot_hardware_test
