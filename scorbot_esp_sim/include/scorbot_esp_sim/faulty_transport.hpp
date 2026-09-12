// Transport decorator that drops or corrupts outgoing frames at configurable rates.
// Lets the simulator (and hardware-interface tests) exercise CRC failures, sequence
// gaps and resynchronisation without a flaky cable.

#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>

#include "scorbot_protocol/transport.hpp"

namespace scorbot_esp_sim
{

class FaultyTransport : public scorbot_protocol::Transport
{
public:
  explicit FaultyTransport(scorbot_protocol::Transport& inner, uint32_t seed = 1) : inner_(inner), rng_(seed) {}

  /// Probability [0,1] that a write() call is silently discarded.
  void setDropProbability(double p) { drop_ = p; }
  /// Probability [0,1] that one byte of a write() call is flipped.
  void setCorruptProbability(double p) { corrupt_ = p; }
  double dropProbability() const { return drop_; }
  double corruptProbability() const { return corrupt_; }
  uint64_t dropped() const { return dropped_; }
  uint64_t corrupted() const { return corrupted_; }

  std::size_t write(const uint8_t* data, std::size_t n) override
  {
    if (drop_ > 0.0 && uniform_(rng_) < drop_)
    {
      ++dropped_;
      return n;  // pretend it went out
    }
    if (corrupt_ > 0.0 && n > 1 && uniform_(rng_) < corrupt_)
    {
      ++corrupted_;
      std::vector<uint8_t> copy(data, data + n);
      const std::size_t idx = static_cast<std::size_t>(rng_() % (n - 1));  // never the delimiter
      copy[idx] ^= static_cast<uint8_t>(1u << (rng_() % 8));
      if (copy[idx] == 0)
        copy[idx] = 0x01;
      return inner_.write(copy.data(), copy.size());
    }
    return inner_.write(data, n);
  }
  std::size_t read(uint8_t* out, std::size_t cap) override { return inner_.read(out, cap); }
  bool waitReadable(std::chrono::milliseconds timeout) override { return inner_.waitReadable(timeout); }
  bool isOpen() const override { return inner_.isOpen(); }
  std::string describe() const override { return "faulty(" + inner_.describe() + ")"; }

private:
  scorbot_protocol::Transport& inner_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  double drop_{0.0};
  double corrupt_{0.0};
  uint64_t dropped_{0};
  uint64_t corrupted_{0};
};

}  // namespace scorbot_esp_sim
