// Link: Envelopes in and out over a Transport, with framing, sequence numbers,
// statistics, and a request/response helper. Single-threaded by design; the hardware
// interface adds its own reader thread around it.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>

#include "scorbot_protocol/framing.hpp"
#include "scorbot_protocol/messages.hpp"
#include "scorbot_protocol/transport.hpp"

namespace scorbot_protocol
{

struct LinkStats
{
  uint64_t tx_envelopes{0};
  uint64_t tx_bytes{0};
  uint64_t tx_errors{0};       ///< encode failures or transport write errors / short writes
  uint64_t rx_envelopes{0};
  uint64_t rx_decode_errors{0};  ///< valid frame but not a parseable Envelope
  uint64_t rx_seq_gaps{0};       ///< frames the peer sent that never arrived
  uint64_t rx_dropped{0};        ///< inbound queue full, oldest discarded
  uint64_t rx_transport_errors{0};  ///< read() reported a closed peer or an I/O error
  DecoderStats decoder;
};

class Link
{
public:
  using ClockFn = std::function<uint64_t()>;  ///< monotonic microseconds

  explicit Link(Transport& transport, ClockFn clock_us = defaultClock, std::size_t max_queue = 64);

  /// New outbound Envelope with the next sequence number and the current time.
  Envelope newEnvelope();

  /// Encode, frame and write. Returns false if it could not be sent in full.
  bool send(const Envelope& envelope);

  /// Read whatever the transport has and decode it. Returns the number of Envelopes
  /// added to the inbound queue. Never blocks.
  std::size_t poll();

  /// Pop the oldest inbound Envelope. Returns false if the queue is empty.
  bool receive(Envelope& out);

  /// Send `request` (request_id assigned here) and wait up to `timeout` for the
  /// matching Response. Other Envelopes arriving meanwhile stay queued for receive().
  /// Returns false on send failure or timeout.
  bool request(const Request& request, Response& response, std::chrono::milliseconds timeout);

  const LinkStats& stats() const { return stats_; }
  Transport& transport() { return transport_; }
  uint64_t now_us() const { return clock_(); }

  static uint64_t defaultClock();

private:
  void enqueue(const Envelope& e);

  Transport& transport_;
  ClockFn clock_;
  std::size_t max_queue_;
  StreamDecoder decoder_;
  std::deque<Envelope> inbound_;
  LinkStats stats_{};
  uint32_t seq_{0};
  uint32_t next_request_id_{1};
  bool have_rx_seq_{false};
  uint32_t last_rx_seq_{0};
  uint8_t payload_[kMaxPayload]{};
  uint8_t frame_[kMaxFrame]{};
  uint8_t read_buf_[4096]{};
};

}  // namespace scorbot_protocol
