#include "scorbot_protocol/link.hpp"

#include <algorithm>

namespace scorbot_protocol
{

uint64_t Link::defaultClock()
{
  using namespace std::chrono;
  return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

Link::Link(Transport& transport, ClockFn clock_us, std::size_t max_queue)
  : transport_(transport), clock_(std::move(clock_us)), max_queue_(max_queue == 0 ? 1 : max_queue)
{
}

Envelope Link::newEnvelope() { return makeEnvelope(++seq_, clock_()); }

bool Link::send(const Envelope& envelope)
{
  std::size_t payload_len = 0;
  if (!encodeEnvelope(envelope, payload_, sizeof(payload_), payload_len))
  {
    ++stats_.tx_errors;
    return false;
  }
  const std::size_t frame_len = encodeFrame(payload_, payload_len, frame_, sizeof(frame_));
  if (frame_len == 0)
  {
    ++stats_.tx_errors;
    return false;
  }
  const std::size_t written = transport_.write(frame_, frame_len);
  if (written != frame_len)
  {
    ++stats_.tx_errors;
    return false;
  }
  ++stats_.tx_envelopes;
  stats_.tx_bytes += frame_len;
  return true;
}

void Link::enqueue(const Envelope& e)
{
  if (inbound_.size() >= max_queue_)
  {
    inbound_.pop_front();
    ++stats_.rx_dropped;
  }
  inbound_.push_back(e);
}

std::size_t Link::poll()
{
  std::size_t added = 0;
  for (;;)
  {
    const std::size_t n = transport_.read(read_buf_, sizeof(read_buf_));
    if (n == Transport::npos)
    {
      ++stats_.rx_transport_errors;
      break;
    }
    if (n == 0)
      break;
    decoder_.feed(read_buf_, n, [&](const uint8_t* payload, std::size_t len) {
      Envelope e;
      if (!decodeEnvelope(payload, len, e))
      {
        ++stats_.rx_decode_errors;
        return;
      }
      if (have_rx_seq_ && e.seq != last_rx_seq_ + 1 && e.seq != 0)
        stats_.rx_seq_gaps += (e.seq > last_rx_seq_) ? (e.seq - last_rx_seq_ - 1) : 1;
      have_rx_seq_ = true;
      last_rx_seq_ = e.seq;
      ++stats_.rx_envelopes;
      enqueue(e);
      ++added;
    });
    if (n < sizeof(read_buf_))
      break;  // drained
  }
  stats_.decoder = decoder_.stats();
  return added;
}

bool Link::receive(Envelope& out)
{
  if (inbound_.empty())
    return false;
  out = inbound_.front();
  inbound_.pop_front();
  return true;
}

bool Link::request(const Request& request, Response& response, std::chrono::milliseconds timeout)
{
  Envelope e = newEnvelope();
  Request& r = setRequest(e, next_request_id_++);
  const uint32_t id = r.request_id;
  r.which_body = request.which_body;
  r.body = request.body;
  if (!send(e))
    return false;

  const uint64_t deadline = clock_() + static_cast<uint64_t>(timeout.count()) * 1000ULL;
  for (;;)
  {
    poll();
    // Scan the queue for our response; leave everything else in order.
    for (auto it = inbound_.begin(); it != inbound_.end(); ++it)
    {
      if (it->which_payload == scorbot_v1_Envelope_response_tag && it->payload.response.request_id == id)
      {
        response = it->payload.response;
        inbound_.erase(it);
        return true;
      }
    }
    const uint64_t now = clock_();
    if (now >= deadline)
      return false;
    const auto remaining = std::chrono::milliseconds(std::max<uint64_t>(1, (deadline - now) / 1000ULL));
    transport_.waitReadable(std::min(remaining, std::chrono::milliseconds(20)));
  }
}

}  // namespace scorbot_protocol
