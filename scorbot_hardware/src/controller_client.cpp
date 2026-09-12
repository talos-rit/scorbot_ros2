#include "scorbot_hardware/controller_client.hpp"

#include <utility>

namespace scorbot_hardware
{

using scorbot_protocol::Envelope;
using scorbot_protocol::Link;

ControllerClient::ControllerClient(std::unique_ptr<scorbot_protocol::Transport> transport,
                                   std::size_t max_events)
  : transport_(std::move(transport)), link_(*transport_), max_events_(max_events == 0 ? 1 : max_events)
{
}

ControllerClient::~ControllerClient() { stop(); }

void ControllerClient::start()
{
  if (running_.exchange(true))
    return;
  reader_ = std::thread([this] { readerLoop(); });
}

void ControllerClient::stop()
{
  if (!running_.exchange(false))
    return;
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    response_cv_.notify_all();  // wake a request() that is still waiting
  }
  if (reader_.joinable())
    reader_.join();
}

bool ControllerClient::send(const Envelope& envelope)
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  return link_.send(envelope);
}

Envelope ControllerClient::newEnvelope()
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  return link_.newEnvelope();
}

bool ControllerClient::request(const scorbot_protocol::Request& req, scorbot_protocol::Response& resp,
                               std::chrono::milliseconds timeout, std::string* error)
{
  std::lock_guard<std::mutex> one_at_a_time(request_mutex_);
  if (!running_.load())
  {
    if (error)
      *error = "client not started";
    return false;
  }
  Envelope e = newEnvelope();
  const uint32_t id = next_request_id_++;
  if (next_request_id_ == 0)
    next_request_id_ = 1;
  scorbot_protocol::Request& out = scorbot_protocol::setRequest(e, id);
  out.which_body = req.which_body;
  out.body = req.body;
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    pending_id_ = id;
    have_response_ = false;
  }
  if (!send(e))
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    pending_id_ = 0;
    if (error)
      *error = std::string("could not send ") + scorbot_protocol::requestName(req);
    return false;
  }
  std::unique_lock<std::mutex> lock(response_mutex_);
  const bool got = response_cv_.wait_for(lock, timeout, [&] { return have_response_ || !running_.load(); });
  pending_id_ = 0;
  if (!got || !have_response_)
  {
    if (error)
      *error = std::string("no response to ") + scorbot_protocol::requestName(req) + " within " +
               std::to_string(timeout.count()) + " ms";
    return false;
  }
  resp = pending_response_;
  return true;
}

StateSnapshot ControllerClient::latestState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_;
}

uint64_t ControllerClient::stateAgeUs() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!latest_.valid)
    return UINT64_MAX;
  const uint64_t now = nowUs();
  return now > latest_.rx_time_us ? now - latest_.rx_time_us : 0;
}

std::size_t ControllerClient::statesReceived() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return states_received_;
}

std::vector<scorbot_protocol::Event> ControllerClient::takeEvents()
{
  std::lock_guard<std::mutex> lock(events_mutex_);
  std::vector<scorbot_protocol::Event> out(events_.begin(), events_.end());
  events_.clear();
  return out;
}

scorbot_protocol::LinkStats ControllerClient::stats() const
{
  std::lock_guard<std::mutex> lock(link_mutex_);
  return link_.stats();
}

void ControllerClient::readerLoop()
{
  using namespace std::chrono_literals;
  while (running_.load())
  {
    // Wait outside the link lock so senders are never blocked by an idle line.
    if (!transport_->waitReadable(5ms))
      continue;
    std::vector<Envelope> batch;
    {
      std::lock_guard<std::mutex> lock(link_mutex_);
      const uint64_t errors_before = link_.stats().rx_transport_errors;
      link_.poll();
      if (link_.stats().rx_transport_errors != errors_before)
        transport_failed_.store(true);
      Envelope e;
      while (link_.receive(e))
        batch.push_back(e);
    }
    for (const Envelope& e : batch)
      dispatch(e);
    if (transport_failed_.load())
    {
      // A closed peer would spin poll(); back off but keep checking so a later
      // stop() returns promptly.
      std::this_thread::sleep_for(20ms);
    }
  }
}

void ControllerClient::dispatch(const Envelope& e)
{
  switch (e.which_payload)
  {
    case scorbot_v1_Envelope_joint_state_tag:
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_.state = e.payload.joint_state;
      latest_.seq = e.seq;
      latest_.rx_time_us = nowUs();
      latest_.valid = true;
      ++states_received_;
      break;
    }
    case scorbot_v1_Envelope_response_tag:
    {
      std::lock_guard<std::mutex> lock(response_mutex_);
      if (pending_id_ != 0 && e.payload.response.request_id == pending_id_)
      {
        pending_response_ = e.payload.response;
        have_response_ = true;
        response_cv_.notify_all();
      }
      break;
    }
    case scorbot_v1_Envelope_event_tag:
    {
      std::lock_guard<std::mutex> lock(events_mutex_);
      if (events_.size() >= max_events_)
        events_.pop_front();
      events_.push_back(e.payload.event);
      break;
    }
    default:
      break;  // commands and requests from the peer are not ours to handle
  }
}

}  // namespace scorbot_hardware
