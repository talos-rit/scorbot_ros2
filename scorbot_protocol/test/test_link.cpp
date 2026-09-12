#include <gtest/gtest.h>

#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "scorbot_protocol/link.hpp"
#include "scorbot_protocol/transport.hpp"

using namespace scorbot_protocol;

namespace
{
struct Pair
{
  TransportPair t;
  Pair() { EXPECT_TRUE(TransportPair::create(t)); }
};

/// Minimal "controller" that answers ping and get_info, used to exercise Link::request.
void serveOnce(Link& controller, bool respond = true)
{
  controller.poll();
  Envelope in;
  while (controller.receive(in))
  {
    if (in.which_payload != scorbot_v1_Envelope_request_tag || !respond)
      continue;
    const Request& req = in.payload.request;
    Envelope out = controller.newEnvelope();
    Response& resp = setResponse(out, req.request_id, scorbot_v1_Result_RESULT_OK);
    if (req.which_body == scorbot_v1_Request_ping_tag)
    {
      resp.which_body = scorbot_v1_Response_pong_tag;
      resp.body.pong = req.body.ping.nonce;
    }
    else if (req.which_body == scorbot_v1_Request_get_info_tag)
    {
      resp.which_body = scorbot_v1_Response_get_info_tag;
      resp.body.get_info.protocol_major = kProtocolMajor;
      std::strcpy(resp.body.get_info.robot_type, "er_4pc");
    }
    else
    {
      resp.result = scorbot_v1_Result_RESULT_UNSUPPORTED;
    }
    EXPECT_TRUE(controller.send(out));
  }
}
}  // namespace

TEST(Link, SendReceiveOverSocketPair)
{
  Pair p;
  Link pi(p.t.a), esp(p.t.b);

  Envelope e = pi.newEnvelope();
  JointCommand& cmd = setJointCommand(e, 5);
  cmd.mode[0] = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
  cmd.velocity[0] = 0.3f;
  ASSERT_TRUE(pi.send(e));
  ASSERT_TRUE(p.t.b.waitReadable(std::chrono::milliseconds(200)));
  EXPECT_EQ(esp.poll(), 1u);
  Envelope in;
  ASSERT_TRUE(esp.receive(in));
  EXPECT_EQ(in.seq, 1u);
  ASSERT_EQ(in.which_payload, scorbot_v1_Envelope_joint_command_tag);
  EXPECT_FLOAT_EQ(in.payload.joint_command.velocity[0], 0.3f);
  EXPECT_FALSE(esp.receive(in));
  EXPECT_EQ(pi.stats().tx_envelopes, 1u);
  EXPECT_EQ(esp.stats().rx_envelopes, 1u);
}

TEST(Link, SequenceGapsAreCounted)
{
  Pair p;
  Link pi(p.t.a), esp(p.t.b);
  for (int i = 0; i < 5; ++i)
  {
    Envelope e = pi.newEnvelope();
    setJointCommand(e, 1);
    if (i == 2)
      continue;  // never sent: seq 3 missing
    ASSERT_TRUE(pi.send(e));
  }
  p.t.b.waitReadable(std::chrono::milliseconds(200));
  EXPECT_EQ(esp.poll(), 4u);
  EXPECT_EQ(esp.stats().rx_seq_gaps, 1u);
}

TEST(Link, RequestResponseWithInterleavedStream)
{
  Pair p;
  Link pi(p.t.a), esp(p.t.b);

  // Controller thread: streams joint states and answers requests for a while.
  std::thread controller([&] {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < end)
    {
      Envelope s = esp.newEnvelope();
      setJointState(s, 5).state = scorbot_v1_SystemState_SYSTEM_STATE_READY;
      esp.send(s);
      serveOnce(esp);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });

  Request req = scorbot_v1_Request_init_zero;
  requestPing(req, 0xABCD);
  Response resp;
  ASSERT_TRUE(pi.request(req, resp, std::chrono::milliseconds(250)));
  EXPECT_EQ(resp.result, scorbot_v1_Result_RESULT_OK);
  ASSERT_EQ(resp.which_body, scorbot_v1_Response_pong_tag);
  EXPECT_EQ(resp.body.pong, 0xABCDu);

  requestGetInfo(req);
  ASSERT_TRUE(pi.request(req, resp, std::chrono::milliseconds(250)));
  ASSERT_EQ(resp.which_body, scorbot_v1_Response_get_info_tag);
  EXPECT_STREQ(resp.body.get_info.robot_type, "er_4pc");

  controller.join();
  // Joint states that arrived during the requests are still available, in order.
  pi.poll();
  Envelope e;
  std::size_t states = 0;
  uint32_t last_seq = 0;
  while (pi.receive(e))
  {
    if (e.which_payload == scorbot_v1_Envelope_joint_state_tag)
    {
      EXPECT_GT(e.seq, last_seq);
      last_seq = e.seq;
      ++states;
    }
  }
  EXPECT_GT(states, 5u);
}

TEST(Link, RequestTimesOutWithoutResponse)
{
  Pair p;
  Link pi(p.t.a), esp(p.t.b);
  Request req = scorbot_v1_Request_init_zero;
  requestPing(req, 1);
  Response resp;
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(pi.request(req, resp, std::chrono::milliseconds(60)));
  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GE(elapsed_ms, 55);
  EXPECT_LT(elapsed_ms, 500);
  serveOnce(esp, false);  // the request did arrive
  EXPECT_EQ(esp.stats().rx_envelopes, 1u);
}

TEST(Link, InboundQueueIsBounded)
{
  Pair p;
  Link pi(p.t.a), esp(p.t.b, Link::defaultClock, 4);
  for (int i = 0; i < 10; ++i)
  {
    Envelope e = pi.newEnvelope();
    setEvent(e).which_body = scorbot_v1_Event_state_changed_tag;
    ASSERT_TRUE(pi.send(e));
  }
  p.t.b.waitReadable(std::chrono::milliseconds(200));
  esp.poll();
  EXPECT_EQ(esp.stats().rx_dropped, 6u);
  Envelope e;
  int n = 0;
  while (esp.receive(e))
    ++n;
  EXPECT_EQ(n, 4);
  EXPECT_EQ(e.seq, 10u) << "newest kept";
}

TEST(Link, CorruptBytesOnTheWireAreSkipped)
{
  Pair p;
  Link pi(p.t.a), esp(p.t.b);
  // Line noise with no trailing delimiter: it fuses with the first frame and costs
  // that one frame (bad CRC). The next frame decodes normally.
  const uint8_t junk[] = {0x01, 0x02, 0x03, 0x00, 0x00, 0x7F};
  p.t.a.write(junk, sizeof(junk));
  for (int i = 0; i < 2; ++i)
  {
    Envelope e = pi.newEnvelope();
    setJointCommand(e, 2);
    ASSERT_TRUE(pi.send(e));
  }
  p.t.b.waitReadable(std::chrono::milliseconds(200));
  EXPECT_EQ(esp.poll(), 1u);
  Envelope in;
  ASSERT_TRUE(esp.receive(in));
  EXPECT_EQ(in.seq, 2u);
  EXPECT_GE(esp.stats().decoder.crc_errors + esp.stats().decoder.cobs_errors + esp.stats().decoder.short_frames, 1u);
}

TEST(SerialTransport, OpensAPtyAtRequestedBaud)
{
  int master = -1, slave = -1;
  char name[128] = {0};
  ASSERT_EQ(::openpty(&master, &slave, name, nullptr, nullptr), 0);
  ::close(slave);  // SerialTransport reopens it by path

  SerialTransport serial;
  std::string error;
  ASSERT_TRUE(serial.open(name, 921600, &error)) << error;
  EXPECT_TRUE(serial.isOpen());
  EXPECT_NE(serial.describe().find("921600"), std::string::npos);

  // Bytes written by the "device" side arrive through the transport.
  FdTransport device(master, true, "pty-master");
  Link pi(serial), esp(device);
  Envelope e = esp.newEnvelope();
  setJointState(e, 5).state = scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE;
  ASSERT_TRUE(esp.send(e));
  ASSERT_TRUE(serial.waitReadable(std::chrono::milliseconds(200)));
  EXPECT_EQ(pi.poll(), 1u);
  Envelope in;
  ASSERT_TRUE(pi.receive(in));
  EXPECT_EQ(in.payload.joint_state.state, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);

  EXPECT_FALSE(serial.open("/dev/definitely-not-a-port", 921600, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(SerialTransport::supportedBaud(12345));
  EXPECT_TRUE(SerialTransport::supportedBaud(115200));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
