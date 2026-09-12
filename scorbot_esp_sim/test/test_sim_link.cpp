// End to end: the simulator behind a Link on one side of a socketpair, a client Link
// on the other, exactly as the hardware interface will use it.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>

#include "scorbot_esp_sim/controller_sim.hpp"
#include "scorbot_esp_sim/faulty_transport.hpp"
#include "scorbot_protocol/link.hpp"

using namespace scorbot_esp_sim;
using scorbot_protocol::Envelope;
using scorbot_protocol::Link;
using scorbot_protocol::TransportPair;

namespace
{

/// Serves the sim over `link` for `seconds` of simulated time, in `dt` steps.
struct Server
{
  ControllerSim& sim;
  Link& link;
  void step(double dt)
  {
    link.poll();
    Envelope in;
    while (link.receive(in))
      sim.handle(in);
    sim.tick(dt);
    Envelope out;
    while (sim.popOutbound(out))
      link.send(out);
  }
  void run(double seconds, double dt = 0.001)
  {
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i)
      step(dt);
  }
};

/// Request through the client link while the server keeps ticking.
bool requestWhileServing(Link& client, Server& server, const scorbot_protocol::Request& req,
                         scorbot_protocol::Response& resp)
{
  // Link::request blocks on the transport; serve in small slices instead.
  Envelope e = client.newEnvelope();
  scorbot_protocol::Request& r = scorbot_protocol::setRequest(e, 1000);
  r.which_body = req.which_body;
  r.body = req.body;
  if (!client.send(e))
    return false;
  for (int i = 0; i < 200; ++i)
  {
    server.step(0.001);
    client.poll();
    Envelope in;
    // Drain, looking for our response; keep others out of the way.
    std::vector<Envelope> keep;
    bool found = false;
    while (client.receive(in))
    {
      if (in.which_payload == scorbot_v1_Envelope_response_tag && in.payload.response.request_id == 1000)
      {
        resp = in.payload.response;
        found = true;
      }
    }
    if (found)
      return true;
  }
  return false;
}

}  // namespace

TEST(SimLink, PingInfoEnableStreamAndCommand)
{
  TransportPair pair;
  ASSERT_TRUE(TransportPair::create(pair));
  ControllerSim sim;
  Link server_link(pair.a), client(pair.b);
  Server server{sim, server_link};
  server.run(0.1);  // boot

  scorbot_protocol::Request req = scorbot_v1_Request_init_zero;
  scorbot_protocol::Response resp;
  scorbot_protocol::requestPing(req, 99);
  ASSERT_TRUE(requestWhileServing(client, server, req, resp));
  EXPECT_EQ(resp.body.pong, 99u);

  scorbot_protocol::requestGetInfo(req);
  ASSERT_TRUE(requestWhileServing(client, server, req, resp));
  ASSERT_EQ(resp.which_body, scorbot_v1_Response_get_info_tag);
  EXPECT_EQ(resp.body.get_info.joint_names_count, 5);

  scorbot_protocol::requestSetWatchdog(req, 0);
  ASSERT_TRUE(requestWhileServing(client, server, req, resp));
  scorbot_protocol::requestEnable(req, true);
  ASSERT_TRUE(requestWhileServing(client, server, req, resp));
  EXPECT_EQ(resp.result, scorbot_v1_Result_RESULT_OK);

  // Stream velocity commands at 100 Hz for one second; states come back at 100 Hz.
  std::size_t states = 0;
  for (int i = 0; i < 100; ++i)
  {
    Envelope e = client.newEnvelope();
    auto& cmd = scorbot_protocol::setJointCommand(e, 5);
    for (int j = 0; j < 5; ++j)
      cmd.mode[j] = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    cmd.mode[2] = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
    cmd.velocity[2] = 0.3f;
    ASSERT_TRUE(client.send(e));
    server.run(0.01);
    client.poll();
    Envelope in;
    while (client.receive(in))
      states += in.which_payload == scorbot_v1_Envelope_joint_state_tag;
  }
  EXPECT_NEAR(static_cast<double>(states), 100.0, 3.0);
  EXPECT_EQ(sim.commandsReceived(), 100u);
  EXPECT_GT(sim.joint(2).reportedPosition(), 0.15);
  EXPECT_EQ(client.stats().rx_seq_gaps, 0u);
}

TEST(SimLink, DroppedAndCorruptedFramesAreVisibleInStats)
{
  TransportPair pair;
  ASSERT_TRUE(TransportPair::create(pair));
  ControllerSim sim;
  FaultyTransport faulty(pair.a, 5);
  faulty.setDropProbability(0.1);
  faulty.setCorruptProbability(0.1);
  Link server_link(faulty), client(pair.b);
  Server server{sim, server_link};

  std::size_t states = 0;
  for (int i = 0; i < 200; ++i)
  {
    server.run(0.01);
    client.poll();
    Envelope in;
    while (client.receive(in))
      states += in.which_payload == scorbot_v1_Envelope_joint_state_tag;
  }
  EXPECT_GT(faulty.dropped(), 5u);
  EXPECT_GT(faulty.corrupted(), 5u);
  EXPECT_GT(client.stats().decoder.crc_errors + client.stats().decoder.cobs_errors, 5u);
  EXPECT_GT(client.stats().rx_seq_gaps, 5u);
  EXPECT_GT(states, 120u) << "most frames still arrive";
  EXPECT_LT(states, 200u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
