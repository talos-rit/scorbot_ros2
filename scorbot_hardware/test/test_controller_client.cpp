#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "scorbot_hardware/controller_client.hpp"
#include "sim_server.hpp"

using namespace scorbot_hardware;
using namespace std::chrono_literals;
using scorbot_hardware_test::SimServer;

TEST(ControllerClient, RequestsAreMatchedByIdAndStatesStream)
{
  SimServer server;
  ControllerClient client(server.clientTransport());
  client.start();

  scorbot_protocol::Request req = scorbot_v1_Request_init_zero;
  scorbot_protocol::Response resp;
  std::string err;
  scorbot_protocol::requestPing(req, 7);
  ASSERT_TRUE(client.request(req, resp, 500ms, &err)) << err;
  EXPECT_EQ(resp.body.pong, 7u);
  scorbot_protocol::requestGetInfo(req);
  ASSERT_TRUE(client.request(req, resp, 500ms, &err)) << err;
  EXPECT_EQ(resp.body.get_info.joint_names_count, 5);

  std::this_thread::sleep_for(150ms);
  EXPECT_GT(client.statesReceived(), 8u);
  EXPECT_LT(client.stateAgeUs(), 100000u);
  const StateSnapshot snap = client.latestState();
  EXPECT_TRUE(snap.valid);
  EXPECT_EQ(snap.state.state, scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
  EXPECT_EQ(client.stats().rx_seq_gaps, 0u);
  client.stop();
}

TEST(ControllerClient, TimesOutWhenNobodyAnswers)
{
  SimServer server;
  server.pause(true);
  ControllerClient client(server.clientTransport());
  client.start();
  scorbot_protocol::Request req = scorbot_v1_Request_init_zero;
  scorbot_protocol::Response resp;
  std::string err;
  scorbot_protocol::requestPing(req, 1);
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.request(req, resp, 50ms, &err));
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count(), 50);
  EXPECT_NE(err.find("no response"), std::string::npos) << err;
  EXPECT_EQ(client.stateAgeUs(), UINT64_MAX);
}

TEST(ControllerClient, EventsAreQueuedAndBounded)
{
  SimServer server;
  ControllerClient client(server.clientTransport(), 4);
  client.start();
  std::this_thread::sleep_for(100ms);  // boot -> UNHOMED event
  for (int i = 0; i < 6; ++i)
    server.withSim([&](scorbot_esp_sim::ControllerSim& sim) {
      sim.injectFault(scorbot_v1_FaultCode_FAULT_OVERCURRENT, 1);
      sim.reboot();  // each cycle: fault event + state change events
      return 0;
    });
  std::this_thread::sleep_for(100ms);
  const auto events = client.takeEvents();
  EXPECT_LE(events.size(), 4u);
  EXPECT_GT(events.size(), 0u);
  EXPECT_TRUE(client.takeEvents().empty());
}

TEST(ControllerClient, ClosedPeerIsReported)
{
  auto server = std::make_unique<SimServer>();
  ControllerClient client(server->clientTransport());
  client.start();
  std::this_thread::sleep_for(50ms);
  server.reset();  // closes the other end of the socketpair
  std::this_thread::sleep_for(100ms);
  EXPECT_TRUE(client.transportFailed());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
