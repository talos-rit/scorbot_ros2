// scorbot_esp_sim: a fake ESP32 controller on a pseudo-terminal.
//
//   scorbot_esp_sim                      # prints the pty path, e.g. /dev/pts/3
//   scorbot_esp_sim --link /tmp/scorbot  # also creates a stable symlink to it
//   scorbot_esp_sim --robot er_v --speed 2.0
//
// Then, from another shell:
//   scorbot_protocol_cli --port /dev/pts/3 ping
//   ros2 launch scorbot_bringup robot.launch.py mock:=false serial_port:=/tmp/scorbot
//
// Injection commands on stdin while running:
//   fault <watchdog|overcurrent|limit_switch|soft_limit|stall|homing_timeout|encoder|driver|internal> [joint]
//   reboot            drop <p>           corrupt <p>          status          quit

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "scorbot_esp_sim/controller_sim.hpp"
#include "scorbot_esp_sim/faulty_transport.hpp"
#include "scorbot_protocol/link.hpp"

using namespace scorbot_esp_sim;
using scorbot_protocol::Envelope;

namespace
{

FaultCode parseFault(const std::string& s)
{
  static const struct { const char* name; FaultCode code; } table[] = {
    {"watchdog", scorbot_v1_FaultCode_FAULT_WATCHDOG}, {"overcurrent", scorbot_v1_FaultCode_FAULT_OVERCURRENT},
    {"limit_switch", scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH}, {"soft_limit", scorbot_v1_FaultCode_FAULT_SOFT_LIMIT},
    {"stall", scorbot_v1_FaultCode_FAULT_STALL}, {"homing_timeout", scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT},
    {"encoder", scorbot_v1_FaultCode_FAULT_ENCODER}, {"driver", scorbot_v1_FaultCode_FAULT_DRIVER},
    {"internal", scorbot_v1_FaultCode_FAULT_INTERNAL},
  };
  for (const auto& t : table)
    if (s == t.name)
      return t.code;
  return scorbot_v1_FaultCode_FAULT_NONE;
}

void printStatus(const ControllerSim& sim, const scorbot_protocol::Link& link, const FaultyTransport& faulty)
{
  std::printf("state=%s fault=%s boot=%u wd=%ums rx=%u ign=%u | tx=%llu tx_err=%llu rx=%llu crc_err=%llu drop=%llu corrupt=%llu\n",
              scorbot_protocol::toString(sim.state()), scorbot_protocol::toString(sim.fault()), sim.bootCount(),
              sim.watchdogMs(), sim.commandsReceived(), sim.commandsIgnored(),
              static_cast<unsigned long long>(link.stats().tx_envelopes),
              static_cast<unsigned long long>(link.stats().tx_errors),
              static_cast<unsigned long long>(link.stats().rx_envelopes),
              static_cast<unsigned long long>(link.stats().decoder.crc_errors),
              static_cast<unsigned long long>(faulty.dropped()), static_cast<unsigned long long>(faulty.corrupted()));
  for (std::size_t i = 0; i < sim.jointCount(); ++i)
  {
    const JointSim& j = sim.joint(i);
    std::printf("  j%zu true=%+.3f rep=%+.3f vel=%+.3f cur=%.2f%s%s%s\n", i, j.true_position, j.reportedPosition(),
                j.velocity, j.current, j.homed ? " homed" : "", j.limit_switch ? " SWITCH" : "", j.fault ? " FAULT" : "");
  }
  std::fflush(stdout);
}

bool handleCommand(const std::string& line, ControllerSim& sim, scorbot_protocol::Link& link, FaultyTransport& faulty)
{
  std::istringstream in(line);
  std::string cmd;
  in >> cmd;
  if (cmd.empty())
    return true;
  if (cmd == "quit" || cmd == "exit")
    return false;
  if (cmd == "status")
    printStatus(sim, link, faulty);
  else if (cmd == "reboot")
  {
    sim.reboot();
    std::printf("rebooted (boot_count=%u)\n", sim.bootCount());
  }
  else if (cmd == "fault")
  {
    std::string name;
    int joint = -1;
    in >> name >> joint;
    const FaultCode code = parseFault(name);
    if (code == scorbot_v1_FaultCode_FAULT_NONE)
      std::printf("unknown fault '%s'\n", name.c_str());
    else
    {
      sim.injectFault(code, joint);
      std::printf("injected %s\n", scorbot_protocol::toString(code));
    }
  }
  else if (cmd == "drop")
  {
    double p = 0;
    in >> p;
    faulty.setDropProbability(p);
    std::printf("drop probability %.3f\n", p);
  }
  else if (cmd == "corrupt")
  {
    double p = 0;
    in >> p;
    faulty.setCorruptProbability(p);
    std::printf("corrupt probability %.3f\n", p);
  }
  else
    std::printf("commands: status | reboot | fault <name> [joint] | drop <p> | corrupt <p> | quit\n");
  std::fflush(stdout);
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  SimOptions opts;
  std::string link_path;
  double speed = 1.0;
  bool quiet = false;
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if (a == "--link" && i + 1 < argc)
      link_path = argv[++i];
    else if (a == "--robot" && i + 1 < argc)
      opts.robot_type = argv[++i];
    else if (a == "--speed" && i + 1 < argc)
      speed = std::atof(argv[++i]);
    else if (a == "--watchdog" && i + 1 < argc)
      opts.watchdog_ms = static_cast<uint32_t>(std::atoi(argv[++i]));
    else if (a == "--quiet")
      quiet = true;
    else
    {
      std::fprintf(stderr, "usage: scorbot_esp_sim [--link PATH] [--robot er_4pc|er_v] [--speed X] [--watchdog MS] [--quiet]\n");
      return a == "-h" || a == "--help" ? 0 : 64;
    }
  }

  int master = -1, slave = -1;
  char name[128] = {0};
  if (::openpty(&master, &slave, name, nullptr, nullptr) != 0)
  {
    std::perror("openpty");
    return 2;
  }
  // Keep the slave open ourselves so the master never sees EOF between clients, and put
  // it in raw mode now: a fresh pty is in cooked mode (canonical buffering, echo), which
  // holds our 100 Hz output in a 4 kB line buffer and echoes it back at us until the first
  // client sets raw mode. With the buffer full, the reply to that client's first request
  // got EAGAIN and was lost.
  termios tio{};
  if (::tcgetattr(slave, &tio) == 0)
  {
    ::cfmakeraw(&tio);
    ::tcsetattr(slave, TCSANOW, &tio);
  }
  if (!link_path.empty())
  {
    ::unlink(link_path.c_str());
    if (::symlink(name, link_path.c_str()) != 0)
      std::perror("symlink");
  }

  scorbot_protocol::FdTransport pty(master, true, std::string("pty ") + name);
  FaultyTransport faulty(pty);
  scorbot_protocol::Link link(faulty);
  ControllerSim sim(opts);

  std::printf("scorbot_esp_sim: robot=%s serial=%s%s%s speed=%.2f\n", opts.robot_type.c_str(), name,
              link_path.empty() ? "" : " link=", link_path.c_str(), speed);
  std::printf("type 'status', 'fault overcurrent 2', 'reboot', 'drop 0.05', 'corrupt 0.01' or 'quit'\n");
  std::fflush(stdout);

  const int stdin_flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
  ::fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK);
  std::string stdin_buf;

  auto last = std::chrono::steady_clock::now();
  SystemState last_state = sim.state();
  bool running = true;
  while (running)
  {
    pollfd fds[2] = {{master, POLLIN, 0}, {STDIN_FILENO, POLLIN, 0}};
    ::poll(fds, 2, 1);

    // Inbound frames.
    link.poll();
    Envelope in;
    while (link.receive(in))
      sim.handle(in);

    // Advance simulated time by the wall-clock step.
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count() * speed;
    last = now;
    sim.tick(std::min(dt, 0.05));

    // Outbound frames. With no client reading the slave, the kernel queue (4 kB) fills
    // and writes come back short: a real UART would simply lose those bytes, so discard
    // the stale unread output rather than stall behind it.
    Envelope out;
    while (sim.popOutbound(out))
      if (!link.send(out))
        ::tcflush(slave, TCIFLUSH);  // counted in the link's tx_errors

    if (!quiet && sim.state() != last_state)
    {
      std::printf("state -> %s%s%s\n", scorbot_protocol::toString(sim.state()),
                  sim.fault() != scorbot_v1_FaultCode_FAULT_NONE ? " fault=" : "",
                  sim.fault() != scorbot_v1_FaultCode_FAULT_NONE ? scorbot_protocol::toString(sim.fault()) : "");
      std::fflush(stdout);
      last_state = sim.state();
    }

    // Operator commands.
    if (fds[1].revents & POLLIN)
    {
      char buf[256];
      const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
      if (n > 0)
        stdin_buf.append(buf, static_cast<std::size_t>(n));
      else if (n == 0)
        fds[1].fd = -1;  // stdin closed; keep serving
      std::size_t nl;
      while ((nl = stdin_buf.find('\n')) != std::string::npos)
      {
        running = handleCommand(stdin_buf.substr(0, nl), sim, link, faulty) && running;
        stdin_buf.erase(0, nl + 1);
      }
    }
  }

  if (!link_path.empty())
    ::unlink(link_path.c_str());
  ::close(slave);
  return 0;
}
