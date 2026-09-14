# scorbot_esp_sim

A fake ESP32 Scorbot controller. It speaks `scorbot_protocol` over a pseudo-terminal
and behaves the way `scorbot_protocol/docs/protocol.md` says the firmware must: the
state machine, homing with hard-stop reversal, the watchdog, soft limits, current and
faults, calibration persistence, reboots. Two jobs:

1. **Test double for the hardware interface.** `robot.launch.py mock:=false
   serial_port:=/tmp/scorbot` drives the simulator exactly as it will drive the ESP32,
   and the hardware interface's tests run against `ControllerSim` directly.
2. **Reference for the firmware.** `test/test_controller_sim.cpp` is the list of
   behaviors the firmware must reproduce, each with the timing that is expected.

## Run it

```bash
ros2 run scorbot_esp_sim scorbot_esp_sim --link /tmp/scorbot
# in another shell
ros2 run scorbot_protocol scorbot_protocol_cli --port /tmp/scorbot ping
ros2 run scorbot_protocol scorbot_protocol_cli --port /tmp/scorbot info
ros2 run scorbot_protocol scorbot_protocol_cli --port /tmp/scorbot home
ros2 run scorbot_protocol scorbot_protocol_cli --port /tmp/scorbot monitor 3
```

Options: `--robot er_4pc|er_v`, `--speed X` (simulated time runs X times faster),
`--watchdog MS`, `--quiet`. The pty is put in raw mode by the simulator, and output
nobody reads is discarded (counted as `tx_err` in `status`), like a real UART.

While it runs, type on its stdin:

| Command | Effect |
| --- | --- |
| `status` | state, counters, link stats, every joint's true and reported position |
| `fault overcurrent 2` | latch a fault (any `FaultCode` name, optional joint index) |
| `reboot` | power cycle: boot count +1, homing forgotten, calibration kept |
| `drop 0.05` | silently discard 5% of outgoing frames (sequence gaps) |
| `corrupt 0.01` | flip a bit in 1% of outgoing frames (CRC errors) |
| `quit` | exit |

## Model

- Each joint integrates velocity with the calibration's acceleration and velocity
  limits. Position mode is a proportional velocity command (time constant 0.1 s),
  velocity mode follows the setpoint.
- Soft limits clamp position setpoints and ramp velocity to zero on approach
  (`at_soft_limit` flag). Mechanical end stops sit 0.15 rad beyond the soft limits;
  pushing into one stalls the motor and pins the current to 1.5× the limit, which
  faults with `FAULT_OVERCURRENT` in ACTIVE.
- The limit switch trips at `home_offset_rad` when approached from `home_direction`
  and stays pressed for 0.02 rad past that edge, mid-range like the Scorbots'. Homing
  drives toward it at a quarter of the joint's maximum speed and zeroes the encoder on
  the edge (reported positions are exactly absolute afterward). Started on the wrong
  side, it reverses once at the hard stop, drives through the switch and re-approaches
  from `home_direction` so the same edge is used; started on the switch, it backs off
  until the switch releases plus 0.02 rad and re-approaches at homing speed. It times out into
  `FAULT_HOMING_TIMEOUT`.
- Positions read 0 at boot and only become absolute after homing. `Enable` with
  `allow_unhomed` removes soft limits (bench use).
- Current is a bias plus speed and acceleration terms; `invert` flips the motor sign
  while joint-space values on the wire keep their sign. `SimOptions::encoder_reversed`
  wires a joint's encoder phases the other way round (as one of the two Scorbots is);
  the calibration's `encoder_invert` must match it or position control runs away.
- `SetCalibration` is validated (positive ratios, ordered limits, ±1 home direction,
  index in range) and refused while moving. Calibration and robot type survive a
  reboot; the watchdog setting does not.

Defaults are a five-joint ER-4pc-like arm starting 0.4 rad away from every switch so
homing has to move. `SimOptions` sets joint names, calibration, initial positions,
timing and whether a pressed switch faults in ACTIVE (off: mid-range switches).
