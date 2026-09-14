# scorbot_hardware

The `ros2_control` hardware interface for the ESP32 Scorbot controller:
`scorbot_hardware/ScorbotSystem`, a `SystemInterface` plugin that talks
`scorbot_protocol` over USB serial. Modeled on `abb_ros2`'s hardware interface, with
two command modes per joint and a service surface through GPIO interfaces (design
D8). Design: `project_documentation/technical/ros2/ros2_architecture.md`, sections 5.5
and 6.

```bash
# Real robot
ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc mock:=false \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102-...
# The simulator instead of a robot (same code path)
ros2 run scorbot_esp_sim scorbot_esp_sim --link /tmp/scorbot
ros2 launch scorbot_bringup robot.launch.py mock:=false serial_port:=/tmp/scorbot
```

## Layout

| File | Depends on ROS | What |
| --- | --- | --- |
| `controller_client.hpp/.cpp` | no | `ControllerClient`: the link with a reader thread. Newest `JointState` without blocking, `request()` matched by id, events queued, link stats. |
| `calibration.hpp/.cpp` | no | `scorbot_description/config/<robot>_calibration.yaml` to `JointCalibration` messages, ordered by the description's joint names (prefix stripped). |
| `session.hpp/.cpp` | no | `Session`: everything the plugin does. Owns the interface storage, configure/activate/read/write, command-mode bookkeeping, the GPIO pulse handling and a worker thread for the requests they trigger. |
| `scorbot_system.hpp/.cpp` | yes | `ScorbotSystem`: the `SystemInterface` adapter over `Session`. Parses `HardwareInfo`, copies between handles and the session. |

The split is deliberate: the three ROS-free files are compiled and tested in-process
against `scorbot_esp_sim::ControllerSim` (`test/test_session.cpp`, 12 cases,
`test/test_controller_client.cpp`, `test/test_calibration.cpp`), which also runs
without a ROS install. `test/test_scorbot_system.cpp` loads the plugin through
`hardware_interface::ResourceManager` and drives it against the simulator executable
over a pty, as `robot.launch.py mock:=false` does.

## Hardware parameters (`<ros2_control>` block)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `transport` | `serial` | only serial for now (UDP later) |
| `serial_port`, `baud_rate` | `/dev/ttyUSB0`, `921600` | the ESP32 link |
| `robot_type` | required | `er_4pc` or `er_v`; must match what the controller reports (an `unset` controller is labeled) |
| `prefix` | `` | joint-name prefix of the description; stripped before matching the controller's joint names |
| `calibration_file` | required | pushed with `SetCalibration` at configure; the file's `robot_type` must match |
| `state_timeout_ms` | 100 | `read()` returns ERROR when no `JointState` arrived for this long |
| `command_watchdog_ms` | 200 | pushed with `SetWatchdog`: the controller faults if commands stop for this long while ACTIVE |
| `request_timeout_ms` | 200 | per request/response |
| `activate_timeout_s` | 3.0 | waiting for the controller to answer, stream and leave BOOT |
| `auto_home_on_activate` | false | home during `on_activate` if the controller is not homed (blocks for the homing time) |
| `allow_unhomed` | false | bench only: `Enable(allow_unhomed)` when the controller is UNHOMED |
| `home_timeout_s` | 90 | `Home` request timeout |

## Lifecycle

- **configure**: open the port, `GetInfo` (protocol major must match, minor warns;
  robot type and joint names in order must match the description), `SetCalibration`
  per joint, `SetWatchdog`, wait for telemetry and for the controller to leave BOOT.
  Any mismatch fails with a log line saying which.
- **activate**: seed every position command with the current position and every
  velocity command with 0. Then, by controller state: READY → `Enable`, wait for
  ACTIVE; UNHOMED → warn (or `Enable(allow_unhomed)`, or home first with
  `auto_home_on_activate`); FAULT/HOMING → warn. Activation succeeds in every case
  where the link is healthy; the controller state is on the GPIO.
- **deactivate**: `Disable` (the controller brakes). **cleanup/error/shutdown**: stop
  threads, close the port.

## `read()` and `write()`

`read()` copies the newest `JointState` into position/velocity/effort (effort is the
motor current in amperes) and the GPIO states. It returns `ERROR` only when the link
is dead (no frame for `state_timeout_ms`, or the port failed) or when the controller
rebooted (`boot_count` changed: calibration and homing are gone, reconfigure). A
controller **FAULT is not a read error**: it is reported on `system/state` and
`system/fault_code`, the joint controllers keep running, the controller ignores their
commands, and `clear_fault` + `enable` recover without touching the lifecycle.

`write()` sends one `JointCommand` per cycle while the controller is ACTIVE, with each
joint in the mode its claimed command interface implies (position, velocity, or none).
NaN commands mean "no command". Nothing is sent in other states, so the controller's
`commands_ignored` counter stays meaningful.

Command modes: `prepare_command_mode_switch` refuses a joint with both `position` and
`velocity` claimed. `perform_command_mode_switch` seeds a joint entering position mode
with its current position and a joint entering velocity mode with 0. Switching while
active is allowed (that is what `switch_controllers` does).

## The `system` GPIO

Commands are pulses: write a nonzero value once, the hardware acts on it in its next
`write()` and resets it to 0. `scorbot_system_controller` wraps them as services.

| Command | Value | Rule |
| --- | --- | --- |
| `home` | joint bitmask (`0x1F` = all) | refused (`last_result` = INVALID_STATE) while any joint command interface is claimed: stop the joint controllers first |
| `enable` | 1, or 2 for `allow_unhomed` | refused if a claimed joint is commanded away from where it is (a stale hold point would jump); allowed when the commands match the state |
| `disable` | nonzero | always |
| `clear_fault` | nonzero | always; the controller decides READY or UNHOMED |

| State | Meaning |
| --- | --- |
| `state`, `fault_code` | `scorbot.v1.SystemState`, `scorbot.v1.FaultCode` |
| `homed_mask`, `limit_mask` | per-joint bits |
| `link_age_ms` | age of the newest `JointState` |
| `firmware_version` | `major*10000 + minor*100 + patch` |
| `boot_count` | the controller's reset counter |
| `last_result` | `scorbot.v1.Result` of the last pulse; -1 while one is in flight |
| `commands_ignored` | the controller's counter of dropped `JointCommand`s |
| `request_count` | pulses consumed so far (accepted or refused); services wait on it |

Only one request is in flight at a time; a second pulse while one runs is dropped with
`last_result` = BUSY.

## Threads

Reader (in `ControllerClient`: waits on the port, decodes, stores the newest state),
worker (in `Session`: runs the requests behind GPIO pulses), and the controller
manager's `read()`/`write()`. The realtime path takes two short mutexes (the link for
one non-blocking write, the state slot for one copy); no allocation after configure.
