# scorbot_ros2

ROS 2 Jazzy stack for the Talos-RIT Scorbot arms: robot descriptions, a
`ros2_control` hardware interface for the custom ESP32 controller, bringup, and a
virtual station that runs on any laptop through Docker.

Design: `project_documentation/technical/ros2/ros2_architecture.md`.

## Packages

| Package | Status | Purpose |
| --- | --- | --- |
| `scorbot_description` | usable | URDF/xacro, meshes, calibration for `er_4pc` (Bluey) and `er_v` (Bingo) |
| `scorbot_bringup` | usable | controllers, `robot.launch.py` (mock = virtual station, or real hardware), `demo_motion` |
| `scorbot_moveit_config` | usable | MoveIt 2 config (SRDF, planning group, IK, OMPL), `demo.launch.py` plans and executes on the station |
| `scorbot_kinematics` | usable | closed-form 5-DOF IK as a MoveIt kinematics plugin (exact position, best reachable orientation) |
| `scorbot_protocol` | usable | protobuf (nanopb) messages, COBS/CRC serial framing, `Link`, bench CLI; spec in `docs/protocol.md` |
| `scorbot_esp_sim` | usable | fake ESP32 over a pty: state machine, homing, watchdog, soft limits, faults, reboot; the firmware's executable spec |
| `scorbot_hardware` | usable | `ros2_control` SystemInterface for the ESP32 link: calibration push, position/velocity modes, fault and reboot handling; tested against the simulator |
| `scorbot_system_controller` | usable | home / enable / disable / clear-fault services and `~/status` through the hardware's GPIO |
| `scorbot_msgs` | usable | `SystemStatus` message, `Home` service |

## Quick start (any OS, Docker only)

Requires Docker Desktop or Docker Engine with the compose plugin. Nothing else is
installed on your machine; RViz runs inside the container and is shown in your browser.

```bash
cd scorbot_ros2
docker compose -f docker/compose.yaml up --build station
```

Then open <http://localhost:6080/vnc.html?autoconnect=1&resize=scale>. You get RViz
with the ER-4pc under the full `ros2_control` stack on mock hardware: this is the
**virtual station**. Make it move from a second terminal:

```bash
docker compose -f docker/compose.yaml exec station ws ros2 run scorbot_bringup demo_motion
```

Planned motion with MoveIt 2 (Plan and Execute from the RViz MotionPlanning panel),
the ER-V instead of the ER-4pc, or the description alone with joint sliders:

```bash
docker compose -f docker/compose.yaml up moveit
ROBOT_TYPE=er_v docker compose -f docker/compose.yaml up station
docker compose -f docker/compose.yaml up view
```

Stop with `Ctrl-C`. The first build downloads the ROS desktop image (a few GB).

### Linux with a local X server

RViz opens as a normal window on the host instead of in the browser (X11 or XWayland,
GPU rendering through `/dev/dri`):

```bash
xhost +local:docker          # once per login
docker compose -f docker/compose.yaml --profile native up station-native
docker compose -f docker/compose.yaml --profile native up moveit-native
```

If the window does not appear, check `echo $DISPLAY` is set in the terminal you run
compose from, and on NVIDIA install the nvidia container toolkit or set
`LIBGL_ALWAYS_SOFTWARE=1`.

### Developing inside the container

```bash
docker compose -f docker/compose.yaml run --rm --service-ports dev
# inside the container:
colcon build --symlink-install && source install/setup.bash
ros2 launch scorbot_description view_robot.launch.py robot_type:=er_v
```

The `dev` service bind-mounts this folder over the copy baked into the image, so edits
on the host are picked up by the next `colcon build`. Run the test suite the way CI
does with `docker compose -f docker/compose.yaml run --rm test`.

## What to try (for the team)

Everything below runs on a laptop with only Docker installed. Use the `*-native`
services on Linux for windows on your desktop, the plain ones for the browser.

1. **Virtual station.** `up station`, open the browser page, run `demo_motion` from a
   second terminal. You are watching the real `ros2_control` stack drive mock hardware.
2. **Drive joints by hand.** In the container: `ws ros2 run rqt_joint_trajectory_controller
   rqt_joint_trajectory_controller`, pick `joint_trajectory_controller`, move sliders.
3. **Plan with MoveIt.** `up moveit`. Drag the marker at the tool, or choose `ready` /
   `folded` under Goal State, then Plan and Execute. The marker follows exactly and the
   pitch and roll rings work; the yaw ring springs back because the arm has no yaw
   freedom at a fixed point (see `scorbot_kinematics/README.md`).
4. **Compare solvers.** `run --rm moveit-native ros2 launch scorbot_moveit_config
   demo.launch.py ik:=pick_ik` (or `ik:=kdl`) and feel the difference while dragging.
5. **Switch to velocity control.** With the station up:
   `ws ros2 control switch_controllers --deactivate joint_trajectory_controller
   --activate forward_velocity_controller`, then publish to
   `/forward_velocity_controller/commands`. This is the mode the tracking loop will use.
6. **Look at the model.** `up view` gives the description alone with joint sliders;
   `xacro` arguments are documented in `scorbot_description/urdf/scorbot.urdf.xacro`.
7. **Run the tests.** `run --rm test` builds and tests every package the way CI does
   (`.github/workflows/ci.yml`).
8. **Talk to a fake controller.** `ws ros2 run scorbot_esp_sim scorbot_esp_sim --link
   /tmp/scorbot` in one shell, then `ws ros2 run scorbot_protocol scorbot_protocol_cli
   --port /tmp/scorbot info` / `home` / `monitor 3` in another. Type `fault overcurrent 2`
   or `reboot` into the simulator to see the events.
9. **The real hardware interface, against the simulator.** `up station-sim` starts
   the simulator and `robot.launch.py mock:=false` against it (the same code path as a
   real ESP32), RViz in the browser; `--profile native up station-sim-native` for a
   native window (Linux). The joint controllers start but the controller is UNHOMED,
   so in another shell: `exec station-sim ws ros2 control switch_controllers --deactivate
   joint_trajectory_controller`, `exec station-sim ws ros2 service call
   /system_controller/home scorbot_msgs/srv/Home "{}"`, the same with
   `/system_controller/enable std_srvs/srv/Trigger`, switch the trajectory controller
   back on, then `demo_motion` moves the simulated robot through the real
   `ScorbotSystem` plugin. `exec station-sim ws ros2 topic echo
   /system_controller/status` shows the controller state. For fault injection run the
   simulator yourself in a `dev` shell (step 8) and launch with `mock:=false
   serial_port:=/tmp/scorbot` in a second one.

Not there yet: firmware on a real ESP32 that speaks the protocol (the simulator is
the stand-in until then), and the camera/vision side.

## Native ROS 2 install (no Docker)

Ubuntu 24.04 with ROS 2 Jazzy desktop:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
ln -s /path/to/scorbot_ros2 .
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc      # virtual station
ros2 run scorbot_bringup demo_motion                                # second terminal
```

## Conventions

- Joint names are fixed across both robots: `base_joint`, `shoulder_joint`,
  `elbow_joint`, `wrist_pitch_joint`, `wrist_roll_joint`. Use the `prefix` argument
  when two arms share one TF tree (`bingo_`, `bluey_`).
- All robot-specific numbers live in `scorbot_description/config/`. Do not put them in
  code, launch files, or firmware.
- Values marked `VERIFY` in the config files have not been checked against a physical
  robot. Remove the mark when you have measured them.
