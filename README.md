# scorbot_ros2

ROS 2 Jazzy stack for the Talos-RIT Scorbot arms: robot descriptions, a `ros2_control` hardware interface for the custom ESP32 controller, bringup, and a virtual station that runs on any computer through Docker.

Design: `project_documentation/technical/ros2/ros2_architecture.md`.

## Packages

| Package | Status | Purpose |
| --- | --- | --- |
| `scorbot_description` | usable | URDF/xacro, meshes, calibration for `er_4pc` (Bluey) and `er_v` (Bingo) |
| `scorbot_bringup` | todo | controllers, `robot.launch.py` (mock = virtual station, or real hardware), `demo_motion` |
| `scorbot_moveit_config` | todo | MoveIt 2 config (SRDF, planning group, IK, OMPL), `demo.launch.py` plans and executes on the station |
|`scorbot_kinematics` | todo | closed-form 5-DOF IK as a MoveIt kinematics plugin (exact position, best reachable orientation) |
| `scorbot_protocol` | todo | protobuf (nanopb) messages, COBS/CRC serial framing, `Link`, bench CLI |
| `scorbot_esp_sim` | todo | fake ESP32 over a pty: state machine, homing, watchdog, soft limits, faults, reboot; the firmware's executable spec |
| `scorbot_hardware` | todo | `ros2_control` SystemInterface for the ESP32 link: calibration push, position/velocity modes, fault and reboot handling |
| `scorbot_system_controller` | todo | home / enable / disable / clear-fault services and `~status` through the hardware's GPIO |
| `scorbot_msgs` | todo | `SystemStatus` message, `Home` service |
