# scorbot_system_controller

A `ros2_control` controller that turns `scorbot_hardware`'s `system` GPIO interfaces
into services and a status topic. Spawned by `robot.launch.py` with `mock:=false`.

```bash
ros2 service call /system_controller/home scorbot_msgs/srv/Home "{}"          # all joints
ros2 service call /system_controller/home scorbot_msgs/srv/Home "{joints: [base_joint]}"
ros2 service call /system_controller/enable std_srvs/srv/Trigger
ros2 service call /system_controller/disable std_srvs/srv/Trigger
ros2 service call /system_controller/clear_fault std_srvs/srv/Trigger
ros2 service call /system_controller/enable_unhomed std_srvs/srv/Trigger       # bench only
ros2 topic echo /system_controller/status
```

Each service blocks until the controller reaches the expected state (`home` until
READY, `enable` until ACTIVE, `disable` until not ACTIVE, `clear_fault` until not
FAULT) or times out, and returns the reason when the hardware or the controller refused
(`home` while joint controllers are active, `enable` when a controller holds a stale
setpoint, a homing fault, ...).

First bringup of a robot, or after a fault:

```bash
ros2 control switch_controllers --deactivate joint_trajectory_controller
ros2 service call /system_controller/home scorbot_msgs/srv/Home "{}"
ros2 service call /system_controller/enable std_srvs/srv/Trigger
ros2 control switch_controllers --activate joint_trajectory_controller
```

Parameters: `gpio_name` (default `system`; `<prefix>system` with a joint prefix),
`joints` (names in controller order, for the `Home` request and the status message),
`home_timeout_s` (90), `request_timeout_s` (2), `status_rate_hz` (10),
`link_timeout_ms` (100).

The handshake with the hardware is race-free: a service writes one pulse, waits for
the hardware's `request_count` to advance and `last_result` to leave `pending`, then
watches `state`. Services run in their own reentrant callback group so a long homing
call does not stall the controller manager; one operation runs at a time.
