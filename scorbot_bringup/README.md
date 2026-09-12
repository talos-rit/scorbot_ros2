# scorbot_bringup

Launch files and controller configuration for one Scorbot arm under `ros2_control`. With `mock:=true` (the default) this is the **virtual station**: the full controller stack running against mock hardware, visible in RViz, no robot required.

```bash
# virtual station, Bluey
ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc
# virtual station, Bingo
ros2 launch scorbot_bringup robot.launch.py robot_type:=er_v

# make it move
ros2 run scorbot_bringup demo_motion
# or drive the joints by hand
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

One robot per launch, one robot per RViz, exactly as on the real Pis. Two stations can share a netowrk by giving each a `name` (namespace) and a `prefix`.

## What `robot.launch.py` starts

| Node | Role |
| --- | --- |
| `robot_state_publisher` | TF and the `robot_description` topic from `scorbot_description` |
| `ros2_control_node` | controller manager at 100 Hz, reads `robot_description` from the topic |
| `joint_state_broadcaster` | publishes `joint_states` |
| `joint_trajectory_controller` | active, point-to-point moves, MoveIt, `demo_motion` |
| `forward_velocity_controller` | loaded, inactive, the tracking loop will use this |
| `rviz2` | optional (`rviz:=false` on a Pi) |

Arguments: `robot_type`, `name`, `prefix`, `mock`, `serial_port`, `calibration_file`, `controllers_file`, `world_frame`, `x y z yaw`, `rviz`, `rviz_config`, `controller_manager_timeout`, Run `ros2 launch scorbot_bringup robot.launch.py --show-args`.


## Switching controllers

Only one of the two joint controllers may own the joints at one time:

```bash
ros2 control switch_controllers \
    --deactivate joint_trajectory_controller --activate forward_velocity_controller
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.2, 0.0, 0.0, 0.0, 0.0]}"
```

## Controller configuration

`config/controllers.yaml` is written once with plain joint and node names. The launch file rewrutes it per robot with `scorbot_bringup/controllers_config.py` before passing it to `ros2_control_node` and the spawners. Edit the source file, never the generated ones.