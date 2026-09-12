# scorbot_moveit_config

MoveIt 2 configuration for one Scorbot arm: planning group `arm` from `base_link` to
`tool0`, named poses, kinematics, OMPL pipeline, and execution through the
`joint_trajectory_controller` that `scorbot_bringup` starts.

```bash
# everything on the virtual station: mock bringup + move_group + RViz MotionPlanning
ros2 launch scorbot_moveit_config demo.launch.py robot_type:=er_4pc

# or beside an already running bringup
ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc rviz:=false
ros2 launch scorbot_moveit_config move_group.launch.py robot_type:=er_4pc rviz:=true
```

In RViz, pick `ready`, `folded` or `zero` under "Goal State" in the MotionPlanning
panel, or drag the marker, then Plan and Execute.

## Files

| File | Purpose |
| --- | --- |
| `config/scorbot.srdf.xacro` | group `arm`, named poses `zero` `ready` `folded`, disabled collision pairs; takes `prefix` |
| `config/kinematics_analytic.yaml` | closed-form Scorbot solver from `scorbot_kinematics` (default, `ik:=analytic`) |
| `config/kinematics.yaml` | KDL, `position_only_ik: true` (`ik:=kdl`) |
| `config/kinematics_pick_ik.yaml` | pick_ik with approximate solutions (`ik:=pick_ik`) |
| `config/joint_limits.yaml` | velocity and acceleration limits for time parameterisation |
| `config/moveit_controllers.yaml` | simple controller manager → `joint_trajectory_controller` |
| `config/ompl_planning.yaml` | OMPL pipeline and planner set |
| `config/moveit.rviz` | RViz with the MotionPlanning display |
| `scorbot_moveit_config/moveit_configs.py` | builds the `MoveItConfigs` from launch arguments |
| `scorbot_moveit_config/config_utils.py` | prefixes joint names in the YAML files at launch |

Hand-written rather than Setup Assistant output so it can follow the URDF's `prefix`
and `robot_type` arguments and stay readable.

## Inverse kinematics on a five-joint arm

The arm has no sixth joint, so most 6-D poses are unreachable and full-pose KDL fails
on nearly every marker drag. Three answers:

- **analytic** (default). `scorbot_kinematics`: exact position, closest reachable
  orientation, branch nearest the current state. Smooth marker dragging, working
  pitch and roll rings; the yaw ring has no effect because the arm has no such freedom.
- **KDL, position only** (`ik:=kdl`). Pose goals keep the tool position and ignore
  orientation. Always available, drifts while dragging.
- **pick_ik** (`ik:=pick_ik`). Local gradient descent from the current state, so the
  marker drags smoothly and the rotation rings act on pitch and roll; the unreachable
  tool yaw is traded off instead of rejected. The Docker image installs
  `ros-jazzy-pick-ik` if the binary exists (check the build log for the warning, or
  `ws ros2 pkg list | grep pick_ik` inside the container).

For camera work the eventual answer is joint-space goals plus "look at" constraints;
MoveIt is for discrete moves, the tracking loop runs on the velocity controller.

## Namespaces and prefixes

Pass the same `name` and `prefix` as the bringup. `move_group` runs in the `name`
namespace so the relative controller name in `moveit_controllers.yaml` resolves to
that robot's `joint_trajectory_controller`; joint keys in `joint_limits.yaml` and
`moveit_controllers.yaml` are prefixed at launch.
