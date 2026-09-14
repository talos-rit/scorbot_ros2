# scorbot_kinematics

Closed-form inverse kinematics for the five-joint Scorbot arms, packaged as a MoveIt 2
`kinematics::KinematicsBase` plugin. Select it in `scorbot_moveit_config` with
`ik:=analytic` (the default).

## Why a custom solver

The arm has five joints, so once the tool position is fixed only two orientation
parameters remain: elevation (pitch) and roll. The tool's yaw is dictated by the base
angle. Generic numerical solvers either reject most 6-D requests (KDL) or drift through
compromise solutions while the RViz marker is dragged (position-only KDL, optimizers).
This solver instead:

1. takes the base angle from the requested point's x, y;
2. projects the requested tool direction into the arm's vertical plane to get the pitch;
3. takes the roll as the closest rotation about the tool axis;
4. solves the planar two-link problem for shoulder and elbow, and the wrist pitch from
   the remainder;
5. evaluates both base branches and both elbow branches, checks joint limits, and
   returns them ordered by distance from the current joint state.

The position is always met exactly when reachable. Orientation is the best reachable
one; the residual is reported as `orientation_error` and, when it exceeds
`orientation_tolerance` (radians, default 3.2 so it never triggers), the solution is
only returned if MoveIt asked for approximate solutions.

## Layout

| File | Purpose |
| --- | --- |
| `include/scorbot_kinematics/analytic_ik.hpp` | ROS-free solver: `Geometry`, `forwardKinematics`, `inverseKinematics` |
| `include/.../scorbot_kinematics_plugin.hpp`, `src/scorbot_kinematics_plugin.cpp` | MoveIt plugin; reads geometry, axis signs, limits and the tool offset from the loaded robot model |
| `test/test_analytic_ik.cpp` | gtest: round trips over random reachable poses for both robots, projection of unreachable yaw, branch selection, limits, axis signs |

## Requirements on the description

Checked at start-up with a clear error if violated:

- exactly five revolute joints in the group, axes z, ±y, ±y, ±y, ±x in that order;
- every arm frame aligned with the base at the zero pose, upper arm and forearm along +x;
- wrist roll axis intersecting the wrist pitch axis;
- the tip frame on the flange axis (any offset along it, none across it). Camera mount
  offsets belong on a separate link, not on `tool0`.

`scorbot_description` satisfies all of these for both robots and any prefix.

## Parameters (`kinematics.yaml`)

```yaml
arm:
  kinematics_solver: scorbot_kinematics/ScorbotKinematicsPlugin
  kinematics_solver_timeout: 0.005
  orientation_tolerance: 3.2   # rad; lower it to reject poses whose orientation is far off
```
