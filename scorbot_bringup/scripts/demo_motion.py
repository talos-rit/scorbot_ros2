#!/usr/bin/env python3
"""Drive the arm through a slow sequence of poses via the joint_trajectory_controller

Initial test for the virtual station and the first thing to run on a 
robot once homing had been completed.
"""

import argparse
import math
import sys

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
]

# Waypoints in degrees, well inside each joint limit, ends back at zero.
WAYPOINTS_DEG = [
    [0, 0, 0, 0, 0],
    [40, 30, -40, 20, 0],
    [40, 60, -90, 30, 90],
    [-40, 60, -90, 30, -90],
    [-40, 20, -20, 0, 0],
    [0, 0, 0, 0, 0],
]

def build_trajectory(prefix: str, scale: float, segment_time: float) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = [f"{prefix}{j}" for j in JOINTS]
    t = 0.0
    for waypoint in WAYPOINTS_DEG:
        t += segment_time
        point = JointTrajectoryPoint()
        point.positions = [math.radians(v) * scale for v in waypoint]
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(seconds=t).to_msg()
        traj.points.append(point)
    return traj

class DemoMotion(Node):
    def __init__(self, namespace: str):
        super().__init__("demo_motion")
        ns = "/" + namespace.strip("/") if namespace.strip("/") else ""
        self.action_name = f"{ns}/joint_trajectory_controller/follow_joint_trajectory"
        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

    def run_once(self, trajectory: JointTrajectory) -> bool:
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f"No action server at {self.action_name}")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Trajectory failed: error_code={result.error_code} {result.error_string}"
            )
            return False
        self.get_logger().info("Trajectory completed")
        return True

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--namespace", default="", help="Robot namespace (launch `name`).")
    parser.add_argument("--prefix", default="", help="Joint name prefix (launch `prefix`).")
    parser.add_argument("--loops", type=int, default=1, help="How many times to run the sequence.")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale every waypoint angle.")
    parser.add_argument("--segment-time", type=float, default=3.0, help="Seconds per waypoint.")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = DemoMotion(args.namespace)
    trajectory = build_trajectory(args.prefix, args.scale, args.segment_time)
    ok = True
    try:
        for i in range(args.loops):
            node.get_logger().info(f"Loop {i + 1}/{args.loops}")
            ok = node.run_once(trajectory)
            if not ok:
                break
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
