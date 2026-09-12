"""Shared launch helpers: common arguments and the MoveItConfigs for one Scorbot arm.

Kept in a module so move_group.launch.py and demo.launch.py build the identical
configuration, and so the argument list matches scorbot_bringup/robot.launch.py.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

from scorbot_moveit_config.config_utils import KINEMATICS_FILES, write_prefixed

COMMON_KEYS = ["robot_type", "name", "prefix", "world_frame", "x", "y", "z", "yaw", "ik"]


def declare_common_arguments() -> list[DeclareLaunchArgument]:
    """Arguments shared with scorbot_bringup/robot.launch.py, plus `ik`."""
    return [
        DeclareLaunchArgument(
            "robot_type",
            default_value="er_4pc",
            choices=["er_4pc", "er_v"],
            description="Which Scorbot: er_4pc (Bluey) or er_v (Bingo).",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="",
            description="Namespace; must match the bringup.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Link/joint prefix; must match the bringup.",
        ),
        DeclareLaunchArgument("world_frame", default_value="world", description="Fixed frame."),
        DeclareLaunchArgument("x", default_value="0.0", description="Base position in world_frame."),
        DeclareLaunchArgument("y", default_value="0.0", description="Base position in world_frame."),
        DeclareLaunchArgument("z", default_value="0.0", description="Base position in world_frame."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Base yaw in world_frame."),
        DeclareLaunchArgument(
            "ik",
            default_value="analytic",
            choices=["analytic", "kdl", "pick_ik"],
            description="Inverse kinematics solver: analytic (closed-form Scorbot solver, "
            "default), kdl (position only), or pick_ik (numerical, needs ros-jazzy-pick-ik).",
        ),
    ]


def build_moveit_configs(context):
    """Build the MoveItConfigs for the launch arguments in `context`."""
    cfg = {key: LaunchConfiguration(key).perform(context) for key in COMMON_KEYS}
    prefix = cfg["prefix"]

    description_share = get_package_share_directory("scorbot_description")
    moveit_share = get_package_share_directory("scorbot_moveit_config")

    urdf = os.path.join(description_share, "urdf", "scorbot.urdf.xacro")
    srdf = os.path.join(moveit_share, "config", "scorbot.srdf.xacro")
    kinematics = os.path.join(moveit_share, "config", KINEMATICS_FILES[cfg["ik"]])
    joint_limits = write_prefixed(
        os.path.join(moveit_share, "config", "joint_limits.yaml"), prefix, "joint_limits"
    )
    controllers = write_prefixed(
        os.path.join(moveit_share, "config", "moveit_controllers.yaml"), prefix, "controllers"
    )

    urdf_mappings = {
        "robot_type": cfg["robot_type"],
        "name": cfg["name"] or cfg["robot_type"],
        "prefix": prefix,
        "world_frame": cfg["world_frame"],
        "x": cfg["x"],
        "y": cfg["y"],
        "z": cfg["z"],
        "yaw": cfg["yaw"],
        # move_group only needs the kinematic model; ros2_control is the bringup's job.
        "use_ros2_control": "false",
    }

    builder = (
        MoveItConfigsBuilder("scorbot", package_name="scorbot_moveit_config")
        .robot_description(file_path=urdf, mappings=urdf_mappings)
        .robot_description_semantic(file_path=srdf, mappings={"prefix": prefix})
        .robot_description_kinematics(file_path=kinematics)
        .joint_limits(file_path=joint_limits)
        .trajectory_execution(file_path=controllers)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            # robot_state_publisher owns /robot_description (it carries the ros2_control
            # tag the controller manager needs); move_group must not publish a second copy.
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
    )
    return builder.to_moveit_configs()
