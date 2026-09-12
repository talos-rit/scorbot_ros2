"""Plan and execute in RViz on the virtual station: bringup (mock) + move_group + RViz.

    ros2 launch scorbot_moveit_config demo.launch.py robot_type:=er_4pc

In RViz: drag the interactive marker or pick a named pose (zero, ready, folded) in the
MotionPlanning panel, press Plan, then Execute. The trajectory runs through the same
joint_trajectory_controller the real robot will use.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from scorbot_moveit_config.moveit_configs import declare_common_arguments


def generate_launch_description():
    arguments = declare_common_arguments() + [
        DeclareLaunchArgument(
            "mock",
            default_value="true",
            choices=["true", "false"],
            description="Mock hardware (virtual station) or the real ESP32 controller.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="ESP32 serial device (mock:=false only).",
        ),
    ]

    forwarded = {
        key: LaunchConfiguration(key)
        for key in ["robot_type", "name", "prefix", "world_frame", "x", "y", "z", "yaw"]
    }

    # Each include runs in its own scope so arguments like `rviz` (false for the
    # bringup, true for move_group) cannot leak from one into the other.
    bringup = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("scorbot_bringup"), "launch", "robot.launch.py"]
                    )
                ),
                launch_arguments={
                    **forwarded,
                    "mock": LaunchConfiguration("mock"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "rviz": "false",
                }.items(),
            )
        ],
    )

    move_group = GroupAction(
        scoped=True,
        forwarding=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("scorbot_moveit_config"), "launch", "move_group.launch.py"]
                    )
                ),
                launch_arguments={
                    **forwarded,
                    "ik": LaunchConfiguration("ik"),
                    "rviz": "true",
                }.items(),
            )
        ],
    )

    return LaunchDescription(arguments + [bringup, move_group])
