"""View one Scorbot arm in RViz with joint sliders.

    ros2 launch scorbot_description view_robot.launch.py robot_type:=er_4pc
    ros2 launch scorbot_description view_robot.launch.py robot_type:=er_v gui:=false

No controllers are involved; joint_state_publisher_gui drives the joints directly.
The virtual station with ros2_control lives in scorbot_bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_type",
            default_value="er_4pc",
            choices=["er_4pc", "er_v"],
            description="Which Scorbot to show: er_4pc (Bluey) or er_v (Bingo).",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for link and joint names.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start joint_state_publisher_gui (sliders). If false, a plain "
            "joint_state_publisher holds every joint at zero.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("scorbot_description"), "rviz", "view_robot.rviz"]
            ),
            description="RViz config file.",
        ),
    ]

    robot_type = LaunchConfiguration("robot_type")
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("scorbot_description"), "urdf", "scorbot.urdf.xacro"]
            ),
            " robot_type:=",
            robot_type,
            " prefix:=",
            prefix,
            " name:=",
            robot_type,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)
