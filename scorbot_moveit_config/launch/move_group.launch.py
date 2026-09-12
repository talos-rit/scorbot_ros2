"""Start move_group for one Scorbot arm, next to a running scorbot_bringup.

    ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc rviz:=false
    ros2 launch scorbot_moveit_config move_group.launch.py robot_type:=er_4pc

Use the same robot_type, name and prefix as the bringup so move_group builds the
same robot model and finds joint_trajectory_controller in the same namespace.
`rviz:=true` adds RViz with the MotionPlanning display.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from scorbot_moveit_config.moveit_configs import build_moveit_configs, declare_common_arguments


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("name").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    moveit_config = build_moveit_configs(context)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_config],
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(rviz),
    )
    return [move_group, rviz_node]


def generate_launch_description():
    arguments = declare_common_arguments() + [
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            choices=["true", "false"],
            description="Start RViz with the MotionPlanning display.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("scorbot_moveit_config"), "config", "moveit.rviz"]
            ),
            description="RViz config with the MotionPlanning display.",
        ),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
