"""Bring up one Scorbot arm under ros2_control.

Virtual station (mock hardware, RViz):
    ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc
    ros2 launch scorbot_bringup robot.launch.py robot_type:=er_v

Real robot (ESP32 over serial through scorbot_hardware), or the ESP simulator:
    ros2 launch scorbot_bringup robot.launch.py robot_type:=er_4pc mock:=false \
        serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102-... rviz:=false
    ros2 run scorbot_esp_sim scorbot_esp_sim --link /tmp/scorbot   # then
    ros2 launch scorbot_bringup robot.launch.py mock:=false serial_port:=/tmp/scorbot

What starts, all in the `name` namespace (root by default):
    robot_state_publisher      publishes TF and the robot_description topic
    ros2_control_node          controller_manager; reads robot_description from the topic
    joint_state_broadcaster    active
    joint_trajectory_controller active   (point-to-point moves, MoveIt, demo_motion)
    forward_velocity_controller inactive (tracking loop; switch to it at runtime)
    system_controller          active, mock:=false only (home/enable/disable/clear_fault
                               services and ~/status; see scorbot_system_controller)
    rviz2                      optional

With mock:=false the joint controllers start but the ESP32 ignores their commands until
it is homed and enabled: stop them, call system_controller/home and /enable, start them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from scorbot_bringup.controllers_config import write_prefixed_controllers

ARGUMENTS = [
    DeclareLaunchArgument(
        "robot_type",
        default_value="er_4pc",
        choices=["er_4pc", "er_v"],
        description="Which Scorbot: er_4pc (Bluey) or er_v (Bingo).",
    ),
    DeclareLaunchArgument(
        "name",
        default_value="",
        description="Namespace for every node and topic. Empty (default) means the root "
        "namespace. Set it when two arms must share one network.",
    ),
    DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="Prefix for link and joint names, e.g. bluey_. Empty by default.",
    ),
    DeclareLaunchArgument(
        "mock",
        default_value="true",
        choices=["true", "false"],
        description="true: mock_components virtual station. false: scorbot_hardware over serial.",
    ),
    DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="ESP32 serial device (mock:=false only).",
    ),
    DeclareLaunchArgument(
        "calibration_file",
        default_value="",
        description="Calibration YAML pushed to the ESP32 (mock:=false only). "
        "Defaults to scorbot_description/config/<robot_type>_calibration.yaml.",
    ),
    DeclareLaunchArgument(
        "auto_home",
        default_value="false",
        choices=["true", "false"],
        description="mock:=false only: home the controller during hardware activation "
        "if it is not homed yet (blocks bringup for the homing time).",
    ),
    DeclareLaunchArgument(
        "allow_unhomed",
        default_value="false",
        choices=["true", "false"],
        description="mock:=false only, bench use: enable the drives without homing "
        "(no soft limits, positions relative to power-up).",
    ),
    DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("scorbot_bringup"), "config", "controllers.yaml"]
        ),
        description="ros2_control controllers YAML (rewritten per name/prefix at launch).",
    ),
    DeclareLaunchArgument(
        "world_frame",
        default_value="world",
        description="Fixed frame the base is attached to. Empty makes base_link the root.",
    ),
    DeclareLaunchArgument("x", default_value="0.0", description="Base position in world_frame."),
    DeclareLaunchArgument("y", default_value="0.0", description="Base position in world_frame."),
    DeclareLaunchArgument("z", default_value="0.0", description="Base position in world_frame."),
    DeclareLaunchArgument("yaw", default_value="0.0", description="Base yaw in world_frame."),
    DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Start RViz.",
    ),
    DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("scorbot_bringup"), "rviz", "robot.rviz"]
        ),
        description="RViz config file.",
    ),
    DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="30",
        description="Seconds the spawners wait for the controller manager.",
    ),
]


def launch_setup(context, *args, **kwargs):
    cfg = {name: LaunchConfiguration(name).perform(context) for name in [
        "robot_type", "name", "prefix", "mock", "serial_port", "calibration_file",
        "auto_home", "allow_unhomed",
        "controllers_file", "world_frame", "x", "y", "z", "yaw", "rviz", "rviz_config",
        "controller_manager_timeout",
    ]}
    namespace = cfg["name"]
    prefix = cfg["prefix"]

    # Controller config: plain file rewritten with the namespace and joint prefix.
    controllers_file = write_prefixed_controllers(
        cfg["controllers_file"], prefix=prefix, namespace=namespace
    )

    xacro_args = [
        " robot_type:=", cfg["robot_type"],
        " name:=", namespace or cfg["robot_type"],
        " prefix:=", prefix,
        " use_ros2_control:=true",
        " mock:=", cfg["mock"],
        " serial_port:=", cfg["serial_port"],
        " auto_home:=", cfg["auto_home"],
        " allow_unhomed:=", cfg["allow_unhomed"],
        " world_frame:=", cfg["world_frame"],
        " x:=", cfg["x"], " y:=", cfg["y"], " z:=", cfg["z"], " yaw:=", cfg["yaw"],
    ]
    if cfg["calibration_file"]:
        xacro_args += [" calibration_file:=", cfg["calibration_file"]]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("scorbot_description"), "urdf", "scorbot.urdf.xacro"]
            ),
            *xacro_args,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    def spawner(controller: str, inactive: bool = False) -> Node:
        args = [
            controller,
            "--controller-manager", "controller_manager",
            "--controller-manager-timeout", cfg["controller_manager_timeout"],
            "--param-file", controllers_file,
        ]
        if inactive:
            args.append("--inactive")
        return Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=args,
            output="screen",
        )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[robot_description],
            output="screen",
        ),
        # controller_manager takes robot_description from the topic published above
        # (same namespace), which is the Jazzy-recommended path.
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=[controllers_file],
            output="screen",
        ),
        spawner("joint_state_broadcaster"),
        spawner("joint_trajectory_controller"),
        spawner("forward_velocity_controller", inactive=True),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            arguments=["-d", cfg["rviz_config"]],
            output="log",
            condition=IfCondition(cfg["rviz"]),
        ),
    ]
    if cfg["mock"] == "false":
        # The system GPIO only exists on the real hardware interface.
        nodes.append(spawner("system_controller"))
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
