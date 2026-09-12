"""controllers.yaml and its per-robot rewriting."""

import os
import sys

import pytest
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)

from scorbot_bringup.controllers_config import (  # noqa: E402
    qualify,
    rewrite_controllers,
    write_prefixed_controllers,
)

CONTROLLERS = os.path.join(PKG_DIR, "config", "controllers.yaml")
JOINTS = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]
CONTROLLER_NODES = [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "forward_velocity_controller",
    "system_controller",
]


@pytest.fixture
def config():
    with open(CONTROLLERS, encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_controllers_yaml_lists_every_joint(config):
    cm = config["controller_manager"]["ros__parameters"]
    assert cm["update_rate"] == 100
    for node in CONTROLLER_NODES:
        assert node in cm, f"{node} not declared in controller_manager"
        assert config[node]["ros__parameters"]["joints"] == JOINTS


def test_trajectory_controller_uses_position_commands(config):
    jtc = config["joint_trajectory_controller"]["ros__parameters"]
    assert jtc["command_interfaces"] == ["position"]
    assert "position" in jtc["state_interfaces"]


@pytest.mark.parametrize(
    "node,namespace,expected",
    [
        ("controller_manager", "", "/controller_manager"),
        ("controller_manager", "bluey", "/bluey/controller_manager"),
        ("controller_manager", "/bluey/", "/bluey/controller_manager"),
        ("/joint_state_broadcaster", "a/b", "/a/b/joint_state_broadcaster"),
    ],
)
def test_qualify(node, namespace, expected):
    assert qualify(node, namespace) == expected


def test_rewrite_prefixes_joints_and_qualifies_nodes(config):
    out = rewrite_controllers(config, prefix="bluey_", namespace="bluey")
    assert set(out) == {"/bluey/controller_manager"} | {f"/bluey/{n}" for n in CONTROLLER_NODES}
    for node in CONTROLLER_NODES:
        assert out[f"/bluey/{node}"]["ros__parameters"]["joints"] == [f"bluey_{j}" for j in JOINTS]
    # The system controller's GPIO is prefixed like the joints (<prefix>system in the xacro).
    assert out["/bluey/system_controller"]["ros__parameters"]["gpio_name"] == "bluey_system"
    # Nothing else changes.
    assert out["/bluey/controller_manager"] == config["controller_manager"]
    assert (
        out["/bluey/joint_trajectory_controller"]["ros__parameters"]["command_interfaces"]
        == ["position"]
    )


def test_rewrite_without_prefix_or_namespace_is_identity_apart_from_keys(config):
    out = rewrite_controllers(config)
    assert set(out) == {f"/{k}" for k in config}
    for key, body in config.items():
        assert out[f"/{key}"] == body


def test_write_prefixed_controllers_round_trips(config, tmp_path):
    path = write_prefixed_controllers(CONTROLLERS, prefix="p_", namespace="ns", dest_dir=str(tmp_path))
    assert os.path.basename(path) == "controllers_ns_p.yaml"
    with open(path, encoding="utf-8") as f:
        written = yaml.safe_load(f)
    assert written == rewrite_controllers(config, prefix="p_", namespace="ns")

    root = write_prefixed_controllers(CONTROLLERS, dest_dir=str(tmp_path))
    assert os.path.basename(root) == "controllers_root.yaml"
