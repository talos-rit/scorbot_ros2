"""SRDF expands, refers only to real links and joints, and named poses respect limits."""

import os
import sys
import xml.etree.ElementTree as ET

import pytest
import xacro
import xacro.substitution_args
import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)
DESCRIPTION_DIR = os.path.join(os.path.dirname(PKG_DIR), "scorbot_description")

# Resolve `$(find scorbot_description)` to the source tree; no ament index in plain pytest.
xacro.substitution_args._eval_find = (
    lambda pkg: DESCRIPTION_DIR if pkg == "scorbot_description" else pkg
)
URDF = os.path.join(DESCRIPTION_DIR, "urdf", "scorbot.urdf.xacro")
SRDF = os.path.join(PKG_DIR, "config", "scorbot.srdf.xacro")
CONFIG = os.path.join(PKG_DIR, "config")

from scorbot_moveit_config.config_utils import (  # noqa: E402
    KINEMATICS_FILES,
    prefix_controllers,
    prefix_joint_limits,
    write_prefixed,
)

JOINTS = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"]


def expand(path, **mappings):
    doc = xacro.process_file(path, mappings={k: str(v) for k, v in mappings.items()})
    return ET.fromstring(doc.toxml())


def load_yaml(name):
    with open(os.path.join(CONFIG, name), encoding="utf-8") as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize("prefix", ["", "bluey_"])
@pytest.mark.parametrize("robot_type", ["er_4pc", "er_v"])
def test_srdf_matches_urdf(robot_type, prefix):
    urdf = expand(URDF, robot_type=robot_type, prefix=prefix)
    srdf = expand(SRDF, prefix=prefix)

    assert srdf.get("name") == urdf.get("name") == "scorbot"

    links = {link.get("name") for link in urdf.findall("link")}
    revolute = {
        j.get("name"): j.find("limit") for j in urdf.findall("joint") if j.get("type") == "revolute"
    }

    chain = srdf.find("group[@name='arm']/chain")
    assert chain.get("base_link") == f"{prefix}base_link"
    assert chain.get("tip_link") == f"{prefix}tool0"
    assert chain.get("base_link") in links and chain.get("tip_link") in links

    for state in srdf.findall("group_state"):
        names = [j.get("name") for j in state.findall("joint")]
        assert names == [f"{prefix}{j}" for j in JOINTS], state.get("name")
        for j in state.findall("joint"):
            limit = revolute[j.get("name")]
            value = float(j.get("value"))
            assert float(limit.get("lower")) <= value <= float(limit.get("upper")), (
                state.get("name"), j.get("name"))

    for dc in srdf.findall("disable_collisions"):
        assert dc.get("link1") in links and dc.get("link2") in links
        assert dc.get("link1") != dc.get("link2")

    # A camera has no joints, so there must be no end_effector entry (MoveIt warns).
    assert srdf.find("end_effector") is None


def test_named_poses_exist():
    srdf = expand(SRDF)
    assert {s.get("name") for s in srdf.findall("group_state")} == {"zero", "ready", "folded"}


def test_joint_limits_cover_every_joint():
    limits = load_yaml("joint_limits.yaml")["joint_limits"]
    assert set(limits) == set(JOINTS)
    for name, lim in limits.items():
        assert lim["has_velocity_limits"] and lim["max_velocity"] > 0, name
        assert lim["has_acceleration_limits"] and lim["max_acceleration"] > 0, name


def test_controllers_match_bringup():
    cfg = load_yaml("moveit_controllers.yaml")
    mgr = cfg["moveit_simple_controller_manager"]
    assert mgr["controller_names"] == ["joint_trajectory_controller"]
    jtc = mgr["joint_trajectory_controller"]
    assert jtc["type"] == "FollowJointTrajectory"
    assert jtc["action_ns"] == "follow_joint_trajectory"
    assert jtc["joints"] == JOINTS
    assert cfg["moveit_controller_manager"].endswith("MoveItSimpleControllerManager")


@pytest.mark.parametrize(
    "name", ["kinematics.yaml", "kinematics_pick_ik.yaml", "kinematics_analytic.yaml"]
)
def test_kinematics_files_target_the_arm_group(name):
    cfg = load_yaml(name)
    assert list(cfg) == ["arm"]
    assert "kinematics_solver" in cfg["arm"]


def test_every_ik_choice_has_a_config_file():
    for choice, name in KINEMATICS_FILES.items():
        assert os.path.exists(os.path.join(CONFIG, name)), choice
    assert (
        load_yaml(KINEMATICS_FILES["analytic"])["arm"]["kinematics_solver"]
        == "scorbot_kinematics/ScorbotKinematicsPlugin"
    )


def test_ompl_pipeline_shape():
    cfg = load_yaml("ompl_planning.yaml")
    assert cfg["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert cfg["arm"]["default_planner_config"] in cfg["planner_configs"]
    for planner in cfg["arm"]["planner_configs"]:
        assert planner in cfg["planner_configs"]


def test_prefix_helpers(tmp_path):
    limits = prefix_joint_limits(load_yaml("joint_limits.yaml"), "p_")
    assert set(limits["joint_limits"]) == {f"p_{j}" for j in JOINTS}
    assert limits["default_velocity_scaling_factor"] == 0.5

    ctrl = prefix_controllers(load_yaml("moveit_controllers.yaml"), "p_")
    assert ctrl["moveit_simple_controller_manager"]["joint_trajectory_controller"]["joints"] == [
        f"p_{j}" for j in JOINTS
    ]
    assert ctrl["moveit_simple_controller_manager"]["controller_names"] == [
        "joint_trajectory_controller"
    ]

    src = os.path.join(CONFIG, "joint_limits.yaml")
    assert write_prefixed(src, "", "joint_limits") == src
    out = write_prefixed(src, "p_", "joint_limits", dest_dir=str(tmp_path))
    with open(out, encoding="utf-8") as f:
        assert yaml.safe_load(f) == limits
    with pytest.raises(ValueError):
        write_prefixed(src, "p_", "nope", dest_dir=str(tmp_path))
