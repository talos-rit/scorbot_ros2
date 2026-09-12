"""Every robot type must expand to a well-formed URDF with the expected topology."""

import math
import os
import xml.etree.ElementTree as ET

import pytest
import xacro
import xacro.substitution_args

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOP = os.path.join(PKG_DIR, "urdf", "scorbot.urdf.xacro")

# `$(find scorbot_description)` needs the ament index, which only exists in an installed
# workspace. Resolve it to the source tree so the tests run with plain `pip install xacro`.
xacro.substitution_args._eval_find = lambda pkg: PKG_DIR if pkg == "scorbot_description" else pkg

JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
]
LINKS = [
    "base_link",
    "turret_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "flange_link",
    "tool0",
]


def expand(**mappings):
    doc = xacro.process_file(TOP, mappings={k: str(v) for k, v in mappings.items()})
    return ET.fromstring(doc.toxml())


@pytest.mark.parametrize("robot_type", ["er_4pc", "er_v"])
def test_topology(robot_type):
    root = expand(robot_type=robot_type)
    links = {link.get("name") for link in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}

    assert set(LINKS) <= links
    assert "world" in links
    assert set(JOINTS) <= joints.keys()

    for j in root.findall("joint"):
        assert j.find("parent").get("link") in links, j.get("name")
        assert j.find("child").get("link") in links, j.get("name")

    # Every child link has exactly one parent joint; the tree has one root.
    children = [j.find("child").get("link") for j in root.findall("joint")]
    assert len(children) == len(set(children))
    roots = links - set(children)
    assert roots == {"world"}


@pytest.mark.parametrize("robot_type", ["er_4pc", "er_v"])
def test_limits_match_manual(robot_type):
    root = expand(robot_type=robot_type)
    lim = {
        j.get("name"): j.find("limit") for j in root.findall("joint") if j.get("type") == "revolute"
    }
    deg = math.degrees
    assert deg(float(lim["base_joint"].get("upper")) - float(lim["base_joint"].get("lower"))) == pytest.approx(310, abs=0.01)
    assert deg(float(lim["shoulder_joint"].get("upper"))) == pytest.approx(130, abs=0.01)
    assert deg(float(lim["shoulder_joint"].get("lower"))) == pytest.approx(-35, abs=0.01)
    assert deg(float(lim["elbow_joint"].get("upper"))) == pytest.approx(130, abs=0.01)
    assert deg(float(lim["wrist_pitch_joint"].get("upper"))) == pytest.approx(130, abs=0.01)
    assert deg(float(lim["wrist_roll_joint"].get("upper"))) == pytest.approx(570, abs=0.01)
    for name, limit in lim.items():
        assert float(limit.get("lower")) < float(limit.get("upper")), name
        assert float(limit.get("velocity")) > 0, name
        assert float(limit.get("effort")) > 0, name


def test_prefix_and_placement():
    root = expand(robot_type="er_v", prefix="bingo_", x=1.0, yaw=0.5)
    names = {link.get("name") for link in root.findall("link")}
    assert "bingo_base_link" in names and "bingo_tool0" in names
    fixed = next(j for j in root.findall("joint") if j.get("name") == "bingo_base_joint_fixed")
    assert fixed.find("parent").get("link") == "world"
    assert fixed.find("origin").get("xyz").split()[0] == "1.0"


def test_no_world_frame_makes_base_link_root():
    root = expand(robot_type="er_4pc", world_frame="")
    links = {link.get("name") for link in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    assert "world" not in links
    assert links - children == {"base_link"}


@pytest.mark.parametrize("mock", [True, False])
def test_ros2_control_block(mock):
    root = expand(robot_type="er_4pc", name="bluey", use_ros2_control=True, mock=str(mock).lower())
    r2c = root.find("ros2_control")
    assert r2c is not None and r2c.get("name") == "bluey"
    plugin = r2c.find("hardware/plugin").text
    joint_names = [j.get("name") for j in r2c.findall("joint")]
    assert joint_names == JOINTS
    if mock:
        assert plugin == "mock_components/GenericSystem"
        assert r2c.find("gpio") is None
    else:
        assert plugin == "scorbot_hardware/ScorbotSystem"
        gpio = r2c.find("gpio")
        assert gpio.get("name") == "system"
        # Must match scorbot_hardware/include/scorbot_hardware/session.hpp.
        assert [c.get("name") for c in gpio.findall("command_interface")] == [
            "home", "enable", "disable", "clear_fault"]
        assert [s.get("name") for s in gpio.findall("state_interface")] == [
            "state", "fault_code", "homed_mask", "limit_mask", "link_age_ms", "firmware_version",
            "boot_count", "last_result", "commands_ignored", "request_count"]
        params = {p.get("name"): p.text for p in r2c.findall("hardware/param")}
        assert params["robot_type"] == "er_4pc"
        assert params["calibration_file"].endswith("er_4pc_calibration.yaml")
        assert params["auto_home_on_activate"] == "false"
        assert params["allow_unhomed"] == "false"
