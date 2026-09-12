"""Prefix-aware rewriting of MoveIt YAML files.

MoveIt's `joint_limits.yaml` keys joints by name and `moveit_controllers.yaml` lists
joint names, so both must be rewritten when the robot is launched with a link/joint
prefix. The SRDF handles the prefix itself through xacro.
"""

from __future__ import annotations

import os
import tempfile
from typing import Any

import yaml

# `ik` launch argument -> kinematics YAML in config/. Kept here (no ROS imports) so
# tests can check it without a ROS installation.
KINEMATICS_FILES = {
    "analytic": "kinematics_analytic.yaml",
    "kdl": "kinematics.yaml",
    "pick_ik": "kinematics_pick_ik.yaml",
}


def prefix_joint_limits(config: dict, prefix: str) -> dict:
    """Return `joint_limits.yaml` content with every joint key prefixed."""
    limits = config.get("joint_limits", {})
    return {**config, "joint_limits": {f"{prefix}{k}": v for k, v in limits.items()}}


def _prefix_joints_lists(value: Any, prefix: str) -> Any:
    if isinstance(value, dict):
        return {
            k: [f"{prefix}{j}" for j in v] if k == "joints" and isinstance(v, list)
            else _prefix_joints_lists(v, prefix)
            for k, v in value.items()
        }
    if isinstance(value, list):
        return [_prefix_joints_lists(v, prefix) for v in value]
    return value


def prefix_controllers(config: dict, prefix: str) -> dict:
    """Return `moveit_controllers.yaml` content with every `joints` list prefixed."""
    return _prefix_joints_lists(config, prefix)


def write_prefixed(source: str, prefix: str, kind: str, dest_dir: str | None = None) -> str:
    """Rewrite `source` for `prefix` and return the generated file path.

    kind: "joint_limits" | "controllers". With an empty prefix the source path is
    returned unchanged so nothing is generated needlessly.
    """
    if not prefix:
        return source
    with open(source, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    if kind == "joint_limits":
        rewritten = prefix_joint_limits(config, prefix)
    elif kind == "controllers":
        rewritten = prefix_controllers(config, prefix)
    else:
        raise ValueError(f"unknown kind {kind!r}")
    dest_dir = dest_dir or tempfile.mkdtemp(prefix="scorbot_moveit_")
    path = os.path.join(dest_dir, f"{kind}_{prefix.strip('_')}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(rewritten, f, sort_keys=False)
    return path
