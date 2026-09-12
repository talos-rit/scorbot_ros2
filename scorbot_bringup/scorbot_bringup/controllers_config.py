"""Rewrite the shared controllers YAML for one robot instance.

`config/controllers.yaml` is written once with plain joint names and plain node names. 
Each launched robot may live in its own namespace and use a link/joint prefix, so the file is 
rewritten at launch time.
"""

from __future__ import annotations

import os
import tempfile
from typing import Any

import yaml

JOINT_LIST_KEYS = ("joints",)
PREFIXED_NAME_KEYS = ("gpio_name",)

def qualify(node: str, namespace: str) -> str:
    """Return the fully qualified node name for `node` inside `namespace`."""
    ns = "/".join(part for part in namespace.split("/") if part)
    node = node.lstrip("/")
    return f"/{ns}/{node}" if ns else f"/{node}"

def _prefix_joints(params: Any, prefix: str) -> Any:
    if isinstance(params, dict):
        out = {}
        for key, value in params.items():
            if key in JOINT_LIST_KEYS and isinstance(value, list):
                out[key] = [f"{prefix}{joint}" for joint in value]
            elif key in PREFIXED_NAME_KEYS and isinstance(value, str):
                out[key] = f"{prefix}{value}"
            else:
                out[key] = _prefix_joints(value, prefix)
        return out
    if isinstance(params, list):
        return [_prefix_joints(v, prefix) for v in params]
    return params

def rewrite_controllers(config: dict, prefix: str = "", namespace: str = "") -> dict:
    """Return a new config with qualified node names and prefixed joint names."""
    result = {}
    for node, body in config.items():
        result[qualify(node, namespace)] = _prefix_joints(body, prefix)
    return result

def write_prefixed_controllers(
        source: str, prefix: str = "", namespace: str = "", dest_dir: str | None = None
) -> str:
    """Rewrite `source` and return the path of the generated YAML file."""
    with open(source, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    rewritten = rewrite_controllers(config, prefix=prefix, namespace=namespace)

    dest_dir = dest_dir or tempfile.mkdtemp(prefix="scorbot_bringup_")
    tag = "_".join(p for p in (namespace.strip("/"), prefix.strip("_")) if p) or "root"
    path = os.path.join(dest_dir, f"controllers_{tag}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(rewritten, f, sort_keys=False)
    return path