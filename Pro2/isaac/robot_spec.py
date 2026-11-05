"""Robot-model specifications shared by Isaac asset checks and launch code.

This module deliberately uses only the Python standard library.  It can be
imported by ordinary ``python3`` for configuration validation, before Isaac's
Python environment or the USD bindings are available.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
from typing import Any


_ROBOT_ID = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class ResolvedRobotAsset:
    """The local asset selected for one validated robot specification."""

    path: Path
    source: str


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _finite_positive(value: Any, name: str) -> float:
    _require(not isinstance(value, bool) and isinstance(value, (int, float)),
             f"{name} must be numeric")
    value = float(value)
    _require(math.isfinite(value) and value > 0, f"{name} must be finite and positive")
    return value


def validate_robot_spec(spec: dict[str, Any]) -> dict[str, Any]:
    """Validate and return ``spec`` without changing paths or model values."""

    _require(isinstance(spec, dict), "robot specification must be a JSON object")
    _require(spec.get("schema_version") == 1, "unsupported robot-spec schema_version")
    robot_id = spec.get("robot_id")
    _require(isinstance(robot_id, str) and _ROBOT_ID.fullmatch(robot_id) is not None,
             "robot_id must use lowercase letters, digits and underscores")
    _require(isinstance(spec.get("display_name"), str) and spec["display_name"].strip(),
             "display_name must be a nonempty string")

    asset = spec.get("asset")
    _require(isinstance(asset, dict), "asset must be an object")
    _require(asset.get("format") == "usd", "the L4 model-swap path currently requires USD")
    env_var = asset.get("environment_variable")
    _require(env_var is None or (isinstance(env_var, str) and env_var.isidentifier()),
             "asset.environment_variable must be a valid variable name or null")
    candidates = asset.get("path_candidates")
    _require(isinstance(candidates, list) and candidates and
             all(isinstance(path, str) and path.strip() for path in candidates),
             "asset.path_candidates must contain at least one nonempty path")

    expected = spec.get("expected_usd")
    _require(isinstance(expected, dict), "expected_usd must be an object")
    for name in ("default_prim", "articulation_root", "base_link",
                 "left_foot_link", "right_foot_link"):
        value = expected.get(name)
        _require(isinstance(value, str) and value.startswith("/"),
                 f"expected_usd.{name} must be an absolute USD prim path")
    _require(expected.get("up_axis") in ("X", "Y", "Z"),
             "expected_usd.up_axis must be X, Y or Z")
    _finite_positive(expected.get("meters_per_unit"), "expected_usd.meters_per_unit")
    _finite_positive(expected.get("total_mass_kg"), "expected_usd.total_mass_kg")
    _require(expected.get("floating_base") is True,
             "the current humanoid workflow requires a floating-base asset")

    joints = spec.get("joints")
    _require(isinstance(joints, dict), "joints must be an object")
    names = joints.get("names")
    _require(isinstance(names, list) and names and
             all(isinstance(name, str) and name for name in names),
             "joints.names must be a nonempty string list")
    _require(len(names) == len(set(names)), "joints.names contains duplicates")
    _require(joints.get("count") == len(names), "joints.count differs from joints.names")
    isaac_order = joints.get("isaac_dof_order")
    _require(isinstance(isaac_order, list) and len(isaac_order) == len(names) and
             set(isaac_order) == set(names),
             "joints.isaac_dof_order must be a permutation of joints.names")
    roles = joints.get("semantic_roles")
    _require(isinstance(roles, dict), "joints.semantic_roles must be an object")
    _require(all(isinstance(name, str) and name in names for name in roles.values()),
             "every semantic joint role must name an actuated joint")
    for role in ("left_hip_yaw", "left_knee", "left_ankle_roll",
                 "right_hip_yaw", "right_knee", "right_ankle_roll"):
        _require(role in roles, f"missing required semantic joint role: {role}")

    stand = spec.get("stand_control")
    _require(isinstance(stand, dict), "stand_control must be an object")
    transition = stand.get("source_transition_interval_s")
    _require(isinstance(transition, list) and len(transition) == 2 and
             all(not isinstance(value, bool) and isinstance(value, (int, float)) and
                 math.isfinite(float(value)) for value in transition),
             "stand_control.source_transition_interval_s must contain two finite numbers")
    _require(float(transition[0]) == 0.0 and float(transition[1]) > 0.0,
             "stand_control source transition must start at zero and end after zero")
    for field in ("position_targets_rad", "kp_nm_per_rad", "kd_nm_s_per_rad"):
        values = stand.get(field)
        _require(isinstance(values, list) and len(values) == len(names),
                 f"stand_control.{field} must match joints.names")
        _require(all(not isinstance(value, bool) and isinstance(value, (int, float)) and
                     math.isfinite(float(value)) for value in values),
                 f"stand_control.{field} must contain finite numbers")
        if field != "position_targets_rad":
            _require(all(float(value) >= 0 for value in values),
                     f"stand_control.{field} must be nonnegative")

    controller = spec.get("controller_target")
    _require(isinstance(controller, dict), "controller_target must be an object")
    _require(controller.get("name") == "openloong_mpc_wbc",
             "controller_target.name must be openloong_mpc_wbc")
    return spec


def load_robot_spec(path: Path) -> dict[str, Any]:
    """Read and validate a UTF-8 robot specification."""

    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"robot specification does not exist: {path}")
    try:
        spec = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid robot specification JSON: {path}: {exc}") from exc
    return validate_robot_spec(spec)


def _candidate_path(text: str, project_root: Path) -> Path:
    expanded = os.path.expandvars(os.path.expanduser(text))
    if "$" in expanded:
        raise ValueError(f"unresolved environment variable in asset path: {text}")
    path = Path(expanded)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def resolve_robot_asset(spec: dict[str, Any], project_root: Path,
                        override: Path | None = None) -> ResolvedRobotAsset:
    """Resolve a local USD without downloading or modifying the source asset."""

    validate_robot_spec(spec)
    project_root = Path(project_root).resolve()
    attempted: list[str] = []
    if override is not None:
        path = Path(override).expanduser().resolve()
        if path.is_file():
            return ResolvedRobotAsset(path, "command_line_override")
        raise FileNotFoundError(f"robot USD override does not exist: {path}")

    asset = spec["asset"]
    env_var = asset.get("environment_variable")
    if env_var and os.environ.get(env_var):
        path = _candidate_path(os.environ[env_var], project_root)
        attempted.append(f"{env_var}={path}")
        if path.is_file():
            return ResolvedRobotAsset(path, f"environment:{env_var}")

    for index, candidate in enumerate(asset["path_candidates"]):
        path = _candidate_path(candidate, project_root)
        attempted.append(str(path))
        if path.is_file():
            return ResolvedRobotAsset(path, f"path_candidates[{index}]")
    raise FileNotFoundError(
        "no robot USD candidate exists; checked: " + ", ".join(attempted))


def robot_spec_path(project_root: Path, robot_id: str) -> Path:
    """Return the conventional specification path after validating its id."""

    if _ROBOT_ID.fullmatch(robot_id) is None:
        raise ValueError("invalid robot id")
    return Path(project_root).resolve() / "Pro2" / "robots" / f"{robot_id}.json"
