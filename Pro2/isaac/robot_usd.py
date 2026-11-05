"""Read-only USD inspection and effort-drive setup for configurable robots.

The inspection functions require the USD Python bindings shipped with Isaac
Sim.  ``Pro2/tools/preflight_l4_robot.py`` starts the correct interpreter, so
users do not need to alter their shell environment.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any


def _usd_modules():
    try:
        from pxr import Usd, UsdGeom, UsdPhysics
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "USD Python bindings are unavailable; run through "
            "python3 Pro2/tools/preflight_l4_robot.py") from exc
    return Usd, UsdGeom, UsdPhysics


def _joint_prims(stage):
    Usd, _, UsdPhysics = _usd_modules()
    return [prim for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.RevoluteJoint) or
            prim.IsA(UsdPhysics.PrismaticJoint)]


def _collision_paths(stage, link_path: str) -> list[str]:
    Usd, _, UsdPhysics = _usd_modules()
    prefix = link_path.rstrip("/") + "/"
    paths: list[str] = []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if (path == link_path or path.startswith(prefix)) and \
                prim.HasAPI(UsdPhysics.CollisionAPI):
            paths.append(path)
    return paths


def inspect_robot_usd(path: Path) -> dict[str, Any]:
    """Return JSON-safe physical and articulation facts from a composed USD."""

    Usd, UsdGeom, UsdPhysics = _usd_modules()
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"robot USD does not exist: {path}")
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"USD stage could not be opened: {path}")

    default_prim = stage.GetDefaultPrim()
    joints = _joint_prims(stage)
    joint_rows: list[dict[str, Any]] = []
    for prim in joints:
        joint = UsdPhysics.Joint(prim)
        drive_axis = ("angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
        drive = UsdPhysics.DriveAPI.Get(prim, drive_axis)
        lower = prim.GetAttribute("physics:lowerLimit").Get()
        upper = prim.GetAttribute("physics:upperLimit").Get()
        joint_rows.append({
            "name": prim.GetName(),
            "path": str(prim.GetPath()),
            "type": prim.GetTypeName(),
            "drive_axis": drive_axis,
            "body0": [str(target) for target in joint.GetBody0Rel().GetTargets()],
            "body1": [str(target) for target in joint.GetBody1Rel().GetTargets()],
            "lower_limit_authored_units": None if lower is None else float(lower),
            "upper_limit_authored_units": None if upper is None else float(upper),
            "drive": bool(drive),
            "drive_type": str(drive.GetTypeAttr().Get()) if drive else None,
            "drive_stiffness": (None if not drive or drive.GetStiffnessAttr().Get() is None
                                else float(drive.GetStiffnessAttr().Get())),
            "drive_damping": (None if not drive or drive.GetDampingAttr().Get() is None
                              else float(drive.GetDampingAttr().Get())),
            "drive_max_force": (None if not drive or drive.GetMaxForceAttr().Get() is None
                                else float(drive.GetMaxForceAttr().Get())),
        })

    rigid_prims = [prim for prim in stage.Traverse()
                   if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    rigid_paths = [str(prim.GetPath()) for prim in rigid_prims]
    masses: dict[str, float] = {}
    for prim in rigid_prims:
        value = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if value is not None:
            masses[str(prim.GetPath())] = float(value)

    articulation_roots = [str(prim.GetPath()) for prim in stage.Traverse()
                          if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    fixed_to_world: list[str] = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        joint = UsdPhysics.Joint(prim)
        body0 = joint.GetBody0Rel().GetTargets()
        body1 = joint.GetBody1Rel().GetTargets()
        if not body0 or not body1:
            fixed_to_world.append(str(prim.GetPath()))

    used_layers: list[dict[str, Any]] = []
    for layer in stage.GetUsedLayers():
        identifier = layer.realPath or layer.identifier
        anonymous = identifier.startswith("anon:")
        used_layers.append({
            "identifier": identifier,
            "anonymous": anonymous,
            "exists": anonymous or Path(identifier).is_file(),
        })

    return {
        "source_usd": str(path),
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "default_prim": str(default_prim.GetPath()) if default_prim else None,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)).upper(),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "articulation_roots": articulation_roots,
        "fixed_to_world_joints": fixed_to_world,
        "floating_base": not fixed_to_world,
        "joint_count": len(joints),
        "joint_names": [row["name"] for row in joint_rows],
        "joints": joint_rows,
        "rigid_body_count": len(rigid_prims),
        "rigid_body_paths": rigid_paths,
        "explicit_mass_count": len(masses),
        "link_masses_kg": masses,
        "total_explicit_mass_kg": sum(masses.values()),
        "used_layers": sorted(used_layers, key=lambda item: item["identifier"]),
    }


def validate_usd_against_spec(observed: dict[str, Any],
                              spec: dict[str, Any]) -> dict[str, Any]:
    """Compare a USD observation with a validated robot specification."""

    expected = spec["expected_usd"]
    checks: list[dict[str, Any]] = []

    def add(name: str, passed: bool, expected_value: Any, actual: Any) -> None:
        checks.append({"name": name, "passed": bool(passed),
                       "expected": expected_value, "actual": actual})

    add("default_prim", observed["default_prim"] == expected["default_prim"],
        expected["default_prim"], observed["default_prim"])
    add("up_axis", observed["up_axis"] == expected["up_axis"],
        expected["up_axis"], observed["up_axis"])
    add("meters_per_unit",
        math.isclose(observed["meters_per_unit"], expected["meters_per_unit"],
                     rel_tol=0.0, abs_tol=1e-12),
        expected["meters_per_unit"], observed["meters_per_unit"])
    add("articulation_root",
        observed["articulation_roots"] == [expected["articulation_root"]],
        [expected["articulation_root"]], observed["articulation_roots"])
    add("floating_base", observed["floating_base"] is expected["floating_base"],
        expected["floating_base"], observed["floating_base"])
    add("joint_count", observed["joint_count"] == spec["joints"]["count"],
        spec["joints"]["count"], observed["joint_count"])
    add("joint_names",
        set(observed["joint_names"]) == set(spec["joints"]["names"]),
        sorted(spec["joints"]["names"]), sorted(observed["joint_names"]))
    add("all_joints_have_drives",
        all(row["drive"] for row in observed["joints"]), True,
        sum(1 for row in observed["joints"] if row["drive"]))
    add("all_joint_body_relations_resolve",
        all(len(row["body0"]) == 1 and len(row["body1"]) == 1
            for row in observed["joints"]), True,
        sum(1 for row in observed["joints"]
            if len(row["body0"]) == 1 and len(row["body1"]) == 1))
    add("rigid_body_count",
        observed["rigid_body_count"] == expected["rigid_body_count"],
        expected["rigid_body_count"], observed["rigid_body_count"])
    add("base_link_is_rigid",
        expected["base_link"] in observed["rigid_body_paths"],
        expected["base_link"], expected["base_link"] in observed["rigid_body_paths"])
    for side in ("left", "right"):
        path = expected[f"{side}_foot_link"]
        add(f"{side}_foot_is_rigid", path in observed["rigid_body_paths"],
            path, path in observed["rigid_body_paths"])

    tolerance = expected.get("mass_relative_tolerance", 1e-6)
    mass = observed["total_explicit_mass_kg"]
    add("all_rigid_bodies_have_explicit_mass",
        observed["explicit_mass_count"] == observed["rigid_body_count"],
        observed["rigid_body_count"], observed["explicit_mass_count"])
    add("total_mass_kg", math.isclose(mass, expected["total_mass_kg"],
                                      rel_tol=tolerance, abs_tol=1e-9),
        expected["total_mass_kg"], mass)
    missing_layers = [row["identifier"] for row in observed["used_layers"]
                      if not row["exists"]]
    add("all_composition_layers_exist", not missing_layers, [], missing_layers)

    # Collision prims may live below instanceable foot subtrees, so inspect the
    # composed stage with instance proxies rather than ordinary Traverse().
    Usd, _, _ = _usd_modules()
    stage = Usd.Stage.Open(observed["source_usd"])
    for side in ("left", "right"):
        link = expected[f"{side}_foot_link"]
        collision_paths = _collision_paths(stage, link)
        observed[f"{side}_foot_collision_paths"] = collision_paths
        add(f"{side}_foot_has_collision", bool(collision_paths), True,
            collision_paths)

    return {"passed": all(check["passed"] for check in checks), "checks": checks}


def configure_effort_drives(stage, robot_prim_path: str,
                            joint_names: list[str]) -> dict[str, float | None]:
    """Disable built-in position gains on a referenced robot stage.

    The source USD remains untouched: callers must set a writable edit target on
    their composed simulation stage.  Existing maximum effort values are kept.
    """

    Usd, _, UsdPhysics = _usd_modules()
    robot = stage.GetPrimAtPath(robot_prim_path)
    if not robot:
        raise ValueError(f"robot prim does not exist: {robot_prim_path}")
    expected = set(joint_names)
    found: dict[str, Any] = {}
    for prim in Usd.PrimRange(robot):
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            if prim.GetName() in found:
                raise ValueError(f"duplicate joint name under {robot_prim_path}: {prim.GetName()}")
            found[prim.GetName()] = prim
    if set(found) != expected:
        raise ValueError(f"USD joints differ from robot specification: {set(found) ^ expected}")

    limits: dict[str, float | None] = {}
    for name in joint_names:
        prim = found[name]
        drive_axis = ("angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
        drive = UsdPhysics.DriveAPI.Apply(prim, drive_axis)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        maximum = drive.GetMaxForceAttr().Get()
        limits[name] = None if maximum is None else float(maximum)
    return limits
