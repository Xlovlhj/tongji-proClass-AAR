"""Build a floating-base, effort-controlled USD using the local Isaac 6 API."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from .model_metadata import prepare_urdf


def _fingerprint(repo_root: Path) -> str:
    digest = hashlib.sha256()
    for path in [Path(__file__), Path(__file__).with_name("model_metadata.py"),
                 repo_root / "models/AzureLoong.urdf", repo_root / "models/AzureLoong.xml"]:
        digest.update(path.read_bytes())
    for mesh in sorted((repo_root / "models/meshes").glob("*")):
        stat = mesh.stat()
        digest.update(f"{mesh.name}:{stat.st_size}:{stat.st_mtime_ns}".encode())
    return digest.hexdigest()


def _configure_asset(usd_path: Path, metadata: dict) -> dict:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None or not stage.GetDefaultPrim():
        raise RuntimeError(f"Imported USD has no default prim: {usd_path}")
    stage.SetEditTarget(stage.GetRootLayer())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    joints, bodies, roots = {}, {}, []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            if prim.GetName() in joints:
                raise RuntimeError(f"Duplicate joint name in USD: {prim.GetName()}")
            joints[prim.GetName()] = prim
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies[prim.GetName()] = prim
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.FixedJoint):
            joint = UsdPhysics.FixedJoint(prim)
            if not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
                raise RuntimeError(f"Floating robot unexpectedly fixed to world: {prim.GetPath()}")
    if set(joints) != set(metadata["joint_names"]):
        raise RuntimeError(f"USD joint names differ from source: {set(joints) ^ set(metadata['joint_names'])}")
    if len(roots) != 1:
        raise RuntimeError(f"Expected one floating articulation root, got {roots}")
    imported_mass = sum(float(UsdPhysics.MassAPI(prim).GetMassAttr().Get() or 0.0)
                        for prim in bodies.values())
    if not math.isclose(imported_mass, metadata["total_link_mass"], rel_tol=1e-5):
        raise RuntimeError(f"Imported link mass {imported_mass} differs from URDF {metadata['total_link_mass']}")
    for name, prim in joints.items():
        values = metadata["joints"][name]
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateMaxForceAttr(values["joint_effort_limit"])
        # The older physxJoint:jointFriction is load dependent, not N*m.
        joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
        joint_api.CreateJointFrictionAttr(0.0)
        joint_api.CreateArmatureAttr(values["armature"])
        if not prim.ApplyAPI("PhysxJointAxisAPI", "angular"):
            raise RuntimeError("Isaac 6 PhysxJointAxisAPI is required for physical damping/friction")
        for key, value in {
            "armature": values["armature"],
            "staticFrictionEffort": values["frictionloss"],
            "dynamicFrictionEffort": values["frictionloss"],
            "viscousFrictionCoefficient": values["damping"] * math.pi / 180.0,
        }.items():
            prim.CreateAttribute(f"physxJointAxis:angular:{key}", Sdf.ValueTypeNames.Float).Set(value)
        # URDF velocity is radians/s, while USD authoring uses degrees/s.
        velocity = values["urdf_velocity_limit"]
        if velocity > 0:
            prim.CreateAttribute("physxJointAxis:angular:maxJointVelocity", Sdf.ValueTypeNames.Float).Set(math.degrees(velocity))

    # Preserve source contact exclusions while allowing non-excluded links to collide.
    for left, right in metadata["collision_excludes"]:
        if left not in bodies or right not in bodies:
            raise RuntimeError(f"Missing excluded collision link: {left}, {right}")
        UsdPhysics.FilteredPairsAPI.Apply(bodies[left]).CreateFilteredPairsRel().AddTarget(bodies[right].GetPath())
    material_base = str(stage.GetDefaultPrim().GetPath()) + "/OpenLoongMaterials"
    UsdGeom.Scope.Define(stage, material_base)
    for link_name, foot in metadata["feet"].items():
        material = UsdShade.Material.Define(stage, material_base + "/" + link_name)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(foot["friction"][0])
        physics_material.CreateDynamicFrictionAttr(foot["friction"][0])
        physics_material.CreateRestitutionAttr(0.0)
        count = 0
        for prim in Usd.PrimRange(bodies[link_name]):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, "physics")
                count += 1
        if not count:
            raise RuntimeError(f"No imported collision geometry for {link_name}")
    stage.GetRootLayer().Save()
    return {"default_prim": str(stage.GetDefaultPrim().GetPath()), "articulation_roots": roots,
            "total_link_mass": imported_mass,
            "joint_paths": {name: str(prim.GetPath()) for name, prim in joints.items()},
            "base_link_path": str(bodies[metadata["base_link"]].GetPath()),
            "foot_link_paths": {name: str(bodies[name].GetPath()) for name in metadata["feet"]}}


def ensure_robot_usd(repo_root: Path, output_dir: Path, force: bool = False) -> Path:
    """Return a validated cached USD, building it if the source changed.

    Call after ``SimulationApp`` starts with ``isaacsim.asset.importer.urdf``
    enabled. ``repo_root`` is the OpenLoong-Dyn-Control checkout. The manifest
    beside ``model_metadata.json`` records actual default/root prim paths.
    """
    repo_root, output_dir = Path(repo_root).resolve(), Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    fingerprint = _fingerprint(repo_root)
    manifest_path = output_dir / "asset_manifest.json"
    if not force and manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text())
        cached = Path(manifest["usd_path"])
        if manifest.get("fingerprint") == fingerprint and cached.is_file():
            return cached
    urdf_path, metadata = prepare_urdf(repo_root, output_dir)
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    config = URDFImporterConfig(
        urdf_path=str(urdf_path), usd_path=str(output_dir / "usd"),
        fix_base=False, merge_fixed_joints=False, merge_mesh=False,
        collision_from_visuals=False, allow_self_collision=True,
        joint_drive_type="force", joint_target_type="none",
        override_joint_stiffness=0.0, override_joint_damping=0.0,
    )
    usd_path = Path(URDFImporter(config).import_urdf()).resolve()
    if not usd_path.is_file():
        raise RuntimeError(f"Isaac importer returned a nonexistent USD: {usd_path}")
    details = _configure_asset(usd_path, metadata)
    manifest_path.write_text(json.dumps({"fingerprint": fingerprint, "usd_path": str(usd_path),
                                        "metadata_path": str(output_dir / "model_metadata.json"),
                                        **details}, indent=2) + "\n")
    return usd_path
