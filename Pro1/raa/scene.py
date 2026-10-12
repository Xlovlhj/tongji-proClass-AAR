"""Export a final robot pose without changing the running physics stage.

Physics exposes world poses, whereas nested USD links need local transforms.
An independent stage is authored from parent links to child links so every
conversion uses the already-correct parent pose.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence


def export_snapshot(stage, link_paths: Sequence[str],
                    positions: Sequence[Sequence[float]],
                    quaternions: Sequence[Sequence[float]],
                    output_path: str | Path) -> float:
    """Clone the root layer, author cached world poses, verify, and export it.

    Quaternions use ``wxyz`` order. Capture all poses from the physics tensor
    backend before calling this helper. The original stage is only read; no
    transforms or physics state are modified. Returns the largest difference
    between the requested and exported link world positions, in metres.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom

    paths = [str(path) for path in link_paths]
    if len(paths) != len(positions) or len(paths) != len(quaternions):
        raise ValueError("link paths, positions and quaternions must have the same length")
    if len(paths) != len(set(paths)):
        raise ValueError("link paths must be unique")
    cached = []
    for path, position, quaternion in zip(paths, positions, quaternions):
        if not path.startswith("/"):
            raise ValueError(f"link path must be absolute: {path}")
        p = tuple(float(value) for value in position)
        q = tuple(float(value) for value in quaternion)
        if len(p) != 3 or len(q) != 4 or not all(math.isfinite(value) for value in (*p, *q)):
            raise ValueError(f"invalid world pose for {path}")
        norm = math.sqrt(sum(value * value for value in q))
        if norm <= 1e-12:
            raise ValueError(f"zero quaternion for {path}")
        q = tuple(value / norm for value in q)
        world = Gf.Matrix4d().SetRotate(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
        world.SetTranslateOnly(Gf.Vec3d(*p))
        cached.append((path, p, world))

    layer = Sdf.Layer.CreateAnonymous("raa_snapshot.usda")
    if not layer.ImportFromString(stage.GetRootLayer().ExportToString()):
        raise RuntimeError("failed to clone the scene root layer")
    snapshot = Usd.Stage.Open(layer, stage.GetPathResolverContext())
    if snapshot is None:
        raise RuntimeError("failed to open the cloned scene")
    # Do not reuse an XformCache while authoring: parent transforms change.
    for path, _, world in sorted(cached, key=lambda value: value[0].count("/")):
        prim = snapshot.GetPrimAtPath(path)
        xform = UsdGeom.Xformable(prim)
        if not prim or not xform:
            raise ValueError(f"link is absent or not transformable in the cloned scene: {path}")
        parent = prim.GetParent()
        parent_xform = UsdGeom.Xformable(parent)
        parent_world = (parent_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        if parent_xform else Gf.Matrix4d(1.0))
        # Gf matrices use row vectors: world = local * parent_world.
        local = world * parent_world.GetInverse()
        xform.ClearXformOpOrder()
        xform.SetResetXformStack(False)
        xform.MakeMatrixXform().Set(local)

    max_error = 0.0
    for path, expected_position, _ in cached:
        actual = UsdGeom.Xformable(snapshot.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        ).ExtractTranslation()
        error = math.sqrt(sum((actual[i] - expected_position[i]) ** 2 for i in range(3)))
        max_error = max(max_error, error)
    if max_error >= 1e-6:
        raise RuntimeError(f"snapshot world-pose verification failed: {max_error:.9g} m")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not layer.Export(str(output)):
        raise RuntimeError(f"failed to export scene snapshot: {output}")
    return max_error
