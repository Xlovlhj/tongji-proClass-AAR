"""Display the force actually sent to PhysX, without adding physical objects.

Isaac Sim 6's RigidPrim.apply_forces applies forces at the link transform.
The arrow tip follows that world-space link origin; its tail lies opposite
the force direction, so the glyph depicts a push into the robot.
"""
from __future__ import annotations

import math


FORCE_ARROW_SCALE_M_PER_N = 0.012


def force_arrow_geometry(application_point, force_world):
    """Return a rendering-independent arrow plan in SI units.

    Total arrow length is linear in force magnitude: 50 N -> 0.6 m and
    100 N -> 1.2 m. A zero vector has no direction and must be hidden.
    """
    point = tuple(float(value) for value in application_point)
    force = tuple(float(value) for value in force_world)
    if len(point) != 3 or len(force) != 3:
        raise ValueError("Force visualization requires three-dimensional vectors")
    if not all(math.isfinite(value) for value in (*point, *force)):
        raise ValueError("Force visualization requires finite values")
    magnitude = math.sqrt(sum(value * value for value in force))
    length = magnitude * FORCE_ARROW_SCALE_M_PER_N
    direction = tuple(value / magnitude for value in force) if magnitude else (0.0, 0.0, 1.0)
    head_length = min(0.18, 0.3 * length)
    return {
        "visible": magnitude > 0.0,
        "force_magnitude_n": magnitude,
        "length_m": length,
        "direction_world": direction,
        "application_point_world_m": point,
        "tail_world_m": tuple(point[i] - length * direction[i] for i in range(3)),
        "shaft_length_m": length - head_length,
        "head_length_m": head_length,
    }


class AppliedForceArrow:
    """A cone/cylinder visual whose only inputs are point and applied force."""

    def __init__(self, stage):
        from pxr import Gf, UsdGeom, Vt

        self._Gf = Gf
        self._UsdGeom = UsdGeom
        self._Vt = Vt
        self.prim_path = "/World/AppliedForceVisualization"
        self._root = UsdGeom.Xform.Define(stage, self.prim_path)
        self._translate = self._root.AddTranslateOp()
        self._orient = self._root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        self._shaft = UsdGeom.Cylinder.Define(stage, self.prim_path + "/Shaft")
        self._head = UsdGeom.Cone.Define(stage, self.prim_path + "/Head")
        for shape, radius, color in (
            (self._shaft, 0.035, (1.0, 0.28, 0.015)),
            (self._head, 0.095, (1.0, 0.045, 0.01)),
        ):
            shape.CreateAxisAttr(UsdGeom.Tokens.z)
            shape.CreateRadiusAttr(radius)
            shape.CreateHeightAttr(0.01)
            shape.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
            shape.CreateDisplayOpacityAttr(Vt.FloatArray([1.0]))
        self._shaft_translate = self._shaft.AddTranslateOp()
        self._head_translate = self._head.AddTranslateOp()
        self._visibility = self._root.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        self.visible = False
        # Deliberately no CollisionAPI, RigidBodyAPI, MassAPI, or physics calls.

    def hide(self):
        if self.visible:
            self._visibility.Set(self._UsdGeom.Tokens.invisible)
            self.visible = False

    def update(self, application_point, force_world):
        plan = force_arrow_geometry(application_point, force_world)
        if not plan["visible"]:
            self.hide()
            return plan
        Gf, Vt = self._Gf, self._Vt
        self._translate.Set(Gf.Vec3d(*plan["application_point_world_m"]))
        rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), Gf.Vec3d(*plan["direction_world"]))
        self._orient.Set(rotation.GetQuat())
        self._shaft.GetHeightAttr().Set(plan["shaft_length_m"])
        self._head.GetHeightAttr().Set(plan["head_length_m"])
        self._shaft_translate.Set(Gf.Vec3d(0.0, 0.0, -plan["length_m"] + plan["shaft_length_m"] / 2))
        self._head_translate.Set(Gf.Vec3d(0.0, 0.0, -plan["head_length_m"] / 2))
        for shape, length in ((self._shaft, plan["shaft_length_m"]),
                              (self._head, plan["head_length_m"])):
            radius = shape.GetRadiusAttr().Get()
            shape.CreateExtentAttr(Vt.Vec3fArray([
                Gf.Vec3f(-radius, -radius, -length / 2),
                Gf.Vec3f(radius, radius, length / 2),
            ]))
        if not self.visible:
            self._visibility.Set(self._UsdGeom.Tokens.inherited)
            self.visible = True
        return plan

    def manifest(self):
        return {
            "enabled": True,
            "prim_path": self.prim_path,
            "geometry": "cylinder shaft and cone head",
            "color": "orange shaft and red-orange head",
            "force_source": "same force_world vector as base.apply_forces and states.csv force_{x,y,z}_n",
            "force_frame": "world",
            "arrow_direction": "parallel to applied force, tail to tip",
            "application_point": "base rigid link transform origin in world coordinates",
            "arrow_tip": "application point",
            "application_point_api_basis": "Isaac Sim 6 RigidPrim.apply_forces and physics.tensors.RigidBodyView.apply_forces: at the link transforms",
            "length_scale_m_per_n": FORCE_ARROW_SCALE_M_PER_N,
            "visibility": "only when actual applied force is nonzero; hidden while timeline paused and after experiment",
            "collision_enabled": False,
            "rigid_body_enabled": False,
            "mass_added": False,
            "physics_modified": False,
        }
