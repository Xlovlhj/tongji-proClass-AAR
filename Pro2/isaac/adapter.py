"""Explicit conventions at the PhysX/controller boundary (SI, world frame, wxyz)."""
from __future__ import annotations
import math


def joint_indices(controller_names, simulator_names):
    if len(set(simulator_names)) != len(simulator_names):
        raise ValueError("仿真关节名重复")
    if len(set(controller_names)) != len(controller_names):
        raise ValueError("控制器关节名重复")
    missing = set(controller_names) - set(simulator_names)
    extra = set(simulator_names) - set(controller_names)
    if missing or extra:
        raise ValueError(f"关节不匹配：missing={sorted(missing)}, extra={sorted(extra)}")
    lookup = {name: i for i, name in enumerate(simulator_names)}
    return [lookup[name] for name in controller_names]


def rotation_wxyz(quaternion):
    w, x, y, z = map(float, quaternion)
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("无效的基座四元数")
    w, x, y, z = (v/norm for v in (w, x, y, z))
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]


def base_origin_velocity(linear_com_world, angular_world, quat_wxyz, com_offset_body):
    """PhysX root velocity is at COM; Pinocchio freeflyer uses link origin."""
    r = rotation_wxyz(quat_wxyz)
    offset = [sum(row[j]*com_offset_body[j] for j in range(3)) for row in r]
    wx, wy, wz = angular_world
    rx, ry, rz = offset
    cross = [wy*rz-wz*ry, wz*rx-wx*rz, wx*ry-wy*rx]
    return [linear_com_world[i]-cross[i] for i in range(3)]


def rpy_wxyz(quaternion):
    r = rotation_wxyz(quaternion)
    return (math.atan2(r[2][1], r[2][2]), math.asin(max(-1, min(1, -r[2][0]))),
            math.atan2(r[1][0], r[0][0]))
