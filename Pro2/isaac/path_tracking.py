"""Deterministic planar path generation and closed-loop command tracking.

The tracker is deliberately independent from Isaac Sim so its geometry,
command limits and goal handling can be unit-tested with the system Python.
It converts a world-frame reference path into body-frame ``vx, vy, wz``
commands consumed by the existing OpenLoong controller bridge.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping, Sequence


def _finite(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    return result


def _positive(value, name: str, *, allow_zero: bool = False) -> float:
    result = _finite(value, name)
    if result < 0 if allow_zero else result <= 0:
        qualifier = "非负" if allow_zero else "正"
        raise ValueError(f"{name} 必须是有限{qualifier}数")
    return result


def _point(value, name: str) -> tuple[float, float]:
    # Isaac returns NumPy slices for measured positions.  Accept any finite,
    # indexable two-element vector instead of limiting the runtime API to JSON
    # list/tuple values used by configuration files.
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{name} 必须是 [x, y]")
    try:
        valid_length = len(value) == 2
    except (TypeError, AttributeError):
        valid_length = False
    if not valid_length:
        raise ValueError(f"{name} 必须是 [x, y]")
    return (_finite(value[0], f"{name}[0]"), _finite(value[1], f"{name}[1]"))


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    value = _finite(angle, "angle")
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _check_keys(value: Mapping, allowed: set[str], name: str) -> None:
    unknown = set(value) - allowed
    if unknown:
        raise ValueError(f"{name} 含未知字段：{', '.join(sorted(unknown))}")


def build_reference_path(spec: Mapping) -> list[tuple[float, float]]:
    """Build sampled path points from a validated JSON-compatible mapping."""
    if not isinstance(spec, Mapping):
        raise ValueError("path_tracking.path 必须是对象")
    kind = spec.get("type")
    if kind == "line":
        _check_keys(spec, {"type", "start", "end", "num_points"}, "line path")
        start = _point(spec.get("start"), "line.start")
        end = _point(spec.get("end"), "line.end")
        count = spec.get("num_points", 101)
        if isinstance(count, bool) or not isinstance(count, int) or count < 2:
            raise ValueError("line.num_points 必须是至少为 2 的整数")
        if math.dist(start, end) < 1e-9:
            raise ValueError("line.start 与 line.end 不能重合")
        return [
            (start[0] + (end[0] - start[0]) * i / (count - 1),
             start[1] + (end[1] - start[1]) * i / (count - 1))
            for i in range(count)
        ]
    if kind == "polyline":
        _check_keys(spec, {"type", "waypoints"}, "polyline path")
        raw = spec.get("waypoints")
        if not isinstance(raw, list) or len(raw) < 2:
            raise ValueError("polyline.waypoints 至少需要两个点")
        points = [_point(p, f"polyline.waypoints[{i}]") for i, p in enumerate(raw)]
        if any(math.dist(a, b) < 1e-9 for a, b in zip(points, points[1:])):
            raise ValueError("polyline 相邻路径点不能重合")
        return points
    if kind == "circle":
        allowed = {"type", "center", "radius_m", "start_angle_rad", "end_angle_rad", "num_points"}
        _check_keys(spec, allowed, "circle path")
        center = _point(spec.get("center"), "circle.center")
        radius = _positive(spec.get("radius_m"), "circle.radius_m")
        start = _finite(spec.get("start_angle_rad"), "circle.start_angle_rad")
        end = _finite(spec.get("end_angle_rad"), "circle.end_angle_rad")
        if abs(end - start) < 1e-6:
            raise ValueError("circle 起止角必须不同；完整圆请相差 2*pi")
        count = spec.get("num_points", 181)
        if isinstance(count, bool) or not isinstance(count, int) or count < 8:
            raise ValueError("circle.num_points 必须是至少为 8 的整数")
        return [
            (center[0] + radius * math.cos(start + (end - start) * i / (count - 1)),
             center[1] + radius * math.sin(start + (end - start) * i / (count - 1)))
            for i in range(count)
        ]
    raise ValueError("path_tracking.path.type 必须是 line、polyline 或 circle")


def path_visualization_plan(
    points: Sequence[Sequence[float]],
    *,
    ground_clearance_m: float = 0.018,
    line_width_m: float = 0.045,
    marker_radius_m: float = 0.055,
    marker_height_m: float = 0.008,
) -> dict:
    """Return a JSON-compatible, visual-only USD plan for a planar path.

    The line is raised slightly above the flat ground to avoid z-fighting.
    Marker geometry intentionally has no collision API, so showing the path
    cannot change the controller or the robot-ground contact result.
    """
    normalized = [_point(point, f"points[{i}]") for i, point in enumerate(points)]
    if len(normalized) < 2:
        raise ValueError("路径可视化至少需要两个点")
    if any(math.dist(a, b) < 1e-9 for a, b in zip(normalized, normalized[1:])):
        raise ValueError("路径可视化的相邻点不能重合")
    clearance = _positive(ground_clearance_m, "ground_clearance_m")
    width = _positive(line_width_m, "line_width_m")
    radius = _positive(marker_radius_m, "marker_radius_m")
    marker_height = _positive(marker_height_m, "marker_height_m")

    # Five-point polylines show every waypoint.  Densely sampled lines and
    # circles use at most twelve markers so the stage stays light-weight.
    marker_limit = 12
    if len(normalized) <= marker_limit:
        marker_indices = list(range(len(normalized)))
    else:
        marker_indices = sorted({
            round(i * (len(normalized) - 1) / (marker_limit - 1))
            for i in range(marker_limit)
        })
    closed = math.dist(normalized[0], normalized[-1]) < 1e-9
    if closed and marker_indices[-1] == len(normalized) - 1:
        marker_indices.pop()

    colors = {
        "start": [0.10, 0.95, 0.25],
        "waypoint": [1.00, 0.80, 0.08],
        "goal": [1.00, 0.12, 0.10],
        "start_goal": [0.95, 0.20, 0.95],
    }
    markers = []
    for marker_number, point_index in enumerate(marker_indices):
        if point_index == 0:
            role = "start_goal" if closed else "start"
        elif point_index == len(normalized) - 1:
            role = "goal"
        else:
            role = "waypoint"
        x, y = normalized[point_index]
        markers.append({
            "prim_path": f"/World/PathVisualization/Marker{marker_number:02d}",
            "point_index": point_index,
            "role": role,
            "center_m": [x, y, marker_height / 2.0 + 0.001],
            "radius_m": radius,
            "height_m": marker_height,
            "color": colors[role],
        })

    curve_path = "/World/PathVisualization/ReferencePath"
    curve_points = [[x, y, clearance] for x, y in normalized]
    half_width = width / 2.0
    ribbon_points = []
    face_vertex_counts = []
    face_vertex_indices = []
    for a, b in zip(normalized, normalized[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        nx, ny = -dy / length * half_width, dx / length * half_width
        first = len(ribbon_points)
        ribbon_points.extend([
            [a[0] + nx, a[1] + ny, clearance],
            [a[0] - nx, a[1] - ny, clearance],
            [b[0] - nx, b[1] - ny, clearance],
            [b[0] + nx, b[1] + ny, clearance],
        ])
        face_vertex_counts.append(4)
        face_vertex_indices.extend(range(first, first + 4))
    extent = [
        [min(point[0] for point in ribbon_points),
         min(point[1] for point in ribbon_points),
         clearance],
        [max(point[0] for point in ribbon_points),
         max(point[1] for point in ribbon_points),
         clearance],
    ]
    return {
        "enabled": True,
        "root_prim_path": "/World/PathVisualization",
        "curve_prim_path": curve_path,
        "reference_point_count": len(normalized),
        "ground_clearance_m": clearance,
        "line_width_m": width,
        "line_color": [0.05, 0.85, 1.00],
        "points_m": curve_points,
        "geometry_type": "ribbon_mesh",
        "ribbon_points_m": ribbon_points,
        "face_vertex_counts": face_vertex_counts,
        "face_vertex_indices": face_vertex_indices,
        "extent_m": extent,
        "markers": markers,
        "primitive_paths": [curve_path] + [item["prim_path"] for item in markers],
        "collision_enabled": False,
    }


def build_path_visualization(stage, points: Sequence[Sequence[float]]) -> dict:
    """Author the visual path in an Isaac USD stage and return its manifest."""
    from pxr import Gf, UsdGeom, Vt

    plan = path_visualization_plan(points)
    UsdGeom.Xform.Define(stage, plan["root_prim_path"])
    curve = UsdGeom.Mesh.Define(stage, plan["curve_prim_path"])
    curve.CreatePointsAttr(Vt.Vec3fArray([
        Gf.Vec3f(*point) for point in plan["ribbon_points_m"]
    ]))
    curve.CreateFaceVertexCountsAttr(Vt.IntArray(plan["face_vertex_counts"]))
    curve.CreateFaceVertexIndicesAttr(Vt.IntArray(plan["face_vertex_indices"]))
    curve.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in plan["extent_m"]]))
    curve.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    curve.CreateDoubleSidedAttr(True)
    curve.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*plan["line_color"])]))

    for item in plan["markers"]:
        marker = UsdGeom.Cylinder.Define(stage, item["prim_path"])
        marker.CreateAxisAttr(UsdGeom.Tokens.z)
        marker.CreateRadiusAttr(item["radius_m"])
        marker.CreateHeightAttr(item["height_m"])
        marker.AddTranslateOp().Set(Gf.Vec3d(*item["center_m"]))
        marker.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*item["color"])]))
        # Do not apply UsdPhysics.CollisionAPI: markers are display-only.
    return plan


@dataclass(frozen=True)
class TrackerSettings:
    mode: str
    lookahead_m: float
    nominal_speed_m_s: float
    gain_longitudinal: float
    gain_lateral: float
    gain_heading: float
    gain_cross_heading: float
    max_vx_m_s: float
    max_vy_m_s: float
    max_wz_rad_s: float
    max_linear_accel_m_s2: float
    max_angular_accel_rad_s2: float
    goal_tolerance_m: float
    stop_at_goal: bool
    open_loop_command: tuple[float, float, float]


def normalize_path_tracking(config: Mapping | None) -> dict | None:
    """Validate the nested run configuration and return a plain normalized dict."""
    if config is None:
        return None
    if not isinstance(config, Mapping):
        raise ValueError("path_tracking 必须是对象或 null")
    allowed = {
        "enabled", "mode", "path", "lookahead_m", "nominal_speed_m_s", "gains", "limits",
        "goal_tolerance_m", "stop_at_goal", "open_loop_command",
    }
    _check_keys(config, allowed, "path_tracking")
    enabled = config.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("path_tracking.enabled 必须是布尔值")
    if not enabled:
        return None
    mode = config.get("mode", "closed_loop")
    if mode not in {"closed_loop", "open_loop"}:
        raise ValueError("path_tracking.mode 必须是 closed_loop 或 open_loop")
    points = build_reference_path(config.get("path"))
    gains = config.get("gains", {})
    limits = config.get("limits", {})
    if not isinstance(gains, Mapping) or not isinstance(limits, Mapping):
        raise ValueError("path_tracking.gains 和 limits 必须是对象")
    _check_keys(gains, {"longitudinal", "lateral", "heading", "cross_heading"}, "path_tracking.gains")
    _check_keys(limits, {"vx_m_s", "vy_m_s", "wz_rad_s", "linear_accel_m_s2", "angular_accel_rad_s2"},
                "path_tracking.limits")
    command = config.get("open_loop_command", [0.15, 0.0, 0.15])
    if not isinstance(command, (list, tuple)) or len(command) != 3:
        raise ValueError("path_tracking.open_loop_command 必须有三个数 [vx, vy, wz]")
    command = tuple(_finite(v, f"path_tracking.open_loop_command[{i}]") for i, v in enumerate(command))
    settings = TrackerSettings(
        mode=mode,
        lookahead_m=_positive(config.get("lookahead_m", 0.30), "path_tracking.lookahead_m"),
        nominal_speed_m_s=_positive(config.get("nominal_speed_m_s", 0.15),
                                    "path_tracking.nominal_speed_m_s", allow_zero=True),
        gain_longitudinal=_positive(gains.get("longitudinal", 0.6), "path_tracking.gains.longitudinal", allow_zero=True),
        gain_lateral=_positive(gains.get("lateral", 0.8), "path_tracking.gains.lateral", allow_zero=True),
        gain_heading=_positive(gains.get("heading", 1.2), "path_tracking.gains.heading", allow_zero=True),
        gain_cross_heading=_positive(gains.get("cross_heading", 0.4),
                                     "path_tracking.gains.cross_heading", allow_zero=True),
        max_vx_m_s=_positive(limits.get("vx_m_s", 0.25), "path_tracking.limits.vx_m_s"),
        max_vy_m_s=_positive(limits.get("vy_m_s", 0.12), "path_tracking.limits.vy_m_s"),
        max_wz_rad_s=_positive(limits.get("wz_rad_s", 0.35), "path_tracking.limits.wz_rad_s"),
        max_linear_accel_m_s2=_positive(limits.get("linear_accel_m_s2", 0.5),
                                        "path_tracking.limits.linear_accel_m_s2"),
        max_angular_accel_rad_s2=_positive(limits.get("angular_accel_rad_s2", 0.8),
                                           "path_tracking.limits.angular_accel_rad_s2"),
        goal_tolerance_m=_positive(config.get("goal_tolerance_m", 0.10), "path_tracking.goal_tolerance_m"),
        stop_at_goal=config.get("stop_at_goal", True),
        open_loop_command=command,
    )
    if not isinstance(settings.stop_at_goal, bool):
        raise ValueError("path_tracking.stop_at_goal 必须是布尔值")
    for i, (value, limit) in enumerate(zip(command, (settings.max_vx_m_s, settings.max_vy_m_s,
                                                      settings.max_wz_rad_s))):
        if abs(value) > limit + 1e-12:
            raise ValueError(f"open_loop_command[{i}] 超出对应 limits")
    return {"settings": settings, "points": points, "source": dict(config)}


class Polyline:
    def __init__(self, points: Sequence[Sequence[float]]):
        self.points = [_point(p, f"points[{i}]") for i, p in enumerate(points)]
        if len(self.points) < 2:
            raise ValueError("路径至少需要两个点")
        self.lengths: list[float] = []
        self.cumulative = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            length = math.dist(a, b)
            if length < 1e-9:
                raise ValueError("路径相邻点不能重合")
            self.lengths.append(length)
            self.cumulative.append(self.cumulative[-1] + length)
        self.total_length = self.cumulative[-1]

    def sample(self, distance_m: float) -> tuple[tuple[float, float], float]:
        s = min(self.total_length, max(0.0, float(distance_m)))
        index = len(self.lengths) - 1
        for i, end in enumerate(self.cumulative[1:]):
            if s <= end + 1e-12:
                index = i
                break
        a, b, length = self.points[index], self.points[index + 1], self.lengths[index]
        ratio = min(1.0, max(0.0, (s - self.cumulative[index]) / length))
        point = (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]))
        return point, math.atan2(b[1] - a[1], b[0] - a[0])

    def project(self, point: Sequence[float], previous_s: float = 0.0) -> tuple[float, tuple[float, float], float, float]:
        px, py = _point(point, "position")
        candidates = []
        for i, (a, b, length) in enumerate(zip(self.points, self.points[1:], self.lengths)):
            dx, dy = b[0] - a[0], b[1] - a[1]
            ratio = min(1.0, max(0.0, ((px - a[0]) * dx + (py - a[1]) * dy) / (length * length)))
            nearest = (a[0] + ratio * dx, a[1] + ratio * dy)
            s = self.cumulative[i] + ratio * length
            distance = math.hypot(px - nearest[0], py - nearest[1])
            heading = math.atan2(dy, dx)
            signed = math.cos(heading) * (py - nearest[1]) - math.sin(heading) * (px - nearest[0])
            # Distance is primary, with a small arc-length continuity term to
            # disambiguate crossings and the coincident start/end point of a
            # sampled full circle.  A near-start measurement a few centimetres
            # outside the circle must not jump directly to the final segment.
            score = distance + 5e-3 * abs(s - previous_s)
            candidates.append((score, s, nearest, signed, heading))
        _, s, nearest, signed, heading = min(candidates, key=lambda item: item[0])
        # Do not move progress backwards because gait oscillation can move the base
        # a few centimetres behind the previous orthogonal projection.
        if s < previous_s:
            s = previous_s
            nearest, heading = self.sample(s)
            signed = math.cos(heading) * (py - nearest[1]) - math.sin(heading) * (px - nearest[0])
        return s, nearest, signed, heading


class PathTracker:
    """Stateful path tracker with rate-limited body-frame commands."""
    diagnostic_names = (
        "target_x_m", "target_y_m", "target_yaw_rad", "nearest_x_m", "nearest_y_m",
        "progress_m", "progress_fraction", "cross_track_error_m", "path_distance_m",
        "heading_error_rad", "goal_distance_m", "reached_goal",
    )

    def __init__(self, config: Mapping):
        normalized = normalize_path_tracking(config)
        if normalized is None:
            raise ValueError("PathTracker 需要启用的 path_tracking 配置")
        self.settings: TrackerSettings = normalized["settings"]
        self.path = Polyline(normalized["points"])
        self.progress_m = 0.0
        self.previous_command = [0.0, 0.0, 0.0]
        self.reached_goal = False
        self.last_diagnostics = {name: 0.0 for name in self.diagnostic_names}

    @property
    def reference_points(self) -> list[tuple[float, float]]:
        return list(self.path.points)

    @staticmethod
    def _clip(value: float, limit: float) -> float:
        return min(limit, max(-limit, value))

    def _rate_limit(self, desired: Iterable[float], dt: float) -> list[float]:
        limits = (self.settings.max_linear_accel_m_s2 * dt,
                  self.settings.max_linear_accel_m_s2 * dt,
                  self.settings.max_angular_accel_rad_s2 * dt)
        result = []
        for old, new, delta in zip(self.previous_command, desired, limits):
            result.append(old + self._clip(float(new) - old, delta))
        self.previous_command = result
        return result

    def update(self, position_xy: Sequence[float], yaw_rad: float, dt: float) -> tuple[list[float], dict[str, float]]:
        dt = _positive(dt, "dt")
        px, py = _point(position_xy, "position_xy")
        yaw = _finite(yaw_rad, "yaw_rad")
        self.progress_m, nearest, signed_cross, _ = self.path.project((px, py), self.progress_m)
        target_s = min(self.path.total_length, self.progress_m + self.settings.lookahead_m)
        target, target_yaw = self.path.sample(target_s)
        dx, dy = target[0] - px, target[1] - py
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        heading_error = wrap_angle(target_yaw - yaw)
        goal_distance = math.dist((px, py), self.path.points[-1])
        near_end = self.progress_m >= self.path.total_length - self.settings.goal_tolerance_m
        # Reaching a terminal goal is an event.  Latch it so normal body sway
        # during Walk2Stand cannot re-enable path commands after the robot has
        # already entered the goal region.
        self.reached_goal = self.reached_goal or bool(
            near_end and goal_distance <= self.settings.goal_tolerance_m)

        if self.reached_goal and self.settings.stop_at_goal:
            desired = [0.0, 0.0, 0.0]
        elif self.settings.mode == "open_loop":
            desired = list(self.settings.open_loop_command)
        else:
            speed = self.settings.nominal_speed_m_s
            if self.settings.stop_at_goal:
                speed *= min(1.0, max(0.0, goal_distance / max(2.0 * self.settings.goal_tolerance_m, 1e-9)))
            desired = [
                self._clip(speed + self.settings.gain_longitudinal * ex, self.settings.max_vx_m_s),
                self._clip(self.settings.gain_lateral * ey, self.settings.max_vy_m_s),
                self._clip(self.settings.gain_heading * heading_error +
                           self.settings.gain_cross_heading * ey, self.settings.max_wz_rad_s),
            ]
        command = self._rate_limit(desired, dt)
        diagnostics = {
            "target_x_m": target[0], "target_y_m": target[1], "target_yaw_rad": target_yaw,
            "nearest_x_m": nearest[0], "nearest_y_m": nearest[1],
            "progress_m": self.progress_m,
            "progress_fraction": self.progress_m / self.path.total_length,
            "cross_track_error_m": signed_cross,
            "path_distance_m": abs(signed_cross),
            "heading_error_rad": heading_error,
            "goal_distance_m": goal_distance,
            "reached_goal": float(self.reached_goal),
        }
        self.last_diagnostics = diagnostics
        return command, diagnostics

    def stop(self, dt: float) -> list[float]:
        """Rate-limit the command toward zero before walking starts or after override."""
        return self._rate_limit((0.0, 0.0, 0.0), _positive(dt, "dt"))

