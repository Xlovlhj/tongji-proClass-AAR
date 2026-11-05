"""Validated, reproducible Isaac terrain descriptions for L4 experiments.

Validation and geometry planning use only the Python standard library.  USD
objects are imported lazily by :func:`build_terrain`, after SimulationApp has
started.  The controller never receives the generated height map, so the
``blind_step`` cases are genuinely unknown to the path/velocity command layer.
"""
from __future__ import annotations

import math
import random
from typing import Mapping


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    return result


def _positive(value, name: str, *, zero: bool = False) -> float:
    result = _number(value, name)
    if result < 0 if zero else result <= 0:
        raise ValueError(f"{name} 必须是{'非负' if zero else '正'}数")
    return result


def _integer(value, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} 必须是不小于 {minimum} 的整数")
    return value


def _keys(config: Mapping, allowed: set[str], name: str) -> None:
    unknown = set(config) - allowed
    if unknown:
        raise ValueError(f"{name} 含未知字段：{', '.join(sorted(unknown))}")


def normalize_terrain(config: Mapping | None) -> dict:
    """Validate a terrain mapping and fill deterministic defaults."""
    if config is None:
        config = {"type": "flat"}
    if not isinstance(config, Mapping):
        raise ValueError("terrain 必须是对象或 null")
    kind = config.get("type", "flat")
    common = {"type", "friction", "restitution", "color"}
    friction = _positive(config.get("friction", 1.0), "terrain.friction", zero=True)
    restitution = _positive(config.get("restitution", 0.0), "terrain.restitution", zero=True)
    if restitution > 1.0:
        raise ValueError("terrain.restitution 不能大于 1")
    color = config.get("color", [0.17, 0.20, 0.23])
    if not isinstance(color, (list, tuple)) or len(color) != 3:
        raise ValueError("terrain.color 必须是三个 [0,1] 数值")
    color = [_number(v, f"terrain.color[{i}]") for i, v in enumerate(color)]
    if any(v < 0 or v > 1 for v in color):
        raise ValueError("terrain.color 必须位于 [0,1]")
    result = {"type": kind, "friction": friction, "restitution": restitution, "color": color}

    if kind == "flat":
        _keys(config, common, "terrain(flat)")
    elif kind == "slope":
        allowed = common | {"angle_deg", "start_x_m", "length_m", "width_m", "thickness_m", "start_height_m"}
        _keys(config, allowed, "terrain(slope)")
        angle = _number(config.get("angle_deg"), "terrain.angle_deg")
        if not 0 < abs(angle) <= 20:
            raise ValueError("terrain.angle_deg 绝对值必须在 (0,20] 度")
        result.update(angle_deg=angle, start_x_m=_number(config.get("start_x_m", 0.8), "terrain.start_x_m"),
                      length_m=_positive(config.get("length_m", 2.5), "terrain.length_m"),
                      width_m=_positive(config.get("width_m", 2.0), "terrain.width_m"),
                      thickness_m=_positive(config.get("thickness_m", 0.08), "terrain.thickness_m"),
                      start_height_m=_number(config.get("start_height_m", 0.0), "terrain.start_height_m"))
    elif kind == "slope_course":
        allowed = common | {"start_x_m", "width_m", "thickness_m", "start_height_m", "segments"}
        _keys(config, allowed, "terrain(slope_course)")
        segments = config.get("segments")
        if not isinstance(segments, list) or not segments:
            raise ValueError("terrain.segments 至少需要一个坡段")
        normalized = []
        for i, segment in enumerate(segments):
            if not isinstance(segment, Mapping):
                raise ValueError(f"terrain.segments[{i}] 必须是对象")
            _keys(segment, {"angle_deg", "length_m"}, f"terrain.segments[{i}]")
            angle = _number(segment.get("angle_deg"), f"terrain.segments[{i}].angle_deg")
            if abs(angle) > 20:
                raise ValueError("坡道课程角度绝对值不能超过 20 度")
            normalized.append({"angle_deg": angle,
                               "length_m": _positive(segment.get("length_m"), f"terrain.segments[{i}].length_m")})
        result.update(start_x_m=_number(config.get("start_x_m", 0.8), "terrain.start_x_m"),
                      width_m=_positive(config.get("width_m", 2.0), "terrain.width_m"),
                      thickness_m=_positive(config.get("thickness_m", 0.08), "terrain.thickness_m"),
                      start_height_m=_number(config.get("start_height_m", 0.0), "terrain.start_height_m"),
                      segments=normalized)
    elif kind == "rough_blocks":
        allowed = common | {"seed", "amplitude_m", "tile_size_m", "rows", "columns", "start_x_m"}
        _keys(config, allowed, "terrain(rough_blocks)")
        result.update(seed=_integer(config.get("seed", 1), "terrain.seed"),
                      amplitude_m=_positive(config.get("amplitude_m"), "terrain.amplitude_m"),
                      tile_size_m=_positive(config.get("tile_size_m", 0.20), "terrain.tile_size_m"),
                      rows=_integer(config.get("rows", 16), "terrain.rows", 2),
                      columns=_integer(config.get("columns", 10), "terrain.columns", 2),
                      start_x_m=_number(config.get("start_x_m", 0.8), "terrain.start_x_m"))
    elif kind == "random_obstacles":
        allowed = common | {"seed", "count", "region_x_m", "region_y_m", "height_range_m",
                            "length_range_m", "width_range_m"}
        _keys(config, allowed, "terrain(random_obstacles)")
        result.update(seed=_integer(config.get("seed", 1), "terrain.seed"),
                      count=_integer(config.get("count", 12), "terrain.count", 1))
        for key, default in (("region_x_m", [0.8, 3.5]), ("region_y_m", [-0.8, 0.8]),
                             ("height_range_m", [0.02, 0.08]), ("length_range_m", [0.10, 0.30]),
                             ("width_range_m", [0.10, 0.35])):
            raw = config.get(key, default)
            if not isinstance(raw, (list, tuple)) or len(raw) != 2:
                raise ValueError(f"terrain.{key} 必须是 [min,max]")
            pair = [_number(v, f"terrain.{key}[{i}]") for i, v in enumerate(raw)]
            if pair[1] <= pair[0]:
                raise ValueError(f"terrain.{key} 必须满足 max>min")
            if key.endswith("range_m") and pair[0] <= 0:
                raise ValueError(f"terrain.{key} 必须为正")
            result[key] = pair
    elif kind == "blind_step":
        allowed = common | {"variant", "start_x_m", "height_m", "length_m", "width_m"}
        _keys(config, allowed, "terrain(blind_step)")
        variant = config.get("variant", "up")
        if variant not in {"up", "down", "uneven"}:
            raise ValueError("terrain.variant 必须是 up、down 或 uneven")
        result.update(variant=variant,
                      start_x_m=_number(config.get("start_x_m", 0.9), "terrain.start_x_m"),
                      height_m=_positive(config.get("height_m", 0.06), "terrain.height_m"),
                      length_m=_positive(config.get("length_m", 4.0), "terrain.length_m"),
                      width_m=_positive(config.get("width_m", 2.0), "terrain.width_m"))
    else:
        raise ValueError("terrain.type 必须是 flat、slope、slope_course、rough_blocks、random_obstacles 或 blind_step")
    return result


def terrain_plan(config: Mapping | None) -> list[dict]:
    """Return cuboids before touching USD; useful for tests and manifests."""
    cfg = normalize_terrain(config)
    items: list[dict] = []

    def box(name, center, size, *, rotate_y_deg=0.0, color=None):
        if min(size) <= 0:
            raise ValueError(f"地形块 {name} 尺寸必须为正")
        items.append({"name": name, "center_m": list(center), "size_m": list(size),
                      "rotate_y_deg": float(rotate_y_deg), "color": list(color or cfg["color"])})

    kind = cfg["type"]
    if kind in {"flat", "slope", "slope_course", "random_obstacles"}:
        box("Ground", (0.0, 0.0, -0.05), (200.0, 200.0, 0.10))
    if kind == "slope":
        theta = math.radians(cfg["angle_deg"])
        length, thickness = cfg["length_m"], cfg["thickness_m"]
        horizontal = length * math.cos(theta)
        center_x = cfg["start_x_m"] + horizontal / 2.0
        center_z = cfg["start_height_m"] + length * math.sin(theta) / 2.0 - thickness * math.cos(theta) / 2.0
        box("Slope00", (center_x, 0.0, center_z), (length, cfg["width_m"], thickness),
            rotate_y_deg=-cfg["angle_deg"])
    elif kind == "slope_course":
        x, z = cfg["start_x_m"], cfg["start_height_m"]
        for i, segment in enumerate(cfg["segments"]):
            theta, length = math.radians(segment["angle_deg"]), segment["length_m"]
            horizontal, rise = length * math.cos(theta), length * math.sin(theta)
            center = (x + horizontal / 2.0, 0.0,
                      z + rise / 2.0 - cfg["thickness_m"] * math.cos(theta) / 2.0)
            box(f"Slope{i:02d}", center, (length, cfg["width_m"], cfg["thickness_m"]),
                rotate_y_deg=-segment["angle_deg"])
            x, z = x + horizontal, z + rise
    elif kind == "rough_blocks":
        rng = random.Random(cfg["seed"])
        amplitude, tile = cfg["amplitude_m"], cfg["tile_size_m"]
        bottom = -amplitude - 0.04
        # A lower safety floor does not mask negative-height tiles.
        box("LowerFloor", (0.0, 0.0, bottom - 0.05), (200.0, 200.0, 0.10))
        approach_start = -5.0
        approach_length = cfg["start_x_m"] - approach_start
        box("Approach", ((approach_start + cfg["start_x_m"]) / 2.0, 0.0, bottom / 2.0),
            (approach_length, cfg["columns"] * tile, -bottom))
        for row in range(cfg["rows"]):
            for column in range(cfg["columns"]):
                top = rng.uniform(-amplitude, amplitude)
                x = cfg["start_x_m"] + (row + 0.5) * tile
                y = (column - (cfg["columns"] - 1) / 2.0) * tile
                box(f"Rough_{row:02d}_{column:02d}", (x, y, (top + bottom) / 2.0),
                    (tile * 1.01, tile * 1.01, top - bottom))
    elif kind == "random_obstacles":
        rng = random.Random(cfg["seed"])
        for i in range(cfg["count"]):
            length = rng.uniform(*cfg["length_range_m"])
            width = rng.uniform(*cfg["width_range_m"])
            height = rng.uniform(*cfg["height_range_m"])
            x = rng.uniform(cfg["region_x_m"][0] + length / 2.0, cfg["region_x_m"][1] - length / 2.0)
            y = rng.uniform(cfg["region_y_m"][0] + width / 2.0, cfg["region_y_m"][1] - width / 2.0)
            box(f"Obstacle{i:02d}", (x, y, height / 2.0), (length, width, height),
                color=(0.45, 0.20, 0.08))
    elif kind == "blind_step":
        start, height, length, width = cfg["start_x_m"], cfg["height_m"], cfg["length_m"], cfg["width_m"]
        bottom = -height - 0.05
        box("LowerFloor", (0.0, 0.0, bottom - 0.05), (200.0, 200.0, 0.10))
        approach_start = -5.0
        approach_length = start - approach_start
        box("Approach", ((approach_start + start) / 2.0, 0.0, bottom / 2.0),
            (approach_length, width, -bottom))
        if cfg["variant"] == "up":
            top_left = top_right = height
        elif cfg["variant"] == "down":
            top_left = top_right = -height
        else:
            top_left, top_right = height, 0.0
        for name, y, top in (("StepLeft", width / 4.0, top_left), ("StepRight", -width / 4.0, top_right)):
            box(name, (start + length / 2.0, y, (top + bottom) / 2.0),
                (length, width / 2.0, top - bottom), color=(0.30, 0.32, 0.35))
    return items


def terrain_manifest(config: Mapping | None) -> dict:
    """Return the exact JSON-compatible geometry manifest for one run."""
    cfg = normalize_terrain(config)
    plan = terrain_plan(cfg)
    paths = ["/World/Terrain/" + item["name"] for item in plan]
    return {
        "config": cfg,
        "plan": plan,
        "primitive_count": len(plan),
        "primitive_paths": paths,
    }


def build_terrain(stage, config: Mapping | None) -> dict:
    """Author the planned cuboids, collisions and a shared physics material."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    manifest = terrain_manifest(config)
    cfg = manifest["config"]
    plan = manifest["plan"]
    material = UsdShade.Material.Define(stage, "/World/TerrainMaterial")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(cfg["friction"])
    physics.CreateDynamicFrictionAttr(cfg["friction"])
    physics.CreateRestitutionAttr(cfg["restitution"])
    paths = []
    for item in plan:
        path = "/World/Terrain/" + item["name"]
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*item["center_m"]))
        if abs(item["rotate_y_deg"]) > 1e-12:
            cube.AddRotateYOp().Set(item["rotate_y_deg"])
        cube.AddScaleOp().Set(Gf.Vec3d(*item["size_m"]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*item["color"])])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")
        paths.append(path)
    if paths != manifest["primitive_paths"]:
        raise RuntimeError("实际创建的地形路径与预生成 manifest 不一致")
    return manifest

