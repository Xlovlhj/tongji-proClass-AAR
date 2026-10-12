"""Load terminal-selected JSON presets without importing Isaac or NumPy.

Precedence is defaults < JSON configuration < explicitly supplied CLI options.
Relative paths inside a JSON file are relative to that file, while CLI paths
are relative to the current working directory.  Validation rejects misspelled
keys, numeric booleans, non-finite values and incompatible adaptive parameters.
"""

from __future__ import annotations

from argparse import Namespace
from collections.abc import Mapping
from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any


CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"
ALIASES = {"pd": "pd.json", "gc-pd": "gc_pd.json", "gc_pd": "gc_pd.json",
           "rbf": "rbf.json", "fuzzy": "fuzzy.json"}
CONTROLLERS = ("pd", "gc-pd", "rbf", "fuzzy")
CONFIG_FIELDS = frozenset({
    "controller", "headless", "output", "isaac_root", "logo_width", "center",
    "draw_z", "speed", "dt", "planning_dt", "max_steps", "hold_seconds",
    "control", "adaptive",
    "disturbance", "record_video", "video_fps", "video_width", "video_height", "video_tail_seconds",
})
RUNTIME_FIELDS = frozenset({"preview", "check_env", "config", "config_path"})
CONTROL_FIELDS = frozenset({"kp", "kd", "torque_limits"})
ADAPTIVE_COMMON_FIELDS = frozenset({"learning_rate", "sliding_lambda", "leakage", "compensation_limits"})
RBF_FIELDS = frozenset({"position_scale", "velocity_scale", "rbf_width", "centers_per_axis"})
FUZZY_FIELDS = frozenset({"error_scale", "velocity_error_scale"})
PANDA_TORQUE_LIMITS = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
DEFAULT_COMPENSATION_LIMITS = (6.0, 6.0, 6.0, 6.0, 2.0, 2.0, 2.0)


def _number(value: Any, name: str, *, minimum: float | None = None,
            strictly_positive: bool = False) -> float:
    # bool subclasses int, so this check must precede the numeric check.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数，不能用布尔值代替")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} 超出有限浮点数范围") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数，不能用布尔值代替")
    if strictly_positive and value <= 0:
        raise ValueError(f"{name} 必须大于 0")
    if minimum is not None and value < minimum:
        raise ValueError(f"{name} 必须不小于 {minimum}")
    return number


def _vector(value: Any, name: str, *, scalar_allowed: bool = False,
            positive: bool = False) -> list[float] | float:
    if scalar_allowed and isinstance(value, (int, float)):
        return _number(value, name, minimum=None if positive else 0.0, strictly_positive=positive)
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise ValueError(f"{name} 必须包含 7 个关节数值" + ("，或使用一个标量" if scalar_allowed else ""))
    return [_number(v, f"{name}[{i}]", minimum=None if positive else 0.0, strictly_positive=positive)
            for i, v in enumerate(value)]


def _path(value: Any, name: str, base_dir: Path) -> Path:
    if not isinstance(value, (str, Path)) or not str(value).strip():
        raise ValueError(f"{name} 必须是非空路径字符串")
    result = Path(value).expanduser()
    if not result.is_absolute():
        result = base_dir / result
    return result.resolve()


def _section(value: Any, name: str, known: frozenset[str]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} 必须是 JSON 对象")
    unknown = set(value) - known
    if unknown:
        raise ValueError(f"{name} 中存在未知字段：{', '.join(sorted(map(str, unknown)))}")
    return deepcopy(dict(value))


def validate_config(data: Mapping[str, Any], *, base_dir: Path | None = None,
                    allow_runtime: bool = False, final: bool = False) -> dict[str, Any]:
    """Return validated, normalized fields; partial configurations are allowed.

    ``control.kp`` and ``control.kd`` scalar gains are expanded to seven values.
    Adaptive scalar/vector forms are retained for the controller factory.
    ``allow_runtime`` is reserved for merged argparse namespaces, not JSON files.
    ``final`` checks effective factory defaults as well as supplied parameters;
    partial files only check relationships whose adaptive fields are supplied.
    """
    if not isinstance(data, Mapping):
        raise ValueError("配置文件的顶层必须是 JSON 对象")
    unknown = set(data) - CONFIG_FIELDS - (RUNTIME_FIELDS if allow_runtime else frozenset())
    if unknown:
        raise ValueError(f"配置中存在未知字段：{', '.join(sorted(map(str, unknown)))}")
    result = deepcopy(dict(data))
    base_dir = Path.cwd() if base_dir is None else Path(base_dir)
    controller = result.get("controller")
    if "controller" in result and controller not in CONTROLLERS:
        raise ValueError(f"controller 必须是 {', '.join(CONTROLLERS)} 之一")
    for name in ("headless", "preview", "check_env"):
        if name in result and not isinstance(result[name], bool):
            raise ValueError(f"{name} 必须是 true 或 false")
    for name in ("logo_width", "speed", "dt", "planning_dt"):
        if name in result:
            result[name] = _number(result[name], name, strictly_positive=True)
    if "draw_z" in result:
        result["draw_z"] = _number(result["draw_z"], "draw_z")
    if "hold_seconds" in result:
        result["hold_seconds"] = _number(result["hold_seconds"], "hold_seconds", minimum=0.0)
    if "video_tail_seconds" in result:
        result["video_tail_seconds"] = _number(result["video_tail_seconds"], "video_tail_seconds", minimum=0.0)
    for name in ("video_fps", "video_width", "video_height"):
        if name in result:
            value = result[name]
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} 必须是正整数")
            if name != "video_fps" and value % 2:
                raise ValueError(f"{name} 必须是偶数")
    if result.get("record_video") is not None:
        result["record_video"] = _path(result["record_video"], "record_video", base_dir)
        if result["record_video"].suffix.lower() != ".mp4":
            raise ValueError("record_video 必须使用 .mp4 扩展名")
        if result.get("video_fps", 30) * result.get("dt", 0.001) > 1:
            raise ValueError("video_fps 不得超过物理控制采样频率 1 / dt")
    if "disturbance" in result:
        disturbance = _section(result["disturbance"], "disturbance", frozenset({"enabled", "pulses"}))
        if "enabled" in disturbance and not isinstance(disturbance["enabled"], bool):
            raise ValueError("disturbance.enabled 必须是 true 或 false")
        if "pulses" in disturbance:
            pulses = disturbance["pulses"]
            if not isinstance(pulses, list):
                raise ValueError("disturbance.pulses 必须是 JSON 数组")
            normalized = []
            previous_end = 0.0
            for i, item in enumerate(pulses):
                pulse = _section(item, f"disturbance.pulses[{i}]", frozenset({"start_s", "end_s", "force_world_n"}))
                if set(pulse) != {"start_s", "end_s", "force_world_n"}:
                    raise ValueError("每个外力脉冲需要 start_s、end_s、force_world_n")
                start = _number(pulse["start_s"], "pulse.start_s", minimum=0.0)
                end = _number(pulse["end_s"], "pulse.end_s", strictly_positive=True)
                if end <= start or start < previous_end:
                    raise ValueError("外力脉冲必须按时间排序、互不重叠且 end_s > start_s")
                values = pulse["force_world_n"]
                if not isinstance(values, (list, tuple)) or len(values) != 3:
                    raise ValueError("force_world_n 必须包含 XYZ 三个力分量")
                vector = [_number(v, "force_world_n") for v in values]
                normalized.append({"start_s": start, "end_s": end, "force_world_n": vector})
                previous_end = end
            disturbance["pulses"] = normalized
        result["disturbance"] = disturbance
    if "max_steps" in result:
        value = result["max_steps"]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("max_steps 必须是非负整数")
    if "center" in result:
        center = result["center"]
        if not isinstance(center, (list, tuple)) or len(center) != 2:
            raise ValueError("center 必须包含两个有限坐标 [X, Y]")
        result["center"] = tuple(_number(value, f"center[{i}]") for i, value in enumerate(center))
    if "output" in result:
        result["output"] = _path(result["output"], "output", base_dir)
    if result.get("isaac_root") is not None:
        result["isaac_root"] = _path(result["isaac_root"], "isaac_root", base_dir)
    if "control" in result:
        control = _section(result["control"], "control", CONTROL_FIELDS)
        for name in ("kp", "kd"):
            if name in control:
                value = _vector(control[name], f"control.{name}", scalar_allowed=True)
                control[name] = [value] * 7 if isinstance(value, float) else value
        if "torque_limits" in control:
            control["torque_limits"] = _vector(control["torque_limits"], "control.torque_limits", positive=True)
        result["control"] = control
    if "adaptive" in result:
        adaptive = _section(result["adaptive"], "adaptive", ADAPTIVE_COMMON_FIELDS | RBF_FIELDS | FUZZY_FIELDS)
        if controller in {"pd", "gc-pd"} and adaptive:
            raise ValueError(f"controller={controller} 不接受 adaptive 参数")
        inappropriate = set(adaptive) & (FUZZY_FIELDS if controller == "rbf" else RBF_FIELDS if controller == "fuzzy" else frozenset())
        if inappropriate:
            raise ValueError(f"controller={controller} 不支持 adaptive 字段：{', '.join(sorted(inappropriate))}")
        for name in ("learning_rate", "sliding_lambda"):
            if name in adaptive:
                adaptive[name] = _vector(adaptive[name], f"adaptive.{name}", scalar_allowed=True, positive=True)
        if "compensation_limits" in adaptive:
            adaptive["compensation_limits"] = _vector(adaptive["compensation_limits"], "adaptive.compensation_limits", positive=True)
        for name in ("position_scale", "velocity_scale", "rbf_width", "error_scale", "velocity_error_scale"):
            if name in adaptive:
                adaptive[name] = _number(adaptive[name], f"adaptive.{name}", strictly_positive=True)
        if "rbf_width" in adaptive and adaptive["rbf_width"] < 1e-6:
            raise ValueError("adaptive.rbf_width 必须不小于 1e-6")
        if "leakage" in adaptive:
            adaptive["leakage"] = _number(adaptive["leakage"], "adaptive.leakage", minimum=0.0)
        if "centers_per_axis" in adaptive:
            value = adaptive["centers_per_axis"]
            if isinstance(value, bool) or not isinstance(value, int) or not 2 <= value <= 9:
                raise ValueError("adaptive.centers_per_axis 必须是 2 到 9 的整数")
        result["adaptive"] = adaptive
    if controller in {"rbf", "fuzzy"}:
        adaptive = result.get("adaptive", {})
        dt = result.get("dt", 0.001)
        if ("dt" in result or final) and dt > 0.01:
            raise ValueError(f"controller={controller} 的 dt 必须不大于 0.01 秒")
        if (final or ("leakage" in adaptive and "dt" in result)) and adaptive.get("leakage", 0.02) * dt > 1:
            raise ValueError("adaptive.leakage * dt 必须不大于 1")
        if final or "compensation_limits" in adaptive:
            compensation = adaptive.get("compensation_limits", DEFAULT_COMPENSATION_LIMITS)
            limits = result.get("control", {}).get("torque_limits", PANDA_TORQUE_LIMITS)
            if any(value > limit for value, limit in zip(compensation, limits)):
                raise ValueError("adaptive.compensation_limits 不得超过对应的 control.torque_limits")
    return result


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"JSON 中存在重复字段：{key}")
        result[key] = value
    return result


def load_config(spec: str | Path | None) -> dict[str, Any]:
    """Load ``pd/gc-pd/rbf/fuzzy`` or a JSON filename, adding ``config_path``.

    ``None`` returns an empty dictionary so the same merge API also supports a
    run without a configuration file.  No directories are created by loading.
    """
    if spec is None:
        return {}
    if not isinstance(spec, (str, Path)) or not str(spec).strip():
        raise ValueError("config 必须是预设名称或非空 JSON 文件路径")
    path = CONFIG_DIR / ALIASES[str(spec)] if str(spec) in ALIASES else Path(spec).expanduser()
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"找不到配置文件：{path}；预设可用 pd、gc-pd、rbf、fuzzy")
    try:
        with path.open("r", encoding="utf-8-sig") as stream:
            data = json.load(stream, object_pairs_hook=_unique_object)
        result = validate_config(data, base_dir=path.parent)
    except (json.JSONDecodeError, ValueError) as exc:
        raise ValueError(f"配置文件 {path} 无效：{exc}") from exc
    result["config_path"] = str(path)
    return result


def merge_config(defaults: Mapping[str, Any] | Namespace,
                 config: Mapping[str, Any] | None = None,
                 overrides: Mapping[str, Any] | Namespace | None = None) -> Namespace:
    """Merge defaults, loaded JSON, then explicit CLI fields into a namespace.

    Use ``argparse.SUPPRESS`` for CLI defaults so absent options do not overwrite
    the file.  The ``control`` and ``adaptive`` objects merge their individual
    keys; each subsequent scalar/vector value replaces the previous one.
    """
    merged: dict[str, Any] = {}
    for layer in (defaults, config or {}, overrides or {}):
        data = vars(layer) if isinstance(layer, Namespace) else layer
        if not isinstance(data, Mapping):
            raise ValueError("配置合并输入必须是映射或 argparse.Namespace")
        for name, value in data.items():
            if name in {"control", "adaptive", "disturbance"} and isinstance(value, Mapping) and isinstance(merged.get(name), Mapping):
                merged[name] = {**merged[name], **deepcopy(dict(value))}
            else:
                merged[name] = deepcopy(value)
    merged.setdefault("control", {})
    merged.setdefault("adaptive", {})
    merged.setdefault("config_path", None)
    return Namespace(**validate_config(merged, allow_runtime=True, final=True))
