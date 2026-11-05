#!/usr/bin/env python3
"""Analyze the four OpenLoong L2 comparison experiments.

This program intentionally uses only the Python standard library and Pillow.  It
reads each run's ``run.json`` and ``states.csv`` and creates, for every
experiment family, a separate ``comparison`` directory containing:

* ``metrics.csv``: machine-readable metrics and experiment provenance;
* compact PNG comparisons: title, axes, units and legend only;
* ``analysis.md``: what changed, what was observed, a cautious mechanism
  interpretation, and the limits of the evidence.

It also creates ``output/L2/summary/L2总指标.csv`` and
``output/L2/summary/L2实验结论.md``.  It never starts Isaac Sim and never edits
the original run data. ``output/L2/active_runs.json``, when present, explicitly
selects the active run root and expected experiment settings for each family.
Historical run roots and their comparisons are not overwritten.

Typical usage from the project root::

    python3 Pro2/tools/analyze_l2_results.py

Analyze one family while experiments are still being collected::

    python3 Pro2/tools/analyze_l2_results.py --group lateral_push

Require all expected run directories and complete data::

    python3 Pro2/tools/analyze_l2_results.py --strict
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover - exercised only on a broken host
    raise SystemExit(
        "缺少 Pillow，无法生成 L2 PNG 对比图。请使用本项目已有的 Python 环境运行。"
    ) from exc


PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_L2_ROOT = PROJECT / "Pro2" / "output" / "L2"

STEADY_START_S = 5.0
DEFAULT_PUSH_AT_S = 7.0
DEFAULT_PUSH_DURATION_S = 0.1
PUSH_TRANSIENT_WINDOW_S = 2.0
RECOVERY_HOLD_S = 0.5
RECOVERY_SPEED_TOL_M_S = 0.05
RECOVERY_ANGLE_TOL_RAD = 0.03

# L2 的正式验收运行采用固定的 12 s / 1 kHz 仿真、100 Hz 记录规格。
# ``--strict`` 不根据某个不完整 run.json 自行放宽要求，而是逐项核对这些
# 已约定的成果规格以及清单、CSV 之间的一致性。
EXPECTED_DURATION_S = 12.0
EXPECTED_STEPS = 12_000
EXPECTED_SAMPLES = 1_200
EXPECTED_LOG_PERIOD_S = 0.01
TIME_ABS_TOL_S = 1e-6

REQUIRED_NUMERIC_COLUMNS = (
    "time_s",
    "base_z_m",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "vx_world_m_s",
    "vy_world_m_s",
    "cmd_vx_body_m_s",
    "cmd_vy_body_m_s",
    "controller:command_vx_mps",
    "controller:command_vy_mps",
    "controller:mpc_qp_status",
    "controller:mpc_solve_s",
    "controller:wbc_qp_status",
    "controller:wbc_solve_s",
    "controller:controller_step_s",
    "controller:left_force_z_est_N",
    "controller:right_force_z_est_N",
)

COLORS = [
    (41, 98, 255),
    (232, 87, 62),
    (38, 166, 91),
    (145, 90, 200),
    (230, 155, 35),
    (32, 161, 173),
]


@dataclass(frozen=True)
class ConditionSpec:
    name: str
    directory_names: tuple[str, ...]
    label: str
    fallback_value: float


@dataclass(frozen=True)
class GroupSpec:
    name: str
    title: str
    variable_name: str
    variable_unit: str
    conditions: tuple[ConditionSpec, ...]
    config_paths: tuple[str, ...]


GROUPS: dict[str, GroupSpec] = {
    "mpc_vx_weight": GroupSpec(
        name="mpc_vx_weight",
        title="MPC 前进速度误差权重",
        variable_name="state_weights[9]",
        variable_unit="1",
        conditions=(
            ConditionSpec("low", ("low",), "低权重 0.2", 0.2),
            ConditionSpec("baseline", ("baseline",), "基线 1.0", 1.0),
            ConditionSpec("high", ("high",), "高权重 5.0", 5.0),
        ),
        config_paths=(
            "Pro2/experiments/L2/mpc_vx_weight/low.json",
            "Pro2/experiments/L2/mpc_vx_weight/baseline.json",
            "Pro2/experiments/L2/mpc_vx_weight/high.json",
        ),
    ),
    "prediction_horizon": GroupSpec(
        name="prediction_horizon",
        title="MPC 预测时域",
        variable_name="mpc_horizon_steps",
        variable_unit="step",
        conditions=(
            ConditionSpec("short", ("short_n5", "short", "short_5", "short_005"), "短时域 N=5", 5.0),
            ConditionSpec("baseline", ("baseline_n10", "baseline", "baseline_10", "baseline_010"), "基线时域 N=10", 10.0),
            ConditionSpec("long", ("long_n15", "long", "long_15", "long_015"), "长时域 N=15", 15.0),
        ),
        config_paths=(
            "Pro2/experiments/L2/prediction_horizon/short_n5.json",
            "Pro2/experiments/L2/prediction_horizon/baseline_n10.json",
            "Pro2/experiments/L2/prediction_horizon/long_n15.json",
        ),
    ),
    "step_height": GroupSpec(
        name="step_height",
        title="摆动脚抬腿高度",
        variable_name="step_height",
        variable_unit="m",
        conditions=(
            ConditionSpec("low_008", ("low_008", "low"), "低抬腿 0.08 m", 0.08),
            ConditionSpec("baseline_012", ("baseline_012", "baseline"), "基线 0.12 m", 0.12),
            ConditionSpec("high_016", ("high_016", "high"), "高抬腿 0.16 m", 0.16),
        ),
        config_paths=(
            "Pro2/experiments/L2/step_height/low_008.json",
            "Pro2/experiments/L2/step_height/baseline_012.json",
            "Pro2/experiments/L2/step_height/high_016.json",
        ),
    ),
    "lateral_push": GroupSpec(
        name="lateral_push",
        title="侧向外力扰动",
        variable_name="push_force_y",
        variable_unit="N",
        conditions=(
            ConditionSpec("no_push", ("no_push", "baseline"), "无扰动 0 N", 0.0),
            ConditionSpec("push_50N", ("push_50N", "push_50n", "medium"), "侧推 50 N", 50.0),
            ConditionSpec("push_100N", ("push_100N", "push_100n", "high"), "侧推 100 N", 100.0),
        ),
        config_paths=(
            "Pro2/experiments/L2/lateral_push/no_push.json",
            "Pro2/experiments/L2/lateral_push/push_50N.json",
            "Pro2/experiments/L2/lateral_push/push_100N.json",
        ),
    ),
}


METRIC_FIELDS = [
    "experiment",
    "condition",
    "condition_label",
    "run_directory",
    "variable_name",
    "variable_value",
    "variable_unit",
    "variable_value_source",
    "mpc_dt_s",
    "prediction_horizon_steps",
    "prediction_horizon_s",
    "run_status",
    "run_outcome",
    "planned_duration_s",
    "actual_duration_s",
    "physical_instability_time_s",
    "termination_reason",
    "data_status",
    "acceptance_validation_status",
    "acceptance_validation_issues",
    "sample_count_total",
    "sample_count_analysis",
    "time_start_s",
    "time_end_s",
    "analysis_start_s",
    "analysis_end_s",
    "vx_external_command_rmse_m_s",
    "vx_internal_reference_rmse_m_s",
    "vy_internal_reference_rmse_m_s",
    "roll_rms_rad",
    "roll_max_abs_rad",
    "pitch_rms_rad",
    "pitch_max_abs_rad",
    "max_tilt_rad",
    "base_height_mean_m",
    "base_height_std_m",
    "base_height_min_m",
    "joint_torque_per_joint_rms_Nm",
    "joint_torque_vector_rms_Nm",
    "joint_torque_max_abs_Nm",
    "joint_torque_squared_integral_Nm2_s",
    "absolute_mechanical_work_proxy_J",
    "left_force_z_est_mean_N",
    "left_force_z_est_peak_N",
    "right_force_z_est_mean_N",
    "right_force_z_est_peak_N",
    "mpc_solve_mean_ms",
    "mpc_solve_p95_ms",
    "mpc_solve_max_ms",
    "wbc_solve_mean_ms",
    "wbc_solve_p95_ms",
    "wbc_solve_max_ms",
    "controller_step_mean_ms",
    "controller_step_p95_ms",
    "controller_step_max_ms",
    "mpc_qp_nonzero_count",
    "wbc_qp_nonzero_count",
    "push_at_s",
    "push_duration_s",
    "push_end_s",
    "push_impulse_y_N_s",
    "logged_push_impulse_y_N_s",
    "logged_push_nonzero_sample_count",
    "logged_push_first_nonzero_s",
    "logged_push_last_nonzero_s",
    "pre_push_roll_mean_rad",
    "pre_push_pitch_mean_rad",
    "pre_push_vx_body_mean_m_s",
    "pre_push_vy_body_mean_m_s",
    "post_push_roll_peak_deviation_rad",
    "post_push_pitch_peak_deviation_rad",
    "post_push_vx_peak_deviation_m_s",
    "post_push_vy_peak_deviation_m_s",
    "post_push_base_height_min_m",
    "post_push_joint_torque_peak_Nm",
    "post_push_left_force_z_est_peak_N",
    "post_push_right_force_z_est_peak_N",
    "push_response_window_start_s",
    "push_response_window_end_s",
    "aligned_no_push_window_start_s",
    "aligned_no_push_window_end_s",
    "aligned_no_push_sample_count",
    "aligned_no_push_roll_max_abs_delta_rad",
    "aligned_no_push_pitch_max_abs_delta_rad",
    "aligned_no_push_vx_max_abs_delta_m_s",
    "aligned_no_push_vy_max_abs_delta_m_s",
    "aligned_no_push_extra_height_drop_max_m",
    "recovery_time_s",
    "recovery_observation_end_s",
    "recovered_in_window",
    "recovery_criterion_applicable",
    "recovery_result_interpretation",
]


@dataclass
class RunData:
    spec: GroupSpec
    condition: ConditionSpec
    directory: Path | None
    run: dict[str, Any] = field(default_factory=dict)
    columns: dict[str, list[float]] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)
    validation_issues: list[str] = field(default_factory=list)
    expected_config: dict[str, Any] = field(default_factory=dict)
    analysis_end_override_s: float | None = None

    @property
    def has_samples(self) -> bool:
        return bool(self.columns.get("time_s"))

    @property
    def label(self) -> str:
        return self.condition.label


@dataclass(frozen=True)
class Series:
    label: str
    x: Sequence[float]
    y: Sequence[float]
    color: tuple[int, int, int]
    dashed: bool = False


@dataclass(frozen=True)
class Panel:
    title: str
    x_label: str
    y_label: str
    series: tuple[Series, ...]
    x_limits: tuple[float, float] | None = None
    y_tick_decimals: int | None = None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="从四类 L2 仿真输出计算指标、绘制对比图并生成分析 Markdown。"
    )
    parser.add_argument(
        "--l2-root",
        type=Path,
        default=DEFAULT_L2_ROOT,
        help="L2 输出根目录（默认 Pro2/output/L2）。",
    )
    parser.add_argument(
        "--group",
        action="append",
        choices=tuple(GROUPS),
        help="只分析指定实验；可重复传入。默认分析四类。",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "严格核对每次运行已完成、12000 步/1200 样本/12 s、CSV 100 Hz 时间序列、"
            "全部数值有限及 run.json 一致性；active_runs.json 显式允许的物理失稳提前停止按真实时长校验。"
            "任一数据检查失败即返回非零状态。"
        ),
    )
    return parser.parse_args(argv)


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def number(value: Any, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def valid(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values if finite(value)]


def mean(values: Iterable[float]) -> float:
    data = valid(values)
    return sum(data) / len(data) if data else math.nan


def rms(values: Iterable[float]) -> float:
    data = valid(values)
    return math.sqrt(sum(value * value for value in data) / len(data)) if data else math.nan


def standard_deviation(values: Iterable[float]) -> float:
    data = valid(values)
    if not data:
        return math.nan
    center = sum(data) / len(data)
    return math.sqrt(sum((value - center) ** 2 for value in data) / len(data))


def maximum(values: Iterable[float]) -> float:
    data = valid(values)
    return max(data) if data else math.nan


def minimum(values: Iterable[float]) -> float:
    data = valid(values)
    return min(data) if data else math.nan


def max_abs(values: Iterable[float]) -> float:
    data = valid(values)
    return max((abs(value) for value in data), default=math.nan)


def percentile(values: Iterable[float], fraction: float) -> float:
    data = sorted(valid(values))
    if not data:
        return math.nan
    position = (len(data) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return data[lower]
    alpha = position - lower
    return data[lower] * (1.0 - alpha) + data[upper] * alpha


def rmse_pairs(values: Sequence[float], references: Sequence[float], indices: Iterable[int]) -> float:
    errors = []
    for index in indices:
        if index < len(values) and index < len(references):
            value, reference = values[index], references[index]
            if finite(value) and finite(reference):
                errors.append(float(value) - float(reference))
    return rms(errors)


def trapezoid(times: Sequence[float], values: Sequence[float], indices: Sequence[int]) -> float:
    total = 0.0
    used = False
    previous: tuple[float, float] | None = None
    for index in indices:
        if index >= len(times) or index >= len(values):
            continue
        t, value = times[index], values[index]
        if not finite(t) or not finite(value):
            previous = None
            continue
        pair = float(t), float(value)
        if previous is not None and pair[0] >= previous[0]:
            total += 0.5 * (previous[1] + pair[1]) * (pair[0] - previous[0])
            used = True
        previous = pair
    return total if used else math.nan


def slice_values(values: Sequence[float], indices: Iterable[int]) -> list[float]:
    return [values[index] for index in indices if index < len(values)]


def indices_between(times: Sequence[float], start: float, end: float | None = None) -> list[int]:
    result = []
    for index, value in enumerate(times):
        if not finite(value):
            continue
        if value >= start and (end is None or value < end):
            result.append(index)
    return result


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def load_states(path: Path) -> dict[str, list[float]]:
    columns: dict[str, list[float]] = {}
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames:
                return {}
            columns = {name: [] for name in reader.fieldnames}
            for row in reader:
                if not finite(number(row.get("time_s"))):
                    continue
                for name in reader.fieldnames:
                    columns[name].append(number(row.get(name)))
    except (OSError, csv.Error):
        return {}
    return columns


def close_to(value: Any, expected: float, tolerance: float = 1e-9) -> bool:
    """Return whether *value* is a finite number close to *expected*."""

    parsed = number(value)
    return finite(parsed) and math.isclose(parsed, expected, rel_tol=0.0, abs_tol=tolerance)


def physical_instability_stop(run: RunData) -> bool:
    """Only accept the explicitly opted-in, recorded physical safety threshold."""

    if (run.spec.name != "lateral_push" or not run.expected_config.get("_accept_physical_instability")
            or run.run.get("status") != "failed"):
        return False
    match = re.fullmatch(r"t=([\d.]+)s：机器人失稳，z=([-\d.]+)m，roll=([-\d.]+)，pitch=([-\d.]+)",
                         str(run.run.get("error", "")))
    if match is None:
        return False
    stopped_at, height, roll, pitch = map(float, match.groups())
    return (
        close_to(run.run.get("simulation_duration_s"), stopped_at, 0.00051)
        and (height <= 0.600 or max(abs(roll), abs(pitch)) >= 1.000)
        and 0 < stopped_at < EXPECTED_DURATION_S
    )


def validate_acceptance_run(run: RunData) -> list[str]:
    """Validate one run against the fixed L2 acceptance data contract.

    This check deliberately reads ``states.csv`` again instead of relying only
    on :func:`load_states`: the normal loader skips a row whose time is invalid,
    whereas strict acceptance must report the malformed row rather than hide
    it.  The returned strings are short enough to store in ``metrics.csv`` and
    specific enough to diagnose a failed ``--strict`` run.
    """

    if run.directory is None:
        return ["运行目录缺失"]

    issues: list[str] = []
    manifest = run.run
    config = get_config(manifest)
    physical_stop = physical_instability_stop(run)
    actual_duration = number(manifest.get("simulation_duration_s"))
    expected_steps = (round(actual_duration / 0.001)
                      if physical_stop else EXPECTED_STEPS)
    expected_samples = math.ceil(expected_steps / 10)

    if manifest.get("status") != "completed" and not physical_stop:
        issues.append(f"run.json.status={manifest.get('status')!r}，应为 'completed'")
    if number(manifest.get("steps")) != expected_steps:
        issues.append(f"run.json.steps={manifest.get('steps')!r}，应为 {expected_steps}")
    if number(manifest.get("samples")) != expected_samples:
        issues.append(f"run.json.samples={manifest.get('samples')!r}，应为 {expected_samples}")
    if not physical_stop and not close_to(manifest.get("simulation_duration_s"), EXPECTED_DURATION_S):
        issues.append(
            f"run.json.simulation_duration_s={manifest.get('simulation_duration_s')!r}，应为 {EXPECTED_DURATION_S:g}"
        )

    if not close_to(config.get("duration"), EXPECTED_DURATION_S):
        issues.append(f"config.duration={config.get('duration')!r}，应为 {EXPECTED_DURATION_S:g}")
    if not close_to(config.get("dt"), EXPECTED_DURATION_S / EXPECTED_STEPS):
        issues.append(
            f"config.dt={config.get('dt')!r}，应为 {EXPECTED_DURATION_S / EXPECTED_STEPS:g}"
        )
    if not close_to(config.get("log_hz"), 1.0 / EXPECTED_LOG_PERIOD_S):
        issues.append(f"config.log_hz={config.get('log_hz')!r}，应为 {1.0 / EXPECTED_LOG_PERIOD_S:g}")

    duration = number(config.get("duration"))
    sim_dt = number(config.get("dt"))
    log_hz = number(config.get("log_hz"))
    if finite(duration) and finite(sim_dt) and sim_dt > 0:
        expected_steps_from_config = (actual_duration if physical_stop else duration) / sim_dt
        if not close_to(manifest.get("steps"), expected_steps_from_config, 1e-6):
            issues.append("run.json.steps 与实际时长/config.dt 不一致")
    if finite(duration) and finite(log_hz) and log_hz > 0:
        expected_samples_from_config = math.ceil(actual_duration * log_hz - 1e-9) if physical_stop else duration * log_hz
        if not close_to(manifest.get("samples"), expected_samples_from_config, 1e-6):
            issues.append("run.json.samples 与实际运行范围和 config.log_hz 不一致")
    if not physical_stop and finite(duration) and not close_to(manifest.get("simulation_duration_s"), duration):
        issues.append("run.json.simulation_duration_s 与 config.duration 不一致")

    config_horizon = number(config.get("mpc_horizon_steps"))
    manifest_horizon = number(manifest.get("mpc_horizon_steps"))
    mpc_dt = number(config.get("mpc_dt"))
    if not finite(config_horizon) or not finite(manifest_horizon):
        issues.append("MPC 预测步数未同时记录在 config 与 run.json 顶层")
    elif not math.isclose(config_horizon, manifest_horizon, rel_tol=0.0, abs_tol=1e-12):
        issues.append("run.json.mpc_horizon_steps 与 config.mpc_horizon_steps 不一致")
    if finite(config_horizon) and finite(mpc_dt):
        expected_prediction_s = config_horizon * mpc_dt
        if not close_to(manifest.get("mpc_prediction_horizon_s"), expected_prediction_s):
            issues.append("run.json.mpc_prediction_horizon_s 与 N*mpc_dt 不一致")
    control_steps = number(manifest.get("mpc_control_horizon_steps"))
    if control_steps != 3:
        issues.append("run.json.mpc_control_horizon_steps 应为固定值 3")
    if finite(control_steps) and finite(mpc_dt):
        if not close_to(manifest.get("mpc_control_horizon_s"), control_steps * mpc_dt):
            issues.append("run.json.mpc_control_horizon_s 与 ch*mpc_dt 不一致")
    if run.spec.name == "lateral_push":
        for name, fallback in (("push_at", DEFAULT_PUSH_AT_S), ("push_duration", DEFAULT_PUSH_DURATION_S)):
            expected = number(run.expected_config.get(name), fallback)
            if not close_to(config.get(name), expected):
                issues.append(f"config.{name} 应为本批验收值 {expected:g} s")
        if physical_stop and actual_duration <= number(config.get("push_at")) + number(config.get("push_duration")):
            issues.append("物理失稳发生在完整施力脉冲结束前，不能作为本批完整冲量对照")

    states_path = run.directory / "states.csv"
    try:
        with states_path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            raw_rows = list(reader)
    except (OSError, csv.Error) as exc:
        issues.append(f"states.csv 无法读取：{exc}")
        return issues

    if len(raw_rows) != expected_samples:
        issues.append(f"states.csv 数据行={len(raw_rows)}，应为 {expected_samples}")
    if len(fieldnames) != len(set(fieldnames)):
        issues.append("states.csv 存在重复列名")
    if finite(number(manifest.get("samples"))) and number(manifest.get("samples")) != len(raw_rows):
        issues.append("run.json.samples 与 states.csv 数据行数不一致")

    torque_names = sorted(name for name in fieldnames if name.startswith("tau_nm:"))
    velocity_names = sorted(name for name in fieldnames if name.startswith("dq_rad_s:"))
    position_names = sorted(name for name in fieldnames if name.startswith("q_rad:"))
    missing = [name for name in REQUIRED_NUMERIC_COLUMNS if name not in fieldnames]
    if not torque_names:
        missing.append("tau_nm:*")
    if not velocity_names:
        missing.append("dq_rad_s:*")
    if missing:
        issues.append("states.csv 缺少必要列：" + ", ".join(missing))

    torque_joints = {name.split(":", 1)[1] for name in torque_names}
    velocity_joints = {name.split(":", 1)[1] for name in velocity_names}
    position_joints = {name.split(":", 1)[1] for name in position_names}
    if torque_joints != velocity_joints or torque_joints != position_joints:
        issues.append("states.csv 的 q_rad、dq_rad_s 与 tau_nm 关节集合不一致")
    manifest_joints = manifest.get("joint_names")
    if not isinstance(manifest_joints, list) or set(manifest_joints) != torque_joints:
        issues.append("run.json.joint_names 与 states.csv 关节列不一致")
    manifest_indices = manifest.get("joint_indices")
    if not isinstance(manifest_indices, list) or len(manifest_indices) != len(torque_joints):
        issues.append("run.json.joint_indices 数量与 states.csv 关节数不一致")

    invalid_cells: list[str] = []
    times: list[float] = []
    for row_number, row in enumerate(raw_rows, start=2):
        parsed_time = number(row.get("time_s"))
        times.append(parsed_time)
        # 当前 states.csv 的全部字段均为数值型控制/状态量。严格模式
        # 对整行做 finite 检查，因而不会漏掉尚未用于汇总但属于验收证据的
        # 关节位置、外力或控制器状态字段。
        for name in fieldnames:
            if not finite(number(row.get(name))):
                if len(invalid_cells) < 8:
                    invalid_cells.append(f"{name}@CSV第{row_number}行")
    if invalid_cells:
        suffix = "（仅列前 8 处）" if len(invalid_cells) == 8 else ""
        issues.append("必要数值存在非有限值：" + ", ".join(invalid_cells) + suffix)

    if run.spec.name == "lateral_push":
        push_at = number(config.get("push_at"))
        push_end = push_at + number(config.get("push_duration"))
        force = config.get("push_force", [])
        force_columns = ("force_x_n", "force_y_n", "force_z_n")
        if len(force) != 3 or any(name not in fieldnames for name in force_columns):
            issues.append("侧推配置或 CSV 缺少三轴外力记录")
        else:
            force_mismatches = []
            for row_number, row in enumerate(raw_rows, start=2):
                t = number(row.get("time_s"))
                active = push_at <= t < push_end
                if any(not close_to(row.get(name), number(force[axis]) if active else 0.0)
                       for axis, name in enumerate(force_columns)):
                    force_mismatches.append(row_number)
            if force_mismatches:
                issues.append("CSV 外力与配置施力区间不一致，异常行：" + ", ".join(map(str, force_mismatches[:8])))

    if times:
        if not close_to(times[0], 0.0, TIME_ABS_TOL_S):
            issues.append(f"states.csv 首个 time_s={times[0]!r}，应约为 0")
        expected_last = (expected_samples - 1) * EXPECTED_LOG_PERIOD_S
        if not close_to(times[-1], expected_last, TIME_ABS_TOL_S):
            issues.append(f"states.csv 末个 time_s={times[-1]!r}，应约为 {expected_last:g}")
        bad_order = []
        bad_period = []
        for index in range(1, len(times)):
            previous, current = times[index - 1], times[index]
            if not finite(previous) or not finite(current):
                continue
            delta = current - previous
            if delta <= 0:
                bad_order.append(index + 2)
            elif not math.isclose(delta, EXPECTED_LOG_PERIOD_S, rel_tol=0.0, abs_tol=TIME_ABS_TOL_S):
                bad_period.append((index + 2, delta))
        if bad_order:
            issues.append(
                "time_s 非严格递增，异常 CSV 行：" + ", ".join(map(str, bad_order[:8]))
            )
        if bad_period:
            preview = ", ".join(f"第{row}行 Δt={delta:g}" for row, delta in bad_period[:8])
            issues.append(f"time_s 采样间隔不约为 0.01 s：{preview}")

    loaded_count = len(run.columns.get("time_s", []))
    if loaded_count != len(raw_rows):
        issues.append("分析器载入样本数与 states.csv 数据行数不一致（可能存在非法 time_s）")
    return issues


def find_run_directory(group_dir: Path, condition: ConditionSpec) -> Path | None:
    candidates: list[Path] = []
    for name in condition.directory_names:
        candidate = group_dir / name
        if candidate.is_dir():
            candidates.append(candidate)
        # Permit informative or retry suffixes such as short_n5_retry1 while
        # keeping the canonical condition mapping deterministic.
        matches = sorted(path for path in group_dir.glob(name + "*") if path.is_dir())
        matches = [path for path in matches if path.name != "comparison"]
        candidates.extend(matches)
    candidates = list(dict.fromkeys(candidates))
    if not candidates:
        return None

    exact_names = set(condition.directory_names)

    def rank(path: Path) -> tuple[int, int, int, float]:
        manifest = load_json(path / "run.json")
        completed = int(manifest.get("status") == "completed")
        has_states = int((path / "states.csv").is_file())
        exact = int(path.name in exact_names)
        try:
            modified = path.stat().st_mtime
        except OSError:
            modified = 0.0
        return completed, has_states, exact, modified

    return max(candidates, key=rank)


def get_config(run: dict[str, Any]) -> dict[str, Any]:
    config = run.get("config", {})
    return config if isinstance(config, dict) else {}


def nested_candidates(run: dict[str, Any], keys: Sequence[str]) -> tuple[float, str] | None:
    config = get_config(run)
    locations = ((config, "run.json:config"), (run, "run.json"))
    for container, prefix in locations:
        for key in keys:
            value = container.get(key)
            if finite(number(value)):
                return float(value), f"{prefix}.{key}"
    return None


def variable_value(run: RunData) -> tuple[float, str]:
    config = get_config(run.run)
    if run.spec.name == "mpc_vx_weight":
        weights = config.get("state_weights")
        if isinstance(weights, list) and len(weights) > 9 and finite(number(weights[9])):
            return float(weights[9]), "run.json:config.state_weights[9]"
    elif run.spec.name == "step_height":
        found = nested_candidates(run.run, ("step_height",))
        if found:
            return found
    elif run.spec.name == "lateral_push":
        force = config.get("push_force")
        if isinstance(force, list) and len(force) > 1 and finite(number(force[1])):
            return float(force[1]), "run.json:config.push_force[1]"
    elif run.spec.name == "prediction_horizon":
        # Keep compatibility with both the current planned name and sensible
        # names used by older/generated manifests.
        found = nested_candidates(
            run.run,
            (
                "mpc_horizon_steps",
                "prediction_horizon_steps",
                "mpc_N",
                "mpc_n",
            ),
        )
        if found:
            return found
        seconds = nested_candidates(run.run, ("prediction_horizon_s", "mpc_horizon_s"))
        dt = nested_candidates(run.run, ("mpc_dt",))
        if seconds and dt and dt[0] > 0:
            return seconds[0] / dt[0], f"derived from {seconds[1]} / {dt[1]}"
    return run.condition.fallback_value, "condition-name fallback (verify run.json)"


def derive_signals(columns: dict[str, list[float]]) -> None:
    count = len(columns.get("time_s", []))
    yaw = columns.get("yaw_rad", [math.nan] * count)
    vx_world = columns.get("vx_world_m_s", [math.nan] * count)
    vy_world = columns.get("vy_world_m_s", [math.nan] * count)
    vx_body, vy_body = [], []
    for index in range(count):
        values = (
            yaw[index] if index < len(yaw) else math.nan,
            vx_world[index] if index < len(vx_world) else math.nan,
            vy_world[index] if index < len(vy_world) else math.nan,
        )
        if not all(finite(value) for value in values):
            vx_body.append(math.nan)
            vy_body.append(math.nan)
            continue
        psi, vx, vy = values
        vx_body.append(math.cos(psi) * vx + math.sin(psi) * vy)
        vy_body.append(-math.sin(psi) * vx + math.cos(psi) * vy)
    columns["derived:vx_body_m_s"] = vx_body
    columns["derived:vy_body_m_s"] = vy_body

    torque_names = [name for name in columns if name.startswith("tau_nm:")]
    velocity_names = [name for name in columns if name.startswith("dq_rad_s:")]
    torque_sq_sum, torque_abs_max, abs_power = [], [], []
    for index in range(count):
        torques = [columns[name][index] for name in torque_names if index < len(columns[name])]
        torques = valid(torques)
        torque_sq_sum.append(sum(value * value for value in torques) if torques else math.nan)
        torque_abs_max.append(max((abs(value) for value in torques), default=math.nan))
        power = 0.0
        used = False
        for torque_name in torque_names:
            joint = torque_name.split(":", 1)[1]
            dq_name = "dq_rad_s:" + joint
            if dq_name not in columns or index >= len(columns[torque_name]) or index >= len(columns[dq_name]):
                continue
            torque, dq = columns[torque_name][index], columns[dq_name][index]
            if finite(torque) and finite(dq):
                power += abs(torque * dq)
                used = True
        abs_power.append(power if used else math.nan)
    columns["derived:torque_sq_sum_Nm2"] = torque_sq_sum
    columns["derived:torque_abs_max_Nm"] = torque_abs_max
    columns["derived:absolute_mechanical_power_W"] = abs_power


def calculate_recovery(
    columns: dict[str, list[float]], push_at: float, push_end: float, analysis_end: float
) -> tuple[float, bool]:
    times = columns.get("time_s", [])
    vx = columns.get("derived:vx_body_m_s", [])
    vy = columns.get("derived:vy_body_m_s", [])
    vx_ref = columns.get("controller:command_vx_mps", columns.get("cmd_vx_body_m_s", []))
    vy_ref = columns.get("controller:command_vy_mps", columns.get("cmd_vy_body_m_s", []))
    roll = columns.get("roll_rad", [])
    pitch = columns.get("pitch_rad", [])
    baseline_indices = indices_between(times, push_at - 1.0, push_at)
    roll_ref = mean(slice_values(roll, baseline_indices))
    pitch_ref = mean(slice_values(pitch, baseline_indices))
    if not finite(roll_ref) or not finite(pitch_ref):
        return math.nan, False

    post_indices = indices_between(times, push_end, analysis_end + 1e-12)
    start_time: float | None = None
    previous_time: float | None = None
    nominal_dt = percentile(
        [times[i] - times[i - 1] for i in range(1, len(times)) if finite(times[i]) and finite(times[i - 1]) and times[i] > times[i - 1]],
        0.5,
    )
    for index in post_indices:
        required = (vx, vy, vx_ref, vy_ref, roll, pitch)
        if any(index >= len(values) for values in required):
            start_time = None
            previous_time = None
            continue
        values = tuple(values[index] for values in required)
        ok = all(finite(value) for value in values)
        if ok:
            vx_i, vy_i, vx_ref_i, vy_ref_i, roll_i, pitch_i = values
            ok = (
                abs(vx_i - vx_ref_i) <= RECOVERY_SPEED_TOL_M_S
                and abs(vy_i - vy_ref_i) <= RECOVERY_SPEED_TOL_M_S
                and abs(roll_i - roll_ref) <= RECOVERY_ANGLE_TOL_RAD
                and abs(pitch_i - pitch_ref) <= RECOVERY_ANGLE_TOL_RAD
            )
        t = times[index]
        if ok:
            # A large logging gap must not count as continuous recovery.
            if (
                start_time is None
                or previous_time is None
                or (finite(nominal_dt) and t - previous_time > max(0.05, 2.5 * nominal_dt))
            ):
                start_time = t
            previous_time = t
            if t - start_time + (nominal_dt if finite(nominal_dt) else 0.0) >= RECOVERY_HOLD_S:
                return max(0.0, start_time - push_end), True
        else:
            start_time = None
            previous_time = None
    return math.nan, False


def compute_metrics(run: RunData) -> dict[str, Any]:
    variable, source = variable_value(run)
    mpc_dt_entry = nested_candidates(run.run, ("mpc_dt",))
    horizon_entry = nested_candidates(
        run.run,
        ("mpc_horizon_steps", "prediction_horizon_steps", "mpc_N", "mpc_n"),
    )
    mpc_dt = mpc_dt_entry[0] if mpc_dt_entry else math.nan
    horizon_steps = horizon_entry[0] if horizon_entry else (
        variable if run.spec.name == "prediction_horizon" else math.nan
    )
    directory_text = "" if run.directory is None else project_relative(run.directory)
    metrics: dict[str, Any] = {name: "" for name in METRIC_FIELDS}
    metrics.update(
        experiment=run.spec.name,
        condition=run.condition.name,
        condition_label=run.condition.label,
        run_directory=directory_text,
        variable_name=run.spec.variable_name,
        variable_value=variable,
        variable_unit=run.spec.variable_unit,
        variable_value_source=source,
        mpc_dt_s=mpc_dt,
        prediction_horizon_steps=horizon_steps,
        prediction_horizon_s=mpc_dt * horizon_steps
        if finite(mpc_dt) and finite(horizon_steps)
        else math.nan,
        run_status=run.run.get("status", "missing" if run.directory is None else "unknown"),
        run_outcome="physical_instability" if physical_instability_stop(run) else run.run.get("status", "missing"),
        planned_duration_s=number(get_config(run.run).get("duration")),
        actual_duration_s=number(run.run.get("simulation_duration_s")),
        physical_instability_time_s=number(run.run.get("simulation_duration_s")) if physical_instability_stop(run) else math.nan,
        termination_reason=run.run.get("error", ""),
        acceptance_validation_status="ok" if not run.validation_issues else "failed",
        acceptance_validation_issues=" | ".join(run.validation_issues),
    )
    if run.directory is None:
        metrics["data_status"] = "missing_run_directory"
        return metrics
    if not run.columns.get("time_s"):
        metrics["data_status"] = "missing_or_empty_states_csv"
        return metrics

    columns = run.columns
    times = columns["time_s"]
    actual_log_end = maximum(times)
    analysis_end = min(actual_log_end, run.analysis_end_override_s) if run.analysis_end_override_s is not None else actual_log_end
    analysis_indices = indices_between(times, STEADY_START_S, analysis_end + 1e-12)
    required_columns = REQUIRED_NUMERIC_COLUMNS[1:]
    missing_columns = [name for name in required_columns if name not in columns]
    if not any(name.startswith("tau_nm:") for name in columns):
        missing_columns.append("tau_nm:*")
    if not any(name.startswith("dq_rad_s:") for name in columns):
        missing_columns.append("dq_rad_s:*")
    config = get_config(run.run)
    expected_duration = number(config.get("duration"))
    log_hz = number(config.get("log_hz"), 100.0)
    end_tolerance = max(0.05, 2.0 / log_hz) if finite(log_hz) and log_hz > 0 else 0.05
    if missing_columns:
        data_status = "missing_columns:" + ";".join(missing_columns)
    elif not analysis_indices:
        data_status = "no_samples_at_or_after_5s"
    elif not physical_instability_stop(run) and finite(expected_duration) and actual_log_end < expected_duration - end_tolerance:
        data_status = "incomplete_states_csv"
    else:
        data_status = "ok"
    metrics.update(
        data_status=data_status,
        sample_count_total=len(times),
        sample_count_analysis=len(analysis_indices),
        time_start_s=minimum(times),
        time_end_s=actual_log_end,
        analysis_start_s=STEADY_START_S,
        analysis_end_s=analysis_end,
    )
    if data_status != "ok":
        return metrics

    derive_signals(columns)

    vx = columns.get("derived:vx_body_m_s", [])
    vy = columns.get("derived:vy_body_m_s", [])
    external_vx = columns.get("cmd_vx_body_m_s", [])
    internal_vx = columns.get("controller:command_vx_mps", external_vx)
    internal_vy = columns.get("controller:command_vy_mps", columns.get("cmd_vy_body_m_s", []))
    roll = columns.get("roll_rad", [])
    pitch = columns.get("pitch_rad", [])
    base_height = columns.get("base_z_m", columns.get("controller:base_height_m", []))
    torque_sq_sum = columns.get("derived:torque_sq_sum_Nm2", [])
    torque_max = columns.get("derived:torque_abs_max_Nm", [])
    abs_power = columns.get("derived:absolute_mechanical_power_W", [])
    torque_names = [name for name in columns if name.startswith("tau_nm:")]
    joint_count = max(1, len(torque_names))

    roll_values = slice_values(roll, analysis_indices)
    pitch_values = slice_values(pitch, analysis_indices)
    torque_sq_values = slice_values(torque_sq_sum, analysis_indices)
    metrics.update(
        vx_external_command_rmse_m_s=rmse_pairs(vx, external_vx, analysis_indices),
        vx_internal_reference_rmse_m_s=rmse_pairs(vx, internal_vx, analysis_indices),
        vy_internal_reference_rmse_m_s=rmse_pairs(vy, internal_vy, analysis_indices),
        roll_rms_rad=rms(roll_values),
        roll_max_abs_rad=max_abs(roll_values),
        pitch_rms_rad=rms(pitch_values),
        pitch_max_abs_rad=max_abs(pitch_values),
        max_tilt_rad=max(max_abs(roll_values), max_abs(pitch_values)),
        base_height_mean_m=mean(slice_values(base_height, analysis_indices)),
        base_height_std_m=standard_deviation(slice_values(base_height, analysis_indices)),
        base_height_min_m=minimum(slice_values(base_height, analysis_indices)),
        joint_torque_per_joint_rms_Nm=math.sqrt(mean(torque_sq_values) / joint_count)
        if finite(mean(torque_sq_values))
        else math.nan,
        joint_torque_vector_rms_Nm=math.sqrt(mean(torque_sq_values))
        if finite(mean(torque_sq_values))
        else math.nan,
        joint_torque_max_abs_Nm=max_abs(slice_values(torque_max, analysis_indices)),
        joint_torque_squared_integral_Nm2_s=trapezoid(times, torque_sq_sum, analysis_indices),
        absolute_mechanical_work_proxy_J=trapezoid(times, abs_power, analysis_indices),
    )

    force_columns = (
        ("left", "controller:left_force_z_est_N"),
        ("right", "controller:right_force_z_est_N"),
    )
    for side, name in force_columns:
        values = slice_values(columns.get(name, []), analysis_indices)
        metrics[f"{side}_force_z_est_mean_N"] = mean(values)
        metrics[f"{side}_force_z_est_peak_N"] = maximum(values)

    timing_columns = (
        ("mpc_solve", "controller:mpc_solve_s"),
        ("wbc_solve", "controller:wbc_solve_s"),
        ("controller_step", "controller:controller_step_s"),
    )
    for prefix, name in timing_columns:
        values_ms = [value * 1000.0 for value in slice_values(columns.get(name, []), analysis_indices) if finite(value)]
        metrics[f"{prefix}_mean_ms"] = mean(values_ms)
        metrics[f"{prefix}_p95_ms"] = percentile(values_ms, 0.95)
        metrics[f"{prefix}_max_ms"] = maximum(values_ms)
    for prefix, name in (
        ("mpc_qp", "controller:mpc_qp_status"),
        ("wbc_qp", "controller:wbc_qp_status"),
    ):
        values = slice_values(columns.get(name, []), analysis_indices)
        metrics[f"{prefix}_nonzero_count"] = sum(1 for value in values if finite(value) and int(round(value)) != 0)

    if run.spec.name == "lateral_push":
        config = get_config(run.run)
        push_at = number(config.get("push_at"), DEFAULT_PUSH_AT_S)
        duration = number(config.get("push_duration"), DEFAULT_PUSH_DURATION_S)
        push_end = push_at + max(0.0, duration)
        baseline_indices = indices_between(times, push_at - 1.0, push_at)
        # The response peak uses one fixed 2 s transient window beginning when
        # the force is switched on.  It therefore includes the configured
        # force-application interval.  Recovery timing starts only after
        # the force has ended and may use the remainder of the log.
        response_end = min(analysis_end, push_at + PUSH_TRANSIENT_WINDOW_S)
        post_indices = indices_between(times, push_at, response_end + 1e-12)
        references = {
            "roll": mean(slice_values(roll, baseline_indices)),
            "pitch": mean(slice_values(pitch, baseline_indices)),
            "vx": mean(slice_values(vx, baseline_indices)),
            "vy": mean(slice_values(vy, baseline_indices)),
        }
        metrics.update(
            push_at_s=push_at,
            push_duration_s=duration,
            push_end_s=push_end,
            push_impulse_y_N_s=number(config.get("push_force", [0, 0, 0])[1]) * duration,
            pre_push_roll_mean_rad=references["roll"],
            pre_push_pitch_mean_rad=references["pitch"],
            pre_push_vx_body_mean_m_s=references["vx"],
            pre_push_vy_body_mean_m_s=references["vy"],
            post_push_roll_peak_deviation_rad=max_abs(
                [roll[i] - references["roll"] for i in post_indices if i < len(roll) and finite(references["roll"])]
            ),
            post_push_pitch_peak_deviation_rad=max_abs(
                [pitch[i] - references["pitch"] for i in post_indices if i < len(pitch) and finite(references["pitch"])]
            ),
            post_push_vx_peak_deviation_m_s=max_abs(
                [vx[i] - references["vx"] for i in post_indices if i < len(vx) and finite(references["vx"])]
            ),
            post_push_vy_peak_deviation_m_s=max_abs(
                [vy[i] - references["vy"] for i in post_indices if i < len(vy) and finite(references["vy"])]
            ),
            post_push_base_height_min_m=minimum(slice_values(base_height, post_indices)),
            post_push_joint_torque_peak_Nm=max_abs(slice_values(torque_max, post_indices)),
            post_push_left_force_z_est_peak_N=maximum(
                slice_values(columns.get("controller:left_force_z_est_N", []), post_indices)
            ),
            post_push_right_force_z_est_peak_N=maximum(
                slice_values(columns.get("controller:right_force_z_est_N", []), post_indices)
            ),
            push_response_window_start_s=push_at,
            push_response_window_end_s=response_end,
        )
        force_y = columns.get("force_y_n", [])
        nonzero = [i for i, value in enumerate(force_y) if finite(value) and abs(value) > 1e-9]
        metrics["logged_push_impulse_y_N_s"] = sum(valid(force_y)) * EXPECTED_LOG_PERIOD_S
        metrics["logged_push_nonzero_sample_count"] = len(nonzero)
        metrics["logged_push_first_nonzero_s"] = times[nonzero[0]] if nonzero else math.nan
        metrics["logged_push_last_nonzero_s"] = times[nonzero[-1]] if nonzero else math.nan
        recovery_time, recovered = calculate_recovery(columns, push_at, push_end, actual_log_end)
        metrics["recovery_time_s"] = recovery_time
        metrics["recovery_observation_end_s"] = actual_log_end
        metrics["recovered_in_window"] = "yes" if recovered else "no"
    return metrics


def append_validation_issue(run: RunData, detail: str) -> None:
    """Attach a cross-run validation error and keep metrics.csv in sync."""

    if detail not in run.validation_issues:
        run.validation_issues.append(detail)
    run.metrics["acceptance_validation_status"] = "failed"
    run.metrics["acceptance_validation_issues"] = " | ".join(run.validation_issues)


def compute_push_aligned_differences(runs: Sequence[RunData]) -> None:
    """Compare at identical samples, bounded by the shortest actual log.

    Pairing is deliberately strict: both runs must contain the same number of
    samples in the response window and every paired timestamp must agree within
    ``TIME_ABS_TOL_S``.  No interpolation or nearest-neighbour matching is used,
    because either would mix gait phases and make the disturbance increment
    dependent on an undocumented alignment choice.
    """

    natural = next((run for run in runs if run.condition.name == "no_push"), None)
    if natural is None:
        for run in runs:
            append_validation_issue(run, "侧推组缺少 no_push 自然步态参照，无法做时间对齐差分")
        return

    raw_natural_recovered = natural.metrics.get("recovered_in_window")
    if raw_natural_recovered == "yes":
        applicable = "yes"
    elif raw_natural_recovered == "no":
        applicable = "no"
    else:
        applicable = "unknown"
    for run in runs:
        run.metrics["recovery_criterion_applicable"] = applicable
        raw = run.metrics.get("recovered_in_window")
        if applicable == "no":
            run.metrics["recovery_result_interpretation"] = (
                "fixed_threshold_invalid_for_periodic_gait; preserve recovered_in_window as raw audit result"
            )
        elif applicable == "yes":
            run.metrics["recovery_result_interpretation"] = (
                "raw_fixed_threshold_recovered" if raw == "yes" else "raw_fixed_threshold_not_recovered"
            )
        else:
            run.metrics["recovery_result_interpretation"] = "criterion_applicability_unknown"

    if not natural.has_samples or natural.metrics.get("data_status") != "ok":
        for run in runs:
            append_validation_issue(run, "no_push 数据不可用，无法做响应窗时间对齐差分")
        return

    window_start = DEFAULT_PUSH_AT_S
    window_end = min(window_start + PUSH_TRANSIENT_WINDOW_S,
                     *(maximum(run.columns.get("time_s", [])) for run in runs))
    natural_times = natural.columns.get("time_s", [])
    natural_indices = indices_between(natural_times, window_start, window_end + TIME_ABS_TOL_S)
    if not natural_indices:
        for run in runs:
            append_validation_issue(run, "no_push 在共同响应窗没有样本")
        return

    signal_names = {
        "aligned_no_push_roll_max_abs_delta_rad": "roll_rad",
        "aligned_no_push_pitch_max_abs_delta_rad": "pitch_rad",
        "aligned_no_push_vx_max_abs_delta_m_s": "derived:vx_body_m_s",
        "aligned_no_push_vy_max_abs_delta_m_s": "derived:vy_body_m_s",
    }
    for run in runs:
        metrics = run.metrics
        metrics["aligned_no_push_window_start_s"] = window_start
        metrics["aligned_no_push_window_end_s"] = window_end
        times = run.columns.get("time_s", [])
        candidate_indices = indices_between(times, window_start, window_end + TIME_ABS_TOL_S)
        if len(candidate_indices) != len(natural_indices):
            append_validation_issue(
                run,
                f"共同响应窗时间对齐失败：{run.condition.name} 有 {len(candidate_indices)} 个样本，"
                f"no_push 有 {len(natural_indices)} 个样本",
            )
            continue
        mismatches = [
            (natural_times[natural_index], times[candidate_index])
            for natural_index, candidate_index in zip(natural_indices, candidate_indices)
            if not math.isclose(
                natural_times[natural_index],
                times[candidate_index],
                rel_tol=0.0,
                abs_tol=TIME_ABS_TOL_S,
            )
        ]
        if mismatches:
            first_reference, first_candidate = mismatches[0]
            append_validation_issue(
                run,
                "共同响应窗时间对齐失败：同序号时间戳不一致 "
                f"(no_push={first_reference:g}, {run.condition.name}={first_candidate:g})",
            )
            continue

        metrics["aligned_no_push_sample_count"] = len(candidate_indices)
        calculation_failed = False
        for metric_name, column_name in signal_names.items():
            reference_values = natural.columns.get(column_name, [])
            candidate_values = run.columns.get(column_name, [])
            differences = []
            for natural_index, candidate_index in zip(natural_indices, candidate_indices):
                if natural_index >= len(reference_values) or candidate_index >= len(candidate_values):
                    calculation_failed = True
                    break
                reference_value = reference_values[natural_index]
                candidate_value = candidate_values[candidate_index]
                if not finite(reference_value) or not finite(candidate_value):
                    calculation_failed = True
                    break
                differences.append(float(candidate_value) - float(reference_value))
            metrics[metric_name] = max_abs(differences) if not calculation_failed else math.nan
        reference_height = natural.columns.get("base_z_m", [])
        candidate_height = run.columns.get("base_z_m", [])
        height_drops = []
        for natural_index, candidate_index in zip(natural_indices, candidate_indices):
            if natural_index >= len(reference_height) or candidate_index >= len(candidate_height):
                calculation_failed = True
                break
            reference_value = reference_height[natural_index]
            candidate_value = candidate_height[candidate_index]
            if not finite(reference_value) or not finite(candidate_value):
                calculation_failed = True
                break
            height_drops.append(float(reference_value) - float(candidate_value))
        metrics["aligned_no_push_extra_height_drop_max_m"] = (
            max(0.0, maximum(height_drops)) if height_drops and not calculation_failed else math.nan
        )
        if calculation_failed:
            append_validation_issue(run, "共同响应窗时间对齐差分包含缺失或非有限状态值")


def project_relative(path: Path) -> str:
    try:
        return path.resolve().relative_to(PROJECT.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def format_csv_value(value: Any) -> Any:
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.10g}"
    return value


def write_metrics(path: Path, rows: Sequence[dict[str, Any]], include_experiment: bool = True) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = METRIC_FIELDS if include_experiment else [name for name in METRIC_FIELDS if name != "experiment"]
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: format_csv_value(row.get(name, "")) for name in fields})


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc" if bold else "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except OSError:
            pass
    return ImageFont.load_default()


def text_size(draw: ImageDraw.ImageDraw, text: str, used_font: ImageFont.ImageFont) -> tuple[int, int]:
    box = draw.textbbox((0, 0), text, font=used_font)
    return box[2] - box[0], box[3] - box[1]


def draw_dashed_line(
    draw: ImageDraw.ImageDraw,
    points: Sequence[tuple[float, float]],
    fill: tuple[int, int, int],
    width: int,
    dash: int = 12,
) -> None:
    for start, end in zip(points, points[1:]):
        x0, y0 = start
        x1, y1 = end
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 0:
            continue
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        position = 0.0
        while position < length:
            segment_end = min(position + dash, length)
            draw.line(
                (
                    x0 + ux * position,
                    y0 + uy * position,
                    x0 + ux * segment_end,
                    y0 + uy * segment_end,
                ),
                fill=fill,
                width=width,
            )
            position += dash * 2


def nice_range(low: float, high: float) -> tuple[float, float]:
    if not finite(low) or not finite(high):
        return 0.0, 1.0
    if low == high:
        margin = max(1e-6, abs(low) * 0.08, 0.01)
        return low - margin, high + margin
    margin = (high - low) * 0.08
    return low - margin, high + margin


def series_bounds(series: Sequence[Series], x_limits: tuple[float, float] | None) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for item in series:
        for x, y in zip(item.x, item.y):
            if not finite(x) or not finite(y):
                continue
            if x_limits is not None and not (x_limits[0] <= x <= x_limits[1]):
                continue
            xs.append(float(x))
            ys.append(float(y))
    if x_limits is not None:
        x_low, x_high = x_limits
    elif xs:
        x_low, x_high = min(xs), max(xs)
    else:
        x_low, x_high = 0.0, 1.0
    if x_low == x_high:
        x_high = x_low + 1.0
    y_low, y_high = nice_range(min(ys) if ys else 0.0, max(ys) if ys else 1.0)
    return x_low, x_high, y_low, y_high


def draw_panel(
    draw: ImageDraw.ImageDraw,
    panel: Panel,
    box: tuple[int, int, int, int],
) -> None:
    left, top, right, bottom = box
    title_font, tick_font, label_font, legend_font = font(28, True), font(21), font(23), font(20)
    plot_left, plot_top = left + 118, top + 58
    # Reserve separate rows below the axes for tick labels, x-axis title and
    # the legend so none of them overlap in the exported report figure.
    plot_right, plot_bottom = right - 35, bottom - 145
    x_low, x_high, y_low, y_high = series_bounds(panel.series, panel.x_limits)

    draw.rectangle((plot_left, plot_top, plot_right, plot_bottom), fill=(255, 255, 255), outline=(70, 76, 86), width=2)
    for tick in range(6):
        fraction = tick / 5
        x = plot_left + fraction * (plot_right - plot_left)
        y = plot_bottom - fraction * (plot_bottom - plot_top)
        draw.line((x, plot_top, x, plot_bottom), fill=(225, 228, 233), width=1)
        draw.line((plot_left, y, plot_right, y), fill=(225, 228, 233), width=1)
        x_value = x_low + fraction * (x_high - x_low)
        y_value = y_low + fraction * (y_high - y_low)
        x_text = f"{x_value:.2f}"
        if panel.y_tick_decimals is None:
            y_text = f"{y_value:.3g}"
        else:
            y_text = f"{y_value:.{panel.y_tick_decimals}f}"
        x_width, _ = text_size(draw, x_text, tick_font)
        y_width, y_height = text_size(draw, y_text, tick_font)
        draw.text((x - x_width / 2, plot_bottom + 12), x_text, fill=(54, 60, 69), font=tick_font)
        draw.text((plot_left - y_width - 12, y - y_height / 2), y_text, fill=(54, 60, 69), font=tick_font)

    for item in panel.series:
        raw_points = []
        for x_value, y_value in zip(item.x, item.y):
            if not finite(x_value) or not finite(y_value):
                continue
            if not (x_low <= x_value <= x_high):
                continue
            x = plot_left + (x_value - x_low) / (x_high - x_low) * (plot_right - plot_left)
            y = plot_bottom - (y_value - y_low) / (y_high - y_low) * (plot_bottom - plot_top)
            raw_points.append((x, y))
        if len(raw_points) > 1800:
            stride = math.ceil(len(raw_points) / 1800)
            raw_points = raw_points[::stride]
        if len(raw_points) >= 2:
            if item.dashed:
                draw_dashed_line(draw, raw_points, item.color, 4)
            else:
                draw.line(raw_points, fill=item.color, width=4, joint="curve")

    title_width, _ = text_size(draw, panel.title, title_font)
    draw.text(((left + right - title_width) / 2, top + 10), panel.title, fill=(28, 33, 41), font=title_font)
    x_width, _ = text_size(draw, panel.x_label, label_font)
    draw.text(((plot_left + plot_right - x_width) / 2, plot_bottom + 48), panel.x_label, fill=(35, 40, 48), font=label_font)
    draw.text((left + 8, plot_top - 31), panel.y_label, fill=(35, 40, 48), font=label_font)

    legend_items = [item for item in panel.series if item.label]
    if legend_items:
        widths = [text_size(draw, item.label, legend_font)[0] + 65 for item in legend_items]
        total_width = sum(widths) + 16 * max(0, len(widths) - 1)
        cursor = max(plot_left, (plot_left + plot_right - total_width) / 2)
        legend_y = plot_bottom + 92
        for item, item_width in zip(legend_items, widths):
            line_y = legend_y + 12
            if item.dashed:
                draw_dashed_line(draw, [(cursor, line_y), (cursor + 42, line_y)], item.color, 4, 7)
            else:
                draw.line((cursor, line_y, cursor + 42, line_y), fill=item.color, width=4)
            draw.text((cursor + 51, legend_y), item.label, fill=(35, 40, 48), font=legend_font)
            cursor += item_width + 16


def save_figure(path: Path, title: str, panels: Sequence[Panel]) -> None:
    width = 1600
    panel_height = 700
    header_height = 92
    height = header_height + panel_height * len(panels) + 30
    image = Image.new("RGB", (width, height), (247, 248, 250))
    draw = ImageDraw.Draw(image)
    used_font = font(38, True)
    text_width, _ = text_size(draw, title, used_font)
    draw.text(((width - text_width) / 2, 23), title, fill=(22, 29, 40), font=used_font)
    for index, panel in enumerate(panels):
        top = header_height + index * panel_height
        draw_panel(draw, panel, (32, top, width - 32, top + panel_height - 18))
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path, format="PNG", optimize=True)


def run_series(
    runs: Sequence[RunData],
    column: str,
    x_limits: tuple[float, float] | None = None,
) -> tuple[Series, ...]:
    result = []
    for index, run in enumerate(runs):
        if not run.has_samples or column not in run.columns:
            continue
        result.append(
            Series(
                run.label,
                run.columns["time_s"],
                run.columns[column],
                COLORS[index % len(COLORS)],
            )
        )
    return tuple(result)


def create_plots(spec: GroupSpec, runs: Sequence[RunData], output: Path) -> list[str]:
    available = [run for run in runs if run.has_samples]
    if not available:
        return []
    x_limits = (6.0, 12.0) if spec.name == "lateral_push" else (5.0, 12.0)
    created: list[str] = []

    def save(name: str, title: str, panels: Sequence[Panel]) -> None:
        if not any(panel.series for panel in panels):
            return
        save_figure(output / name, title, panels)
        created.append(name)

    if spec.name in ("mpc_vx_weight", "prediction_horizon"):
        speed_series = list(run_series(available, "derived:vx_body_m_s", x_limits))
        reference = next((run for run in available if "controller:command_vx_mps" in run.columns), None)
        if reference:
            speed_series.append(
                Series(
                    "内部速度参考",
                    reference.columns["time_s"],
                    reference.columns["controller:command_vx_mps"],
                    (80, 84, 92),
                    True,
                )
            )
        save(
            "speed_tracking.png",
            f"{spec.title}：速度跟踪",
            (Panel("机身前进速度", "时间 (s)", "速度 (m/s)", tuple(speed_series), x_limits),),
        )
        save(
            "posture.png",
            f"{spec.title}：机身姿态",
            (
                Panel("Roll", "时间 (s)", "角度 (rad)", run_series(available, "roll_rad", x_limits), x_limits),
                Panel("Pitch", "时间 (s)", "角度 (rad)", run_series(available, "pitch_rad", x_limits), x_limits),
            ),
        )
        if spec.name == "mpc_vx_weight":
            save(
                "control_effort.png",
                f"{spec.title}：关节力矩",
                (Panel("31 关节瞬时最大绝对力矩", "时间 (s)", "力矩 (N·m)", run_series(available, "derived:torque_abs_max_Nm", x_limits), x_limits),),
            )
        else:
            converted = []
            for index, run in enumerate(available):
                source = run.columns.get("controller:mpc_solve_s", [])
                converted.append(
                    Series(run.label, run.columns["time_s"], [value * 1000.0 for value in source], COLORS[index])
                )
            save(
                "solver_time.png",
                f"{spec.title}：MPC qpOASES QP CPU time",
                (Panel("MPC qpOASES QP CPU time", "时间 (s)", "CPU time (ms)", tuple(converted), x_limits),),
            )
    elif spec.name == "step_height":
        save(
            "posture.png",
            "抬腿高度：机身姿态",
            (
                Panel("Roll", "时间 (s)", "角度 (rad)", run_series(available, "roll_rad", x_limits), x_limits),
                Panel("Pitch", "时间 (s)", "角度 (rad)", run_series(available, "pitch_rad", x_limits), x_limits),
            ),
        )
        save(
            "base_height.png",
            "抬腿高度：基座高度",
            (
                Panel(
                    "基座高度",
                    "时间 (s)",
                    "高度 (m)",
                    run_series(available, "base_z_m", x_limits),
                    x_limits,
                    y_tick_decimals=3,
                ),
            ),
        )
        save(
            "joint_torque.png",
            "抬腿高度：关节力矩",
            (Panel("31 关节瞬时最大绝对力矩", "时间 (s)", "力矩 (N·m)", run_series(available, "derived:torque_abs_max_Nm", x_limits), x_limits),),
        )
    elif spec.name == "lateral_push":
        save(
            "disturbance_response.png",
            "侧向扰动：机身速度响应",
            (
                Panel("机身前进速度", "时间 (s)", "速度 (m/s)", run_series(available, "derived:vx_body_m_s", x_limits), x_limits),
                Panel("机身横向速度", "时间 (s)", "速度 (m/s)", run_series(available, "derived:vy_body_m_s", x_limits), x_limits),
            ),
        )
        save(
            "recovery.png",
            "侧向扰动：姿态响应",
            (
                Panel("Roll", "时间 (s)", "角度 (rad)", run_series(available, "roll_rad", x_limits), x_limits),
                Panel("Pitch", "时间 (s)", "角度 (rad)", run_series(available, "pitch_rad", x_limits), x_limits),
            ),
        )
        save(
            "control_effort.png",
            "侧向扰动：关节力矩",
            (Panel("31 关节瞬时最大绝对力矩", "时间 (s)", "力矩 (N·m)", run_series(available, "derived:torque_abs_max_Nm", x_limits), x_limits),),
        )
    return created


def md_number(value: Any, digits: int = 4) -> str:
    if finite(value):
        return f"{float(value):.{digits}g}"
    if value in ("yes", "no"):
        return str(value)
    return "—"


def percent_change(value: Any, baseline: Any) -> str:
    if not finite(value) or not finite(baseline):
        return "无法计算"
    if abs(float(baseline)) < 1e-12:
        return f"由 {md_number(baseline)} 变为 {md_number(value)}"
    change = (float(value) - float(baseline)) / abs(float(baseline)) * 100.0
    direction = "增加" if change > 0 else "降低" if change < 0 else "不变"
    return f"{direction} {abs(change):.1f}%"


def mechanism_text(group: str) -> str:
    texts = {
        "mpc_vx_weight": (
            "`state_weights[9]` 增大后，MPC 目标函数会提高前进速度误差的代价，求解器可能分配更积极的接触 wrench。"
            "是否因此改善闭环表现，仍要同时检查姿态、足底估计力、关节力矩和约束求解状态；权重更大本身不能证明更优。"
        ),
        "prediction_horizon": (
            "预测步数 `N` 改变了状态预测矩阵及其覆盖的未来时间窗；`N=5/10/15` 在 `mpc_dt=0.005 s` 下分别对应 "
            "25/50/75 ms。当前实现的控制时域固定为 `ch=3`，单步输入维数 `nu=13`，因此 qpOASES 的决策变量维数始终是 "
            "`nu*ch=39`，不会随 `N` 增大。`N` 仍会改变压缩前的预测矩阵、组装结果和 QP 系数，但日志字段 "
            "`controller:mpc_solve_s` 只记录 qpOASES QP CPU time，不包含预测矩阵与 QP 组装时间。"
        ),
        "step_height": (
            "`step_height` 改变摆动足轨迹，WBC 必须用关节加速度和力矩跟踪新的足端任务。较高轨迹可能改变腿部控制努力和机身扰动。"
            "当前日志没有足端世界坐标，因此本实验只能确认配置的期望抬腿高度，不能把它当作实测离地间隙。"
        ),
        "lateral_push": (
            "侧向外力先改变基座横向速度和姿态，MPC 通过允许的接触 wrench 修正，WBC 再把期望广义力映射到全身关节。"
            "恢复能力同时受摩擦、足底力矩约束、支撑切换和关节力矩限幅影响，不能仅由某一个峰值解释。"
        ),
    }
    return texts[group]


def controlled_config(run: RunData) -> dict[str, Any]:
    """Return the experiment-relevant config with the intended variable masked."""

    source = get_config(run.run)
    keys = (
        "mode",
        "duration",
        "dt",
        "mpc_dt",
        "walk_start",
        "vx",
        "vy",
        "wz",
        "swing_time",
        "step_height",
        "stand_height",
        "push_at",
        "push_duration",
        "push_force",
        "u_weight",
        "state_weights",
        "input_weights",
        "mpc_horizon_steps",
    )
    result = {key: source.get(key) for key in keys if key in source}
    if run.spec.name == "mpc_vx_weight":
        weights = list(result.get("state_weights", []))
        if len(weights) > 9:
            weights[9] = "<single-variable>"
        result["state_weights"] = weights
    elif run.spec.name == "prediction_horizon":
        result["mpc_horizon_steps"] = "<single-variable>"
    elif run.spec.name == "step_height":
        result["step_height"] = "<single-variable>"
    elif run.spec.name == "lateral_push":
        result["push_force"] = "<single-variable>"
    return result


def single_variable_issues(runs: Sequence[RunData]) -> list[str]:
    available = [run for run in runs if run.run]
    if len(available) != len(runs):
        return ["部分 run.json 缺失，无法完成单变量检查"]
    issues = []
    for run in available:
        actual, source = variable_value(run)
        if source.startswith("condition-name fallback"):
            issues.append(
                f"{run.condition.name} 的 run.json 未记录 `{run.spec.variable_name}`，无法核验实际取值"
            )
        elif not math.isclose(
            actual,
            run.condition.fallback_value,
            rel_tol=1e-9,
            abs_tol=1e-12,
        ):
            issues.append(
                f"{run.condition.name} 的实际值为 {actual:g}，设计值为 {run.condition.fallback_value:g}"
            )
    reference = controlled_config(available[0])
    for run in available[1:]:
        current = controlled_config(run)
        keys = sorted(set(reference) | set(current))
        differences = [key for key in keys if reference.get(key) != current.get(key)]
        if differences:
            issues.append(
                f"{available[0].condition.name} 与 {run.condition.name} 还存在非目标字段差异："
                + ", ".join(f"`{key}`" for key in differences)
            )
    return issues


def write_analysis(spec: GroupSpec, runs: Sequence[RunData], plots: Sequence[str], path: Path) -> None:
    rows = [run.metrics for run in runs]
    baseline = next(
        (
            row
            for row in rows
            if row.get("condition") in ("baseline", "baseline_012", "no_push")
        ),
        rows[0] if rows else {},
    )
    complete = [
        row
        for row in rows
        if row.get("data_status") == "ok"
        and row.get("run_outcome") in ("completed", "physical_instability")
        and row.get("acceptance_validation_status") == "ok"
    ]
    lines = [
        f"# {spec.title}对照实验分析",
        "",
        "> 本文由 `Pro2/tools/analyze_l2_results.py` 从各运行目录的 `run.json` 和 `states.csv` 生成。数值结果保存在同目录的 `metrics.csv`。",
        "",
        "## 1. 改了什么",
        "",
        f"本组唯一自变量是 `{spec.variable_name}`，单位为 `{spec.variable_unit}`。实验配置路径为：",
        "",
    ]
    lines += [f"- `{config_path}`" for config_path in spec.config_paths]
    lines += [""]
    if spec.name == "prediction_horizon":
        lines.append(
            "各条件的实际预测步数优先读取 `run.json`；若运行清单未记录该字段，则按条件目录的设计值暂填，"
            "并在 `metrics.csv` 的 `variable_value_source` 中明确标记，验收前必须人工复核。"
        )
    else:
        lines.append("各条件的实际取值读取自 `run.json`，具体来源记录在 `metrics.csv` 的 `variable_value_source` 列。")
    config_issues = single_variable_issues(runs)
    if config_issues:
        lines += ["", "**单变量配置检查未通过：**"]
        lines += [f"- {issue}" for issue in config_issues]
    else:
        lines += [
            "",
            "**单变量配置检查通过：**在运动、仿真和控制相关字段中，三次运行只改变了本组指定的自变量。",
        ]
    lines += [
        "",
        "## 2. 观察到什么",
        "",
        f"本批共同分析窗口为 `5.0–{md_number(baseline.get('analysis_end_s'))} s`。"
        "完整日志保留到各自实际结束时刻；比较指标使用共同可用时间段。"
        "`最大 Roll/Pitch 分量` 定义为该窗口内 `max(max|Roll|, max|Pitch|)`。",
        "",
        "| 条件 | 自变量 | 运行/数据/验收校验 | 分析样本 | 内部参考 vx RMSE (m/s) | 最大 Roll/Pitch 分量 (rad) | 高度标准差 (m) | 力矩平方积分 (N²·m²·s) | MPC qpOASES QP CPU time P95 (ms) | MPC+WBC 非零状态样本数 | 恢复时间原始值 (s) |",
        "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        status = (
            f"{row.get('run_status', '—')} / {row.get('data_status', '—')} / "
            f"{row.get('acceptance_validation_status', '—')}"
        )
        qp_count = "—"
        if finite(row.get("mpc_qp_nonzero_count")) and finite(row.get("wbc_qp_nonzero_count")):
            qp_count = str(int(row["mpc_qp_nonzero_count"]) + int(row["wbc_qp_nonzero_count"]))
        lines.append(
            "| {condition} | {variable} | {status} | {samples} | {speed} | {tilt} | {height} | {torque} | {mpc} | {qp} | {recovery} |".format(
                condition=row.get("condition_label", row.get("condition", "—")),
                variable=md_number(row.get("variable_value")),
                status=status,
                samples=row.get("sample_count_analysis", "—") or "—",
                speed=md_number(row.get("vx_internal_reference_rmse_m_s")),
                tilt=md_number(row.get("max_tilt_rad")),
                height=md_number(row.get("base_height_std_m")),
                torque=md_number(row.get("joint_torque_squared_integral_Nm2_s")),
                mpc=md_number(row.get("mpc_solve_p95_ms")),
                qp=qp_count,
                recovery=md_number(row.get("recovery_time_s")),
            )
        )
    lines += [""]
    if spec.name == "lateral_push":
        natural = next((row for row in rows if row.get("condition") == "no_push"), {})
        criterion_applicable = natural.get("recovery_criterion_applicable") == "yes"
        push_duration = md_number(natural.get("push_duration_s"))
        push_at = md_number(natural.get("push_at_s"))
        push_end = md_number(natural.get("push_end_s"))
        aligned_window = f"{md_number(natural.get('aligned_no_push_window_start_s'))}–{md_number(natural.get('aligned_no_push_window_end_s'))} s"
        lines += [
            f"本批施力区间为 `{push_at}–{push_end} s`，持续 `{push_duration} s`；同批三档力为 0 / 50 / 100 N。",
            "",
            "| 条件 | 配置持续时间 (s) | 配置冲量 (N·s) | 日志矩形积分 (N·s) | 非零外力样本 | 首个非零时刻 (s) | 最后非零时刻 (s) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
        for row in rows:
            lines.append(
                f"| {row.get('condition_label')} | {md_number(row.get('push_duration_s'))} | "
                f"{md_number(row.get('push_impulse_y_N_s'))} | {md_number(row.get('logged_push_impulse_y_N_s'))} | "
                f"{row.get('logged_push_nonzero_sample_count', '—')} | "
                f"{md_number(row.get('logged_push_first_nonzero_s'))} | {md_number(row.get('logged_push_last_nonzero_s'))} |"
            )
        lines += [
            "",
            "计划时长均为 12 s；发生物理失稳时保留原始 `failed` 状态和真实停止时刻，"
            "不补写状态数据。该失稳是抗扰实验的结果，数据完整性校验通过不表示机器人保持稳定。",
            "",
            "| 条件 | 运行结果 | 实际时长 (s) | 记录样本数 | 停止原因 |",
            "|---|---|---:|---:|---|",
        ]
        for row in rows:
            lines.append(f"| {row.get('condition_label')} | {row.get('run_outcome')} | "
                         f"{md_number(row.get('actual_duration_s'))} | {row.get('sample_count_total')} | "
                         f"{row.get('termination_reason') or '完成计划时长'} |")
        lines += [
            "",
            "`no_push` 是**自然步态参照**：它保留相同的行走命令和时间窗，但不施加外力，用于识别正常步态本身的周期起伏。",
            "扰动峰值最多观察施力开始后的 2.0 s，并截断到三组日志共同末尾，即 "
            f"`{aligned_window}`；该窗口包含 {push_duration} s 的施力阶段。",
            "恢复计时从施力结束 `push_at + push_duration` 开始。每个连续样本必须同时满足："
            "机身坐标系 `vx`、`vy` 相对控制器内部速度参考的误差均不超过 `0.05 m/s`，"
            "Roll、Pitch 相对施力前 1.0 s 均值的偏差均不超过 `0.03 rad`，并连续保持至少 `0.5 s`。"
            "若相邻记录间隔大于 `max(0.05 s, 2.5×中位采样间隔)`，连续计时会重置。",
            "恢复判据原始审计仍使用每组到实际停止时刻的完整日志，"
            "观察末端另存 `recovery_observation_end_s`；同刻差分与常规比较指标使用上面的共同窗口。",
            "",
        ]
        if not criterion_applicable:
            lines += [
                "**恢复判据适用性结论：**`no_push` 自然步态参照本身也未连续满足上述固定阈值。"
                "这说明正常周期步态的速度或姿态起伏已经超出该阈值。周期步态下该固定阈值判据无效，"
                "无法给出有效恢复时间，也不能据此判断 50 N/100 N 组是否恢复。"
                "下表仍保留 `recovered_in_window` 与 `recovery_time_s` 的原始计算结果供审计。",
                "",
            ]
        lines += [
            "下面的原始峰值以每次运行自身的施力前 1 s 均值为基准。它们用于查看单次响应，"
            "不用于计算相对 `no_push` 的扰动增量。",
            "",
            "| 条件 | 响应窗 Roll 峰值偏差 (rad) | 响应窗 Pitch 峰值偏差 (rad) | 响应窗 vx 峰值偏差 (m/s) | 响应窗 vy 峰值偏差 (m/s) | 响应窗最低高度 (m) | 响应窗峰值力矩 (N·m) | 恢复状态 | 恢复时间 (s) |",
            "|---|---:|---:|---:|---:|---:|---:|---|---:|",
        ]
        for row in rows:
            recovered = row.get("recovered_in_window")
            raw_text = "原始判据：满足" if recovered == "yes" else "原始判据：未满足" if recovered == "no" else "原始判据：无结果"
            if not criterion_applicable and row.get("condition") == "no_push":
                recovered_text = f"自然步态参照；{raw_text}"
            elif not criterion_applicable:
                recovered_text = f"周期步态下判据无效；{raw_text}"
            else:
                recovered_text = (
                    "固定阈值判据：恢复" if recovered == "yes" else "固定阈值判据：未恢复" if recovered == "no" else "—"
                )
            lines.append(
                "| {condition} | {roll} | {pitch} | {vx} | {vy} | {height} | {torque} | {recovered} | {time} |".format(
                    condition=row.get("condition_label", row.get("condition", "—")),
                    roll=md_number(row.get("post_push_roll_peak_deviation_rad")),
                    pitch=md_number(row.get("post_push_pitch_peak_deviation_rad")),
                    vx=md_number(row.get("post_push_vx_peak_deviation_m_s")),
                    vy=md_number(row.get("post_push_vy_peak_deviation_m_s")),
                    height=md_number(row.get("post_push_base_height_min_m")),
                    torque=md_number(row.get("post_push_joint_torque_peak_Nm")),
                    recovered=recovered_text,
                    time=md_number(row.get("recovery_time_s")),
                )
            )
        lines += [""]
        lines += [
            f"相对自然步态的扰动增量采用**同时间样本差分**：只在 `{aligned_window}` 比较，"
            "要求两次运行的时间戳逐点相等（容差 `1e-6 s`），不插值、不做最近邻匹配。"
            "每个状态先计算 `push(t)-no_push(t)`，再统计最大绝对差；额外高度下降定义为 "
            "`max(0, max(no_push_z(t)-push_z(t)))`。",
            "",
            "| 条件 | 对齐样本 | 最大 ΔRoll 绝对值 (rad) | 最大 ΔPitch 绝对值 (rad) | 最大 Δvx 绝对值 (m/s) | 最大 Δvy 绝对值 (m/s) | 相对 no_push 最大额外高度下降 (m) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
        for row in rows:
            lines.append(
                "| {condition} | {samples} | {roll} | {pitch} | {vx} | {vy} | {height} |".format(
                    condition=row.get("condition_label", row.get("condition", "—")),
                    samples=row.get("aligned_no_push_sample_count", "—") or "—",
                    roll=md_number(row.get("aligned_no_push_roll_max_abs_delta_rad")),
                    pitch=md_number(row.get("aligned_no_push_pitch_max_abs_delta_rad")),
                    vx=md_number(row.get("aligned_no_push_vx_max_abs_delta_m_s")),
                    vy=md_number(row.get("aligned_no_push_vy_max_abs_delta_m_s")),
                    height=md_number(row.get("aligned_no_push_extra_height_drop_max_m")),
                )
            )
        lines += [""]
        if natural and all(row.get("data_status") == "ok" for row in rows):
            lines.append("按上述时间对齐定义，本批侧推相对自然步态参照的差分结果为：")
            for row in rows:
                if row is natural:
                    continue
                lines.append(
                    f"- **{row.get('condition_label')}**：max|ΔRoll| "
                    f"{md_number(row.get('aligned_no_push_roll_max_abs_delta_rad'))} rad，max|ΔPitch| "
                    f"{md_number(row.get('aligned_no_push_pitch_max_abs_delta_rad'))} rad，max|Δvx| "
                    f"{md_number(row.get('aligned_no_push_vx_max_abs_delta_m_s'))} m/s，max|Δvy| "
                    f"{md_number(row.get('aligned_no_push_vy_max_abs_delta_m_s'))} m/s，额外高度下降 "
                    f"{md_number(row.get('aligned_no_push_extra_height_drop_max_m'))} m。"
                )
            lines += [""]
    if len(complete) == len(spec.conditions):
        speed_base = baseline.get("vx_internal_reference_rmse_m_s")
        tilt_base = baseline.get("max_tilt_rad")
        torque_base = baseline.get("joint_torque_squared_integral_Nm2_s")
        for row in rows:
            if row is baseline:
                continue
            lines.append(
                f"- 相对基线，**{row.get('condition_label')}** 的内部参考速度 RMSE {percent_change(row.get('vx_internal_reference_rmse_m_s'), speed_base)}，"
                f"最大 Roll/Pitch 分量 {percent_change(row.get('max_tilt_rad'), tilt_base)}，"
                f"力矩平方积分 {percent_change(row.get('joint_torque_squared_integral_Nm2_s'), torque_base)}。"
            )
    else:
        lines.append(
            f"- 当前只有 {len(complete)}/{len(spec.conditions)} 个条件具备 5 s 之后的有效数据，本组不能形成完整验收结论。"
        )
    if plots:
        lines += ["", "对应对比图："]
        lines += [f"- [`{name}`]({name})" for name in plots]
    lines += [
        "",
        "## 3. 为什么可能出现这些变化",
        "",
        mechanism_text(spec.name),
        "",
        "这段解释给出与控制结构一致的机制假设。因果判断以本组单变量配置有效、运行完成且重复实验趋势一致为前提。",
        "",
        "## 4. 结论边界",
        "",
        "- 常规对比指标使用 `5.0 s` 至本组共同日志末尾；扰动恢复姿态基准使用施力前 1.0 s。",
        "- `controller:left_force_z_est_N` 与 `controller:right_force_z_est_N` 是动力学反算值，不是 PhysX 接触传感器实测值。",
        "- 力矩平方积分是控制努力代理量；绝对机械功也是机械侧代理量，二者都不是电机电能。",
        "- `controller:mpc_solve_s` 与 `controller:wbc_solve_s` 是 qpOASES 报告的 QP CPU time，"
        "由本实验 CSV 以 100 Hz 抽样记录；它们不包含预测矩阵构造、QP 组装，也不等于完整控制循环耗时。",
        "- `MPC+WBC 非零状态样本数` 是统一分析窗口内两个求解器状态字段非零样本数之和，不等于互不重复的时间点数。",
        "- 当前结果来自 Isaac Sim/PhysX 中的一次确定配置运行。没有重复试验时，只能说明本次仿真的差异。",
        "- 图中只放曲线、标题、坐标轴和图例；全部数值、样本数与条件保存在 `metrics.csv`。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def active_group(root: Path, name: str) -> tuple[Path, dict[str, Any]]:
    """Resolve explicit active-batch selection, never silently substitute history."""

    manifest_path = root / "active_runs.json"
    if not manifest_path.is_file():
        return root / name, {}
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entry = manifest.get("groups", {}).get(name)
    if entry is None:
        return root / name, {}
    if not isinstance(entry, dict) or not isinstance(entry.get("directory"), str):
        raise ValueError(f"{manifest_path}: {name} 必须指定 directory")
    directory = (root / entry["directory"]).resolve()
    if not directory.is_relative_to((root / name).resolve()):
        raise ValueError(f"{manifest_path}: {name} 的运行目录必须位于该实验类别内")
    expected = entry.get("expected_config", {})
    if not isinstance(expected, dict):
        raise ValueError(f"{manifest_path}: {name}.expected_config 必须为对象")
    return directory, dict(expected, _accept_physical_instability=bool(entry.get("accept_physical_instability", False)))


def load_group(root: Path, spec: GroupSpec) -> list[RunData]:
    group_dir, expected_config = active_group(root, spec.name)
    runs = []
    for condition in spec.conditions:
        directory = find_run_directory(group_dir, condition)
        run = RunData(spec, condition, directory)
        run.expected_config = expected_config
        if directory is not None:
            run.run = load_json(directory / "run.json")
            run.columns = load_states(directory / "states.csv")
        run.validation_issues = validate_acceptance_run(run)
        run.metrics = compute_metrics(run)
        runs.append(run)
    if spec.name == "lateral_push":
        if all(run.has_samples for run in runs):
            common_end = min(maximum(run.columns["time_s"]) for run in runs)
            for run in runs:
                run.analysis_end_override_s = common_end
                run.metrics = compute_metrics(run)
        compute_push_aligned_differences(runs)
    return runs


def write_push_presentation_summary(runs: Sequence[RunData], comparison: Path) -> None:
    """Keep video captions and the PPT tied to the analyzed source recordings."""

    natural = next(run for run in runs if run.condition.name == "no_push")
    row = natural.metrics
    complete = all(run.metrics.get("acceptance_validation_status") == "ok" for run in runs)
    conditions = [
        {
            "name": run.condition.name,
            "force_y_N": run.metrics.get("variable_value"),
            "impulse_N_s": run.metrics.get("push_impulse_y_N_s"),
            "max_abs_delta_vy_m_s": run.metrics.get("aligned_no_push_vy_max_abs_delta_m_s"),
            "max_abs_delta_roll_rad": run.metrics.get("aligned_no_push_roll_max_abs_delta_rad"),
            "max_abs_delta_pitch_rad": run.metrics.get("aligned_no_push_pitch_max_abs_delta_rad"),
            "max_abs_delta_vx_m_s": run.metrics.get("aligned_no_push_vx_max_abs_delta_m_s"),
            "extra_height_drop_m": run.metrics.get("aligned_no_push_extra_height_drop_max_m"),
            "run_directory": project_relative(run.directory) if run.directory else None,
            "run_status": run.metrics.get("run_status"),
            "status": run.metrics.get("run_outcome"),
            "simulation_end_s": run.metrics.get("actual_duration_s"),
            "video_duration_s": run.run.get("video_duration_s"),
            "failure_reason": run.metrics.get("termination_reason"),
            "acceptance_validation_status": run.metrics.get("acceptance_validation_status"),
        }
        for run in runs
    ]
    conclusions = [
        f"侧推持续 {md_number(row.get('push_duration_s'))} s，冲量为 "
        + " / ".join(md_number(run.metrics.get("push_impulse_y_N_s")) for run in runs) + " N·s。",
        " / ".join(f"{run.condition.label}：{run.metrics.get('run_outcome')}，"
                   f"{md_number(run.metrics.get('actual_duration_s'))} s" for run in runs),
        f"相对同刻无扰动轨迹比较，统计窗口为 {md_number(row.get('aligned_no_push_window_start_s'))}–"
        f"{md_number(row.get('aligned_no_push_window_end_s'))} s。",
    ]
    applicable = row.get("recovery_criterion_applicable") == "yes"
    if not applicable:
        conclusions.append("自然步态也未通过固定恢复阈值，不能由该判据给出有效恢复时间。")
    payload = {
        "run_root": project_relative(comparison.parent),
        "figure": project_relative(comparison / "disturbance_response.png"),
        "metrics": project_relative(comparison / "metrics.csv"),
        "push_at_s": row.get("push_at_s"),
        "push_duration_s": row.get("push_duration_s"),
        "push_end_s": row.get("push_end_s"),
        "window_start_s": row.get("aligned_no_push_window_start_s"),
        "window_end_s": row.get("aligned_no_push_window_end_s"),
        "conditions": conditions,
        "conclusions": conclusions,
        "recovery_criterion_applicable": applicable,
        "acceptance_validation_passed": complete,
    }
    failed = [run for run in runs if run.metrics.get("run_outcome") == "physical_instability"]
    stable_push = [run for run in runs if run.condition.name != "no_push" and run.metrics.get("run_status") == "completed"]
    outcome_lines = [f"{run.metrics.get('variable_value'):g} N 完成 12 s" for run in stable_push]
    outcome_lines += [f"{run.metrics.get('variable_value'):g} N：{run.metrics.get('actual_duration_s'):g} s 失稳" for run in failed]
    outcome_lines += ["失稳属于正式对照结果" if failed else "结果仅限本次测试条件"]
    payload["ppt"] = {
        "condition": f"世界 +Y 侧推 0 / 50 / 100 N，{row.get('push_at_s'):g}–{row.get('push_end_s'):g} s；"
                     f"Δvy 峰值窗口 {row.get('aligned_no_push_window_start_s'):.2f}–{row.get('aligned_no_push_window_end_s'):.2f} s",
        "table_rows": [["侧推 (N)", "|Δvy| (m/s)"]] + [
            [f"{run.metrics.get('variable_value'):g}", f"{run.metrics.get('aligned_no_push_vy_max_abs_delta_m_s'):.4f}"]
            for run in runs
        ],
        "conclusion": "\n".join(outcome_lines),
        "reason": f"外力持续 {row.get('push_duration_s'):g} s，冲量为 0 / "
                  + " / ".join(f"{run.metrics.get('push_impulse_y_N_s'):g}" for run in runs if run.condition.name != "no_push")
                  + " N·s。\n" + ("100 N 组失稳；QP 状态为 0 不代表闭环稳定。" if failed
                                   else "固定恢复阈值对自然步态无效，不报告恢复时间。"),
        "config_path": "Pro2/experiments/L2/lateral_push/push_100N.json",
    }
    (comparison / "presentation_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )


def write_summary(root: Path, all_runs: dict[str, list[RunData]]) -> None:
    summary_dir = root / "summary"
    rows = [run.metrics for runs in all_runs.values() for run in runs]
    write_metrics(summary_dir / "L2总指标.csv", rows)
    complete_groups = sum(
        all(
            run.metrics.get("data_status") == "ok"
            and run.metrics.get("run_outcome") in ("completed", "physical_instability")
            and run.metrics.get("acceptance_validation_status") == "ok"
            for run in runs
        )
        and not single_variable_issues(runs)
        for runs in all_runs.values()
    )
    lines = [
        "# L2 四类对照实验结论",
        "",
        "> 本文由 `Pro2/tools/analyze_l2_results.py` 根据真实运行文件生成，不使用 L0 数据补齐 L2 结果。",
        "",
        "## 验收状态",
        "",
        f"当前原始证据完整且可分析的实验类别：**{complete_groups}/{len(all_runs)}**。"
        "物理失稳提前停止按真实记录验收，不等于成功走满计划时长。",
        "",
        "| 实验类别 | 有效条件 | 完成12s条件 | 失稳停止条件 | 单变量/清单检查 | 指标文件 | 分析文件 |",
        "|---|---:|---:|---:|---|---|---|",
    ]
    for name, runs in all_runs.items():
        group_dir, _ = active_group(root, name)
        comparison_relative = (group_dir / "comparison").relative_to(root).as_posix()
        valid_count = sum(
            run.metrics.get("data_status") == "ok"
            and run.metrics.get("acceptance_validation_status") == "ok"
            for run in runs
        )
        completed_count = sum(
            run.metrics.get("data_status") == "ok"
            and run.metrics.get("run_status") == "completed"
            and run.metrics.get("acceptance_validation_status") == "ok"
            for run in runs
        )
        config_status = "通过" if not single_variable_issues(runs) else "未通过"
        stopped_count = sum(run.metrics.get("run_outcome") == "physical_instability" for run in runs)
        lines.append(
            f"| {runs[0].spec.title if runs else name} | {valid_count}/{len(runs)} | {completed_count}/{len(runs)} | {stopped_count}/{len(runs)} | {config_status} | "
            f"[`metrics.csv`](../{comparison_relative}/metrics.csv) | [`analysis.md`](../{comparison_relative}/analysis.md) |"
        )
    lines += [
        "",
        "完整数值见 [`L2总指标.csv`](L2总指标.csv)。四类实验必须分别保存在各自目录，不能把某一类的基线运行复制成另一类成果。",
        "",
        "## 指标口径",
        "",
        "- 常规指标从 `5.0 s` 开始；侧推组三条件统一截断到最短日志末尾，其余三组到 `11.99 s`。`最大 Roll/Pitch 分量` 为 "
        "`max(max|Roll|, max|Pitch|)`。",
        "- `MPC+WBC 非零状态样本数` 是统一窗口内 MPC 与 WBC 求解状态字段非零样本数之和。",
        "- MPC/WBC 时间字段是 qpOASES 报告的 QP CPU time，并以 100 Hz 写入 CSV；"
        "它不含预测矩阵构造和 QP 组装，不代表完整 MPC/WBC 计算时间。",
        "- 预测时域实验只改变 `N`。`N` 增大会扩展预测矩阵和未来窗口；控制时域固定 `ch=3`、"
        "输入维数 `nu=13`，所以 qpOASES 决策维数固定为 `39`，没有随 `N` 扩大。",
        "- 侧推实验中，`no_push` 是自然步态参照。峰值统计窗从施力开始持续 2.0 s（含施力阶段）。"
        "恢复计时从施力结束开始；`vx`、`vy` 相对内部参考误差分别不超过 `0.05 m/s`，"
        "Roll、Pitch 相对施力前 1.0 s 均值的偏差分别不超过 `0.03 rad`，四项连续满足至少 `0.5 s` 才算恢复。"
        "记录中断超过 `max(0.05 s, 2.5×中位采样间隔)` 时连续计时重置。",
        "- 相对 `no_push` 的正式扰动增量从 `7.00 s` 开始，最多观察至 `9.00 s`，并截断到最短日志末尾；按同时间戳逐样本计算。时间戳容差为 `1e-6 s`，"
        "不插值、不做最近邻匹配；时间网格不一致会使严格验收失败。Roll、Pitch、vx、vy 记录最大绝对差，"
        "额外高度下降记录 `max(0, max(no_push_z(t)-push_z(t)))`。",
        "",
    ]
    push_runs = all_runs.get("lateral_push", [])
    natural = next((run for run in push_runs if run.condition.name == "no_push"), None)
    lines += ["## 侧推恢复判据与时间对齐差分", ""]
    if natural is None:
        lines += ["缺少 `no_push` 自然步态参照，无法判断恢复判据适用性或计算时间对齐差分。", ""]
    elif natural.metrics.get("recovery_criterion_applicable") != "yes":
        lines += [
            "`no_push` 自然步态参照本身未连续满足固定恢复阈值，说明周期步态的正常起伏已超出该阈值。"
            "周期步态下该固定阈值判据无效，无法给出有效恢复时间，也不能据此判断 50 N/100 N 组是否恢复；"
            "原始 `recovered_in_window` 与 `recovery_time_s` 仍保存在指标 CSV 中供审计。",
            "",
        ]
    else:
        lines += ["`no_push` 自然步态参照通过固定恢复阈值的适用性检查。", ""]
    if push_runs:
        lines += [
            f"当前侧推批次：`{project_relative(active_group(root, 'lateral_push')[0])}`。"
            f"施力从 {md_number(push_runs[0].metrics.get('push_at_s'))} s 开始，"
            f"持续 {md_number(push_runs[0].metrics.get('push_duration_s'))} s；"
            f"共同扰动观察窗为 {md_number(push_runs[0].metrics.get('aligned_no_push_window_start_s'))}–"
            f"{md_number(push_runs[0].metrics.get('aligned_no_push_window_end_s'))} s。"
            "实际外力时序与配置逐样本核对，冲量与样本数保存在 `L2总指标.csv`。",
            "",
        ]
        lines += [
            "| 条件 | 对齐样本 | 最大 ΔRoll 绝对值 (rad) | 最大 ΔPitch 绝对值 (rad) | 最大 Δvx 绝对值 (m/s) | 最大 Δvy 绝对值 (m/s) | 额外高度下降 (m) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
        for run in push_runs:
            row = run.metrics
            lines.append(
                "| {condition} | {samples} | {roll} | {pitch} | {vx} | {vy} | {height} |".format(
                    condition=row.get("condition_label", row.get("condition", "—")),
                    samples=row.get("aligned_no_push_sample_count", "—") or "—",
                    roll=md_number(row.get("aligned_no_push_roll_max_abs_delta_rad")),
                    pitch=md_number(row.get("aligned_no_push_pitch_max_abs_delta_rad")),
                    vx=md_number(row.get("aligned_no_push_vx_max_abs_delta_m_s")),
                    vy=md_number(row.get("aligned_no_push_vy_max_abs_delta_m_s")),
                    height=md_number(row.get("aligned_no_push_extra_height_drop_max_m")),
                )
            )
        lines += [""]
    if complete_groups == len(all_runs):
        lines += [
            "四类实验均已具备三条件的有效原始数据，计划时长均为 12 s。"
            "正常完成条件按 12000 步/1200 样本验收；显式记录的物理失稳条件按真实停止时刻、"
            "步数和应有日志数量验收，不能补齐或伪装成完整运行。"
            "自动检查已核对运行状态、100 Hz 时间序列、必要数值有限性和清单一致性；"
            "最终验收仍需结合各组图像、视频和人工源码分析检查结论。",
            "",
        ]
    else:
        lines += [
            "当前自动数据完整性检查尚未全部通过，因此不能把 L2 写成已完成验收。补齐缺失运行后重新执行同一 Python 命令即可刷新本页。",
            "",
        ]
    summary_dir.mkdir(parents=True, exist_ok=True)
    (summary_dir / "L2实验结论.md").write_text("\n".join(lines), encoding="utf-8")


def analyze(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    root = args.l2_root.expanduser().resolve()
    names = args.group or list(GROUPS)
    all_runs: dict[str, list[RunData]] = {}
    issues: list[str] = []
    for name in names:
        spec = GROUPS[name]
        runs = load_group(root, spec)
        comparison = active_group(root, name)[0] / "comparison"
        comparison.mkdir(parents=True, exist_ok=True)
        write_metrics(comparison / "metrics.csv", [run.metrics for run in runs], include_experiment=False)
        plots = create_plots(spec, runs, comparison)
        write_analysis(spec, runs, plots, comparison / "analysis.md")
        if name == "lateral_push" and all(run.metrics.get("acceptance_validation_status") == "ok" for run in runs):
            write_push_presentation_summary(runs, comparison)
        all_runs[name] = runs
        for detail in single_variable_issues(runs):
            issues.append(f"{name}: 单变量/清单一致性检查：{detail}")
        for run in runs:
            if (
                run.metrics.get("run_outcome") not in ("completed", "physical_instability")
                or run.metrics.get("data_status") != "ok"
                or run.metrics.get("acceptance_validation_status") != "ok"
            ):
                issues.append(
                    f"{name}/{run.condition.name}: run_status={run.metrics.get('run_status')}, "
                    f"data_status={run.metrics.get('data_status')}, "
                    f"acceptance={run.metrics.get('acceptance_validation_status')}"
                )
                issues.extend(
                    f"{name}/{run.condition.name}: {detail}" for detail in run.validation_issues
                )
        print(f"[L2 analysis] {name}: {len(plots)} 张图，{project_relative(comparison / 'metrics.csv')}")
    # The global summary always represents all four experiment families, even
    # when --group was used to refresh only one comparison directory.
    summary_runs = {
        name: all_runs[name] if name in all_runs else load_group(root, spec)
        for name, spec in GROUPS.items()
    }
    write_summary(root, summary_runs)
    if issues:
        print("[L2 analysis] 数据完整性提示：", file=sys.stderr)
        for issue in issues:
            print(f"  - {issue}", file=sys.stderr)
        if args.strict:
            return 2
    print(f"[L2 analysis] 汇总：{project_relative(root / 'summary' / 'L2实验结论.md')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(analyze())
