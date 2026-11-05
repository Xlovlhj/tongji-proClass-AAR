#!/usr/bin/env python3
"""Validate and analyze the three-run L0 MPC input-weight tuning experiment.

The program never starts Isaac Sim.  It expects these completed run folders::

    Pro2/output/L0/mpc_input_weight/low_1e-7/
    Pro2/output/L0/mpc_input_weight/baseline_1e-6/
    Pro2/output/L0/mpc_input_weight/high_1e-5/

Each folder must contain ``run.json``, ``config.json`` and ``states.csv``.  The
data contract is deliberately strict: 12 s at 1 kHz, 1200 logged samples at
100 Hz, finite numeric data, a completed manifest, and a single changed
variable (``u_weight``).  Successful analysis writes the machine-readable
metrics, three figures and a short report into the sibling ``comparison``
folder.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    from analyze_l2_results import COLORS, Panel, Series, save_figure
except ImportError:  # imported as ``tools.analyze_l0_tuning`` in unit tests
    from .analyze_l2_results import COLORS, Panel, Series, save_figure


PROJECT = Path(__file__).resolve().parents[2]
CONFIG_ROOT = PROJECT / "Pro2" / "experiments" / "L0" / "mpc_input_weight"
DEFAULT_OUTPUT_ROOT = PROJECT / "Pro2" / "output" / "L0" / "mpc_input_weight"
ANALYSIS_START_S = 5.0
EXPECTED_DURATION_S = 12.0
EXPECTED_STEPS = 12_000
EXPECTED_SAMPLES = 1_200
EXPECTED_LOG_PERIOD_S = 0.01
TIME_TOLERANCE_S = 1e-7

DEFAULT_STATE_WEIGHTS = [1, 1, 1, 0.001, 100, 1, 0.001, 0.001, 0.001, 1, 100, 1]
DEFAULT_INPUT_WEIGHTS = [1] * 13


@dataclass(frozen=True)
class Condition:
    name: str
    label: str
    u_weight: float
    config_name: str


CONDITIONS = (
    Condition("low_1e-7", "u=1e-7", 1e-7, "low_1e-7.json"),
    Condition("baseline_1e-6", "u=1e-6", 1e-6, "baseline_1e-6.json"),
    Condition("high_1e-5", "u=1e-5", 1e-5, "high_1e-5.json"),
)

# Every field that can affect this comparison is pinned.  Runtime provenance
# fields such as isaac_root, output, jobs and config path are intentionally not
# compared because they do not change the simulated controller experiment.
EXPECTED_CONTROLLED_CONFIG: dict[str, Any] = {
    "mode": "walk",
    "duration": 12.0,
    "dt": 0.001,
    "mpc_dt": 0.005,
    "mpc_horizon_steps": 10,
    "swing_time": 0.4,
    "step_height": 0.12,
    "stand_height": 1.08,
    "walk_start": 3.0,
    "vx": 0.2,
    "vy": 0.0,
    "wz": 0.0,
    "render_hz": 30.0,
    "log_hz": 100.0,
    "headless": True,
    "record_video": False,
    "push_at": None,
    "push_duration": 0.1,
    "push_force": [0.0, 0.0, 0.0],
    "state_weights": DEFAULT_STATE_WEIGHTS,
    "input_weights": DEFAULT_INPUT_WEIGHTS,
}

REQUIRED_COLUMNS = (
    "time_s",
    "base_z_m",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "vx_world_m_s",
    "vy_world_m_s",
    "controller:command_vx_mps",
    "controller:mpc_qp_status",
    "controller:mpc_solve_s",
    "controller:wbc_qp_status",
)

METRIC_FIELDS = (
    "condition",
    "label",
    "u_weight",
    "validation_status",
    "sample_count_total",
    "sample_count_analysis",
    "analysis_start_s",
    "analysis_end_s",
    "body_vx_mean_m_s",
    "body_vx_internal_reference_rmse_m_s",
    "roll_rms_rad",
    "roll_max_abs_rad",
    "pitch_rms_rad",
    "pitch_max_abs_rad",
    "tilt_rms_rad",
    "tilt_max_rad",
    "base_height_mean_m",
    "base_height_std_m",
    "base_height_min_m",
    "peak_joint_torque_Nm",
    "joint_torque_vector_rms_Nm",
    "joint_torque_squared_integral_Nm2_s",
    "mpc_qp_failure_samples",
    "wbc_qp_failure_samples",
    "mpc_solve_mean_ms",
    "mpc_solve_p95_ms",
    "mpc_solve_max_ms",
    "balanced_score",
)


@dataclass
class RunData:
    condition: Condition
    directory: Path
    manifest: dict[str, Any] = field(default_factory=dict)
    config: dict[str, Any] = field(default_factory=dict)
    columns: dict[str, list[float]] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)
    issues: list[str] = field(default_factory=list)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="严格校验 L0 的三组 u_weight 运行并生成对比指标、图和分析。"
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="三组运行所在目录；默认 Pro2/output/L0/mpc_input_weight。",
    )
    return parser.parse_args(argv)


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def numeric_close(actual: Any, expected: float, tolerance: float = 1e-12) -> bool:
    return finite(actual) and math.isclose(float(actual), expected, rel_tol=1e-9, abs_tol=tolerance)


def mean(values: Iterable[float]) -> float:
    data = list(values)
    return sum(data) / len(data) if data else math.nan


def rms(values: Iterable[float]) -> float:
    data = list(values)
    return math.sqrt(sum(value * value for value in data) / len(data)) if data else math.nan


def standard_deviation(values: Iterable[float]) -> float:
    data = list(values)
    if not data:
        return math.nan
    center = mean(data)
    return math.sqrt(sum((value - center) ** 2 for value in data) / len(data))


def percentile(values: Iterable[float], fraction: float) -> float:
    data = sorted(values)
    if not data:
        return math.nan
    position = (len(data) - 1) * fraction
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return data[lower]
    alpha = position - lower
    return data[lower] * (1.0 - alpha) + data[upper] * alpha


def trapezoid(times: Sequence[float], values: Sequence[float]) -> float:
    return sum(
        0.5 * (values[index - 1] + values[index]) * (times[index] - times[index - 1])
        for index in range(1, min(len(times), len(values)))
    )


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"缺少 {path.name}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path.name} 无法读取：{exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} 顶层必须是 JSON 对象")
    return value


def _same_value(actual: Any, expected: Any) -> bool:
    if isinstance(expected, list):
        return (
            isinstance(actual, list)
            and len(actual) == len(expected)
            and all(_same_value(a, e) for a, e in zip(actual, expected))
        )
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        return actual == expected
    return numeric_close(actual, float(expected))


def controlled_config_issues(config: dict[str, Any], condition: Condition, source: str) -> list[str]:
    issues: list[str] = []
    for key, expected in EXPECTED_CONTROLLED_CONFIG.items():
        if key not in config:
            issues.append(f"{source} 缺少配置字段 {key}")
        elif not _same_value(config[key], expected):
            issues.append(f"{source}.{key}={config[key]!r}，应为 {expected!r}")
    if not numeric_close(config.get("u_weight"), condition.u_weight):
        issues.append(
            f"{source}.u_weight={config.get('u_weight')!r}，应为 {condition.u_weight:g}"
        )
    return issues


def validate_planned_configs(config_root: Path = CONFIG_ROOT) -> list[str]:
    """Verify that the committed designs differ only in ``u_weight``."""

    issues: list[str] = []
    masked: list[dict[str, Any]] = []
    for condition in CONDITIONS:
        try:
            config = load_json(config_root / condition.config_name)
        except ValueError as exc:
            issues.append(f"{condition.name}: {exc}")
            continue
        issues.extend(controlled_config_issues(config, condition, condition.config_name))
        current = dict(config)
        current.pop("u_weight", None)
        masked.append(current)
    if len(masked) == len(CONDITIONS) and any(item != masked[0] for item in masked[1:]):
        issues.append("三份实验 JSON 除 u_weight 外仍有其他字段差异")
    return issues


def load_states_strict(path: Path) -> tuple[dict[str, list[float]], list[str]]:
    issues: list[str] = []
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            raw_rows = list(reader)
    except FileNotFoundError as exc:
        raise ValueError("缺少 states.csv") from exc
    except (OSError, csv.Error) as exc:
        raise ValueError(f"states.csv 无法读取：{exc}") from exc
    if not fieldnames:
        raise ValueError("states.csv 没有表头")
    if len(fieldnames) != len(set(fieldnames)):
        issues.append("states.csv 存在重复列名")
    missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]
    torque_names = [name for name in fieldnames if name.startswith("tau_nm:")]
    if not torque_names:
        missing.append("tau_nm:*")
    if missing:
        issues.append("states.csv 缺少必要列：" + ", ".join(missing))

    columns = {name: [] for name in fieldnames}
    invalid: list[str] = []
    for row_number, row in enumerate(raw_rows, start=2):
        for name in fieldnames:
            try:
                value = float(row.get(name, ""))
            except (TypeError, ValueError):
                value = math.nan
            if not math.isfinite(value):
                if len(invalid) < 8:
                    invalid.append(f"{name}@第{row_number}行")
            columns[name].append(value)
    if invalid:
        issues.append("states.csv 含非有限或非数值单元格：" + ", ".join(invalid))
    return columns, issues


def validate_time_axis(times: Sequence[float]) -> list[str]:
    issues: list[str] = []
    if len(times) != EXPECTED_SAMPLES:
        issues.append(f"states.csv 数据行={len(times)}，应为 {EXPECTED_SAMPLES}")
        return issues
    if not numeric_close(times[0], 0.0, TIME_TOLERANCE_S):
        issues.append(f"首个 time_s={times[0]!r}，应为 0")
    expected_last = EXPECTED_DURATION_S - EXPECTED_LOG_PERIOD_S
    if not numeric_close(times[-1], expected_last, TIME_TOLERANCE_S):
        issues.append(f"末个 time_s={times[-1]!r}，应为 {expected_last:g}")
    for index, (previous, current) in enumerate(zip(times, times[1:]), start=3):
        if not math.isfinite(previous) or not math.isfinite(current):
            continue
        if current <= previous:
            issues.append(f"time_s 在 CSV 第 {index} 行未严格递增")
            break
        if not math.isclose(
            current - previous,
            EXPECTED_LOG_PERIOD_S,
            rel_tol=0.0,
            abs_tol=TIME_TOLERANCE_S,
        ):
            issues.append(
                f"time_s 在 CSV 第 {index} 行的间隔为 {current - previous:g} s，应为 0.01 s"
            )
            break
    return issues


def load_and_validate_run(output_root: Path, condition: Condition) -> RunData:
    run = RunData(condition, output_root / condition.name)
    if not run.directory.is_dir():
        run.issues.append(f"缺少运行目录 {run.directory}")
        return run
    try:
        run.manifest = load_json(run.directory / "run.json")
    except ValueError as exc:
        run.issues.append(str(exc))
    try:
        run.config = load_json(run.directory / "config.json")
    except ValueError as exc:
        run.issues.append(str(exc))

    manifest_config = run.manifest.get("config") if isinstance(run.manifest, dict) else None
    if run.manifest:
        if run.manifest.get("status") != "completed":
            run.issues.append(f"run.json.status={run.manifest.get('status')!r}，应为 'completed'")
        if run.manifest.get("steps") != EXPECTED_STEPS:
            run.issues.append(f"run.json.steps={run.manifest.get('steps')!r}，应为 {EXPECTED_STEPS}")
        if run.manifest.get("samples") != EXPECTED_SAMPLES:
            run.issues.append(f"run.json.samples={run.manifest.get('samples')!r}，应为 {EXPECTED_SAMPLES}")
        if not numeric_close(run.manifest.get("simulation_duration_s"), EXPECTED_DURATION_S):
            run.issues.append("run.json.simulation_duration_s 应为 12.0")
        if not isinstance(manifest_config, dict):
            run.issues.append("run.json.config 缺失或不是对象")
        else:
            run.issues.extend(
                controlled_config_issues(manifest_config, condition, "run.json.config")
            )
    if run.config:
        run.issues.extend(controlled_config_issues(run.config, condition, "config.json"))
    if isinstance(manifest_config, dict) and run.config:
        for key in (*EXPECTED_CONTROLLED_CONFIG, "u_weight"):
            if run.config.get(key) != manifest_config.get(key):
                run.issues.append(f"config.json 与 run.json.config 的 {key} 不一致")

    try:
        run.columns, states_issues = load_states_strict(run.directory / "states.csv")
        run.issues.extend(states_issues)
    except ValueError as exc:
        run.issues.append(str(exc))
    if run.columns:
        run.issues.extend(validate_time_axis(run.columns.get("time_s", [])))
        if run.manifest.get("samples") != len(run.columns.get("time_s", [])):
            run.issues.append("run.json.samples 与 states.csv 数据行数不一致")
    return run


def validate_actual_single_variable(runs: Sequence[RunData]) -> list[str]:
    issues: list[str] = []
    if any(not run.config for run in runs):
        return ["部分 config.json 缺失，无法核验三次实际运行的单变量条件"]
    reference = {
        key: runs[0].config.get(key) for key in EXPECTED_CONTROLLED_CONFIG
    }
    for run in runs[1:]:
        current = {key: run.config.get(key) for key in EXPECTED_CONTROLLED_CONFIG}
        differences = [key for key in reference if reference[key] != current[key]]
        if differences:
            issues.append(
                f"{runs[0].condition.name} 与 {run.condition.name} 除 u_weight 外还不同："
                + ", ".join(differences)
            )
    values = [run.config.get("u_weight") for run in runs]
    expected = [condition.u_weight for condition in CONDITIONS]
    if not all(numeric_close(value, target) for value, target in zip(values, expected)):
        issues.append(f"实际 u_weight 序列 {values!r} 与设计值 {expected!r} 不一致")
    return issues


def analysis_indices(times: Sequence[float]) -> list[int]:
    return [index for index, value in enumerate(times) if value >= ANALYSIS_START_S]


def body_forward_velocity(columns: dict[str, list[float]]) -> list[float]:
    return [
        math.cos(yaw) * vx + math.sin(yaw) * vy
        for yaw, vx, vy in zip(
            columns["yaw_rad"], columns["vx_world_m_s"], columns["vy_world_m_s"]
        )
    ]


def compute_metrics(run: RunData) -> dict[str, Any]:
    columns = run.columns
    times = columns["time_s"]
    indices = analysis_indices(times)
    if not indices:
        raise ValueError(f"{run.condition.name} 在 5.0 s 后没有数据")
    vx_body = body_forward_velocity(columns)
    columns["derived:body_vx_m_s"] = vx_body
    reference = columns["controller:command_vx_mps"]
    roll = [columns["roll_rad"][index] for index in indices]
    pitch = [columns["pitch_rad"][index] for index in indices]
    height = [columns["base_z_m"][index] for index in indices]
    speed_errors = [vx_body[index] - reference[index] for index in indices]
    tilt = [math.hypot(r, p) for r, p in zip(roll, pitch)]

    torque_names = sorted(name for name in columns if name.startswith("tau_nm:"))
    torque_square_sum: list[float] = []
    torque_peak: list[float] = []
    for index in indices:
        values = [columns[name][index] for name in torque_names]
        torque_square_sum.append(sum(value * value for value in values))
        torque_peak.append(max(abs(value) for value in values))
    full_peak_signal = []
    for index in range(len(times)):
        full_peak_signal.append(max(abs(columns[name][index]) for name in torque_names))
    columns["derived:peak_joint_torque_Nm"] = full_peak_signal
    analysis_times = [times[index] for index in indices]

    # Solver failures are a whole-run safety/validity check, while tracking,
    # posture and effort are evaluated only in the settled walking window.
    mpc_status = columns["controller:mpc_qp_status"]
    wbc_status = columns["controller:wbc_qp_status"]
    solve_ms = [columns["controller:mpc_solve_s"][index] * 1000.0 for index in indices]
    return {
        "condition": run.condition.name,
        "label": run.condition.label,
        "u_weight": run.condition.u_weight,
        "validation_status": "passed",
        "sample_count_total": len(times),
        "sample_count_analysis": len(indices),
        "analysis_start_s": analysis_times[0],
        "analysis_end_s": analysis_times[-1],
        "body_vx_mean_m_s": mean(vx_body[index] for index in indices),
        "body_vx_internal_reference_rmse_m_s": rms(speed_errors),
        "roll_rms_rad": rms(roll),
        "roll_max_abs_rad": max(abs(value) for value in roll),
        "pitch_rms_rad": rms(pitch),
        "pitch_max_abs_rad": max(abs(value) for value in pitch),
        "tilt_rms_rad": rms(tilt),
        "tilt_max_rad": max(tilt),
        "base_height_mean_m": mean(height),
        "base_height_std_m": standard_deviation(height),
        "base_height_min_m": min(height),
        "peak_joint_torque_Nm": max(torque_peak),
        "joint_torque_vector_rms_Nm": math.sqrt(mean(torque_square_sum)),
        "joint_torque_squared_integral_Nm2_s": trapezoid(
            analysis_times, torque_square_sum
        ),
        "mpc_qp_failure_samples": sum(int(round(value)) != 0 for value in mpc_status),
        "wbc_qp_failure_samples": sum(int(round(value)) != 0 for value in wbc_status),
        "mpc_solve_mean_ms": mean(solve_ms),
        "mpc_solve_p95_ms": percentile(solve_ms, 0.95),
        "mpc_solve_max_ms": max(solve_ms),
    }


def add_balanced_scores(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Add a transparent min-max score and return the selected condition."""

    weights = {
        "body_vx_internal_reference_rmse_m_s": 0.40,
        "tilt_max_rad": 0.20,
        "base_height_std_m": 0.15,
        "joint_torque_squared_integral_Nm2_s": 0.20,
        "mpc_solve_p95_ms": 0.05,
    }
    bounds = {
        field: (min(float(row[field]) for row in rows), max(float(row[field]) for row in rows))
        for field in weights
    }
    for row in rows:
        score = 0.0
        for field, weight in weights.items():
            low, high = bounds[field]
            normalized = 0.0 if math.isclose(low, high) else (float(row[field]) - low) / (high - low)
            score += weight * normalized
        row["balanced_score"] = score
    best_score = min(float(row["balanced_score"]) for row in rows)
    tied = [
        row
        for row in rows
        if math.isclose(float(row["balanced_score"]), best_score, rel_tol=0.0, abs_tol=1e-12)
    ]
    selected = next(
        (row for row in tied if row["condition"] == "baseline_1e-6"),
        min(tied, key=lambda row: float(row["u_weight"])),
    )
    return {
        "condition": selected["condition"],
        "u_weight": selected["u_weight"],
        "balanced_score": selected["balanced_score"],
        "method": "min-max weighted score; lower is better",
        "weights": weights,
    }


def write_metrics_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=METRIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: f"{value:.10g}" if isinstance(value, float) else value
                    for key, value in row.items()
                    if key in METRIC_FIELDS
                }
            )


def create_plots(runs: Sequence[RunData], output: Path) -> None:
    x_limits = (ANALYSIS_START_S, EXPECTED_DURATION_S)
    speed_series = [
        Series(
            run.condition.label,
            run.columns["time_s"],
            run.columns["derived:body_vx_m_s"],
            COLORS[index],
        )
        for index, run in enumerate(runs)
    ]
    reference = runs[1]
    speed_series.append(
        Series(
            "内部参考",
            reference.columns["time_s"],
            reference.columns["controller:command_vx_mps"],
            (75, 79, 88),
            True,
        )
    )
    save_figure(
        output / "speed_tracking.png",
        "MPC 输入权重：速度跟踪",
        (
            Panel(
                "机身前进速度",
                "时间 (s)",
                "速度 (m/s)",
                tuple(speed_series),
                x_limits,
            ),
        ),
    )

    def series(column: str) -> tuple[Series, ...]:
        return tuple(
            Series(
                run.condition.label,
                run.columns["time_s"],
                run.columns[column],
                COLORS[index],
            )
            for index, run in enumerate(runs)
        )

    save_figure(
        output / "posture.png",
        "MPC 输入权重：机身状态",
        (
            Panel("Roll", "时间 (s)", "角度 (rad)", series("roll_rad"), x_limits),
            Panel("Pitch", "时间 (s)", "角度 (rad)", series("pitch_rad"), x_limits),
            Panel("基座高度", "时间 (s)", "高度 (m)", series("base_z_m"), x_limits, 3),
        ),
    )
    save_figure(
        output / "control_effort.png",
        "MPC 输入权重：关节力矩",
        (
            Panel(
                "瞬时最大绝对关节力矩",
                "时间 (s)",
                "力矩 (N·m)",
                series("derived:peak_joint_torque_Nm"),
                x_limits,
            ),
        ),
    )


def markdown_number(value: Any, digits: int = 5) -> str:
    return f"{float(value):.{digits}g}" if finite(value) else "—"


def percent_change(value: float, baseline: float) -> str:
    if math.isclose(baseline, 0.0, abs_tol=1e-15):
        return "基线为零，不能计算百分比"
    change = (value - baseline) / abs(baseline) * 100.0
    action = "增加" if change > 0 else "降低" if change < 0 else "不变"
    return f"{action} {abs(change):.2f}%"


def write_analysis(path: Path, rows: Sequence[dict[str, Any]], recommendation: dict[str, Any]) -> None:
    baseline = next(row for row in rows if row["condition"] == "baseline_1e-6")
    selected = next(row for row in rows if row["condition"] == recommendation["condition"])
    lines = [
        "# L0：MPC 输入权重三点调参分析",
        "",
        "> 数据来自同目录三次 Isaac Sim 运行的 `run.json`、`config.json` 和 `states.csv`。分析程序只读取结果，不启动仿真。",
        "",
        "## 1. 实验设计",
        "",
        "本实验只改变凸 MPC 的输入代价总系数 `u_weight`：`1e-7`、`1e-6` 和 `1e-5`。其余仿真、运动、MPC、步态、默认状态权重与输入通道权重完全相同，并且不施加外力。三份配置分别位于：",
        "",
        "- `Pro2/experiments/L0/mpc_input_weight/low_1e-7.json`",
        "- `Pro2/experiments/L0/mpc_input_weight/baseline_1e-6.json`",
        "- `Pro2/experiments/L0/mpc_input_weight/high_1e-5.json`",
        "",
        "统一分析窗口为 `5.0 s` 到日志末尾。世界坐标速度先按机身偏航角转换为机身前向速度，再与控制器内部参考比较：",
        "",
        "$$",
        "v_x^B=\\cos(\\psi)v_x^W+\\sin(\\psi)v_y^W",
        "$$",
        "",
        "控制努力使用全部关节力矩平方和的时间积分：",
        "",
        "$$",
        "J_{\\tau}=\\int_{5\\,\\mathrm{s}}^{t_{\\mathrm{end}}}\\sum_j \\tau_j^2(t)\\,\\mathrm{d}t",
        "$$",
        "",
        "## 2. 数据完整性",
        "",
        "三次运行均通过严格检查：`status=completed`、`12000` 个物理步、`1200` 个日志样本、`0.01 s` 等间隔时间轴、所有数值有限，并且实际运行配置中只有 `u_weight` 不同。",
        "",
        "## 3. 指标结果",
        "",
        "| 条件 | vx RMSE (m/s) | 最大倾角 (rad) | 高度标准差 (m) | 最低高度 (m) | 峰值关节力矩 (N·m) | 力矩平方积分 (N²·m²·s) | MPC QP 失败样本 | MPC 求解 P95 (ms) | 综合分数 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {label} | {rmse} | {tilt} | {hstd} | {hmin} | {peak} | {effort} | {fail} | {p95} | {score} |".format(
                label=row["label"],
                rmse=markdown_number(row["body_vx_internal_reference_rmse_m_s"]),
                tilt=markdown_number(row["tilt_max_rad"]),
                hstd=markdown_number(row["base_height_std_m"]),
                hmin=markdown_number(row["base_height_min_m"]),
                peak=markdown_number(row["peak_joint_torque_Nm"]),
                effort=markdown_number(row["joint_torque_squared_integral_Nm2_s"]),
                fail=int(row["mpc_qp_failure_samples"]),
                p95=markdown_number(row["mpc_solve_p95_ms"]),
                score=markdown_number(row["balanced_score"]),
            )
        )
    lines += [
        "",
        "![速度跟踪](speed_tracking.png)",
        "",
        "![机身姿态与高度](posture.png)",
        "",
        "![控制努力](control_effort.png)",
        "",
        "## 4. 调参结论",
        "",
        f"按本实验的平衡评价，推荐值为 `{recommendation['u_weight']:g}`。该条件的综合分数为 `{recommendation['balanced_score']:.5g}`，三组中最低。",
        "",
        "综合分数先在三组内部对各指标做最小—最大归一化，再按以下权重相加：速度 RMSE 40%、最大倾角 20%、高度标准差 15%、力矩平方积分 20%、MPC 求解时间 P95 5%。分数越低，表示本次无扰动直行任务中的速度跟踪、稳定性、控制努力与计算时间折中越好。",
        "",
        f"与原始基线 `u_weight=1e-6` 相比，推荐条件的速度 RMSE {percent_change(selected['body_vx_internal_reference_rmse_m_s'], baseline['body_vx_internal_reference_rmse_m_s'])}，最大倾角 {percent_change(selected['tilt_max_rad'], baseline['tilt_max_rad'])}，力矩平方积分 {percent_change(selected['joint_torque_squared_integral_Nm2_s'], baseline['joint_torque_squared_integral_Nm2_s'])}。",
        "",
        "从控制意义上看，减小 `u_weight` 会降低对接触 wrench 偏离名义重力支撑输入的惩罚，控制器可以更积极地追踪状态参考，但可能增大关节控制努力；增大 `u_weight` 会更强地约束这种偏离，可能降低控制努力，也可能牺牲速度跟踪。实际结果必须同时查看速度、姿态、高度、力矩和 QP 状态，不能只按某一个指标判断。",
        "",
        "本结论针对固定地形、固定速度、无扰动和单次确定性运行。若用于更广的最终参数选择，还需要在转向、外力和不同地形条件下复核。完整数值保存在 `metrics.csv` 和 `metrics.json`。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def analyze(output_root: Path = DEFAULT_OUTPUT_ROOT) -> tuple[list[RunData], dict[str, Any]]:
    comparison = output_root / "comparison"
    comparison.mkdir(parents=True, exist_ok=True)
    planned_issues = validate_planned_configs()
    runs = [load_and_validate_run(output_root, condition) for condition in CONDITIONS]
    actual_issues = validate_actual_single_variable(runs)
    all_issues = {
        "planned_configs": planned_issues,
        "actual_single_variable": actual_issues,
        "runs": {run.condition.name: run.issues for run in runs},
    }
    if planned_issues or actual_issues or any(run.issues for run in runs):
        (comparison / "validation_errors.json").write_text(
            json.dumps(all_issues, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        details = sum(len(value) for value in all_issues["runs"].values())
        details += len(planned_issues) + len(actual_issues)
        raise ValueError(
            f"L0 调参数据严格校验失败，共 {details} 项；详见 "
            f"{comparison / 'validation_errors.json'}"
        )

    for run in runs:
        run.metrics = compute_metrics(run)
    rows = [run.metrics for run in runs]
    recommendation = add_balanced_scores(rows)
    write_metrics_csv(comparison / "metrics.csv", rows)
    payload = {
        "experiment": "L0 MPC input-weight tuning",
        "analysis_window_s": [
            rows[0]["analysis_start_s"],
            rows[0]["analysis_end_s"],
        ],
        "validation": {
            "status": "passed",
            "single_variable": "u_weight",
            "expected_steps_per_run": EXPECTED_STEPS,
            "expected_samples_per_run": EXPECTED_SAMPLES,
        },
        "recommendation": recommendation,
        "runs": rows,
    }
    (comparison / "metrics.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    create_plots(runs, comparison)
    write_analysis(comparison / "analysis.md", rows, recommendation)
    error_path = comparison / "validation_errors.json"
    if error_path.exists():
        error_path.unlink()
    return runs, payload


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        _, result = analyze(args.output_root)
    except ValueError as exc:
        print(f"错误：{exc}")
        return 1
    recommendation = result["recommendation"]
    print("L0 三点调参分析完成")
    print(f"推荐 u_weight={recommendation['u_weight']:g}")
    print(f"成果目录：{args.output_root / 'comparison'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
