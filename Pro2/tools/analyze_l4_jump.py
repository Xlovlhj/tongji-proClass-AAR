#!/usr/bin/env python3
"""Validate one isolated L4 vertical-jump run and draw its evidence plots.

The input directory is read-only.  Results are written to ``analysis/`` below
that run unless ``--output`` names another, still isolated, child directory.
An existing output directory is never overwritten.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median
from typing import Iterable, Sequence

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover - broken project environment
    raise SystemExit("缺少项目已有依赖 Pillow，无法生成跳跃验收图。") from exc


PHASES = (
    "prepare", "crouch", "thrust", "flight_up", "flight_down",
    "landing", "recovery", "complete",
)
NUMERIC_FIELDS = (
    "time_s", "base_x_m", "base_y_m", "base_z_m", "roll_rad",
    "pitch_rad", "vx_world_m_s", "vy_world_m_s", "vz_world_m_s",
    "force_x_n", "force_y_n", "force_z_n", "controller:mpc_enabled",
    "controller:mpc_qp_status", "controller:wbc_qp_status",
    "jump:decision_left_contact_force_z_n",
    "jump:decision_right_contact_force_z_n",
    "jump:height_reference_m",
)
STRING_FIELDS = ("jump:phase",)

TERMINAL_WINDOW_S = 0.50
CONTACT_FORCE_N = 20.0
CONTACT_FRACTION_MIN = 0.95
SECOND_FLIGHT_MAX_S = 0.05
TERMINAL_HORIZONTAL_SPEED_MAX_M_S = 0.20
TERMINAL_VERTICAL_SPEED_MAX_M_S = 0.15
TERMINAL_TILT_MAX_RAD = 0.20
FINAL_HEIGHT_ERROR_MAX_M = 0.12
PLANAR_DRIFT_BASE_MAX_M = 0.30
QP_SUCCESS_STATUS = 0

COLORS = (
    (25, 105, 170),
    (214, 118, 0),
    (45, 145, 93),
    (179, 62, 68),
)
GRID = (222, 226, 232)
INK = (42, 47, 55)


def _load_json(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"缺少文件：{path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} 顶层必须是 JSON 对象")
    return value


def _load_states(path: Path) -> dict[str, list]:
    if not path.is_file():
        raise FileNotFoundError(f"缺少文件：{path}")
    wanted = set(NUMERIC_FIELDS + STRING_FIELDS)
    result: dict[str, list] = {name: [] for name in wanted}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)):
            raise ValueError(f"CSV 含重复列名：{path}")
        missing = sorted(wanted - set(fields))
        if missing:
            raise KeyError("states.csv 缺少：" + ", ".join(missing))
        for line_number, row in enumerate(reader, 2):
            for name in STRING_FIELDS:
                value = row.get(name)
                if value is None or not value.strip():
                    raise ValueError(f"{path}:{line_number} 的 {name} 为空")
                result[name].append(value.strip())
            for name in NUMERIC_FIELDS:
                try:
                    value = float(row[name])
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(
                        f"{path}:{line_number} 的 {name} 不是数值"
                    ) from exc
                if not math.isfinite(value):
                    raise ValueError(f"{path}:{line_number} 的 {name} 含 NaN/Inf")
                result[name].append(value)
    if not result["time_s"]:
        raise ValueError(f"CSV 没有数据行：{path}")
    times = result["time_s"]
    if any(right <= left for left, right in zip(times, times[1:])):
        raise ValueError("states.csv 的 time_s 必须严格递增")
    return result


def _transitions(phases: Sequence[str]) -> list[str]:
    result: list[str] = []
    for phase in phases:
        if not result or phase != result[-1]:
            result.append(phase)
    return result


def _indices_at_or_after(times: Sequence[float], start: float) -> list[int]:
    return [index for index, value in enumerate(times) if value >= start]


def _maximum_true_duration(mask: Sequence[bool], sample_dt: float) -> float:
    longest = current = 0
    for value in mask:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest * sample_dt


def _true_runs(mask: Sequence[bool]) -> list[tuple[int, int]]:
    """Return inclusive index ranges for every contiguous true interval."""
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, value in enumerate(mask):
        if value and start is None:
            start = index
        elif not value and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(mask) - 1))
    return runs


def _finite_number(value, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    if minimum is not None and result < minimum:
        raise ValueError(f"{name} 必须 >= {minimum:g}")
    return result


def _max_abs(values: Iterable[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def _check(name: str, passed: bool, actual, requirement: str) -> dict:
    return {
        "name": name,
        "passed": bool(passed),
        "actual": actual,
        "requirement": requirement,
    }


def compute_acceptance(run_dir: Path) -> tuple[dict, dict[str, list]]:
    """Read and independently validate one jump run without writing files."""
    run_dir = run_dir.expanduser().resolve()
    run = _load_json(run_dir / "run.json")
    jump = _load_json(run_dir / "jump_metrics.json")
    columns = _load_states(run_dir / "states.csv")

    config = run.get("config", {})
    jump_config = config.get("jump", {}) if isinstance(config, dict) else {}
    if not isinstance(jump_config, dict):
        jump_config = {}
    stand_height = _finite_number(
        jump_config.get("stand_height_m", 1.08),
        "run.json.config.jump.stand_height_m")
    takeoff_force_threshold = _finite_number(
        jump_config.get("takeoff_force_threshold_n"),
        "run.json.config.jump.takeoff_force_threshold_n", minimum=0.0)
    landing_force_threshold = _finite_number(
        jump_config.get("landing_force_threshold_n"),
        "run.json.config.jump.landing_force_threshold_n", minimum=0.0)
    if takeoff_force_threshold >= landing_force_threshold:
        raise ValueError("起跳接触力阈值必须小于落地接触力阈值")
    contact_confirm_duration = _finite_number(
        jump_config.get("contact_confirm_duration_s", 0.0),
        "run.json.config.jump.contact_confirm_duration_s", minimum=0.0)
    jump_start_time = _finite_number(
        jump_config.get("start_time_s"),
        "run.json.config.jump.start_time_s", minimum=0.0)

    summary_minimum_rise = _finite_number(
        jump.get("minimum_required_rise_m"),
        "jump_metrics.json.minimum_required_rise_m", minimum=0.0)
    target_height = jump_config.get("target_height_m")
    if target_height is not None:
        target_height = _finite_number(
            target_height, "run.json.config.jump.target_height_m", minimum=0.0)
        # Keep this synchronized with simulation.py. Recomputing from the run
        # configuration prevents a modified summary from weakening the limit.
        minimum_rise = max(0.03, 0.5 * target_height)
        minimum_rise_source = (
            "max(0.03, 0.5 * run.json.config.jump.target_height_m)")
        minimum_rise_threshold_consistent = math.isclose(
            summary_minimum_rise, minimum_rise, rel_tol=0.0, abs_tol=1e-9)
    else:
        minimum_rise = summary_minimum_rise
        minimum_rise_source = "jump_metrics.json.minimum_required_rise_m"
        minimum_rise_threshold_consistent = True
    landing_advance = _finite_number(
        jump_config.get("landing_foot_advance_m", 0.0),
        "run.json.config.jump.landing_foot_advance_m", minimum=0.0)
    drift_limit = max(PLANAR_DRIFT_BASE_MAX_M, landing_advance + 0.10)

    times = columns["time_s"]
    sample_dt = median(right - left for left, right in zip(times, times[1:])) \
        if len(times) > 1 else math.inf
    event_time_tolerance = sample_dt + contact_confirm_duration
    terminal = _indices_at_or_after(times, times[-1] - TERMINAL_WINDOW_S)
    if len(terminal) < 2:
        raise ValueError("states.csv 末段不足两个采样点，无法验收稳定性")

    phases = _transitions(columns["jump:phase"])
    manifest_phases = [
        row.get("phase") for row in jump.get("phase_sequence", [])
        if isinstance(row, dict)
    ]
    takeoff_time = jump.get("takeoff_time_s")
    touchdown_time = jump.get("touchdown_time_s")
    valid_events = (
        isinstance(takeoff_time, (int, float)) and
        not isinstance(takeoff_time, bool) and
        isinstance(touchdown_time, (int, float)) and
        not isinstance(touchdown_time, bool) and
        math.isfinite(float(takeoff_time)) and
        math.isfinite(float(touchdown_time)) and
        float(touchdown_time) > float(takeoff_time)
    )

    flight_indices = [
        index for index, phase in enumerate(columns["jump:phase"])
        if phase in {"flight_up", "flight_down"}
    ]
    flight_mpc_enabled_count = sum(
        columns["controller:mpc_enabled"][index] >= 0.5
        for index in flight_indices
    )
    external_force_peak = max(
        math.sqrt(
            columns["force_x_n"][index] ** 2 +
            columns["force_y_n"][index] ** 2 +
            columns["force_z_n"][index] ** 2
        )
        for index in range(len(times))
    )

    left_force = columns["jump:decision_left_contact_force_z_n"]
    right_force = columns["jump:decision_right_contact_force_z_n"]

    # Reconstruct takeoff from raw forces after the configured thrust start.
    # This rejects sensor warm-up zeros without using summary event times or
    # trusting the logged phase label to prove contact loss.
    airborne_mask = [
        time_s >= jump_start_time and
        left <= takeoff_force_threshold and
        right <= takeoff_force_threshold
        for time_s, left, right in zip(
            times, left_force, right_force)
    ]
    airborne_runs = [
        (start, end) for start, end in _true_runs(airborne_mask)
        if end > start and
        times[end] - times[start] + 1e-12 >= contact_confirm_duration
    ]
    # The first interval that is sustained for at least two samples is the
    # conservative logged takeoff. It is selected without consulting JSON.
    airborne_run = airborne_runs[0] if airborne_runs else None
    csv_takeoff_index = airborne_run[0] if airborne_run is not None else None
    csv_airborne_end_index = airborne_run[1] if airborne_run is not None else None

    # The first bilateral threshold crossing after flight is the raw landing.
    # Later terminal-contact checks independently verify that support persists.
    csv_landing_index = None
    if csv_airborne_end_index is not None:
        csv_landing_index = next((
            index for index in range(csv_airborne_end_index + 1, len(times))
            if left_force[index] >= landing_force_threshold and
            right_force[index] >= landing_force_threshold
        ), None)

    raw_events_valid = (
        csv_takeoff_index is not None and
        csv_airborne_end_index is not None and
        csv_landing_index is not None and
        csv_landing_index > csv_airborne_end_index
    )
    if raw_events_valid:
        csv_takeoff_time = times[csv_takeoff_index]
        csv_landing_time = times[csv_landing_index]
        airborne_sample_count = csv_airborne_end_index - csv_takeoff_index + 1
        airborne_duration = (
            times[csv_airborne_end_index] - times[csv_takeoff_index])
        apex_index = max(
            range(csv_takeoff_index, csv_landing_index + 1),
            key=lambda index: columns["base_z_m"][index])
        raw_actual_rise = (
            columns["base_z_m"][apex_index] -
            columns["base_z_m"][csv_takeoff_index])
        post_touchdown = range(csv_landing_index, len(times))
        both_airborne = [
            left_force[index] <= takeoff_force_threshold and
            right_force[index] <= takeoff_force_threshold
            for index in post_touchdown
        ]
        second_flight_duration = _maximum_true_duration(
            both_airborne, sample_dt)
    else:
        csv_takeoff_time = None
        csv_landing_time = None
        airborne_sample_count = 0
        airborne_duration = 0.0
        apex_index = None
        raw_actual_rise = None
        second_flight_duration = math.inf

    takeoff_time_delta = (
        csv_takeoff_time - float(takeoff_time)
        if raw_events_valid and valid_events else None)
    touchdown_time_delta = (
        csv_landing_time - float(touchdown_time)
        if raw_events_valid and valid_events else None)
    event_timing_consistent = (
        raw_events_valid and valid_events and
        abs(takeoff_time_delta) <= event_time_tolerance + 1e-12 and
        abs(touchdown_time_delta) <= event_time_tolerance + 1e-12
    )

    terminal_left_fraction = sum(
        left_force[index] >= CONTACT_FORCE_N for index in terminal
    ) / len(terminal)
    terminal_right_fraction = sum(
        right_force[index] >= CONTACT_FORCE_N for index in terminal
    ) / len(terminal)
    terminal_bilateral_fraction = sum(
        left_force[index] >= CONTACT_FORCE_N and
        right_force[index] >= CONTACT_FORCE_N for index in terminal
    ) / len(terminal)

    terminal_horizontal_speeds = [
        math.hypot(columns["vx_world_m_s"][index],
                   columns["vy_world_m_s"][index])
        for index in terminal
    ]
    terminal_height_errors = [
        abs(columns["base_z_m"][index] - stand_height) for index in terminal
    ]
    terminal_tilts = [
        max(abs(columns["roll_rad"][index]),
            abs(columns["pitch_rad"][index]))
        for index in terminal
    ]
    max_terminal_horizontal_speed = max(terminal_horizontal_speeds)
    max_terminal_vertical_speed = _max_abs(
        columns["vz_world_m_s"][index] for index in terminal)
    max_terminal_tilt = max(terminal_tilts)
    max_terminal_height_error = max(terminal_height_errors)

    planar_drift = math.hypot(
        columns["base_x_m"][-1] - columns["base_x_m"][0],
        columns["base_y_m"][-1] - columns["base_y_m"][0],
    )

    active_mpc = [
        index for index, enabled in enumerate(columns["controller:mpc_enabled"])
        if enabled >= 0.5
    ]
    mpc_qp_failures = sum(
        round(columns["controller:mpc_qp_status"][index]) != QP_SUCCESS_STATUS
        for index in active_mpc
    )
    active_wbc = [
        index for index, phase in enumerate(columns["jump:phase"])
        if phase not in {"flight_up", "flight_down"}
    ]
    wbc_qp_failures = sum(
        round(columns["controller:wbc_qp_status"][index]) != QP_SUCCESS_STATUS
        for index in active_wbc
    )

    actual_rise = jump.get("actual_rise_m")
    summary_rise_valid = (
        isinstance(actual_rise, (int, float)) and
        not isinstance(actual_rise, bool) and
        math.isfinite(float(actual_rise)))
    rise_ok = (
        raw_actual_rise is not None and
        raw_actual_rise + 1e-12 >= minimum_rise)
    metrics = {
        "logged_samples": len(times),
        "logged_start_time_s": times[0],
        "logged_end_time_s": times[-1],
        "median_log_period_s": sample_dt,
        "phase_sequence_from_states": phases,
        "phase_sequence_from_jump_metrics": manifest_phases,
        "takeoff_time_s": takeoff_time,
        "touchdown_time_s": touchdown_time,
        "csv_takeoff_sample_index": csv_takeoff_index,
        "csv_takeoff_time_s": csv_takeoff_time,
        "csv_takeoff_base_height_m": (
            columns["base_z_m"][csv_takeoff_index]
            if csv_takeoff_index is not None else None),
        "csv_airborne_last_sample_index": csv_airborne_end_index,
        "csv_airborne_last_time_s": (
            times[csv_airborne_end_index]
            if csv_airborne_end_index is not None else None),
        "csv_airborne_sample_count": airborne_sample_count,
        "csv_airborne_observed_duration_s": airborne_duration,
        "csv_landing_sample_index": csv_landing_index,
        "csv_landing_time_s": csv_landing_time,
        "csv_landing_base_height_m": (
            columns["base_z_m"][csv_landing_index]
            if csv_landing_index is not None else None),
        "csv_apex_sample_index": apex_index,
        "csv_apex_time_s": times[apex_index] if apex_index is not None else None,
        "csv_apex_base_height_m": (
            columns["base_z_m"][apex_index] if apex_index is not None else None),
        "csv_actual_rise_m": raw_actual_rise,
        "jump_metrics_actual_rise_m": actual_rise,
        # Kept for downstream readers of schema v1; acceptance no longer uses it.
        "actual_rise_m": actual_rise,
        "rise_difference_jump_metrics_minus_csv_m": (
            float(actual_rise) - raw_actual_rise
            if summary_rise_valid and raw_actual_rise is not None else None),
        "minimum_required_rise_m": minimum_rise,
        "minimum_required_rise_from_jump_metrics_m": summary_minimum_rise,
        "minimum_required_rise_source": minimum_rise_source,
        "csv_takeoff_minus_jump_metrics_s": takeoff_time_delta,
        "csv_landing_minus_jump_metrics_s": touchdown_time_delta,
        "event_time_tolerance_s": event_time_tolerance,
        "flight_mpc_enabled_sample_count": flight_mpc_enabled_count,
        "external_force_peak_n": external_force_peak,
        "maximum_post_touchdown_both_airborne_duration_s": second_flight_duration,
        "terminal_window_s": TERMINAL_WINDOW_S,
        "terminal_sample_count": len(terminal),
        "terminal_left_contact_fraction": terminal_left_fraction,
        "terminal_right_contact_fraction": terminal_right_fraction,
        "terminal_bilateral_contact_fraction": terminal_bilateral_fraction,
        "terminal_max_horizontal_speed_m_s": max_terminal_horizontal_speed,
        "terminal_max_abs_vertical_speed_m_s": max_terminal_vertical_speed,
        "terminal_max_tilt_rad": max_terminal_tilt,
        "terminal_max_height_error_m": max_terminal_height_error,
        "final_base_position_m": [
            columns["base_x_m"][-1], columns["base_y_m"][-1],
            columns["base_z_m"][-1],
        ],
        "planar_drift_m": planar_drift,
        "planar_drift_limit_m": drift_limit,
        "active_mpc_sample_count": len(active_mpc),
        "mpc_qp_failure_sample_count": mpc_qp_failures,
        "active_wbc_sample_count": len(active_wbc),
        "wbc_qp_failure_sample_count": wbc_qp_failures,
    }

    checks = [
        _check("run_completed", run.get("status") == "completed",
               run.get("status"), "run.json.status == completed"),
        _check("complete_phase_sequence",
               phases == list(PHASES) and manifest_phases == list(PHASES) and
               jump.get("completed_phase_sequence") is True and
               jump.get("final_phase") == "complete",
               {"states": phases, "jump_metrics": manifest_phases,
                "final_phase": jump.get("final_phase")},
               "状态与汇总均依次包含 prepare 至 complete 八阶段"),
        _check("takeoff_and_touchdown", valid_events,
               {"takeoff_time_s": takeoff_time,
                "touchdown_time_s": touchdown_time},
               "起跳和落地时刻均存在，且落地晚于起跳"),
        _check("sustained_airborne_interval", airborne_run is not None,
               {"start_time_s": csv_takeoff_time,
                "end_time_s": (times[csv_airborne_end_index]
                               if csv_airborne_end_index is not None else None),
                "samples": airborne_sample_count,
                "observed_duration_s": airborne_duration},
               "配置起跳时刻后至少两个连续样本的双足力均不高于起跳阈值"),
        _check("bilateral_landing_after_airborne", raw_events_valid,
               {"landing_time_s": csv_landing_time,
                "left_force_n": (left_force[csv_landing_index]
                                 if csv_landing_index is not None else None),
                "right_force_n": (right_force[csv_landing_index]
                                  if csv_landing_index is not None else None)},
               "持续腾空后出现双足力均不低于落地阈值的接触"),
        _check("event_timing_consistency", event_timing_consistent,
               {"takeoff_delta_s": takeoff_time_delta,
                "touchdown_delta_s": touchdown_time_delta,
                "tolerance_s": event_time_tolerance},
               "CSV 独立事件与高频 jump_metrics 事件之差均不超过日志周期加确认时长"),
        _check("minimum_rise_threshold_consistency",
               minimum_rise_threshold_consistent,
               {"recomputed_m": minimum_rise,
                "jump_metrics_m": summary_minimum_rise,
                "source": minimum_rise_source},
               "配置可推导阈值时，jump_metrics 阈值必须与配置重算值一致"),
        _check("minimum_jump_rise", rise_ok,
               {"csv_actual_rise_m": raw_actual_rise,
                "jump_metrics_actual_rise_m": actual_rise},
               f"states.csv 独立重算高度增量 >= {minimum_rise:.3f} m；"
               "JSON actual_rise 不单独决定结果"),
        _check("jump_metrics_minimum_jump_rise",
               summary_rise_valid and
               float(actual_rise) + 1e-12 >= summary_minimum_rise,
               actual_rise,
               "jump_metrics.actual_rise_m 也必须是有限值且达到其记录阈值"),
        _check("flight_mpc_disabled",
               bool(flight_indices) and flight_mpc_enabled_count == 0 and
               jump.get("flight_mpc_disabled") is True,
               {"flight_samples": len(flight_indices),
                "enabled_samples": flight_mpc_enabled_count,
                "summary_flag": jump.get("flight_mpc_disabled")},
               "腾空阶段有日志且所有样本 controller:mpc_enabled == 0"),
        _check("external_force_assist_disabled",
               jump.get("external_force_assist") is False and
               external_force_peak <= 1e-9,
               {"summary_flag": jump.get("external_force_assist"),
                "peak_logged_force_n": external_force_peak},
               "外力辅助标志为 false，且三轴外力日志均为 0"),
        _check("no_second_flight_after_touchdown",
               raw_events_valid and
               second_flight_duration <= SECOND_FLIGHT_MAX_S,
               second_flight_duration,
               f"落地后双脚同时失去接触不连续超过 {SECOND_FLIGHT_MAX_S:.2f} s"),
        _check("terminal_bilateral_contact",
               min(terminal_left_fraction, terminal_right_fraction,
                   terminal_bilateral_fraction) >= CONTACT_FRACTION_MIN,
               {"left": terminal_left_fraction,
                "right": terminal_right_fraction,
                "bilateral": terminal_bilateral_fraction},
               f"末 {TERMINAL_WINDOW_S:.2f} s 左、右和双足接触占比均 >= "
               f"{CONTACT_FRACTION_MIN:.0%}（接触阈值 {CONTACT_FORCE_N:g} N）"),
        _check("terminal_horizontal_speed",
               max_terminal_horizontal_speed <= TERMINAL_HORIZONTAL_SPEED_MAX_M_S,
               max_terminal_horizontal_speed,
               f"末段最大水平速度 <= {TERMINAL_HORIZONTAL_SPEED_MAX_M_S:.2f} m/s"),
        _check("terminal_vertical_speed",
               max_terminal_vertical_speed <= TERMINAL_VERTICAL_SPEED_MAX_M_S,
               max_terminal_vertical_speed,
               f"末段最大竖直速度绝对值 <= {TERMINAL_VERTICAL_SPEED_MAX_M_S:.2f} m/s"),
        _check("terminal_posture", max_terminal_tilt <= TERMINAL_TILT_MAX_RAD,
               max_terminal_tilt,
               f"末段最大 |roll|/|pitch| <= {TERMINAL_TILT_MAX_RAD:.2f} rad"),
        _check("terminal_height", max_terminal_height_error <= FINAL_HEIGHT_ERROR_MAX_M,
               max_terminal_height_error,
               f"末段基座高度相对 {stand_height:.3f} m 的最大误差 <= "
               f"{FINAL_HEIGHT_ERROR_MAX_M:.2f} m"),
        _check("planar_drift", planar_drift <= drift_limit, planar_drift,
               f"全程水平净漂移 <= {drift_limit:.2f} m"),
        _check("mpc_qp_success", bool(active_mpc) and mpc_qp_failures == 0,
               {"active_samples": len(active_mpc),
                "failure_samples": mpc_qp_failures},
               "MPC 开启样本均为 QP status 0"),
        _check("wbc_qp_success", bool(active_wbc) and wbc_qp_failures == 0,
               {"active_samples": len(active_wbc),
                "failure_samples": wbc_qp_failures},
               "非腾空 WBC 样本均为 QP status 0"),
        _check("builtin_jump_pass", jump.get("pass") is True,
               jump.get("pass"), "仿真内建 jump_metrics.pass == true"),
    ]
    result = {
        "schema_version": 2,
        "run_directory": str(run_dir),
        "source_files": ["run.json", "jump_metrics.json", "states.csv"],
        "thresholds": {
            "terminal_window_s": TERMINAL_WINDOW_S,
            "contact_force_n": CONTACT_FORCE_N,
            "takeoff_force_threshold_n": takeoff_force_threshold,
            "landing_force_threshold_n": landing_force_threshold,
            "contact_confirm_duration_s": contact_confirm_duration,
            "jump_start_time_s": jump_start_time,
            "event_time_tolerance_s": event_time_tolerance,
            "minimum_required_rise_m": minimum_rise,
            "minimum_required_rise_source": minimum_rise_source,
            "minimum_terminal_contact_fraction": CONTACT_FRACTION_MIN,
            "maximum_second_flight_duration_s": SECOND_FLIGHT_MAX_S,
            "maximum_terminal_horizontal_speed_m_s":
                TERMINAL_HORIZONTAL_SPEED_MAX_M_S,
            "maximum_terminal_vertical_speed_m_s":
                TERMINAL_VERTICAL_SPEED_MAX_M_S,
            "maximum_terminal_tilt_rad": TERMINAL_TILT_MAX_RAD,
            "maximum_terminal_height_error_m": FINAL_HEIGHT_ERROR_MAX_M,
            "maximum_planar_drift_m": drift_limit,
            "qp_success_status": QP_SUCCESS_STATUS,
        },
        "metrics": metrics,
        "checks": checks,
        "pass": all(check["passed"] for check in checks),
    }
    return result, columns


def _font(size: int, bold: bool = False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    paths = (
        Path("/usr/share/fonts/truetype/dejavu") / name,
        Path("/usr/share/fonts/dejavu") / name,
    )
    for path in paths:
        if path.is_file():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def _bounds(series: Sequence[Sequence[float]]) -> tuple[float, float]:
    values = [value for line in series for value in line]
    low, high = min(values), max(values)
    if math.isclose(low, high):
        padding = max(0.05, abs(low) * 0.1)
    else:
        padding = 0.08 * (high - low)
    return low - padding, high + padding


def _line_chart(path: Path, title: str, x: Sequence[float],
                series: Sequence[tuple[str, Sequence[float], tuple[int, int, int]]],
                x_label: str, y_label: str) -> None:
    width, height = 1400, 820
    left, top, right, bottom = 125, 105, 50, 105
    plot = (left, top, width - right, height - bottom)
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    title_font, label_font, tick_font = _font(30, True), _font(23), _font(19)
    draw.text((width / 2, 42), title, fill=INK, font=title_font, anchor="mm")
    x0, x1 = min(x), max(x)
    if math.isclose(x0, x1):
        x1 = x0 + 1.0
    y0, y1 = _bounds([line for _, line, _ in series])
    for step in range(6):
        fraction = step / 5
        px = plot[0] + fraction * (plot[2] - plot[0])
        py = plot[3] - fraction * (plot[3] - plot[1])
        draw.line((px, plot[1], px, plot[3]), fill=GRID, width=1)
        draw.line((plot[0], py, plot[2], py), fill=GRID, width=1)
        draw.text((px, plot[3] + 18), f"{x0 + fraction*(x1-x0):.2f}",
                  fill=INK, font=tick_font, anchor="ma")
        draw.text((plot[0] - 15, py), f"{y0 + fraction*(y1-y0):.2f}",
                  fill=INK, font=tick_font, anchor="rm")
    draw.rectangle(plot, outline=INK, width=2)
    for name, values, color in series:
        points = [
            (plot[0] + (value_x - x0) / (x1 - x0) * (plot[2] - plot[0]),
             plot[3] - (value_y - y0) / (y1 - y0) * (plot[3] - plot[1]))
            for value_x, value_y in zip(x, values)
        ]
        if len(points) >= 2:
            draw.line(points, fill=color, width=3, joint="curve")
    draw.text(((plot[0] + plot[2]) / 2, height - 45), x_label,
              fill=INK, font=label_font, anchor="mm")
    draw.text((34, (plot[1] + plot[3]) / 2), y_label, fill=INK,
              font=label_font, anchor="mm")
    legend_x = plot[2] - 15
    legend_y = plot[1] + 18
    for name, _, color in reversed(series):
        text_box = draw.textbbox((0, 0), name, font=label_font)
        text_width = text_box[2] - text_box[0]
        legend_x -= text_width + 54
        draw.line((legend_x, legend_y, legend_x + 32, legend_y), fill=color, width=4)
        draw.text((legend_x + 40, legend_y), name, fill=INK,
                  font=label_font, anchor="lm")
        legend_x -= 22
    image.save(path)


def _trajectory_chart(path: Path, xs: Sequence[float], ys: Sequence[float]) -> None:
    width = height = 900
    margin = 115
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    title_font, label_font, tick_font = _font(30, True), _font(23), _font(19)
    draw.text((width / 2, 42), "Base planar trajectory", fill=INK,
              font=title_font, anchor="mm")
    x0, x1 = _bounds([xs])
    y0, y1 = _bounds([ys])
    span = max(x1 - x0, y1 - y0)
    x_mid, y_mid = (x0 + x1) / 2, (y0 + y1) / 2
    x0, x1 = x_mid - span / 2, x_mid + span / 2
    y0, y1 = y_mid - span / 2, y_mid + span / 2
    plot = (margin, 95, width - 55, height - margin)
    for step in range(6):
        fraction = step / 5
        px = plot[0] + fraction * (plot[2] - plot[0])
        py = plot[3] - fraction * (plot[3] - plot[1])
        draw.line((px, plot[1], px, plot[3]), fill=GRID)
        draw.line((plot[0], py, plot[2], py), fill=GRID)
        draw.text((px, plot[3] + 18), f"{x0 + fraction*(x1-x0):.2f}",
                  fill=INK, font=tick_font, anchor="ma")
        draw.text((plot[0] - 14, py), f"{y0 + fraction*(y1-y0):.2f}",
                  fill=INK, font=tick_font, anchor="rm")
    points = [
        (plot[0] + (x - x0) / (x1 - x0) * (plot[2] - plot[0]),
         plot[3] - (y - y0) / (y1 - y0) * (plot[3] - plot[1]))
        for x, y in zip(xs, ys)
    ]
    if len(points) >= 2:
        draw.line(points, fill=COLORS[0], width=4, joint="curve")
    radius = 7
    for point, color in ((points[0], COLORS[2]), (points[-1], COLORS[3])):
        draw.ellipse((point[0]-radius, point[1]-radius,
                      point[0]+radius, point[1]+radius), fill=color)
    draw.rectangle(plot, outline=INK, width=2)
    draw.text(((plot[0] + plot[2]) / 2, height - 42), "x (m)",
              fill=INK, font=label_font, anchor="mm")
    draw.text((30, (plot[1] + plot[3]) / 2), "y (m)", fill=INK,
              font=label_font, anchor="mm")
    legend_y = plot[1] + 18
    legend_x = plot[2] - 240
    for name, color in (("Start", COLORS[2]), ("End", COLORS[3])):
        draw.ellipse((legend_x, legend_y-6, legend_x+12, legend_y+6), fill=color)
        draw.text((legend_x + 20, legend_y), name, fill=INK,
                  font=label_font, anchor="lm")
        legend_x += 115
    image.save(path)


def _markdown(result: dict) -> str:
    metrics = result["metrics"]
    rows = [
        f"| `{check['name']}` | {'通过' if check['passed'] else '失败'} | "
        f"{json.dumps(check['actual'], ensure_ascii=False)} | {check['requirement']} |"
        for check in result["checks"]
    ]
    return "\n".join([
        "# L4 竖直跳跃独立验收",
        "",
        f"结论：**{'通过' if result['pass'] else '未通过'}**。分析器直接读取该次运行的 "
        "`run.json`、`jump_metrics.json` 和 `states.csv`，没有改写原始实验数据。",
        "",
        "## 关键结果",
        "",
        f"- `states.csv` 独立重算起跳高度增量：`{metrics['csv_actual_rise_m']}` m；",
        f"- `jump_metrics.json` 汇总高度增量：`{metrics['jump_metrics_actual_rise_m']}` m；",
        f"- CSV 离地/落地时刻：`{metrics['csv_takeoff_time_s']}` / "
        f"`{metrics['csv_landing_time_s']}` s；",
        f"- 落地后最长双脚同时离地：`{metrics['maximum_post_touchdown_both_airborne_duration_s']:.4f}` s；",
        f"- 末段双足接触占比：`{metrics['terminal_bilateral_contact_fraction']:.3f}`；",
        f"- 末段最大水平/竖直速度：`{metrics['terminal_max_horizontal_speed_m_s']:.4f}` / "
        f"`{metrics['terminal_max_abs_vertical_speed_m_s']:.4f}` m/s；",
        f"- 末段最大倾角：`{metrics['terminal_max_tilt_rad']:.4f}` rad；",
        f"- 末段最大高度误差：`{metrics['terminal_max_height_error_m']:.4f}` m；",
        f"- 全程水平净漂移：`{metrics['planar_drift_m']:.4f}` m；",
        f"- MPC/WBC QP 失败样本：`{metrics['mpc_qp_failure_sample_count']}` / "
        f"`{metrics['wbc_qp_failure_sample_count']}`。",
        "",
        "## 验收项",
        "",
        "| 检查 | 结果 | 实际值 | 判据 |",
        "|---|---:|---|---|",
        *rows,
        "",
        "## 曲线",
        "",
        "- `height.png`：基座高度与高度参考；",
        "- `velocity.png`：基座三轴速度；",
        "- `contact_forces.png`：左右脚 PhysX 法向接触力；",
        "- `attitude.png`：横滚角与俯仰角；",
        "- `planar_trajectory.png`：基座水平轨迹。",
        "",
    ])


def write_analysis(run_dir: Path, output: Path | None = None) -> dict:
    """Compute acceptance and create a new, non-overwriting analysis folder."""
    run_dir = run_dir.expanduser().resolve()
    output = (run_dir / "analysis" if output is None else output.expanduser().resolve())
    try:
        output.relative_to(run_dir)
    except ValueError as exc:
        raise ValueError("分析输出必须位于本次跳跃运行目录内部") from exc
    if output.exists():
        raise FileExistsError(f"分析目录已存在，请指定新的 --output：{output}")
    result, columns = compute_acceptance(run_dir)
    output.mkdir(parents=True)
    (output / "acceptance.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    times = columns["time_s"]
    _line_chart(
        output / "height.png", "Jump height", times,
        (("Base z", columns["base_z_m"], COLORS[0]),
         ("Reference", columns["jump:height_reference_m"], COLORS[1])),
        "time (s)", "height (m)",
    )
    _line_chart(
        output / "velocity.png", "Base velocity", times,
        (("vx", columns["vx_world_m_s"], COLORS[0]),
         ("vy", columns["vy_world_m_s"], COLORS[1]),
         ("vz", columns["vz_world_m_s"], COLORS[2])),
        "time (s)", "velocity (m/s)",
    )
    _line_chart(
        output / "contact_forces.png", "Foot normal contact forces", times,
        (("Left", columns["jump:decision_left_contact_force_z_n"], COLORS[0]),
         ("Right", columns["jump:decision_right_contact_force_z_n"], COLORS[1])),
        "time (s)", "force (N)",
    )
    _line_chart(
        output / "attitude.png", "Base attitude", times,
        (("Roll", columns["roll_rad"], COLORS[0]),
         ("Pitch", columns["pitch_rad"], COLORS[1])),
        "time (s)", "angle (rad)",
    )
    _trajectory_chart(
        output / "planar_trajectory.png",
        columns["base_x_m"], columns["base_y_m"],
    )
    (output / "README.md").write_text(_markdown(result), encoding="utf-8")
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path,
                        help="含 run.json、jump_metrics.json、states.csv 的独立跳跃目录")
    parser.add_argument("--output", type=Path,
                        help="新的分析子目录；默认 <run>/analysis，禁止覆盖")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = arguments(argv)
    result = write_analysis(args.run, args.output)
    output = args.output.expanduser().resolve() if args.output else \
        args.run.expanduser().resolve() / "analysis"
    print(f"[L4 jump] {'通过' if result['pass'] else '未通过'}：{output}")
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
