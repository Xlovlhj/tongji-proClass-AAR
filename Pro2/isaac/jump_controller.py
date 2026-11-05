"""High-level jump phases for the Isaac port of ``demo/jump_mpc.cpp``.

The Python state machine owns timing and event validation. The native bridge
receives one :class:`JumpReference` every physics step and executes the same
low-level modes as the upstream demo: contact-force MPC for thrust/recovery and
leg IK/PV while airborne.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping


class UnsupportedJumpRuntime(RuntimeError):
    """Raised when a requested jump cannot be executed by the controller bridge."""


class JumpRuntimeError(RuntimeError):
    """Raised when takeoff or landing is not observed within its deadline."""


PHASE_CODES = {
    "prepare": 0,
    "crouch": 1,
    "thrust": 2,
    "flight_up": 3,
    "flight_down": 4,
    "landing": 5,
    "recovery": 6,
    "complete": 7,
}

REQUIRED_CAPABILITIES = {
    "dynamic_base_height_reference": "运行时修改质心/基座高度参考",
    "flight_phase_control": "腾空时关闭接触力 MPC 并切换腿部轨迹",
    "bilateral_contact_feedback": "使用左右脚估算力判定离地与落地",
}


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是有限数值")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数值")
    return value


def _positive(value, name: str, *, zero: bool = False) -> float:
    value = _number(value, name)
    if value < 0 if zero else value <= 0:
        raise ValueError(f"{name} 必须是{'非负' if zero else '正'}数")
    return value


def normalize_jump(config: Mapping | None) -> dict | None:
    if config is None:
        return None
    if not isinstance(config, Mapping):
        raise ValueError("jump 必须是对象或 null")
    allowed = {
        "enabled", "start_time_s", "stand_height_m", "crouch_height_m",
        "target_height_m", "crouch_duration_s", "crouch_speed_m_s",
        "thrust_duration_s", "landing_foot_advance_m",
        "landing_foot_swing_duration_s",
        "takeoff_timeout_s", "landing_timeout_s", "landing_settle_duration_s",
        "landing_settle_timeout_s", "landing_vertical_velocity_threshold_m_s",
        "recovery_duration_s", "recovery_mpc_duration_s",
        "takeoff_force_threshold_n",
        "landing_force_threshold_n", "contact_confirm_duration_s",
    }
    unknown = set(config) - allowed
    if unknown:
        raise ValueError(f"jump 含未知字段：{', '.join(sorted(unknown))}")
    enabled = config.get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("jump.enabled 必须是布尔值")
    if not enabled:
        return None

    stand = _positive(config.get("stand_height_m", 1.08), "jump.stand_height_m")
    crouch = _positive(config.get("crouch_height_m", 0.90), "jump.crouch_height_m")
    target = _positive(config.get("target_height_m", 0.12), "jump.target_height_m")
    if crouch >= stand:
        raise ValueError("jump.crouch_height_m 必须小于 stand_height_m")
    if not 0.75 <= crouch <= 1.05:
        raise ValueError("jump.crouch_height_m 必须在 0.75 至 1.05 米之间")
    if not 0.02 <= target <= 0.30:
        raise ValueError("jump.target_height_m 必须在 0.02 至 0.30 米之间")

    landing_advance = _positive(
        config.get("landing_foot_advance_m", 0.05),
        "jump.landing_foot_advance_m", zero=True)
    if landing_advance > 0.20:
        raise ValueError("jump.landing_foot_advance_m 必须在 0 至 0.20 米之间")
    landing_swing_duration = _positive(
        config.get("landing_foot_swing_duration_s", 0.10),
        "jump.landing_foot_swing_duration_s")
    if landing_swing_duration > 0.50:
        raise ValueError("jump.landing_foot_swing_duration_s 必须不超过 0.50 秒")

    takeoff_force = _positive(
        config.get("takeoff_force_threshold_n", 80.0),
        "jump.takeoff_force_threshold_n", zero=True)
    landing_force = _positive(
        config.get("landing_force_threshold_n", 1000.0),
        "jump.landing_force_threshold_n")
    if takeoff_force >= landing_force:
        raise ValueError("jump.takeoff_force_threshold_n 必须小于 landing_force_threshold_n")

    landing_settle_duration = _positive(
        config.get("landing_settle_duration_s", 0.10),
        "jump.landing_settle_duration_s")
    landing_settle_timeout = _positive(
        config.get("landing_settle_timeout_s", 0.40),
        "jump.landing_settle_timeout_s")
    if landing_settle_timeout < landing_settle_duration:
        raise ValueError(
            "jump.landing_settle_timeout_s 必须不小于 landing_settle_duration_s")

    return {
        "enabled": True,
        "start_time_s": _positive(config.get("start_time_s", 8.5), "jump.start_time_s", zero=True),
        "stand_height_m": stand,
        "crouch_height_m": crouch,
        "target_height_m": target,
        "crouch_duration_s": _positive(config.get("crouch_duration_s", 5.5), "jump.crouch_duration_s"),
        "crouch_speed_m_s": _positive(
            config.get("crouch_speed_m_s", 0.10), "jump.crouch_speed_m_s"),
        "thrust_duration_s": _positive(config.get("thrust_duration_s", 0.18), "jump.thrust_duration_s"),
        "landing_foot_advance_m": landing_advance,
        "landing_foot_swing_duration_s": landing_swing_duration,
        "takeoff_timeout_s": _positive(config.get("takeoff_timeout_s", 0.35), "jump.takeoff_timeout_s"),
        "landing_timeout_s": _positive(config.get("landing_timeout_s", 1.5), "jump.landing_timeout_s"),
        "landing_settle_duration_s": landing_settle_duration,
        "landing_settle_timeout_s": landing_settle_timeout,
        "landing_vertical_velocity_threshold_m_s": _positive(
            config.get("landing_vertical_velocity_threshold_m_s", 0.20),
            "jump.landing_vertical_velocity_threshold_m_s"),
        "recovery_duration_s": _positive(config.get("recovery_duration_s", 2.0), "jump.recovery_duration_s"),
        "recovery_mpc_duration_s": _positive(
            config.get("recovery_mpc_duration_s", 0.60),
            "jump.recovery_mpc_duration_s"),
        "takeoff_force_threshold_n": takeoff_force,
        "landing_force_threshold_n": landing_force,
        "contact_confirm_duration_s": _positive(
            config.get("contact_confirm_duration_s", 0.001),
            "jump.contact_confirm_duration_s", zero=True),
    }


def jump_preflight(config: Mapping | None, capabilities: Mapping[str, bool],
                   *, raise_on_missing: bool = True) -> dict:
    """Return an auditable capability report before Isaac starts."""
    cfg = normalize_jump(config)
    report = {
        "requested": cfg is not None,
        "required_capabilities": dict(REQUIRED_CAPABILITIES),
        "available_capabilities": {
            key: bool(capabilities.get(key, False)) for key in REQUIRED_CAPABILITIES
        },
        "ready": True,
    }
    if cfg is None:
        return report
    missing = [
        REQUIRED_CAPABILITIES[key]
        for key in REQUIRED_CAPABILITIES if not capabilities.get(key, False)
    ]
    report["missing"] = missing
    report["ready"] = not missing
    if missing and raise_on_missing:
        raise UnsupportedJumpRuntime(
            "跳跃预检未通过，当前 Isaac 控制桥缺少：" + "、".join(missing))
    return report


@dataclass(frozen=True)
class JumpReference:
    phase: str
    phase_code: int
    phase_progress: float
    base_height_m: float
    vertical_velocity_m_s: float
    landing_foot_advance_m: float
    mpc_enabled: bool
    contact_mode: str


class JumpStateMachine:
    """Event-checked phase sequence modelled after the upstream jump demo."""

    phases = tuple(PHASE_CODES)

    def __init__(self, config: Mapping):
        normalized = normalize_jump(config)
        if normalized is None:
            raise ValueError("JumpStateMachine 需要启用的 jump 配置")
        self.config = normalized
        self.phase = "prepare"
        self.phase_started_s = 0.0
        self._previous_time = -math.inf
        self._flight_started_s: float | None = None
        self._contact_started_s: float | None = None
        self.takeoff_time_s: float | None = None
        self.touchdown_time_s: float | None = None
        self._ascending_observed = False
        self.touchdown_height_m: float | None = None
        self.recovery_start_height_m: float | None = None
        self.failed_reason: str | None = None

        crouch_start = normalized["start_time_s"] - normalized["crouch_duration_s"]
        if crouch_start < 0.0:
            raise ValueError("jump.crouch_duration_s 不能早于仿真开始")
        descent_time = ((normalized["stand_height_m"] - normalized["crouch_height_m"])
                        / normalized["crouch_speed_m_s"])
        if normalized["crouch_duration_s"] + 1e-12 < descent_time:
            raise ValueError(
                "jump.crouch_duration_s 太短，无法按 crouch_speed_m_s 到达下蹲高度")

    @property
    def complete(self) -> bool:
        return self.phase == "complete"

    @property
    def left_ground(self) -> bool:
        return self.takeoff_time_s is not None

    def _set_phase(self, phase: str, t: float) -> None:
        if phase not in self.phases:
            raise ValueError(f"未知跳跃阶段：{phase}")
        if phase != self.phase:
            self.phase = phase
            self.phase_started_s = t

    def _fail(self, reason: str) -> None:
        self.failed_reason = reason
        raise JumpRuntimeError(reason)

    def update(self, time_s: float, base_height_m: float, vertical_velocity_m_s: float,
               left_force_z_n: float, right_force_z_n: float) -> JumpReference:
        t = _number(time_s, "time_s")
        base_height = _number(base_height_m, "base_height_m")
        vz = _number(vertical_velocity_m_s, "vertical_velocity_m_s")
        left_force = _number(left_force_z_n, "left_force_z_n")
        right_force = _number(right_force_z_n, "right_force_z_n")
        if t < self._previous_time:
            raise ValueError("跳跃状态机时间不能倒退")
        self._previous_time = t
        cfg = self.config

        crouch_start = cfg["start_time_s"] - cfg["crouch_duration_s"]
        if self.phase == "prepare" and t >= crouch_start:
            self._set_phase("crouch", t)

        elapsed = max(0.0, t - self.phase_started_s)
        # As in jump_mpc.cpp, start_time_s is the beginning of thrust.  The
        # crouch starts earlier and may hold its final height before takeoff.
        if self.phase == "crouch" and t >= cfg["start_time_s"]:
            self._set_phase("thrust", t)
        elif self.phase == "thrust" and elapsed >= cfg["thrust_duration_s"]:
            self._set_phase("flight_up", t)
            self._flight_started_s = t

        in_flight = self.phase in {"flight_up", "flight_down"}
        both_released = (
            left_force <= cfg["takeoff_force_threshold_n"] and
            right_force <= cfg["takeoff_force_threshold_n"]
        )
        if in_flight and self.takeoff_time_s is None and both_released:
            self.takeoff_time_s = t

        if self.phase == "flight_up":
            if vz > 0.05:
                self._ascending_observed = True
            flight_elapsed = t - (self._flight_started_s if self._flight_started_s is not None else t)
            if self.takeoff_time_s is None and flight_elapsed >= cfg["takeoff_timeout_s"]:
                self._fail("起跳失败：蹬伸后未检测到双脚离地")
            if self.takeoff_time_s is not None and self._ascending_observed and vz <= 0.0:
                self._set_phase("flight_down", t)

        if self.phase == "flight_down":
            both_landed = (
                left_force >= cfg["landing_force_threshold_n"] and
                right_force >= cfg["landing_force_threshold_n"]
            )
            if both_landed:
                if self._contact_started_s is None:
                    self._contact_started_s = t
                if t - self._contact_started_s >= cfg["contact_confirm_duration_s"]:
                    self.touchdown_time_s = t
                    self.touchdown_height_m = base_height
                    self._set_phase("landing", t)
            else:
                self._contact_started_s = None
            flight_elapsed = t - (self._flight_started_s if self._flight_started_s is not None else t)
            if self.phase == "flight_down" and flight_elapsed >= cfg["landing_timeout_s"]:
                self._fail("落地失败：腾空后未在时限内检测到双脚同时着地")

        elapsed = max(0.0, t - self.phase_started_s)
        if self.phase == "landing":
            support_confirmed = (
                left_force > cfg["takeoff_force_threshold_n"] and
                right_force > cfg["takeoff_force_threshold_n"]
            )
            impact_absorbed = abs(vz) <= cfg["landing_vertical_velocity_threshold_m_s"]
            if (elapsed >= cfg["landing_settle_duration_s"] and
                    support_confirmed and impact_absorbed):
                # Recovery starts from the height reached after impact
                # absorption, rather than the first-contact height.
                self.recovery_start_height_m = base_height
                self._set_phase("recovery", t)
            elif elapsed >= cfg["landing_settle_timeout_s"]:
                self._fail(
                    "落地吸收失败：双脚接触或竖直速度未在时限内稳定")
        elif self.phase == "recovery":
            # jump_mpc.cpp never steps the height reference from the crouched
            # landing pose straight to 1.08 m.  It keeps applying
            # Ramp(..., 1.08, 0.1 * dt) until the standing height is reached.
            # Treat recovery_duration_s as a minimum dwell time, and also wait
            # for that source-equivalent height ramp to finish.  This keeps the
            # recovery -> complete transition continuous for every touchdown
            # height instead of relying on one hand-tuned duration.
            start = (self.recovery_start_height_m
                     if self.recovery_start_height_m is not None
                     else self.touchdown_height_m
                     if self.touchdown_height_m is not None
                     else cfg["crouch_height_m"])
            height_ramp_duration = max(
                0.0, (cfg["stand_height_m"] - start) / 0.10)
            if elapsed >= max(cfg["recovery_duration_s"], height_ramp_duration):
                self._set_phase("complete", t)

        return self._reference(t, vz)

    def _reference(self, t: float, measured_vz: float) -> JumpReference:
        cfg = self.config
        elapsed = max(0.0, t - self.phase_started_s)
        stand, crouch = cfg["stand_height_m"], cfg["crouch_height_m"]
        launch_velocity = math.sqrt(2.0 * 9.81 * cfg["target_height_m"])
        progress = 0.0
        landing_advance = 0.0

        if self.phase == "prepare":
            height, velocity, enabled, contact = stand, 0.0, False, "double_support"
        elif self.phase == "crouch":
            height = max(crouch, stand - cfg["crouch_speed_m_s"] * elapsed)
            progress = min(1.0, (stand - height) / (stand - crouch))
            velocity, enabled, contact = 0.0, False, "double_support"
        elif self.phase == "thrust":
            progress = min(1.0, elapsed / cfg["thrust_duration_s"])
            velocity = progress * launch_velocity
            # Integral of the linearly ramped velocity, matching jump_mpc.cpp.
            height = crouch + 0.5 * launch_velocity * cfg["thrust_duration_s"] * progress * progress
            enabled, contact = True, "double_support"
        elif self.phase == "flight_up":
            # The upstream demo retracts the legs at 5 m/s in a world-oriented
            # foot vector while MPC is disabled.
            height = max(crouch, stand - 5.0 * elapsed)
            progress = min(1.0, (stand - height) / (stand - crouch))
            velocity, enabled, contact = measured_vz, False, "flight"
        elif self.phase == "flight_down":
            # The source demo linearly advances both feet by 0.2 m over 20 ms.
            # Isaac keeps the same apex-anchored forward-swing logic, with a
            # configurable amplitude and duration for its contact geometry.
            progress = min(1.0, elapsed / cfg["landing_foot_swing_duration_s"])
            landing_advance = cfg["landing_foot_advance_m"]
            height, velocity, enabled, contact = crouch, measured_vz, False, "flight"
        elif self.phase == "landing":
            progress = min(1.0, elapsed / cfg["landing_settle_duration_s"])
            height = self.touchdown_height_m if self.touchdown_height_m is not None else crouch
            velocity, enabled, contact = 0.0, True, "double_support"
        elif self.phase == "recovery":
            start = (self.recovery_start_height_m
                     if self.recovery_start_height_m is not None
                     else self.touchdown_height_m
                     if self.touchdown_height_m is not None else crouch)
            height = min(stand, start + 0.10 * elapsed)
            # Use the upstream landing MPC as a finite impact/braking window,
            # then keep the same height ramp under fixed-foot WBC.  Progress is
            # deliberately the MPC-window progress because the native bridge
            # uses 1.0 as the continuous hand-off signal.
            progress = min(1.0, elapsed / cfg["recovery_mpc_duration_s"])
            velocity = 0.0
            enabled = elapsed < cfg["recovery_mpc_duration_s"]
            contact = "double_support"
        else:
            progress = 1.0
            height, velocity, enabled, contact = stand, 0.0, False, "double_support"

        return JumpReference(
            phase=self.phase,
            phase_code=PHASE_CODES[self.phase],
            phase_progress=progress,
            base_height_m=height,
            vertical_velocity_m_s=velocity,
            landing_foot_advance_m=landing_advance,
            mpc_enabled=enabled,
            contact_mode=contact,
        )
