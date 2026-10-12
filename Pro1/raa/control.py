"""Joint torque control and time-aligned tracking metrics for raa-v0.

The simulator supplies positions, velocities, and (when requested) gravity
torques in the same seven-joint order. No Isaac dependency lives in this file.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


PANDA_TORQUE_LIMITS = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)


def _joint_vector(values: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain seven finite joint values")
    return result


@dataclass(frozen=True)
class PDConfig:
    """Panda defaults; tune gains together with the physics time step.

    Position errors are in radians, velocities in radians per second, and
    torques in newton metres. These limits apply to the arm, not its fingers.
    """

    kp: tuple[float, ...] = (160.0, 160.0, 120.0, 120.0, 70.0, 55.0, 35.0)
    kd: tuple[float, ...] = (22.0, 22.0, 18.0, 18.0, 2.0, 1.4, 0.8)
    torque_limits: tuple[float, ...] = PANDA_TORQUE_LIMITS
    gravity_compensation: bool = True

    def __post_init__(self) -> None:
        for name in ("kp", "kd", "torque_limits"):
            values = _joint_vector(getattr(self, name), name)
            if np.any(values < 0) or (name == "torque_limits" and np.any(values == 0)):
                raise ValueError(f"{name} values must be {'positive' if name == 'torque_limits' else 'nonnegative'}")
            # Store immutable values even when the caller supplied numpy arrays.
            object.__setattr__(self, name, tuple(float(value) for value in values))


class TorquePD:
    """Explicit PD torque controller with optional model gravity compensation.

    ``gravity`` must be the actuator torque needed to balance gravity, not the
    gravity force acting on the robot. The adapter owns that sign convention.
    Both feedback and feed-forward gravity are clipped together at arm limits.
    """

    def __init__(self, config: PDConfig | None = None) -> None:
        self.config = config if config is not None else PDConfig()
        self.kp = np.asarray(self.config.kp, dtype=np.float64)
        self.kd = np.asarray(self.config.kd, dtype=np.float64)
        self.torque_limits = np.asarray(self.config.torque_limits, dtype=np.float64)
        self.reset()

    def reset(self) -> None:
        self.last_raw_torque = np.zeros(7, dtype=np.float64)
        self.last_torque = np.zeros(7, dtype=np.float64)
        self.saturation_mask = np.zeros(7, dtype=bool)
        self.last_compensation = np.zeros(7, dtype=np.float64)
        self.last_feedback_torque = np.zeros(7, dtype=np.float64)
        self.last_gravity_torque = np.zeros(7, dtype=np.float64)

    def compute(
        self,
        q: Sequence[float],
        dq: Sequence[float],
        q_ref: Sequence[float],
        dq_ref: Sequence[float],
        gravity: Sequence[float] | None = None,
        *, dt: float | None = None, adapt: bool = True,
    ) -> np.ndarray:
        q = _joint_vector(q, "q")
        dq = _joint_vector(dq, "dq")
        q_ref = _joint_vector(q_ref, "q_ref")
        dq_ref = _joint_vector(dq_ref, "dq_ref")
        self.last_feedback_torque = self.kp * (q_ref - q) + self.kd * (dq_ref - dq)
        self.last_gravity_torque = np.zeros(7, dtype=np.float64)
        if self.config.gravity_compensation:
            if gravity is None:
                raise ValueError("gravity compensation requires seven gravity torques")
            self.last_gravity_torque = _joint_vector(gravity, "gravity").copy()
        torque = self.last_feedback_torque + self.last_gravity_torque
        self.last_raw_torque = torque.copy()
        self.last_torque = np.clip(torque, -self.torque_limits, self.torque_limits)
        self.saturation_mask = np.abs(torque) > self.torque_limits
        return self.last_torque.copy()

    def state_summary(self) -> dict:
        return {"kind": "gc-pd" if self.config.gravity_compensation else "pd",
                "gravity_compensation": self.config.gravity_compensation,
                "kp": self.kp.tolist(), "kd": self.kd.tolist(),
                "torque_limits": self.torque_limits.tolist(), "online_learning": False}


def tracking_metrics(
    target_xyz: Sequence[Sequence[float]],
    actual_xyz: Sequence[Sequence[float]],
    drawing_mask: Sequence[bool] | None = None,
    torques: Sequence[Sequence[float]] | None = None,
    torque_limits: Sequence[float] | None = None,
    dt: float | None = None,
) -> dict[str, float | int | None]:
    """Summarize paired samples in metres; never use nearest-contour distance.

    Target and measured position at row ``i`` must describe the same simulation
    instant. Full-trajectory metrics include transfers and pen lifts; metrics
    prefixed ``drawing_`` use only the supplied pen-down mask. If no mask is
    given, every sample counts as drawing. An empty drawing mask gives null
    drawing metrics, which remain compatible with JSON output.

    Torque RMS is across time and joints. Saturation fraction counts individual
    joint samples at the limit (clipped torque) or exceeding it (raw torque).
    """

    target = np.asarray(target_xyz, dtype=np.float64)
    actual = np.asarray(actual_xyz, dtype=np.float64)
    if target.ndim != 2 or target.shape[1:] != (3,) or len(target) == 0:
        raise ValueError("target_xyz must have nonempty shape (samples, 3)")
    if actual.shape != target.shape:
        raise ValueError("actual_xyz must have the same shape as target_xyz")
    if not np.all(np.isfinite(target)) or not np.all(np.isfinite(actual)):
        raise ValueError("tracking positions must be finite")

    count = len(target)
    if drawing_mask is None:
        mask = np.ones(count, dtype=bool)
    else:
        supplied_mask = np.asarray(drawing_mask)
        if supplied_mask.shape != (count,):
            raise ValueError("drawing_mask must match the number of samples")
        if supplied_mask.dtype != np.bool_ and not np.all(np.isin(supplied_mask, (0, 1))):
            raise ValueError("drawing_mask must contain only booleans or 0/1")
        mask = supplied_mask.astype(bool)

    error = actual - target
    tip_errors = np.linalg.norm(error, axis=1)
    xy_errors = np.linalg.norm(error[:, :2], axis=1)
    result: dict[str, float | int | None] = {
        "sample_count": count,
        "drawing_sample_count": int(np.count_nonzero(mask)),
        "tip_rmse_m": float(np.sqrt(np.mean(tip_errors**2))),
        "tip_max_error_m": float(np.max(tip_errors)),
        "xy_rmse_m": float(np.sqrt(np.mean(xy_errors**2))),
        "xy_max_error_m": float(np.max(xy_errors)),
        "drawing_tip_rmse_m": None,
        "drawing_tip_max_error_m": None,
        "drawing_xy_rmse_m": None,
        "drawing_xy_max_error_m": None,
    }
    if np.any(mask):
        for prefix, values in (("drawing_tip", tip_errors[mask]), ("drawing_xy", xy_errors[mask])):
            result[f"{prefix}_rmse_m"] = float(np.sqrt(np.mean(values**2)))
            result[f"{prefix}_max_error_m"] = float(np.max(values))

    if dt is not None:
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        result["duration_s"] = float((count - 1) * dt)

    if torques is not None:
        torque = np.asarray(torques, dtype=np.float64)
        if torque.shape != (count, 7) or not np.all(np.isfinite(torque)):
            raise ValueError("torques must have finite shape (samples, 7)")
        limits = _joint_vector(PANDA_TORQUE_LIMITS if torque_limits is None else torque_limits, "torque_limits")
        if np.any(limits <= 0):
            raise ValueError("torque_limits must be positive")
        result["torque_rms_nm"] = float(np.sqrt(np.mean(torque**2)))
        result["torque_max_abs_nm"] = float(np.max(np.abs(torque)))
        result["torque_saturation_fraction"] = float(np.mean(np.abs(torque) >= limits * (1.0 - 1e-9)))
    elif torque_limits is not None:
        raise ValueError("torque_limits requires torque samples")
    return result
