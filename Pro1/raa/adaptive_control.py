"""Bounded online residual learning on the same gravity-compensated PD base.

RBF: each joint has a two-input Gaussian hidden layer with learned linear
output weights. Fuzzy: each joint has a 5 x 5 product-rule, zero-order Sugeno
system with learned consequent torques. Both update from tracking feedback;
there is no pretrained network, offline dataset, or claimed global stability
proof. Projection bounds learned torques, not the robot's tracking error.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from .control import PDConfig, TorquePD, _joint_vector


ADAPTIVE_COMMON_FIELDS = frozenset({
    "learning_rate", "sliding_lambda", "leakage", "compensation_limits",
})
RBF_FIELDS = ADAPTIVE_COMMON_FIELDS | {
    "position_scale", "velocity_scale", "rbf_width", "centers_per_axis",
}
FUZZY_FIELDS = ADAPTIVE_COMMON_FIELDS | {"error_scale", "velocity_error_scale"}


def _positive_joint_values(value: float | Sequence[float], name: str) -> tuple[float, ...]:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 0:
        array = np.full(7, float(array))
    array = _joint_vector(array, name)
    if np.any(array <= 0):
        raise ValueError(f"{name} must be positive")
    return tuple(float(v) for v in array)


def _positive_scalar(value: float, name: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _time_step(value: float) -> float:
    result = _positive_scalar(value, "dt")
    if result > 0.01:
        raise ValueError("adaptive dt must be <= 0.01 s; default is 0.001 s")
    return result


@dataclass(frozen=True)
class _AdaptiveConfig:
    pd: PDConfig = field(default_factory=PDConfig)
    dt: float = 0.001
    learning_rate: float | tuple[float, ...] = 20.0
    sliding_lambda: float | tuple[float, ...] = 6.0
    leakage: float = 0.02
    compensation_limits: tuple[float, ...] = (6.0, 6.0, 6.0, 6.0, 2.0, 2.0, 2.0)

    def __post_init__(self) -> None:
        if not isinstance(self.pd, PDConfig) or not self.pd.gravity_compensation:
            raise ValueError("adaptive controllers require a gravity-compensated PDConfig")
        object.__setattr__(self, "dt", _time_step(self.dt))
        for name in ("learning_rate", "sliding_lambda"):
            object.__setattr__(self, name, _positive_joint_values(getattr(self, name), name))
        limits = _joint_vector(self.compensation_limits, "compensation_limits")
        if np.any(limits <= 0) or np.any(limits > np.asarray(self.pd.torque_limits)):
            raise ValueError("compensation_limits must be positive and <= joint torque limits")
        object.__setattr__(self, "compensation_limits", tuple(float(v) for v in limits))
        leakage = float(self.leakage)
        if not np.isfinite(leakage) or leakage < 0:
            raise ValueError("leakage must be finite and nonnegative")
        if leakage * self.dt > 1:
            raise ValueError("leakage * dt must be <= 1")
        object.__setattr__(self, "leakage", leakage)


@dataclass(frozen=True)
class RBFConfig(_AdaptiveConfig):
    learning_rate: float | tuple[float, ...] = 30.0
    position_scale: float = float(np.pi)
    velocity_scale: float = 2.5
    rbf_width: float = 0.65
    centers_per_axis: int = 3

    def __post_init__(self) -> None:
        super().__post_init__()
        for name in ("position_scale", "velocity_scale", "rbf_width"):
            object.__setattr__(self, name, _positive_scalar(getattr(self, name), name))
        if (isinstance(self.centers_per_axis, bool)
                or not isinstance(self.centers_per_axis, (int, np.integer))
                or not 2 <= self.centers_per_axis <= 9):
            raise ValueError("centers_per_axis must be an integer in [2, 9]")
        object.__setattr__(self, "centers_per_axis", int(self.centers_per_axis))
        if self.rbf_width < 1e-6:
            raise ValueError("rbf_width must be >= 1e-6 to avoid ill-conditioned features")


@dataclass(frozen=True)
class FuzzyConfig(_AdaptiveConfig):
    error_scale: float = 0.02
    velocity_error_scale: float = 0.2

    def __post_init__(self) -> None:
        super().__post_init__()
        for name in ("error_scale", "velocity_error_scale"):
            object.__setattr__(self, name, _positive_scalar(getattr(self, name), name))


class _AdaptiveTorque:
    controller_name = "adaptive"

    def __init__(self, config: _AdaptiveConfig, feature_count: int) -> None:
        self.config = config
        self._base = TorquePD(config.pd)
        self.kp, self.kd = self._base.kp, self._base.kd
        self.torque_limits = self._base.torque_limits
        self.learning_rate = np.asarray(config.learning_rate)
        self.sliding_lambda = np.asarray(config.sliding_lambda)
        self.compensation_limits = np.asarray(config.compensation_limits)
        self.feature_count = feature_count
        self.reset()

    def reset(self) -> None:
        """Start an independent trial with zero weights and zero diagnostics."""
        self._base.reset()
        self.weights = np.zeros((7, self.feature_count), dtype=np.float64)
        self.last_used_weights = np.zeros_like(self.weights)
        self.last_used_weight_norms = np.zeros(7)
        self.last_raw_torque = np.zeros(7)
        self.last_torque = np.zeros(7)
        self.saturation_mask = np.zeros(7, dtype=bool)
        self.last_compensation = np.zeros(7)
        self.last_activation = np.zeros_like(self.weights)
        self.last_sliding_error = np.zeros(7)
        self.last_learning_mask = np.zeros(7, dtype=bool)
        self.last_weight_update_norm = 0.0
        self.training_step_count = 0
        self.saturated_joint_sample_count = 0
        self.antiwindup_joint_sample_count = 0
        self.last_dt = self.config.dt

    def _activations(self, q, dq, error, velocity_error):
        raise NotImplementedError

    def compute(
        self,
        q: Sequence[float],
        dq: Sequence[float],
        q_ref: Sequence[float],
        dq_ref: Sequence[float],
        gravity: Sequence[float] | None = None,
        *,
        dt: float | None = None,
        adapt: bool = True,
    ) -> np.ndarray:
        """Use old weights for this torque, then learn for the next time step.

        Positive e=q_ref-q / s increases positive compensation. Each step uses
        dt*(Gamma*activation*s - leakage*weights), followed by projection. If
        the actuator is saturated, learning that would push further into its
        saturation is disabled for that joint; leakage can still reduce weights.
        ``adapt=False`` freezes both learning and leakage (e.g. GUI hold).
        """
        time_step = _time_step(self.config.dt if dt is None else dt)
        if self.config.leakage * time_step > 1:
            raise ValueError("leakage * dt must be <= 1")
        if not isinstance(adapt, (bool, np.bool_)):
            raise ValueError("adapt must be boolean")
        q = _joint_vector(q, "q")
        dq = _joint_vector(dq, "dq")
        q_ref = _joint_vector(q_ref, "q_ref")
        dq_ref = _joint_vector(dq_ref, "dq_ref")
        error, velocity_error = q_ref - q, dq_ref - dq
        sliding = velocity_error + self.sliding_lambda * error
        if not np.all(np.isfinite(sliding)):
            raise ValueError("tracking errors must remain finite")
        self._base.compute(q, dq, q_ref, dq_ref, gravity)
        activation = self._activations(q, dq, error, velocity_error)
        self.last_used_weights = self.weights.copy()
        self.last_used_weight_norms = np.linalg.norm(self.last_used_weights, axis=1)
        compensation = np.sum(self.last_used_weights * activation, axis=1)
        raw = self._base.last_raw_torque + compensation
        if not np.all(np.isfinite(raw)):
            raise ValueError("raw adaptive torque must remain finite")
        self.last_raw_torque = raw.copy()
        self.last_torque = np.clip(raw, -self.torque_limits, self.torque_limits)
        self.saturation_mask = np.abs(raw) > self.torque_limits
        self.last_compensation = compensation.copy()
        self.last_activation = activation.copy()
        self.last_sliding_error = sliding.copy()
        self.last_dt = time_step
        outward = ((raw >= self.torque_limits) & (sliding > 0)
            | (raw <= -self.torque_limits) & (sliding < 0))
        self.last_learning_mask = (~outward) if adapt else np.zeros(7, dtype=bool)
        self.last_weight_update_norm = 0.0
        if adapt:
            # Clip the learning signal at 1 rad/s to bound individual updates
            # during gross transients. The unclipped s remains in diagnostics.
            signal = np.clip(sliding, -1.0, 1.0)
            learning = self.learning_rate[:, None] * activation * signal[:, None]
            learning[outward] = 0.0
            candidate = self.weights + time_step * (learning - self.config.leakage * self.weights)
            updated = np.clip(candidate, -self.compensation_limits[:, None], self.compensation_limits[:, None])
            self.last_weight_update_norm = float(np.linalg.norm(updated - self.weights))
            self.weights = updated
            self.training_step_count += 1
            self.saturated_joint_sample_count += int(np.count_nonzero(self.saturation_mask))
            self.antiwindup_joint_sample_count += int(np.count_nonzero(outward))
        return self.last_torque.copy()

    def state_summary(self) -> dict[str, Any]:
        """Export the final learned state; safe to serialize with json.dumps."""
        return {
            "controller": self.controller_name,
            "configuration": asdict(self.config),
            "started_from_zero_weights": True,
            "training_step_count": self.training_step_count,
            "feature_count_per_joint": self.feature_count,
            "weights_shape": list(self.weights.shape),
            "weights": self.weights.tolist(),
            "weights_l2_norm": float(np.linalg.norm(self.weights)),
            "weight_norms_by_joint": self.weight_norms.tolist(),
            "last_used_weight_norms_by_joint": self.last_used_weight_norms.tolist(),
            "weight_state_timing": "weights are after the latest update; last_used_weights generated the latest torque",
            "weights_max_abs": float(np.max(np.abs(self.weights))),
            "learning_rate": self.learning_rate.tolist(),
            "sliding_lambda": self.sliding_lambda.tolist(),
            "leakage": self.config.leakage,
            "compensation_limits_nm": self.compensation_limits.tolist(),
            "last_compensation_nm": self.last_compensation.tolist(),
            "last_sliding_error_rad_s": self.last_sliding_error.tolist(),
            "last_weight_update_norm": self.last_weight_update_norm,
            "last_dt_s": self.last_dt,
            "saturated_joint_sample_count": self.saturated_joint_sample_count,
            "antiwindup_joint_sample_count": self.antiwindup_joint_sample_count,
            "activation_normalization": "nonnegative features with sum=1 for each joint",
            "basis_centers": self.centers.tolist(),
            "learning_signal_clip_rad_s": 1.0,
            "guarantee": "bounded weights and compensation; no global tracking stability proof",
        }

    @property
    def weight_norms(self) -> np.ndarray:
        """Seven row L2 norms, for per-step CSV diagnostics."""
        return np.linalg.norm(self.weights, axis=1)

    def state_arrays(self) -> dict[str, np.ndarray]:
        """Copies suitable for np.savez_compressed; no object/pickle arrays."""
        result = {
            "weights": self.weights.copy(),
            "last_used_weights": self.last_used_weights.copy(),
            "basis_centers": self.centers.copy(),
            "kp": self.kp.copy(),
            "kd": self.kd.copy(),
            "torque_limits": self.torque_limits.copy(),
            "learning_rate": self.learning_rate.copy(),
            "sliding_lambda": self.sliding_lambda.copy(),
            "compensation_limits": self.compensation_limits.copy(),
            "last_activation": self.last_activation.copy(),
            "last_compensation": self.last_compensation.copy(),
            "training_step_count": np.array(self.training_step_count, dtype=np.int64),
            "dt": np.array(self.last_dt),
            "leakage": np.array(self.config.leakage),
        }
        if isinstance(self.config, RBFConfig):
            result["normalization_scales"] = np.array([self.config.position_scale, self.config.velocity_scale])
            result["rbf_width"] = np.array(self.config.rbf_width)
        else:
            result["normalization_scales"] = np.array([self.config.error_scale, self.config.velocity_error_scale])
        return result


class RBFAdaptive(_AdaptiveTorque):
    """Seven independent Gaussian RBF networks with online output weights."""

    controller_name = "rbf"

    def __init__(self, config: RBFConfig | None = None) -> None:
        config = RBFConfig() if config is None else config
        if not isinstance(config, RBFConfig):
            raise TypeError("RBFAdaptive requires RBFConfig")
        centers = np.linspace(-1.0, 1.0, config.centers_per_axis)
        self.centers = np.array([(p, v) for p in centers for v in centers])
        super().__init__(config, len(self.centers))

    def _activations(self, q, dq, error, velocity_error):
        normalized = np.column_stack((q / self.config.position_scale, dq / self.config.velocity_scale))
        normalized = np.clip(normalized, -2.0, 2.0)
        difference = (normalized[:, None, :] - self.centers[None, :, :]) / self.config.rbf_width
        logits = -0.5 * np.sum(difference**2, axis=2)
        # Log-space normalization avoids underflow for far-away activations.
        logits -= np.max(logits, axis=1, keepdims=True)
        basis = np.exp(logits)
        return basis / np.sum(basis, axis=1, keepdims=True)


class FuzzyAdaptive(_AdaptiveTorque):
    """5 x 5 triangular product rules with online zero-order Sugeno outputs.

    Labels are negative-large / negative-small / zero / positive-small /
    positive-large. At most four rules fire per joint; the torque consequents,
    rather than the PD gains, are learned from feedback.
    """

    controller_name = "fuzzy"

    def __init__(self, config: FuzzyConfig | None = None) -> None:
        config = FuzzyConfig() if config is None else config
        if not isinstance(config, FuzzyConfig):
            raise TypeError("FuzzyAdaptive requires FuzzyConfig")
        self.centers = np.linspace(-1.0, 1.0, 5)
        super().__init__(config, 25)

    def _activations(self, q, dq, error, velocity_error):
        e = np.clip(error / self.config.error_scale, -1.0, 1.0)
        de = np.clip(velocity_error / self.config.velocity_error_scale, -1.0, 1.0)
        error_membership = np.maximum(1.0 - np.abs(e[:, None] - self.centers) / 0.5, 0.0)
        velocity_membership = np.maximum(1.0 - np.abs(de[:, None] - self.centers) / 0.5, 0.0)
        rules = (error_membership[:, :, None] * velocity_membership[:, None, :]).reshape(7, 25)
        return rules / np.sum(rules, axis=1, keepdims=True)


def make_adaptive_controller(
    name: str,
    *,
    pd_config: PDConfig | None = None,
    dt: float = 0.001,
    parameters: Mapping[str, Any] | None = None,
) -> RBFAdaptive | FuzzyAdaptive:
    """Strict factory for the config selector; reject misspelled parameters."""
    configurations = {"rbf": (RBFConfig, RBFAdaptive, RBF_FIELDS),
                      "fuzzy": (FuzzyConfig, FuzzyAdaptive, FUZZY_FIELDS)}
    if name not in configurations:
        raise ValueError(f"unknown adaptive controller {name!r}; choose rbf or fuzzy")
    config_type, controller_type, fields = configurations[name]
    parameters = dict(parameters or {})
    unknown = set(parameters) - fields
    if unknown:
        raise ValueError(f"unknown {name} adaptive parameters: {', '.join(sorted(unknown))}")
    config = config_type(pd=PDConfig() if pd_config is None else pd_config, dt=dt, **parameters)
    return controller_type(config)
