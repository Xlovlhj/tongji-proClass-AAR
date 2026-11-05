"""Nonlinear SRBD MPC for the L4 algorithm comparison.

The implementation uses nonlinear orientation kinematics, world-frame inertia,
the gyroscopic term and circular friction cones.  It is solved with SciPy SLSQP
as a direct single-shooting nonlinear program, which keeps the dependency set
compatible with the installed Isaac Sim Python.  This is a real nonlinear
finite-horizon optimizer, but it is a research/reference controller: whether it
meets the 200 Hz OpenLoong deadline must be measured in the Isaac closed loop.

Run the deterministic mathematical self-check with:

    python3 Pro2/run_l4_control.py self-check --controller nmpc-slsqp
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

try:
    from .srbd_model import (STATE_SIZE, WRENCH_SIZE, SrbdParameters, as_contact_schedule,
                             as_foot_positions, as_reference, as_state,
                             euler_rates_from_world_omega, nominal_wrench,
                             rotation_rpy, wrench_constraint_values)
except ImportError:
    from srbd_model import (STATE_SIZE, WRENCH_SIZE, SrbdParameters, as_contact_schedule,
                            as_foot_positions, as_reference, as_state,
                            euler_rates_from_world_omega, nominal_wrench,
                            rotation_rpy, wrench_constraint_values)

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("请使用 Isaac Sim Python 运行，当前解释器缺少 NumPy") from exc


DEFAULT_STATE_WEIGHTS = np.array([
    4.0, 4.0, 2.0, 1.0, 30.0, 30.0,
    .1, .1, .1, 5.0, 20.0, 5.0,
])


@dataclass
class NmpcResult:
    wrench: np.ndarray
    control_sequence: np.ndarray
    predicted_states: np.ndarray
    success: bool
    status: str
    objective: float
    solve_time_s: float
    iterations: int
    minimum_constraint_margin: float


class NonlinearSrbdMpc:
    """Direct single-shooting nonlinear MPC solved by SLSQP."""

    def __init__(self, *, dt=.02, horizon=5, parameters=None,
                 state_weights=None, input_weight=2e-6,
                 wrench_rate_weight=1e-7, max_iterations=60, tolerance=1e-5):
        self.dt = float(dt)
        self.horizon = int(horizon)
        self.parameters = parameters or SrbdParameters()
        self.parameters.validate()
        self.state_weights = np.asarray(
            DEFAULT_STATE_WEIGHTS if state_weights is None else state_weights,
            dtype=float)
        self.input_weight = float(input_weight)
        self.wrench_rate_weight = float(wrench_rate_weight)
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)
        if not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError("dt 必须为有限正数")
        if self.horizon < 1:
            raise ValueError("horizon 必须为正整数")
        if self.state_weights.shape != (STATE_SIZE,) or np.any(self.state_weights < 0):
            raise ValueError("state_weights 必须包含 12 个非负数")
        if self.input_weight <= 0 or self.wrench_rate_weight < 0:
            raise ValueError("输入权重必须为正，变化率权重必须非负")
        if self.max_iterations < 1 or self.tolerance <= 0:
            raise ValueError("max_iterations 和 tolerance 必须为正")
        self._warm_start = None
        self._previous_wrench = None

    def derivative(self, state, wrench, foot_positions_world):
        """Full nonlinear SRBD derivative in the shared state convention."""

        state = as_state(state)
        wrench = np.asarray(wrench, dtype=float)
        feet = as_foot_positions(foot_positions_world)
        if wrench.shape != (WRENCH_SIZE,) or not np.isfinite(wrench).all():
            raise ValueError("wrench 必须包含 12 个有限数值")
        params = self.parameters
        rotation = rotation_rpy(state[:3])
        inertia_world = rotation @ params.inertia @ rotation.T
        forces = wrench.reshape(2, 6)[:, :3]
        moments = wrench.reshape(2, 6)[:, 3:]
        total_force = forces.sum(axis=0)
        total_moment = moments.sum(axis=0)
        for foot in range(2):
            total_moment += np.cross(feet[foot] - state[3:6], forces[foot])
        derivative = np.zeros(STATE_SIZE)
        derivative[:3] = euler_rates_from_world_omega(state[:3], state[6:9])
        derivative[3:6] = state[9:12]
        derivative[6:9] = np.linalg.solve(
            inertia_world,
            total_moment - np.cross(state[6:9], inertia_world @ state[6:9]))
        derivative[9:12] = total_force / params.mass + np.array([0.0, 0.0, -params.gravity])
        return derivative

    def integrate(self, state, wrench, foot_positions_world):
        """One RK4 prediction step."""

        h = self.dt
        k1 = self.derivative(state, wrench, foot_positions_world)
        k2 = self.derivative(state + .5 * h * k1, wrench, foot_positions_world)
        k3 = self.derivative(state + .5 * h * k2, wrench, foot_positions_world)
        k4 = self.derivative(state + h * k3, wrench, foot_positions_world)
        result = state + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        # Keep Euler angles on a continuous local branch near zero/reference.
        result[:3] = np.arctan2(np.sin(result[:3]), np.cos(result[:3]))
        return result

    def rollout(self, state, control_sequence, foot_positions_world):
        state = as_state(state).copy()
        controls = np.asarray(control_sequence, dtype=float).reshape(self.horizon, WRENCH_SIZE)
        prediction = np.empty((self.horizon, STATE_SIZE))
        for step, wrench in enumerate(controls):
            state = self.integrate(state, wrench, foot_positions_world)
            prediction[step] = state
        return prediction

    def _initial_guess(self, contacts):
        if self._warm_start is not None and self._warm_start.shape == (self.horizon, WRENCH_SIZE):
            guess = self._warm_start.copy()
        else:
            guess = np.vstack([nominal_wrench(self.parameters, mode) for mode in contacts])
        for step, mode in enumerate(contacts):
            for foot in range(2):
                if not mode[foot]:
                    guess[step, foot * 6:(foot + 1) * 6] = 0.0
        return guess

    def _bounds(self, contacts):
        from scipy.optimize import Bounds

        lower, upper = [], []
        params = self.parameters
        active_low = [-1000.0, -1000.0, 0.0,
                      -params.max_roll_moment, -params.max_pitch_moment,
                      -params.max_yaw_moment]
        active_high = [1000.0, 1000.0, params.max_normal_force,
                       params.max_roll_moment, params.max_pitch_moment,
                       params.max_yaw_moment]
        for mode in contacts:
            for active in mode:
                lower.extend(active_low if active else [0.0] * 6)
                upper.extend(active_high if active else [0.0] * 6)
        return Bounds(np.asarray(lower), np.asarray(upper))

    def solve(self, state, reference, foot_positions_world, contacts=(True, True)):
        try:
            from scipy.optimize import minimize
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("NonlinearSrbdMpc 需要 SciPy；Isaac Sim Python 已自带") from exc

        x0 = as_state(state)
        reference = as_reference(reference, self.horizon)
        feet = as_foot_positions(foot_positions_world)
        contacts = as_contact_schedule(contacts, self.horizon)
        nominal = np.vstack([nominal_wrench(self.parameters, mode) for mode in contacts])
        guess = self._initial_guess(contacts)
        q = self.state_weights

        def objective(flat):
            controls = flat.reshape(self.horizon, WRENCH_SIZE)
            predicted = self.rollout(x0, controls, feet)
            error = predicted - reference
            error[:, :3] = np.arctan2(np.sin(error[:, :3]), np.cos(error[:, :3]))
            value = float(np.sum(error * error * q[None, :]))
            value += self.input_weight * float(np.sum((controls - nominal) ** 2))
            previous = self._previous_wrench if self._previous_wrench is not None else nominal[0]
            changes = np.vstack((controls[0] - previous, np.diff(controls, axis=0)))
            value += self.wrench_rate_weight * float(np.sum(changes ** 2))
            return value

        def constraints(flat):
            controls = flat.reshape(self.horizon, WRENCH_SIZE)
            return np.concatenate([
                wrench_constraint_values(controls[step], contacts[step], self.parameters,
                                         circular_friction=True)
                for step in range(self.horizon)
            ])

        start = time.perf_counter()
        solution = minimize(
            objective, guess.reshape(-1), method="SLSQP",
            bounds=self._bounds(contacts),
            constraints={"type": "ineq", "fun": constraints},
            options={"maxiter": self.max_iterations, "ftol": self.tolerance,
                     "disp": False},
        )
        elapsed = time.perf_counter() - start
        controls = np.asarray(solution.x, dtype=float).reshape(self.horizon, WRENCH_SIZE)
        margins = constraints(controls.reshape(-1))
        success = bool(solution.success and np.isfinite(controls).all()
                       and margins.min() >= -5e-5)
        if success:
            self._warm_start = np.vstack((controls[1:], controls[-1:]))
            self._previous_wrench = controls[0].copy()
        prediction = self.rollout(x0, controls, feet)
        return NmpcResult(
            wrench=controls[0].copy(),
            control_sequence=controls,
            predicted_states=prediction,
            success=success,
            status=str(solution.message),
            objective=float(solution.fun),
            solve_time_s=elapsed,
            iterations=int(solution.nit),
            minimum_constraint_margin=float(margins.min()),
        )


def self_check():
    controller = NonlinearSrbdMpc(dt=.02, horizon=3, max_iterations=30)
    state = np.zeros(STATE_SIZE)
    state[5] = 1.08
    feet = np.array([[0.0, .115, 0.0], [0.0, -.115, 0.0]])
    nominal = nominal_wrench(controller.parameters, (True, True))
    derivative = controller.derivative(state, nominal, feet)
    result = controller.solve(state, state, feet, (True, True))
    report = {
        "success": result.success,
        "stationary_derivative_norm": float(np.linalg.norm(derivative)),
        "minimum_constraint_margin": result.minimum_constraint_margin,
        "status": result.status,
        "iterations": result.iterations,
        "solve_time_s": result.solve_time_s,
        "first_wrench": result.wrench.tolist(),
    }
    report["passed"] = bool(report["success"]
                            and report["stationary_derivative_norm"] < 1e-10
                            and report["minimum_constraint_margin"] >= -5e-5)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if not args.self_check:
        parser.error("当前命令行入口需要 --self-check")
    report = self_check()
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
