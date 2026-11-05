"""Executable Python reference for the OpenLoong-style linear Convex MPC.

This is intentionally a readable formula/reference implementation.  It uses a
12-D physical wrench and an affine gravity term, whereas the upstream C++ QP
uses a 13th input fixed to ``m*g``.  Both representations describe the same
gravity acceleration, but their raw QP matrices are therefore not byte-for-byte
identical.  The controller retains OpenLoong's state order, yaw linearization,
Euler discretization and move blocking (the last control block is held).

Run the deterministic mathematical self-check with Isaac's Python:

    python3 Pro2/run_l4_control.py self-check --controller convex
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
                             as_foot_positions, as_reference, as_state, nominal_wrench,
                             rotation_rpy, skew, wrench_constraint_values)
except ImportError:  # direct ``python3 Pro2/control/...py`` execution
    from srbd_model import (STATE_SIZE, WRENCH_SIZE, SrbdParameters, as_contact_schedule,
                            as_foot_positions, as_reference, as_state, nominal_wrench,
                            rotation_rpy, skew, wrench_constraint_values)

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("请使用 Isaac Sim Python 运行，当前解释器缺少 NumPy") from exc


DEFAULT_STATE_WEIGHTS = np.array([
    1.0, 1.0, 1.0, 1e-3, 100.0, 1.0,
    1e-3, 1e-3, 1e-3, 1.0, 100.0, 1.0,
])


@dataclass
class ConvexMpcResult:
    wrench: np.ndarray
    control_blocks: np.ndarray
    predicted_states: np.ndarray
    success: bool
    status: str
    objective: float
    solve_time_s: float
    iterations: int
    hessian_min_eigenvalue: float
    minimum_constraint_margin: float


class ConvexMpcReference:
    """Condensed linear SRBD QP solved by OSQP."""

    def __init__(self, *, dt=0.005, horizon=10, control_horizon=3,
                 parameters=None, state_weights=None, input_weight=1e-6):
        self.dt = float(dt)
        self.horizon = int(horizon)
        self.control_horizon = int(control_horizon)
        self.parameters = parameters or SrbdParameters()
        self.parameters.validate()
        self.state_weights = np.asarray(
            DEFAULT_STATE_WEIGHTS if state_weights is None else state_weights,
            dtype=float)
        self.input_weight = float(input_weight)
        if not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError("dt 必须为有限正数")
        if self.horizon < 1 or not 1 <= self.control_horizon <= self.horizon:
            raise ValueError("要求 1 <= control_horizon <= horizon")
        if self.state_weights.shape != (STATE_SIZE,) or np.any(self.state_weights < 0):
            raise ValueError("state_weights 必须包含 12 个非负数")
        if not math.isfinite(self.input_weight) or self.input_weight <= 0:
            raise ValueError("input_weight 必须为有限正数")
        self._warm_start = None

    def linear_model(self, state, foot_positions_world):
        """Return Euler-discretized ``x+ = A x + B u + d``."""

        x = as_state(state)
        feet = as_foot_positions(foot_positions_world)
        params = self.parameters
        a_c = np.zeros((STATE_SIZE, STATE_SIZE))
        # The C++ baseline retains only yaw in this kinematic linearization.
        yaw_rotation = rotation_rpy([0.0, 0.0, x[2]])
        a_c[0:3, 6:9] = yaw_rotation.T
        a_c[3:6, 9:12] = np.eye(3)
        b_c = np.zeros((STATE_SIZE, WRENCH_SIZE))
        inertia_world = yaw_rotation @ params.inertia @ yaw_rotation.T
        inertia_world_inv = np.linalg.inv(inertia_world)
        for foot in range(2):
            start = foot * 6
            lever = feet[foot] - x[3:6]
            b_c[6:9, start:start + 3] = inertia_world_inv @ skew(lever)
            b_c[6:9, start + 3:start + 6] = inertia_world_inv
            b_c[9:12, start:start + 3] = np.eye(3) / params.mass
        a = np.eye(STATE_SIZE) + self.dt * a_c
        b = self.dt * b_c
        d = np.zeros(STATE_SIZE)
        d[11] = -self.dt * params.gravity
        return a, b, d

    def condensed_prediction(self, state, foot_positions_world):
        """Return matrices in ``X = Sx*x0 + Su*U + sd``."""

        x = as_state(state)
        a, b, d = self.linear_model(x, foot_positions_world)
        rows = self.horizon * STATE_SIZE
        cols = self.control_horizon * WRENCH_SIZE
        sx = np.zeros((rows, STATE_SIZE))
        su = np.zeros((rows, cols))
        sd = np.zeros(rows)
        map_x = np.eye(STATE_SIZE)
        map_u = np.zeros((STATE_SIZE, cols))
        offset = np.zeros(STATE_SIZE)
        for step in range(self.horizon):
            block = min(step, self.control_horizon - 1)
            map_x = a @ map_x
            map_u = a @ map_u
            map_u[:, block * WRENCH_SIZE:(block + 1) * WRENCH_SIZE] += b
            offset = a @ offset + d
            sl = slice(step * STATE_SIZE, (step + 1) * STATE_SIZE)
            sx[sl], su[sl], sd[sl] = map_x, map_u, offset
        return sx, su, sd

    def _constraint_matrix(self, contacts):
        """Build affine pyramid/CoP constraints for the independent blocks."""

        rows, lower, upper = [], [], []
        columns = self.control_horizon * WRENCH_SIZE
        params = self.parameters

        def add(block, coefficients, lo=-np.inf, hi=np.inf):
            row = np.zeros(columns)
            start = block * WRENCH_SIZE
            row[start:start + WRENCH_SIZE] = coefficients
            rows.append(row)
            lower.append(lo)
            upper.append(hi)

        for block in range(self.control_horizon):
            for foot in range(2):
                base = foot * 6
                if not contacts[block, foot]:
                    for axis in range(6):
                        coefficients = np.zeros(WRENCH_SIZE)
                        coefficients[base + axis] = 1.0
                        add(block, coefficients, 0.0, 0.0)
                    continue
                variable_bounds = (
                    (-1000.0, 1000.0), (-1000.0, 1000.0),
                    (0.0, params.max_normal_force),
                    (-params.max_roll_moment, params.max_roll_moment),
                    (-params.max_pitch_moment, params.max_pitch_moment),
                    (-params.max_yaw_moment, params.max_yaw_moment),
                )
                for axis, (lo, hi) in enumerate(variable_bounds):
                    coefficients = np.zeros(WRENCH_SIZE)
                    coefficients[base + axis] = 1.0
                    add(block, coefficients, lo, hi)
                # The sqrt(2) factor follows the upstream four-sided pyramid.
                mu = params.friction / math.sqrt(2.0)
                for tangent in (0, 1):
                    for sign in (-1.0, 1.0):
                        coefficients = np.zeros(WRENCH_SIZE)
                        coefficients[base + tangent] = sign
                        coefficients[base + 2] = -mu
                        add(block, coefficients, hi=0.0)
                # Simple rectangular centre-of-pressure bounds in foot axes.
                for moment_axis, extent in ((3, params.foot_half_width),
                                            (4, params.foot_front)):
                    for sign in (-1.0, 1.0):
                        coefficients = np.zeros(WRENCH_SIZE)
                        coefficients[base + moment_axis] = sign
                        coefficients[base + 2] = -extent
                        add(block, coefficients, hi=0.0)
        return np.asarray(rows), np.asarray(lower), np.asarray(upper)

    def solve(self, state, reference, foot_positions_world, contacts=(True, True)):
        try:
            import osqp
            from scipy import sparse
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "ConvexMpcReference 需要 OSQP 和 SciPy；Isaac Sim Python 已自带这两个包"
            ) from exc

        x0 = as_state(state)
        reference = as_reference(reference, self.horizon)
        contacts = as_contact_schedule(contacts, self.horizon)
        sx, su, sd = self.condensed_prediction(x0, foot_positions_world)
        qbar = np.diag(np.tile(self.state_weights, self.horizon))
        rbar = np.eye(self.control_horizon * WRENCH_SIZE) * self.input_weight
        nominal = np.concatenate([
            nominal_wrench(self.parameters, contacts[block])
            for block in range(self.control_horizon)
        ])
        error_without_control = sx @ x0 + sd - reference.reshape(-1)
        hessian = 2.0 * (su.T @ qbar @ su + rbar)
        hessian = 0.5 * (hessian + hessian.T) + np.eye(hessian.shape[0]) * 1e-10
        gradient = 2.0 * (su.T @ qbar @ error_without_control - rbar @ nominal)
        constraints, lower, upper = self._constraint_matrix(contacts)
        solver = osqp.OSQP()
        solver.setup(P=sparse.csc_matrix(sparse.triu(sparse.csc_matrix(hessian))), q=gradient,
                     A=sparse.csc_matrix(constraints), l=lower, u=upper,
                     verbose=False, polishing=True, eps_abs=1e-6, eps_rel=1e-6,
                     max_iter=10000)
        if self._warm_start is not None and self._warm_start.shape == gradient.shape:
            solver.warm_start(x=self._warm_start)
        start = time.perf_counter()
        solution = solver.solve()
        elapsed = time.perf_counter() - start
        success = solution.info.status_val in (1, 2) and solution.x is not None
        if not success:
            controls = nominal
        else:
            controls = np.asarray(solution.x, dtype=float)
            shifted = controls.reshape(self.control_horizon, WRENCH_SIZE)
            self._warm_start = np.vstack((shifted[1:], shifted[-1:])).reshape(-1)
        predicted = (sx @ x0 + su @ controls + sd).reshape(self.horizon, STATE_SIZE)
        first_contacts = contacts[0]
        margins = wrench_constraint_values(
            controls[:WRENCH_SIZE], first_contacts, self.parameters,
            circular_friction=False)
        return ConvexMpcResult(
            wrench=controls[:WRENCH_SIZE].copy(),
            control_blocks=controls.reshape(self.control_horizon, WRENCH_SIZE),
            predicted_states=predicted,
            success=success,
            status=str(solution.info.status),
            objective=float(solution.info.obj_val),
            solve_time_s=elapsed,
            iterations=int(solution.info.iter),
            hessian_min_eigenvalue=float(np.linalg.eigvalsh(hessian).min()),
            minimum_constraint_margin=float(margins.min()),
        )


def self_check():
    controller = ConvexMpcReference(dt=.02, horizon=5, control_horizon=3)
    state = np.zeros(STATE_SIZE)
    state[5] = 1.08
    reference = state.copy()
    feet = np.array([[0.0, 0.115, 0.0], [0.0, -0.115, 0.0]])
    result = controller.solve(state, reference, feet, (True, True))
    vertical = float(result.wrench[2] + result.wrench[8])
    expected = controller.parameters.mass * controller.parameters.gravity
    checks = {
        "success": result.success,
        "hessian_positive_definite": result.hessian_min_eigenvalue > 0,
        "constraints_satisfied": result.minimum_constraint_margin >= -1e-5,
        "vertical_force_relative_error": abs(vertical - expected) / expected,
        "status": result.status,
        "iterations": result.iterations,
        "solve_time_s": result.solve_time_s,
        "first_wrench": result.wrench.tolist(),
    }
    checks["passed"] = bool(checks["success"] and checks["hessian_positive_definite"]
                            and checks["constraints_satisfied"]
                            and checks["vertical_force_relative_error"] < .25)
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-check", action="store_true", help="运行静态双支撑数学自检")
    parser.add_argument("--output", type=Path, help="将自检 JSON 写入该路径")
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
