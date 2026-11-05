"""One-SQP-step real-time-iteration NMPC for the L4 Isaac comparison.

Each control update rolls out the nonlinear SRBD model, linearizes its RK4 map
along that trajectory, condenses the local state sensitivities and solves one
constrained QP with OSQP.  This is the standard SQP-RTI execution pattern: it
does not iterate to full NLP convergence at each sample, so its local solution
must not be described as the global optimum of the nonlinear walking problem.
"""
from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
from pathlib import Path
import time

try:
    from .convex_mpc_reference import ConvexMpcReference
    from .nmpc_srbd import DEFAULT_STATE_WEIGHTS, NonlinearSrbdMpc
    from .srbd_model import (STATE_SIZE, WRENCH_SIZE, as_contact_schedule,
                             as_foot_positions, as_reference, as_state,
                             nominal_wrench, wrench_constraint_values)
except ImportError:
    from convex_mpc_reference import ConvexMpcReference
    from nmpc_srbd import DEFAULT_STATE_WEIGHTS, NonlinearSrbdMpc
    from srbd_model import (STATE_SIZE, WRENCH_SIZE, as_contact_schedule,
                            as_foot_positions, as_reference, as_state,
                            nominal_wrench, wrench_constraint_values)

import numpy as np


@dataclass
class RtiNmpcResult:
    wrench: np.ndarray
    control_sequence: np.ndarray
    predicted_states: np.ndarray
    success: bool
    status: str
    objective: float
    solve_time_s: float
    iterations: int
    minimum_constraint_margin: float
    qp_iterations: int
    linearization_time_s: float
    qp_solve_time_s: float


class RtiNmpc:
    """Nonlinear SRBD MPC with one local SQP/QP update per control cycle."""

    def __init__(self, *, dt=.02, horizon=5, parameters=None,
                 state_weights=None, input_weight=2e-6,
                 wrench_rate_weight=1e-7, state_difference=1e-5,
                 wrench_difference=1e-3):
        self.model = NonlinearSrbdMpc(
            dt=dt, horizon=horizon, parameters=parameters,
            state_weights=(DEFAULT_STATE_WEIGHTS if state_weights is None else state_weights),
            input_weight=input_weight, wrench_rate_weight=wrench_rate_weight,
            max_iterations=1)
        self.dt = self.model.dt
        self.horizon = self.model.horizon
        self.parameters = self.model.parameters
        self.state_weights = self.model.state_weights
        self.input_weight = self.model.input_weight
        self.wrench_rate_weight = self.model.wrench_rate_weight
        self.state_difference = float(state_difference)
        self.wrench_difference = float(wrench_difference)
        if self.state_difference <= 0 or self.wrench_difference <= 0:
            raise ValueError("有限差分步长必须为正")
        self._warm_start = None
        self._previous_wrench = None
        self._constraint_builder = ConvexMpcReference(
            dt=dt, horizon=horizon, control_horizon=horizon,
            parameters=self.parameters, state_weights=self.state_weights,
            input_weight=input_weight)

    def _linearize_step(self, state, wrench, feet):
        # Linearize the continuous nonlinear vector field with forward finite
        # differences, then use the same first-order discretization as the C++
        # baseline for the RTI sensitivity.  The nominal rollout itself remains
        # RK4.  This cuts the preparation cost enough for online experimentation
        # while retaining the full nonlinear operating-point dependence.
        base = self.model.derivative(state, wrench, feet)
        a = np.eye(STATE_SIZE)
        b = np.empty((STATE_SIZE, WRENCH_SIZE))
        for column in range(STATE_SIZE):
            perturb = np.zeros(STATE_SIZE)
            scale = self.state_difference * max(1.0, abs(state[column]))
            perturb[column] = scale
            a[:, column] += self.dt * (
                self.model.derivative(state + perturb, wrench, feet) - base
            ) / scale
        for column in range(WRENCH_SIZE):
            perturb = np.zeros(WRENCH_SIZE)
            scale = self.wrench_difference * max(1.0, abs(wrench[column]))
            perturb[column] = scale
            b[:, column] = self.dt * (
                self.model.derivative(state, wrench + perturb, feet) - base
            ) / scale
        return a, b

    def _nominal_controls(self, contacts):
        if self._warm_start is not None and self._warm_start.shape == (self.horizon, WRENCH_SIZE):
            controls = self._warm_start.copy()
        else:
            controls = np.vstack([nominal_wrench(self.parameters, mode) for mode in contacts])
        for step, mode in enumerate(contacts):
            for foot, active in enumerate(mode):
                if not active:
                    controls[step, foot * 6:(foot + 1) * 6] = 0.0
        return controls

    def solve(self, state, reference, foot_positions_world, contacts=(True, True)):
        import osqp
        from scipy import sparse

        begin = time.perf_counter()
        x0 = as_state(state)
        reference = as_reference(reference, self.horizon)
        feet = as_foot_positions(foot_positions_world)
        contacts = as_contact_schedule(contacts, self.horizon)
        controls = self._nominal_controls(contacts)
        nominal_states = self.model.rollout(x0, controls, feet)

        linearization_begin = time.perf_counter()
        state_at_step = np.vstack((x0[None, :], nominal_states[:-1]))
        linearizations = [
            self._linearize_step(state_at_step[step], controls[step], feet)
            for step in range(self.horizon)
        ]
        columns = self.horizon * WRENCH_SIZE
        sensitivity = np.zeros((self.horizon * STATE_SIZE, columns))
        propagated = np.zeros((STATE_SIZE, columns))
        for step, (a, b) in enumerate(linearizations):
            propagated = a @ propagated
            propagated[:, step * WRENCH_SIZE:(step + 1) * WRENCH_SIZE] += b
            sensitivity[step * STATE_SIZE:(step + 1) * STATE_SIZE] = propagated
        linearization_time = time.perf_counter() - linearization_begin

        qbar = np.diag(np.tile(self.state_weights, self.horizon))
        rbar = np.eye(columns) * self.input_weight
        nominal_support = np.vstack([
            nominal_wrench(self.parameters, mode) for mode in contacts
        ]).reshape(-1)
        controls_flat = controls.reshape(-1)
        error = nominal_states.reshape(-1) - reference.reshape(-1)
        # Angle error must stay on its local wrapped branch.
        error.reshape(self.horizon, STATE_SIZE)[:, :3] = np.arctan2(
            np.sin(error.reshape(self.horizon, STATE_SIZE)[:, :3]),
            np.cos(error.reshape(self.horizon, STATE_SIZE)[:, :3]))
        hessian = 2.0 * (sensitivity.T @ qbar @ sensitivity + rbar)
        gradient = 2.0 * (sensitivity.T @ qbar @ error
                          + rbar @ (controls_flat - nominal_support))

        # Penalize the change from the previously applied wrench and between
        # consecutive horizon nodes.  E maps absolute U to those differences.
        if self.wrench_rate_weight > 0:
            difference = np.zeros((columns, columns))
            for step in range(self.horizon):
                sl = slice(step * WRENCH_SIZE, (step + 1) * WRENCH_SIZE)
                difference[sl, sl] = np.eye(WRENCH_SIZE)
                if step > 0:
                    previous = slice((step - 1) * WRENCH_SIZE, step * WRENCH_SIZE)
                    difference[sl, previous] = -np.eye(WRENCH_SIZE)
            offset = np.zeros(columns)
            if self._previous_wrench is not None:
                offset[:WRENCH_SIZE] = -self._previous_wrench
            else:
                offset[:WRENCH_SIZE] = -nominal_support[:WRENCH_SIZE]
            rate_error = difference @ controls_flat + offset
            hessian += 2.0 * self.wrench_rate_weight * difference.T @ difference
            gradient += 2.0 * self.wrench_rate_weight * difference.T @ rate_error
        hessian = .5 * (hessian + hessian.T) + np.eye(columns) * 1e-10

        constraint_matrix, lower, upper = self._constraint_builder._constraint_matrix(contacts)
        current_constraint = constraint_matrix @ controls_flat
        solver = osqp.OSQP()
        solver.setup(
            P=sparse.csc_matrix(sparse.triu(sparse.csc_matrix(hessian))), q=gradient,
            A=sparse.csc_matrix(constraint_matrix),
            l=lower - current_constraint, u=upper - current_constraint,
            verbose=False, polishing=False, eps_abs=1e-5, eps_rel=1e-5,
            max_iter=4000, warm_starting=True)
        qp_begin = time.perf_counter()
        solution = solver.solve()
        qp_time = time.perf_counter() - qp_begin
        success = solution.info.status_val in (1, 2) and solution.x is not None
        if success:
            controls = (controls_flat + np.asarray(solution.x)).reshape(
                self.horizon, WRENCH_SIZE)
        predicted = self.model.rollout(x0, controls, feet)
        margins = np.concatenate([
            wrench_constraint_values(controls[step], contacts[step], self.parameters,
                                     circular_friction=True)
            for step in range(self.horizon)
        ])
        success = bool(success and np.isfinite(controls).all() and margins.min() >= -1e-4)
        if success:
            self._warm_start = np.vstack((controls[1:], controls[-1:]))
            self._previous_wrench = controls[0].copy()
        predicted_error = predicted - reference
        predicted_error[:, :3] = np.arctan2(np.sin(predicted_error[:, :3]),
                                             np.cos(predicted_error[:, :3]))
        objective = float(np.sum(predicted_error ** 2 * self.state_weights[None, :])
                          + self.input_weight * np.sum((controls.reshape(-1)
                                                        - nominal_support) ** 2))
        return RtiNmpcResult(
            wrench=controls[0].copy(), control_sequence=controls,
            predicted_states=predicted, success=success,
            status=str(solution.info.status), objective=objective,
            solve_time_s=time.perf_counter() - begin,
            iterations=1, minimum_constraint_margin=float(margins.min()),
            qp_iterations=int(solution.info.iter),
            linearization_time_s=linearization_time,
            qp_solve_time_s=qp_time,
        )


def self_check():
    state = np.zeros(STATE_SIZE)
    state[0] = .03
    state[5] = 1.08
    reference = np.zeros(STATE_SIZE)
    reference[5] = 1.08
    feet = np.array([[0.0, .115, 0.0], [0.0, -.115, 0.0]])
    controller = RtiNmpc(dt=.02, horizon=3)
    result = controller.solve(state, reference, feet, (True, True))
    report = {
        "success": result.success,
        "status": result.status,
        "sqp_iterations": result.iterations,
        "qp_iterations": result.qp_iterations,
        "solve_time_s": result.solve_time_s,
        "linearization_time_s": result.linearization_time_s,
        "qp_solve_time_s": result.qp_solve_time_s,
        "minimum_constraint_margin": result.minimum_constraint_margin,
        "first_wrench": result.wrench.tolist(),
        "test_condition": "0.03 rad initial roll (about 1.72 deg), double support, N=3, dt=0.02 s",
    }
    report["passed"] = bool(result.success and result.iterations == 1
                            and result.minimum_constraint_margin >= -1e-4)
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
