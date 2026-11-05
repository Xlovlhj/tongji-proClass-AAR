"""Small bridge between a Python wrench optimizer and the native WBC.

The Isaac loop owns state acquisition and calls :meth:`update` at physics rate.
The optimizer is evaluated at ``update_period_s``; its first wrench is retained
by the C++ bridge between updates.  On any failed nonlinear solve the bridge is
given a nonzero status, so it automatically uses the internal Convex MPC wrench
for that cycle instead of applying an untrusted result.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class ExternalMpcUpdate:
    updated: bool
    applied_external: bool
    success: bool
    status: str
    solve_time_s: float
    iterations: int


class ExternalMpcRuntime:
    def __init__(self, solver, *, update_period_s=.02):
        self.solver = solver
        self.update_period_s = float(update_period_s)
        if not math.isfinite(self.update_period_s) or self.update_period_s <= 0:
            raise ValueError("update_period_s 必须为有限正数")
        self.next_update_s = 0.0
        self.last_result = None

    def update(self, controller, sim_time_s, state, reference,
               foot_positions_world, contacts):
        """Solve when due and configure ``controller``'s external wrench mode."""

        sim_time_s = float(sim_time_s)
        if not math.isfinite(sim_time_s) or sim_time_s < 0:
            raise ValueError("sim_time_s 必须为有限非负数")
        if sim_time_s + 1e-12 < self.next_update_s:
            result = self.last_result
            return ExternalMpcUpdate(
                updated=False,
                applied_external=bool(result and result.success),
                success=bool(result and result.success),
                status="held" if result is not None else "waiting",
                solve_time_s=0.0,
                iterations=0,
            )
        result = self.solver.solve(state, reference, foot_positions_world, contacts)
        self.last_result = result
        reported_iterations = int(getattr(result, "qp_iterations", result.iterations))
        # Advance from the scheduled time to avoid drift if one solve is slow.
        while self.next_update_s <= sim_time_s + 1e-12:
            self.next_update_s += self.update_period_s
        controller.set_external_mpc_wrench(
            result.wrench,
            solve_time=result.solve_time_s,
            solver_status=0 if result.success else 1,
            iterations=reported_iterations,
        )
        return ExternalMpcUpdate(
            updated=True,
            applied_external=result.success,
            success=result.success,
            status=result.status,
            solve_time_s=result.solve_time_s,
            iterations=reported_iterations,
        )

    def disable(self, controller):
        controller.set_external_mpc_wrench(None)
        self.last_result = None
        self.next_update_s = 0.0
