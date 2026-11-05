"""Run with Isaac's Python (NumPy/SciPy/OSQP are bundled).

python3 Pro2/run_l4_control.py test
"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

CONTROL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CONTROL_DIR))

import numpy as np

from convex_mpc_reference import ConvexMpcReference
from nmpc_srbd import NonlinearSrbdMpc
from nmpc_rti import RtiNmpc
from runtime_adapter import ExternalMpcRuntime
from srbd_model import (STATE_SIZE, SrbdParameters, nominal_wrench,
                        predict_contact_schedule, velocity_reference)


def standing_case():
    state = np.zeros(STATE_SIZE)
    state[5] = 1.08
    feet = np.array([[0.0, .115, 0.0], [0.0, -.115, 0.0]])
    return state, feet


class ConvexReferenceTests(unittest.TestCase):
    def test_prediction_dimensions_and_move_blocking(self):
        state, feet = standing_case()
        solver = ConvexMpcReference(horizon=7, control_horizon=3)
        sx, su, sd = solver.condensed_prediction(state, feet)
        self.assertEqual(sx.shape, (7 * 12, 12))
        self.assertEqual(su.shape, (7 * 12, 3 * 12))
        self.assertEqual(sd.shape, (7 * 12,))
        # The held third block must still influence the final prediction step.
        self.assertGreater(np.linalg.norm(su[-12:, -12:]), 0)

    def test_standing_qp_is_feasible_and_strictly_convex(self):
        state, feet = standing_case()
        result = ConvexMpcReference(dt=.02, horizon=5).solve(
            state, state, feet, (True, True))
        self.assertTrue(result.success, result.status)
        self.assertGreater(result.hessian_min_eigenvalue, 0)
        self.assertGreaterEqual(result.minimum_constraint_margin, -1e-5)
        expected = SrbdParameters().mass * SrbdParameters().gravity
        self.assertAlmostEqual(result.wrench[2] + result.wrench[8], expected, delta=1.0)

    def test_shared_reference_and_contact_prediction(self):
        state, _ = standing_case()
        reference = velocity_reference(state, [.2, 0.0, .1], dt=.02, horizon=4)
        self.assertEqual(reference.shape, (4, 12))
        self.assertGreater(reference[-1, 3], reference[0, 3])
        self.assertGreater(reference[-1, 2], reference[0, 2])
        schedule = predict_contact_schedule(0, 1, .98, dt=.02, horizon=3,
                                            swing_time=.4)
        np.testing.assert_array_equal(schedule,
                                      [[True, False], [False, True], [False, True]])


class NonlinearReferenceTests(unittest.TestCase):
    def test_nominal_wrench_is_stationary(self):
        state, feet = standing_case()
        solver = NonlinearSrbdMpc(horizon=2)
        wrench = nominal_wrench(solver.parameters, (True, True))
        np.testing.assert_allclose(solver.derivative(state, wrench, feet), 0.0, atol=1e-12)

    def test_nonlinear_solver_returns_feasible_wrench(self):
        state, feet = standing_case()
        result = NonlinearSrbdMpc(dt=.02, horizon=2, max_iterations=20).solve(
            state, state, feet, (True, True))
        self.assertTrue(result.success, result.status)
        self.assertGreaterEqual(result.minimum_constraint_margin, -5e-5)

    def test_rti_nmpc_tracks_nonlinear_rollout_and_is_feasible(self):
        state, feet = standing_case()
        state[0] = .03
        solver = RtiNmpc(dt=.02, horizon=2)
        result = solver.solve(state, np.array([0, 0, 0, 0, 0, 1.08,
                                               0, 0, 0, 0, 0, 0]),
                              feet, (True, True))
        self.assertTrue(result.success, result.status)
        self.assertEqual(result.iterations, 1)
        self.assertGreater(result.qp_iterations, 0)
        self.assertGreaterEqual(result.minimum_constraint_margin, -1e-4)


class RuntimeAdapterTests(unittest.TestCase):
    class Controller:
        def __init__(self):
            self.calls = []

        def set_external_mpc_wrench(self, wrench, **metadata):
            self.calls.append((wrench, metadata))

    def test_update_rate_and_success_metadata(self):
        state, feet = standing_case()
        controller = self.Controller()
        adapter = ExternalMpcRuntime(
            NonlinearSrbdMpc(dt=.02, horizon=2, max_iterations=20),
            update_period_s=.02)
        first = adapter.update(controller, 0.0, state, state, feet, (True, True))
        held = adapter.update(controller, .001, state, state, feet, (True, True))
        self.assertTrue(first.updated)
        self.assertTrue(first.applied_external)
        self.assertFalse(held.updated)
        self.assertEqual(len(controller.calls), 1)
        self.assertEqual(controller.calls[0][1]["solver_status"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
