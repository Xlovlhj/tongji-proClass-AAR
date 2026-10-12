"""Online learning contracts and a disturbance-rejection plant experiment.

Run with a Python environment containing numpy:
python3 -m unittest discover -s Pro1/tests -v
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raa.adaptive_control import FuzzyAdaptive, FuzzyConfig, RBFAdaptive, RBFConfig, make_adaptive_controller
from raa.control import PDConfig, TorquePD


class AdaptiveControlTests(unittest.TestCase):
    def test_both_learn_causally_and_reset_independent_trials(self):
        zeros = np.zeros(7)
        reference = np.full(7, 0.005)
        for factory in (RBFAdaptive, FuzzyAdaptive):
            with self.subTest(controller=factory.__name__):
                controller = factory()
                expected_pd = controller.kp * reference
                first = controller.compute(zeros, zeros, reference, zeros, zeros)
                np.testing.assert_allclose(first, expected_pd)
                self.assertGreater(np.linalg.norm(controller.weights), 0)
                second = controller.compute(zeros, zeros, reference, zeros, zeros)
                self.assertTrue(np.all(second > first))
                self.assertEqual(controller.training_step_count, 2)
                controller.reset()
                self.assertEqual(controller.training_step_count, 0)
                np.testing.assert_array_equal(controller.weights, np.zeros_like(controller.weights))
                np.testing.assert_allclose(controller.compute(zeros, zeros, reference, zeros, zeros), first)

    def test_learning_scales_with_elapsed_time_and_freezes_during_hold(self):
        zeros = np.zeros(7)
        reference = np.full(7, 0.001)
        for factory in (RBFAdaptive, FuzzyAdaptive):
            with self.subTest(controller=factory.__name__):
                fast, slow = factory(), factory()
                fast.compute(zeros, zeros, reference, zeros, zeros, dt=0.001)
                slow.compute(zeros, zeros, reference, zeros, zeros, dt=0.002)
                np.testing.assert_allclose(slow.weights, 2 * fast.weights)
                saved, count = fast.weights.copy(), fast.training_step_count
                torque = fast.compute(zeros, zeros, reference, zeros, zeros, adapt=False)
                self.assertTrue(np.all(torque > fast.kp * reference))
                np.testing.assert_array_equal(fast.weights, saved)
                self.assertEqual(fast.training_step_count, count)
                self.assertFalse(np.any(fast.last_learning_mask))

    def test_normalized_basis_and_fuzzy_local_rules(self):
        zeros = np.zeros(7)
        rbf = RBFAdaptive()
        rbf.compute(np.full(7, -1.2), zeros, zeros, zeros, zeros, adapt=False)
        negative_pattern = rbf.last_activation.copy()
        rbf.compute(np.full(7, 1.2), zeros, zeros, zeros, zeros, adapt=False)
        self.assertFalse(np.allclose(negative_pattern, rbf.last_activation))
        self.assertEqual(rbf.last_activation.shape, (7, 9))
        np.testing.assert_allclose(rbf.last_activation.sum(axis=1), np.ones(7))
        self.assertTrue(np.all(rbf.last_activation >= 0))
        fuzzy = FuzzyAdaptive()
        fuzzy.compute(zeros, zeros, np.full(7, 0.005), np.full(7, 0.05), zeros, adapt=False)
        self.assertEqual(fuzzy.last_activation.shape, (7, 25))
        np.testing.assert_allclose(fuzzy.last_activation.sum(axis=1), np.ones(7))
        np.testing.assert_array_equal((fuzzy.last_activation > 0).sum(axis=1), np.full(7, 4))
        fuzzy.compute(zeros, zeros, np.full(7, 2.0), np.full(7, 5.0), zeros, adapt=False)
        np.testing.assert_allclose(fuzzy.last_activation.sum(axis=1), np.ones(7))
        self.assertTrue(np.all(np.isfinite(fuzzy.last_torque)))

    def test_projection_bounds_compensation_under_aggressive_learning(self):
        zeros = np.zeros(7)
        for factory, config in ((RBFAdaptive, RBFConfig), (FuzzyAdaptive, FuzzyConfig)):
            with self.subTest(controller=factory.__name__):
                controller = factory(config(learning_rate=1e8, leakage=0))
                for _ in range(5):
                    torque = controller.compute(zeros, zeros, np.full(7, 0.002), zeros, zeros)
                    self.assertTrue(np.all(np.abs(controller.weights) <= controller.compensation_limits[:, None]))
                    self.assertTrue(np.all(np.abs(controller.last_compensation) <= controller.compensation_limits + 1e-12))
                    self.assertTrue(np.all(np.abs(torque) <= controller.torque_limits))
                self.assertTrue(np.any(np.isclose(controller.weights, controller.compensation_limits[:, None])))

    def test_saturation_antiwindup_retains_leakage_and_allows_inward_learning(self):
        zeros = np.zeros(7)
        for factory in (RBFAdaptive, FuzzyAdaptive):
            with self.subTest(controller=factory.__name__):
                controller = factory()
                controller.weights.fill(0.3)
                old = controller.weights.copy()
                torque = controller.compute(zeros, zeros, np.ones(7), zeros, zeros)
                self.assertTrue(np.all(controller.saturation_mask))
                self.assertFalse(np.any(controller.last_learning_mask))
                np.testing.assert_allclose(torque, controller.torque_limits)
                np.testing.assert_allclose(controller.weights, old * (1 - controller.config.leakage * controller.config.dt))
                controller.compute(zeros, zeros, np.full(7, 10.0), np.full(7, -61.0), zeros)
                self.assertTrue(np.all(controller.saturation_mask))
                self.assertTrue(np.all(controller.last_learning_mask))
                self.assertTrue(np.all(controller.last_sliding_error < 0))

    def test_zero_error_leakage_and_serializable_state_exports(self):
        zeros = np.zeros(7)
        for factory in (RBFAdaptive, FuzzyAdaptive):
            with self.subTest(controller=factory.__name__):
                controller = factory()
                controller.weights.fill(0.5)
                old = controller.weights.copy()
                controller.compute(zeros, zeros, zeros, zeros, zeros)
                self.assertLess(np.linalg.norm(controller.weights), np.linalg.norm(old))
                summary = controller.state_summary()
                json.dumps(summary, allow_nan=False)
                self.assertEqual(summary["training_step_count"], 1)
                arrays = controller.state_arrays()
                self.assertEqual(arrays["weights"].shape, controller.weights.shape)
                self.assertTrue(all(value.dtype != object for value in arrays.values()))
                arrays["weights"].fill(0)
                self.assertGreater(np.linalg.norm(controller.weights), 0)

    def test_invalid_config_or_state_cannot_silently_start_learning(self):
        zeros = np.zeros(7)
        for factory in (RBFAdaptive, FuzzyAdaptive):
            with self.subTest(controller=factory.__name__):
                controller = factory()
                for dt in (0, -0.001, np.nan, np.inf, 0.02):
                    with self.assertRaises(ValueError):
                        controller.compute(zeros, zeros, zeros, zeros, zeros, dt=dt)
                with self.assertRaises(ValueError):
                    controller.compute([np.nan] * 7, zeros, zeros, zeros, zeros)
                with self.assertRaises(ValueError):
                    controller.compute(zeros, zeros, zeros, zeros)
                self.assertEqual(controller.training_step_count, 0)
                self.assertFalse(np.any(controller.weights))
        for kwargs in ({"learning_rate": 0}, {"sliding_lambda": -1}, {"leakage": -0.1},
                       {"compensation_limits": [0] * 7}, {"pd": PDConfig(gravity_compensation=False)}):
            with self.assertRaises(ValueError):
                RBFConfig(**kwargs)
        with self.assertRaises(ValueError):
            RBFConfig(centers_per_axis=3.5)
        with self.assertRaises(ValueError):
            make_adaptive_controller("rbf", parameters={"learningrate": 30})
        with self.assertRaises(ValueError):
            make_adaptive_controller("rbf", parameters={"error_scale": 0.02})

    def test_online_learning_reduces_unseen_constant_disturbance_error(self):
        """Closed-loop evidence of adaptation beyond changing a weight array.

        Independent seven-axis inertial plant, starting at rest with a constant
        unmodelled external torque. Identical PD gains/limits and initial state.
        This is an algorithm check, not a claim about the Panda's performance.
        """
        inertia = np.array([1., 1., 1., 1., 0.1, 0.1, 0.05])
        disturbance = np.array([1., -0.8, 0.6, -0.4, 0.2, -0.2, 0.15])
        zeros = np.zeros(7)
        def experiment(controller):
            q, dq = zeros.copy(), zeros.copy()
            squared_error = 0.0
            for step in range(8000):
                torque = controller.compute(q, dq, zeros, zeros, zeros)
                dq += 0.001 * (torque + disturbance) / inertia
                q += 0.001 * dq
                if step >= 7000:
                    squared_error += float(q @ q)
            return np.sqrt(squared_error / (1000 * 7))
        baseline = experiment(TorquePD())
        for factory in (RBFAdaptive, FuzzyAdaptive):
            with self.subTest(controller=factory.__name__):
                controller = factory()
                adapted = experiment(controller)
                self.assertLess(adapted, baseline * 0.7)
                self.assertEqual(controller.training_step_count, 8000)
                self.assertTrue(np.all(np.isfinite(controller.weights)))


if __name__ == "__main__":
    unittest.main()
