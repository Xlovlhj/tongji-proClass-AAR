import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.analyze_l0_tuning import (
    CONDITIONS,
    EXPECTED_CONTROLLED_CONFIG,
    RunData,
    compute_metrics,
    validate_actual_single_variable,
    validate_planned_configs,
    validate_time_axis,
)


class L0TuningConfigTests(unittest.TestCase):
    def test_committed_configs_are_a_true_single_variable_sweep(self):
        self.assertEqual(validate_planned_configs(), [])

    def test_expected_time_axis_is_accepted(self):
        times = [index * 0.01 for index in range(1200)]
        self.assertEqual(validate_time_axis(times), [])

    def test_actual_single_variable_check_detects_an_extra_change(self):
        runs = []
        for condition in CONDITIONS:
            config = dict(EXPECTED_CONTROLLED_CONFIG)
            config["state_weights"] = list(EXPECTED_CONTROLLED_CONFIG["state_weights"])
            config["input_weights"] = list(EXPECTED_CONTROLLED_CONFIG["input_weights"])
            config["push_force"] = list(EXPECTED_CONTROLLED_CONFIG["push_force"])
            config["u_weight"] = condition.u_weight
            runs.append(RunData(condition, Path(condition.name), config=config))
        runs[-1].config["vx"] = 0.3
        issues = validate_actual_single_variable(runs)
        self.assertEqual(len(issues), 1)
        self.assertIn("vx", issues[0])


class L0TuningMetricTests(unittest.TestCase):
    def test_body_frame_rmse_and_torque_integral(self):
        condition = CONDITIONS[1]
        run = RunData(condition, Path(condition.name))
        run.columns = {
            "time_s": [5.0, 5.01, 5.02],
            "base_z_m": [1.08, 1.07, 1.08],
            "roll_rad": [0.0, 0.01, 0.0],
            "pitch_rad": [0.0, 0.0, -0.02],
            "yaw_rad": [math.pi / 2] * 3,
            "vx_world_m_s": [0.0, 0.0, 0.0],
            "vy_world_m_s": [0.2, 0.2, 0.2],
            "controller:command_vx_mps": [0.2, 0.2, 0.2],
            "controller:mpc_qp_status": [0.0, 1.0, 0.0],
            "controller:wbc_qp_status": [0.0, 0.0, 0.0],
            "controller:mpc_solve_s": [0.001, 0.002, 0.003],
            "tau_nm:j1": [1.0, 1.0, 1.0],
            "tau_nm:j2": [2.0, 2.0, 2.0],
        }
        result = compute_metrics(run)
        self.assertAlmostEqual(result["body_vx_internal_reference_rmse_m_s"], 0.0)
        self.assertAlmostEqual(result["joint_torque_squared_integral_Nm2_s"], 0.1)
        self.assertAlmostEqual(result["peak_joint_torque_Nm"], 2.0)
        self.assertEqual(result["mpc_qp_failure_samples"], 1)
        self.assertAlmostEqual(result["mpc_solve_p95_ms"], 2.9)


if __name__ == "__main__":
    unittest.main()
