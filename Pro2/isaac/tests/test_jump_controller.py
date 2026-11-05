import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.jump_controller import (
    JumpRuntimeError,
    JumpStateMachine,
    UnsupportedJumpRuntime,
    jump_preflight,
    normalize_jump,
)


CONFIG = {
    "enabled": True,
    "start_time_s": 3.0,
    "stand_height_m": 1.08,
    "crouch_height_m": 0.90,
    "target_height_m": 0.12,
    "crouch_duration_s": 2.0,
    "crouch_speed_m_s": 0.10,
    "thrust_duration_s": 0.2,
    "landing_foot_advance_m": 0.05,
    "landing_foot_swing_duration_s": 0.10,
    "takeoff_timeout_s": 0.35,
    "landing_timeout_s": 1.0,
    "landing_settle_duration_s": 0.2,
    "recovery_duration_s": 0.5,
    "takeoff_force_threshold_n": 80.0,
    "landing_force_threshold_n": 1000.0,
    "contact_confirm_duration_s": 0.01,
}


class JumpTests(unittest.TestCase):
    def test_preflight_reports_missing_and_supported_capabilities(self):
        missing = {
            "dynamic_base_height_reference": False,
            "flight_phase_control": False,
            "bilateral_contact_feedback": False,
        }
        report = jump_preflight(CONFIG, missing, raise_on_missing=False)
        self.assertFalse(report["ready"])
        self.assertEqual(len(report["missing"]), 3)
        with self.assertRaises(UnsupportedJumpRuntime):
            jump_preflight(CONFIG, missing)

        supported = {key: True for key in missing}
        report = jump_preflight(CONFIG, supported)
        self.assertTrue(report["ready"])
        self.assertEqual(report["missing"], [])

    def test_dynamic_crouch_and_thrust_references(self):
        machine = JumpStateMachine(CONFIG)
        self.assertEqual(machine.update(0.0, 1.08, 0.0, 400.0, 400.0).phase, "prepare")
        start = machine.update(1.0, 1.08, 0.0, 400.0, 400.0)
        middle = machine.update(1.9, 1.00, 0.0, 400.0, 400.0)
        crouched = machine.update(2.9, 0.90, 0.0, 400.0, 400.0)
        thrust = machine.update(3.0, 0.90, 0.0, 400.0, 400.0)
        half = machine.update(3.10, 0.94, 0.0, 400.0, 400.0)

        self.assertEqual(start.phase, "crouch")
        self.assertGreater(start.base_height_m, middle.base_height_m)
        self.assertGreater(middle.base_height_m, crouched.base_height_m)
        self.assertEqual(thrust.phase, "thrust")
        launch = math.sqrt(2.0 * 9.81 * CONFIG["target_height_m"])
        self.assertAlmostEqual(half.vertical_velocity_m_s, 0.5 * launch)
        expected_height = CONFIG["crouch_height_m"] + 0.5 * launch * \
            CONFIG["thrust_duration_s"] * 0.5**2
        self.assertAlmostEqual(half.base_height_m, expected_height)
        self.assertTrue(half.mpc_enabled)

    def test_full_sequence_requires_takeoff_and_bilateral_debounced_landing(self):
        machine = JumpStateMachine(CONFIG)
        machine.update(0.0, 1.08, 0.0, 400.0, 400.0)
        machine.update(1.0, 1.08, 0.0, 400.0, 400.0)
        machine.update(3.0, 0.90, 0.0, 400.0, 400.0)
        flight = machine.update(3.21, 1.00, 0.5, 400.0, 400.0)
        self.assertEqual(flight.phase, "flight_up")
        self.assertFalse(machine.left_ground)
        self.assertFalse(flight.mpc_enabled)

        flight = machine.update(3.22, 1.01, 1.0, 0.0, 0.0)
        self.assertTrue(machine.left_ground)
        self.assertEqual(flight.phase, "flight_up")
        apex = machine.update(3.40, 1.05, -0.1, 0.0, 0.0)
        self.assertEqual(apex.phase, "flight_down")
        self.assertEqual(apex.landing_foot_advance_m, 0.05)
        descended = machine.update(3.50, 0.98, -0.5, 1200.0, 900.0)
        self.assertEqual(descended.phase, "flight_down")
        self.assertAlmostEqual(descended.phase_progress, 1.0)
        self.assertEqual(machine.update(3.51, 0.96, -0.5, 1200.0, 1200.0).phase, "flight_down")
        self.assertEqual(machine.update(3.52, 0.95, -0.5, 1200.0, 1200.0).phase, "landing")
        self.assertIsNotNone(machine.touchdown_time_s)
        self.assertEqual(
            machine.update(3.73, 0.95, 0.0, 400.0, 400.0).phase,
            "recovery")
        # Recovery rises from 0.95 m to 1.08 m at the source demo's 0.1 m/s,
        # so the configured 0.5 s minimum is not yet enough at 4.23 s.
        recovery = machine.update(4.23, 1.00, 0.0, 400.0, 400.0)
        self.assertEqual(recovery.phase, "recovery")
        self.assertTrue(recovery.mpc_enabled)
        complete = machine.update(5.04, 1.08, 0.0, 400.0, 400.0)
        self.assertEqual(complete.phase, "complete")
        self.assertFalse(complete.mpc_enabled)
        self.assertTrue(machine.complete)

    def test_takeoff_and_landing_timeouts_fail_explicitly(self):
        no_takeoff = JumpStateMachine(CONFIG)
        for args in ((0.0, 1.08, 0.0, 400.0, 400.0),
                     (1.0, 1.08, 0.0, 400.0, 400.0),
                     (3.0, 0.90, 0.0, 400.0, 400.0),
                     (3.21, 1.00, 0.5, 400.0, 400.0)):
            no_takeoff.update(*args)
        with self.assertRaisesRegex(JumpRuntimeError, "未检测到双脚离地"):
            no_takeoff.update(3.57, 1.02, 0.5, 400.0, 400.0)

        no_landing = JumpStateMachine(CONFIG)
        for args in ((0.0, 1.08, 0.0, 400.0, 400.0),
                     (1.0, 1.08, 0.0, 400.0, 400.0),
                     (3.0, 0.90, 0.0, 400.0, 400.0),
                     (3.21, 1.00, 0.5, 0.0, 0.0),
                     (3.22, 1.01, 1.0, 0.0, 0.0),
                     (3.40, 1.05, -0.1, 0.0, 0.0)):
            no_landing.update(*args)
        with self.assertRaisesRegex(JumpRuntimeError, "未在时限内"):
            no_landing.update(4.22, 0.80, -1.0, 0.0, 0.0)

    def test_invalid_config_and_clock_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "小于"):
            normalize_jump(dict(CONFIG, crouch_height_m=1.1))
        with self.assertRaisesRegex(ValueError, "未知字段"):
            normalize_jump(dict(CONFIG, typo=1))
        with self.assertRaisesRegex(ValueError, "有限数值"):
            normalize_jump(dict(CONFIG, target_height_m=math.nan))
        with self.assertRaisesRegex(ValueError, "必须小于"):
            normalize_jump(dict(CONFIG, takeoff_force_threshold_n=1200.0))
        with self.assertRaisesRegex(ValueError, "0 至 0.20"):
            normalize_jump(dict(CONFIG, landing_foot_advance_m=0.21))
        with self.assertRaisesRegex(ValueError, "太短"):
            JumpStateMachine(dict(CONFIG, crouch_duration_s=1.0))
        machine = JumpStateMachine(CONFIG)
        machine.update(1.0, 1.08, 0.0, 400.0, 400.0)
        with self.assertRaisesRegex(ValueError, "不能倒退"):
            machine.update(0.9, 1.08, 0.0, 400.0, 400.0)


if __name__ == "__main__":
    unittest.main()
