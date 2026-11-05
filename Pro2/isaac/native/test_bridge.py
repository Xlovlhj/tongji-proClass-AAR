"""C ABI regression tests; runs with stock Python 3 and no Isaac installation.

python3 Pro2/isaac/native/test_bridge.py
These tests exercise the actual compiled controller, not a physics simulation.
"""
from __future__ import annotations

import ctypes as ct
import json
import math
from pathlib import Path
import unittest

ISAAC_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = ISAAC_DIR.parent / "OpenLoong-Dyn-Control"
DOUBLE_P = ct.POINTER(ct.c_double)


def array(values):
    return (ct.c_double * len(values))(*values)


class Native:
    def __init__(self, *, dt=0.001, mpc_dt=0.005, height=1.08):
        self.lib = ct.CDLL(str(ISAAC_DIR / "build/mpc_n10/libopenloong_controller.so"))
        lib = self.lib
        lib.ol_create.argtypes = [ct.c_char_p] + [ct.c_double] * 5
        lib.ol_create.restype = ct.c_void_p
        lib.ol_last_error.restype = ct.c_char_p
        lib.ol_destroy.argtypes = [ct.c_void_p]
        lib.ol_joint_count.argtypes = [ct.c_void_p]
        lib.ol_joint_name.argtypes = [ct.c_void_p, ct.c_int]
        lib.ol_joint_name.restype = ct.c_char_p
        lib.ol_initial_joints.argtypes = [ct.c_void_p, DOUBLE_P, ct.c_int]
        lib.ol_diagnostic_count.argtypes = [ct.c_void_p]
        lib.ol_diagnostic_name.argtypes = [ct.c_void_p, ct.c_int]
        lib.ol_diagnostic_name.restype = ct.c_char_p
        lib.ol_diagnostics.argtypes = [ct.c_void_p, DOUBLE_P, ct.c_int]
        lib.ol_control_signal_count.argtypes = [ct.c_void_p]
        lib.ol_control_signal_name.argtypes = [ct.c_void_p, ct.c_int]
        lib.ol_control_signal_name.restype = ct.c_char_p
        lib.ol_control_signals.argtypes = [ct.c_void_p, DOUBLE_P, ct.c_int]
        lib.ol_set_external_mpc_wrench.argtypes = [ct.c_void_p, ct.c_int,
                                                   DOUBLE_P, ct.c_int,
                                                   ct.c_double, ct.c_int,
                                                   ct.c_int]
        lib.ol_set_mpc_weights.argtypes = [ct.c_void_p, ct.c_double, DOUBLE_P, DOUBLE_P]
        lib.ol_step.argtypes = [ct.c_void_p, ct.c_double] + [DOUBLE_P] * 7 + [ct.c_int, DOUBLE_P]
        lib.ol_step_jump.argtypes = ([ct.c_void_p, ct.c_double] + [DOUBLE_P] * 6 +
                                     [ct.c_int] + [ct.c_double] * 4 + [DOUBLE_P])
        self.handle = lib.ol_create(str(REPO_ROOT).encode(), dt, mpc_dt, 0.4, 0.12, height)
        if not self.handle:
            raise RuntimeError(self.error)
        self.joints = array([0.0] * 31)
        if lib.ol_initial_joints(self.handle, self.joints, 31):
            raise RuntimeError(self.error)
        self.velocities = array([0.0] * 31)
        self.position = array([0.0, 0.0, height])
        self.quaternion = array([1.0, 0.0, 0.0, 0.0])
        self.linear = array([0.0, 0.0, 0.0])
        self.angular = array([0.0, 0.0, 0.0])
        self.command = array([0.0, 0.0, 0.0])
        self.torque = array([0.0] * 31)

    @property
    def error(self):
        return self.lib.ol_last_error().decode()

    def close(self):
        if self.handle:
            self.lib.ol_destroy(self.handle)
            self.handle = None

    def step(self, time, walking=False):
        return self.lib.ol_step(self.handle, time, self.joints, self.velocities,
                                self.position, self.quaternion, self.linear,
                                self.angular, self.command, int(walking), self.torque)

    def step_jump(self, time, phase, height, vertical_velocity=0.0,
                  progress=0.0, landing_foot_advance=0.0):
        return self.lib.ol_step_jump(
            self.handle, time, self.joints, self.velocities, self.position,
            self.quaternion, self.linear, self.angular, phase, height,
            vertical_velocity, progress, landing_foot_advance, self.torque)

    def diagnostics(self):
        count = self.lib.ol_diagnostic_count(self.handle)
        data = array([0.0] * count)
        if self.lib.ol_diagnostics(self.handle, data, count):
            raise RuntimeError(self.error)
        return {self.lib.ol_diagnostic_name(self.handle, i).decode(): data[i]
                for i in range(count)}

    def control_signals(self):
        count = self.lib.ol_control_signal_count(self.handle)
        data = array([0.0] * count)
        if self.lib.ol_control_signals(self.handle, data, count):
            raise RuntimeError(self.error)
        return {self.lib.ol_control_signal_name(self.handle, i).decode(): data[i]
                for i in range(count)}

    def set_external_wrench(self, wrench, *, solve_time=0.0, status=0, iterations=0):
        values = array(wrench)
        return self.lib.ol_set_external_mpc_wrench(
            self.handle, 1, values, len(values), solve_time, status, iterations)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = Native()
        self.addCleanup(self.controller.close)

    def test_joint_contract_and_initial_ik(self):
        c = self.controller
        self.assertEqual(c.lib.ol_joint_count(c.handle), 31)
        self.assertEqual(c.lib.ol_joint_name(c.handle, 0), b"J_arm_l_01")
        self.assertEqual(c.lib.ol_joint_name(c.handle, 30), b"J_ankle_r_roll")
        self.assertIsNone(c.lib.ol_joint_name(c.handle, 31))
        self.assertIn("out of range", c.error)
        self.assertTrue(all(map(math.isfinite, c.joints)))
        self.assertLess(c.diagnostics()["initial_ik_error"], 1e-4)

    def test_output_is_joint_torque_before_gear_division(self):
        c = self.controller
        cfg = json.loads((REPO_ROOT / "common/joint_ctrl_config.json").read_text())["J_waist_pitch"]
        c.joints[16] += 0.01
        self.assertEqual(c.step(0), 0, c.error)
        alpha = 0.001 / (0.001 + 1.0 / (2.0 * 3.1415 * cfg["PVT_LPF_Fc"]))
        expected = -0.01 * cfg["kp"] * alpha
        self.assertAlmostEqual(c.torque[16], expected, places=8)
        self.assertGreater(abs(c.torque[16]), abs(expected / cfg["gear"]) * 80)
        signals = c.control_signals()
        name = "J_waist_pitch"
        self.assertEqual(len(signals), 39 + 5 * 31)
        self.assertAlmostEqual(signals[f"prelimit_Nm:{name}"],
                               signals[f"wbc_feedforward_Nm:{name}"] +
                               signals[f"pd_filtered_Nm:{name}"], places=12)
        self.assertAlmostEqual(signals[f"command_Nm:{name}"], c.torque[16], places=12)
        self.assertLess(signals[f"pd_raw_Nm:{name}"], 0)

    def test_stand_then_walk_at_native_frequency(self):
        c = self.controller
        limits = json.loads((REPO_ROOT / "common/joint_ctrl_config.json").read_text())
        for index in range(3003):
            self.assertEqual(c.step(index * 0.001, walking=index >= 3001), 0, c.error)
        diag = c.diagnostics()
        self.assertEqual(diag["ready"], 1)
        self.assertEqual(diag["motion_state"], 1)
        self.assertEqual(diag["mpc_qp_status"], 0)
        self.assertEqual(diag["wbc_qp_status"], 0)
        self.assertIn(diag["next_leg_state"], (0, 1, 2))
        self.assertTrue(all(map(math.isfinite, diag.values())))
        for i, value in enumerate(c.torque):
            name = c.lib.ol_joint_name(c.handle, i).decode()
            self.assertTrue(math.isfinite(value))
            self.assertLessEqual(abs(value), limits[name]["maxTorque"] + 1e-8)

        # Match the external wrench contact mask to the support selected by the
        # gait scheduler.  The bridge deliberately rejects a one-step-old
        # wrench for the opposite support foot.
        left = [10.0, -5.0, 760.0, 1.0, 2.0, 3.0]
        right = [-10.0, 5.0, 760.0, -1.0, -2.0, -3.0]
        leg_state = int(diag["leg_state"])
        wrench = ((left + [0.0] * 6) if leg_state == 0 else
                  ([0.0] * 6 + right) if leg_state == 1 else
                  ([*left[:2], 380.0, *left[3:],
                    *right[:2], 380.0, *right[3:]]))
        self.assertEqual(c.set_external_wrench(
            wrench, solve_time=.0012, status=0, iterations=7), 0, c.error)
        self.assertEqual(c.step(3.003, walking=True), 0, c.error)
        diag = c.diagnostics()
        self.assertEqual(diag["wrench_source_external"], 1)
        self.assertEqual(diag["external_mpc_status"], 0)
        self.assertAlmostEqual(diag["external_mpc_solve_s"], .0012)
        self.assertEqual(diag["external_mpc_iterations"], 7)
        signals = c.control_signals()
        for component, expected in zip((
            "left_fx_N", "left_fy_N", "left_fz_N", "left_mx_Nm",
            "left_my_Nm", "left_mz_Nm", "right_fx_N", "right_fy_N",
            "right_fz_N", "right_mx_Nm", "right_my_Nm", "right_mz_Nm"), wrench):
            self.assertAlmostEqual(signals[f"mpc_wrench:{component}"], expected)

    def test_nonfinite_input_rejected_and_output_cleared(self):
        c = self.controller
        c.torque[0] = 123
        c.joints[0] = math.nan
        self.assertLess(c.step(0), 0)
        self.assertIn("NaN/Inf", c.error)
        self.assertTrue(all(value == 0 for value in c.torque))

    def test_jump_modes_switch_mpc_and_keep_torque_finite(self):
        c = self.controller
        limits = json.loads((REPO_ROOT / "common/joint_ctrl_config.json").read_text())
        for index in range(3001):
            self.assertEqual(c.step_jump(index * .001, 0, 1.08), 0, c.error)

        sequence = (
            (1, .90, 0.0, 1.0, 0.0, 0, (0.0, 0.0), None),
            (2, .91, 1.0, .5, 0.0, 1, (0.0, 0.0), None),
            (3, .96, .5, .3, 0.0, 0, (0.0, 0.0), None),
            (4, 1.08, -.5, 1.0, .05, 0, (0.0, 0.0), None),
            (5, .90, 0.0, 0.0, 0.0, 1, (0.0, 0.0), None),
            (6, 1.00, 0.0, .5, 0.0, 1, (0.0, 0.0), None),
            # After the source-MPC window, -K*v is split equally across the
            # feet.  Phase 7 keeps the same law and the 150 N/foot vector-norm
            # limit remains available for larger recovery velocities.
            (6, 1.04, 0.0, 1.0, 0.0, 0, (-.12, .08), (27.0, -18.0)),
            (7, 1.08, 0.0, 1.0, 0.0, 0, (.3, .4), (-67.5, -90.0)),
        )
        for offset, (phase, height, velocity, progress, advance,
                     mpc_enabled, base_velocity, recovery_force) in enumerate(sequence, 1):
            c.linear = array([*base_velocity, 0.0])
            self.assertEqual(
                c.step_jump((3000 + offset) * .001, phase, height, velocity,
                            progress, advance),
                0, c.error)
            diag = c.diagnostics()
            self.assertEqual(diag["jump_active"], 1)
            self.assertEqual(diag["jump_phase"], phase)
            self.assertEqual(diag["mpc_enabled"], mpc_enabled)
            self.assertAlmostEqual(diag["jump_height_reference_m"], height)
            self.assertAlmostEqual(diag["jump_vertical_velocity_reference_m_s"], velocity)
            self.assertAlmostEqual(diag["jump_landing_foot_advance_m"], advance)
            self.assertTrue(all(map(math.isfinite, c.torque)))
            if phase == 5:
                signals = c.control_signals()
                for component in ("left_fx_N", "left_fy_N",
                                  "right_fx_N", "right_fy_N"):
                    self.assertAlmostEqual(
                        signals[f"wbc_wrench:{component}"],
                        signals[f"mpc_wrench:{component}"], places=8)
            if recovery_force is not None:
                signals = c.control_signals()
                expected_fx, expected_fy = recovery_force
                for side in ("left", "right"):
                    self.assertAlmostEqual(
                        signals[f"mpc_wrench:{side}_fx_N"], expected_fx,
                        places=8)
                    self.assertAlmostEqual(
                        signals[f"mpc_wrench:{side}_fy_N"], expected_fy,
                        places=8)
                    self.assertLessEqual(
                        math.hypot(signals[f"mpc_wrench:{side}_fx_N"],
                                   signals[f"mpc_wrench:{side}_fy_N"]),
                        150.0 + 1e-8)
                    self.assertAlmostEqual(
                        signals[f"wbc_wrench:{side}_fx_N"], expected_fx,
                        places=8)
                    self.assertAlmostEqual(
                        signals[f"wbc_wrench:{side}_fy_N"], expected_fy,
                        places=8)
                total_wbc_fx = sum(signals[f"wbc_wrench:{side}_fx_N"]
                                   for side in ("left", "right"))
                total_wbc_fy = sum(signals[f"wbc_wrench:{side}_fy_N"]
                                   for side in ("left", "right"))
                self.assertLess(total_wbc_fx * base_velocity[0] +
                                total_wbc_fy * base_velocity[1], 0.0)
            for index, torque in enumerate(c.torque):
                name = c.lib.ol_joint_name(c.handle, index).decode()
                self.assertLessEqual(abs(torque), limits[name]["maxTorque"] + 1e-8)

        c.torque[0] = 123.0
        self.assertLess(c.step_jump((3001 + len(sequence)) * .001,
                                    8, 1.08), 0)
        self.assertIn("phase", c.error.lower())
        self.assertTrue(all(value == 0 for value in c.torque))

    def test_reset_time_is_rejected(self):
        c = self.controller
        self.assertEqual(c.step(0), 0, c.error)
        self.assertLess(c.step(0), 0)
        self.assertIn("physics dt", c.error)
        self.assertTrue(all(value == 0 for value in c.torque))

    def test_jump_recovery_base_pitch_task_limits_adverse_acceleration(self):
        c = self.controller
        for index in range(3001):
            self.assertEqual(c.step_jump(index * .001, 0, 1.08), 0, c.error)

        # Enter landing first because that transition enables the dedicated
        # fixed-contact recovery task and captures the landing reference.
        self.assertEqual(c.step_jump(3.001, 5, .90), 0, c.error)
        pitch = -0.1
        c.quaternion = array([
            math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
        self.assertEqual(
            c.step_jump(3.002, 6, 1.00, progress=1.0), 0, c.error)

        signals = c.control_signals()
        # CoMXY_HipRPY asks for a positive pitch acceleration to recover from
        # the negative pitch.  Pz, arm and head tasks come after it and must
        # remain in the accumulated contact/posture null space instead of
        # reversing that command.  The subsequent contact-wrench QP may alter
        # the base acceleration to satisfy floating-base dynamics, but it must
        # remain feasible and finite.
        self.assertGreater(signals["wbc_kin_ddq_base:wy"], 0.0)
        for name, value in signals.items():
            if name.startswith(("wbc_kin_ddq_base:", "wbc_opt_ddq_base:",
                                "wbc_wrench:")):
                self.assertTrue(math.isfinite(value), name)
        self.assertEqual(c.diagnostics()["wbc_qp_status"], 0)

    def test_wxyz_quaternion_and_nonunit_normalization(self):
        c = self.controller
        roll, pitch = 0.05, -0.04
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        c.quaternion = array([2 * cr * cp, 2 * sr * cp, 2 * cr * sp, -2 * sr * sp])
        self.assertEqual(c.step(0), 0, c.error)
        diag = c.diagnostics()
        self.assertAlmostEqual(diag["roll_rad"], roll, places=10)
        self.assertAlmostEqual(diag["pitch_rad"], pitch, places=10)

    def test_bad_weights_and_bad_clock_ratio(self):
        c = self.controller
        result = c.lib.ol_set_mpc_weights(c.handle, -1, array([1] * 12), array([1] * 13))
        self.assertLess(result, 0)
        self.assertIn("positive", c.error)
        with self.assertRaisesRegex(RuntimeError, "integer multiple"):
            Native(dt=0.001, mpc_dt=0.0055)

    def test_yaw_is_continuous_across_pi_boundary(self):
        c = self.controller
        for index, raw_yaw in enumerate((math.pi - 0.001, -math.pi + 0.001)):
            c.quaternion = array([math.cos(raw_yaw / 2), 0, 0, math.sin(raw_yaw / 2)])
            self.assertEqual(c.step(index * 0.001), 0, c.error)
        self.assertAlmostEqual(c.diagnostics()["base_yaw_rad"], math.pi + 0.001, places=10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
