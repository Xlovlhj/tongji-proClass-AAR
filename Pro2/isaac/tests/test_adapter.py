"""Simulator-independent checks of the conventions at the C++/PhysX boundary.

Run from the repository root with:
    python3 -m unittest discover -s Pro2/isaac/tests -p 'test_*.py'
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.adapter import base_origin_velocity, joint_indices, rpy_wxyz


class JointMappingTests(unittest.TestCase):
    def test_reversed_simulator_order_preserves_named_states_and_efforts(self):
        controller_names = ["left_hip", "left_knee", "right_hip", "right_knee"]
        simulator_names = list(reversed(controller_names))
        indices = joint_indices(controller_names, simulator_names)
        simulator_positions = [4.0, 3.0, 2.0, 1.0]
        self.assertEqual([simulator_positions[i] for i in indices], [1.0, 2.0, 3.0, 4.0])

        # The same mapping must address each named actuator on the write path.
        simulator_efforts = [0.0] * 4
        for index, effort in zip(indices, [10.0, 20.0, 30.0, 40.0]):
            simulator_efforts[index] = effort
        self.assertEqual(dict(zip(simulator_names, simulator_efforts)), {
            "left_hip": 10.0, "left_knee": 20.0,
            "right_hip": 30.0, "right_knee": 40.0,
        })

    def test_mismatched_names_rejected_even_when_counts_match(self):
        with self.assertRaises(ValueError):
            joint_indices(["hip", "knee"], ["hip", "ankle"])

    def test_extra_or_missing_joint_rejected(self):
        for simulator_names in (["hip"], ["hip", "knee", "ankle"]):
            with self.subTest(simulator_names=simulator_names), self.assertRaises(ValueError):
                joint_indices(["hip", "knee"], simulator_names)

    def test_duplicate_names_rejected_on_both_sides(self):
        for controller_names, simulator_names in (
            (["hip", "knee"], ["hip", "hip"]),
            (["hip", "hip"], ["hip", "knee"]),
        ):
            with self.subTest(controller=controller_names, simulator=simulator_names):
                with self.assertRaises(ValueError):
                    joint_indices(controller_names, simulator_names)


class BaseVelocityTests(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=12)

    def test_yaw_rotates_local_com_offset_before_velocity_correction(self):
        # A +90 degree yaw places the COM 2 m along world +Y. Rotation at
        # +3 rad/s around world Z adds -6 m/s along X to its linear velocity.
        yaw90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
        result = base_origin_velocity(
            [-5.0, 2.0, 3.0], [0.0, 0.0, 3.0], yaw90, [2.0, 0.0, 0.0]
        )
        self.assertVectorAlmostEqual(result, [1.0, 2.0, 3.0])

    def test_general_angular_velocity_and_rotated_offset(self):
        # This 120 degree rotation about (1,1,1) maps X->Y, Y->Z, Z->X.
        # The local offset (1,2,3) is therefore (3,1,2) in the world.
        # Contributions from omega=(2,-1,3) sum to (-5,5,5) m/s at COM.
        result = base_origin_velocity(
            [-1.0, 3.0, 5.5], [2.0, -1.0, 3.0],
            [0.5, 0.5, 0.5, 0.5], [1.0, 2.0, 3.0],
        )
        self.assertVectorAlmostEqual(result, [4.0, -2.0, 0.5])

    def test_com_at_origin_does_not_change_linear_velocity(self):
        result = base_origin_velocity(
            [1.2, -3.4, 5.6], [7.0, -8.0, 9.0],
            [0.5, 0.5, 0.5, 0.5], [0.0, 0.0, 0.0],
        )
        self.assertVectorAlmostEqual(result, [1.2, -3.4, 5.6])


class OrientationTests(unittest.TestCase):
    # Intrinsic ZYX orientation with roll=60, pitch=30, yaw=45 degrees;
    # the reference quaternion is specified independently of the adapter.
    REFERENCE_WXYZ = (
        0.8223631719059994, 0.3604234056503559,
        0.3919038373291199, 0.20056212114657512,
    )

    def test_nontrivial_orientation_in_wxyz_order(self):
        result = rpy_wxyz(self.REFERENCE_WXYZ)
        for actual, expected_degrees in zip(result, [60.0, 30.0, 45.0]):
            self.assertAlmostEqual(actual, math.radians(expected_degrees), places=12)

    def test_quaternion_scale_and_sign_do_not_change_orientation(self):
        for scale in (7.0, -3.0):
            with self.subTest(scale=scale):
                result = rpy_wxyz([scale * v for v in self.REFERENCE_WXYZ])
                for actual, expected_degrees in zip(result, [60.0, 30.0, 45.0]):
                    self.assertAlmostEqual(actual, math.radians(expected_degrees), places=12)

    def test_invalid_quaternions_rejected(self):
        for quaternion in ([0.0] * 4, [math.nan, 0.0, 0.0, 0.0], [math.inf, 0.0, 0.0, 0.0]):
            with self.subTest(quaternion=quaternion), self.assertRaises(ValueError):
                rpy_wxyz(quaternion)


if __name__ == "__main__":
    unittest.main()
