"""Standard-library tests for L4 robot specification discovery."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import tempfile
import unittest

from Pro2.isaac.robot_spec import (load_robot_spec, resolve_robot_asset,
                                   validate_robot_spec)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SPEC_PATH = PROJECT_ROOT / "Pro2/robots/unitree_h1_2_handless.json"


class RobotSpecTests(unittest.TestCase):
    def test_checked_in_h1_spec_is_self_consistent(self):
        spec = load_robot_spec(SPEC_PATH)
        self.assertEqual(spec["robot_id"], "unitree_h1_2_handless")
        self.assertEqual(spec["joints"]["count"], 27)
        self.assertEqual(len(spec["joints"]["names"]), 27)
        self.assertEqual(set(spec["joints"]["isaac_dof_order"]),
                         set(spec["joints"]["names"]))
        self.assertEqual(spec["expected_usd"]["rigid_body_count"], 28)
        self.assertEqual(len(spec["stand_control"]["position_targets_rad"]), 27)
        self.assertEqual(spec["stand_control"]["source_transition_interval_s"], [0.0, 3.0])
        self.assertEqual(len(spec["stand_control"]["kp_nm_per_rad"]), 27)
        self.assertEqual(len(spec["stand_control"]["kd_nm_s_per_rad"]), 27)

    def test_explicit_asset_override_is_used_without_mutating_spec(self):
        spec = load_robot_spec(SPEC_PATH)
        original = copy.deepcopy(spec)
        with tempfile.TemporaryDirectory() as directory:
            asset = Path(directory) / "robot.usd"
            asset.write_bytes(b"test")
            resolved = resolve_robot_asset(spec, PROJECT_ROOT, asset)
        self.assertEqual(resolved.source, "command_line_override")
        self.assertEqual(spec, original)

    def test_environment_asset_has_priority_over_candidates(self):
        spec = load_robot_spec(SPEC_PATH)
        key = spec["asset"]["environment_variable"]
        previous = os.environ.get(key)
        try:
            with tempfile.TemporaryDirectory() as directory:
                asset = Path(directory) / "from_env.usd"
                asset.write_bytes(b"test")
                os.environ[key] = str(asset)
                resolved = resolve_robot_asset(spec, PROJECT_ROOT)
                self.assertEqual(resolved.path, asset.resolve())
                self.assertEqual(resolved.source, f"environment:{key}")
        finally:
            if previous is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = previous

    def test_duplicate_joint_names_are_rejected(self):
        spec = load_robot_spec(SPEC_PATH)
        spec["joints"]["names"][-1] = spec["joints"]["names"][0]
        with self.assertRaisesRegex(ValueError, "duplicates"):
            validate_robot_spec(spec)

    def test_missing_required_semantic_role_is_rejected(self):
        spec = load_robot_spec(SPEC_PATH)
        del spec["joints"]["semantic_roles"]["left_knee"]
        with self.assertRaisesRegex(ValueError, "left_knee"):
            validate_robot_spec(spec)

    def test_isaac_order_must_cover_each_joint_once(self):
        spec = load_robot_spec(SPEC_PATH)
        order = spec["joints"]["isaac_dof_order"]
        order[-1] = order[0]
        with self.assertRaisesRegex(ValueError, "isaac_dof_order"):
            validate_robot_spec(spec)


if __name__ == "__main__":
    unittest.main()
