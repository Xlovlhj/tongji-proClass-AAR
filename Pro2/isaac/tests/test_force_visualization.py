"""Check the physical meaning of the arrow without needing Isaac or a GPU."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.force_visualization import AppliedForceArrow, force_arrow_geometry


class ForceArrowTests(unittest.TestCase):
    def test_lateral_push_tip_is_at_application_point_and_points_world_positive_y(self):
        point = (2.0, 0.3, 1.08)
        plan = force_arrow_geometry(point, (0.0, 50.0, 0.0))
        self.assertTrue(plan["visible"])
        self.assertEqual(plan["application_point_world_m"], point)
        self.assertEqual(plan["direction_world"], (0.0, 1.0, 0.0))
        self.assertEqual(plan["tail_world_m"], (2.0, -0.3, 1.08))
        self.assertAlmostEqual(plan["length_m"], 0.6)
        self.assertAlmostEqual(plan["shaft_length_m"] + plan["head_length_m"], 0.6)

    def test_length_is_proportional_and_sign_changes_direction_not_length(self):
        low = force_arrow_geometry((0, 0, 0), (0, 50, 0))
        high = force_arrow_geometry((0, 0, 0), (0, 100, 0))
        reversed_force = force_arrow_geometry((0, 0, 0), (0, -100, 0))
        self.assertAlmostEqual(high["length_m"], 2 * low["length_m"])
        self.assertEqual(reversed_force["length_m"], high["length_m"])
        self.assertEqual(reversed_force["direction_world"], (0, -1, 0))

    def test_arbitrary_direction_keeps_tip_and_tail_on_force_line(self):
        point = (2, -4, 1)
        force = (3, -4, 12)
        plan = force_arrow_geometry(point, force)
        self.assertAlmostEqual(plan["force_magnitude_n"], 13)
        for i in range(3):
            self.assertAlmostEqual(point[i] - plan["tail_world_m"][i], 0.012 * force[i])

    def test_force_off_hides_previously_visible_arrow(self):
        arrow = AppliedForceArrow.__new__(AppliedForceArrow)
        arrow._UsdGeom = SimpleNamespace(Tokens=SimpleNamespace(invisible="invisible"))
        arrow._visibility = Mock()
        arrow.visible = True
        plan = arrow.update((0, 0, 1), (0, 0, 0))
        self.assertFalse(plan["visible"])
        self.assertFalse(arrow.visible)
        arrow._visibility.Set.assert_called_once_with("invisible")
        arrow.hide()
        arrow._visibility.Set.assert_called_once_with("invisible")


if __name__ == "__main__":
    unittest.main()
