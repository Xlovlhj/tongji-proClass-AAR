import math
from array import array
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.path_tracking import (
    PathTracker,
    Polyline,
    build_reference_path,
    normalize_path_tracking,
    path_visualization_plan,
    wrap_angle,
)


def closed_loop_config(path=None):
    return {
        "mode": "closed_loop",
        "path": path or {"type": "line", "start": [0, 0], "end": [2, 0], "num_points": 5},
        "lookahead_m": 0.3,
        "nominal_speed_m_s": 0.15,
        "limits": {"vx_m_s": 0.25, "vy_m_s": 0.12, "wz_rad_s": 0.35,
                   "linear_accel_m_s2": 0.5, "angular_accel_rad_s2": 0.8},
    }


class PathGeometryTests(unittest.TestCase):
    def test_ground_visualization_plan_is_visible_and_non_colliding(self):
        points = [(0.0, 0.0), (1.0, 0.0), (1.4, 0.5), (1.4, 1.2), (2.1, 1.2)]
        plan = path_visualization_plan(points)
        self.assertEqual(plan["curve_prim_path"], "/World/PathVisualization/ReferencePath")
        self.assertEqual(plan["reference_point_count"], 5)
        self.assertEqual([point[:2] for point in plan["points_m"]], [list(point) for point in points])
        self.assertTrue(all(point[2] > 0.01 for point in plan["points_m"]))
        self.assertGreater(plan["line_width_m"], 0.0)
        self.assertEqual(plan["geometry_type"], "ribbon_mesh")
        self.assertEqual(plan["face_vertex_counts"], [4, 4, 4, 4])
        self.assertEqual(len(plan["ribbon_points_m"]), 16)
        self.assertFalse(plan["collision_enabled"])
        self.assertEqual([marker["role"] for marker in plan["markers"]],
                         ["start", "waypoint", "waypoint", "waypoint", "goal"])

    def test_indexable_runtime_position_is_accepted(self):
        tracker = PathTracker({
            "path": {"type": "line", "start": [0, 0], "end": [1, 0]},
        })
        command, diagnostics = tracker.update(array("d", [0.0, 0.0]), 0.0, 0.001)
        self.assertEqual(len(command), 3)
        self.assertTrue(math.isfinite(diagnostics["path_distance_m"]))

    def test_line_and_circle_generation(self):
        line = build_reference_path({"type": "line", "start": [0, 0], "end": [1, 2], "num_points": 3})
        self.assertEqual(line, [(0.0, 0.0), (0.5, 1.0), (1.0, 2.0)])
        circle = build_reference_path({"type": "circle", "center": [0, 1], "radius_m": 1,
                                       "start_angle_rad": -math.pi / 2, "end_angle_rad": 3 * math.pi / 2,
                                       "num_points": 21})
        self.assertAlmostEqual(circle[0][0], 0.0, places=12)
        self.assertAlmostEqual(circle[0][1], 0.0, places=12)
        self.assertAlmostEqual(circle[-1][0], 0.0, places=12)
        self.assertAlmostEqual(circle[-1][1], 0.0, places=12)

    def test_projection_is_monotonic(self):
        path = Polyline([(0, 0), (1, 0), (2, 0)])
        first = path.project((1.2, 0.1), 0.0)
        second = path.project((0.8, 0.1), first[0])
        self.assertAlmostEqual(first[0], 1.2)
        self.assertAlmostEqual(second[0], 1.2)
        self.assertAlmostEqual(second[2], 0.1)

    def test_closed_circle_near_start_does_not_jump_to_end(self):
        points = build_reference_path({
            "type": "circle", "center": [0, 1], "radius_m": 1,
            "start_angle_rad": -math.pi / 2,
            "end_angle_rad": 3 * math.pi / 2, "num_points": 361,
        })
        path = Polyline(points)
        progress, _, _, _ = path.project((-0.02, -0.013), 0.0)
        self.assertLess(progress, 0.1)

    def test_wrap_angle_interval(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(-3 * math.pi), -math.pi)


class TrackerTests(unittest.TestCase):
    def test_closed_loop_corrects_left_offset_and_obeys_rate_limit(self):
        tracker = PathTracker(closed_loop_config())
        command, diagnostics = tracker.update((0.0, 0.2), 0.0, 0.1)
        self.assertAlmostEqual(command[0], 0.05)
        self.assertAlmostEqual(command[1], -0.05)
        self.assertLess(command[2], 0.0)
        self.assertAlmostEqual(diagnostics["cross_track_error_m"], 0.2)

    def test_goal_detection_stops(self):
        tracker = PathTracker(closed_loop_config())
        command, diagnostics = tracker.update((2.0, 0.0), 0.0, 0.1)
        self.assertTrue(tracker.reached_goal)
        self.assertEqual(command, [0.0, 0.0, 0.0])
        self.assertEqual(diagnostics["reached_goal"], 1.0)
        command, diagnostics = tracker.update((2.5, 0.5), 0.0, 0.1)
        self.assertTrue(tracker.reached_goal)
        self.assertEqual(command, [0.0, 0.0, 0.0])
        self.assertEqual(diagnostics["reached_goal"], 1.0)

    def test_open_loop_command_is_limited_but_not_feedback_modified(self):
        config = closed_loop_config()
        config.update(mode="open_loop", open_loop_command=[0.1, 0.0, 0.1])
        tracker = PathTracker(config)
        command, _ = tracker.update((0.0, 0.5), 1.0, 1.0)
        self.assertEqual(command, [0.1, 0.0, 0.1])

    def test_unknown_nested_key_rejected(self):
        config = closed_loop_config()
        config["limits"]["typo"] = 1
        with self.assertRaises(ValueError):
            normalize_path_tracking(config)


if __name__ == "__main__":
    unittest.main()

