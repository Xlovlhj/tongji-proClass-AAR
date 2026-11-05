import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.analyze_l4_mpc_comparison import (
    KEY_EXPERIMENT_CONFIG_FIELDS,
    analyze as analyze_mpc_comparison,
    external_update_indices,
    manifest_comparison_checks,
    metrics,
)
from tools.analyze_l4_path_terrain import (
    STEP_FRONT_PROXY_NOTE, TERRAIN_SUMMARY_FIELDS, analyze_run,
    blind_step_front_metrics, terrain_bounds, terrain_traversal_metrics,
    update_terrain_summary,
)


class NmpcDeduplicationTests(unittest.TestCase):
    def columns(self):
        count = 5
        return {
            "time_s": [0.0, 0.01, 0.02, 0.03, 0.04],
            "base_z_m": [1.0] * count,
            "roll_rad": [0.0] * count,
            "pitch_rad": [0.0] * count,
            "yaw_rad": [0.0] * count,
            "vx_world_m_s": [0.1] * count,
            "vy_world_m_s": [0.0] * count,
            "cmd_vx_body_m_s": [0.1] * count,
            "cmd_vy_body_m_s": [0.0] * count,
            "controller:external_mpc_update_count": [0.0, 1.0, 1.0, 2.0, 2.0],
            "controller:external_mpc_solve_s": [0.0, 0.01, 0.01, 0.03, 0.03],
            "controller:external_mpc_status": [0.0, 0.0, 0.0, 1.0, 1.0],
            "controller:wrench_source_external": [0.0, 1.0, 1.0, 0.0, 0.0],
        }

    def test_timing_and_status_use_one_row_per_update(self):
        result = metrics(self.columns(), "nmpc")
        self.assertEqual(result["external_updates_observed"], 2)
        self.assertEqual(result["solver_evaluations_logged"], 2)
        self.assertEqual(result["solver_held_samples_removed"], 2)
        self.assertEqual(result["solver_failure_samples"], 1)
        self.assertAlmostEqual(result["solve_time_mean_s"], 0.02)
        self.assertAlmostEqual(result["solve_time_max_s"], 0.03)

    def test_counter_regression_is_rejected(self):
        columns = self.columns()
        columns["controller:external_mpc_update_count"] = [0.0, 2.0, 2.0, 1.0, 1.0]
        with self.assertRaises(ValueError):
            external_update_indices(columns)

    def test_convex_timing_excludes_zero_stand_phase_when_flagged(self):
        columns = self.columns()
        columns.pop("controller:external_mpc_update_count")
        columns.pop("controller:external_mpc_solve_s")
        columns.pop("controller:external_mpc_status")
        columns.pop("controller:wrench_source_external")
        columns.update({
            "walking_command": [0.0, 0.0, 1.0, 1.0, 1.0],
            "controller:mpc_solve_s": [0.0, 0.0, 0.001, 0.003, 0.005],
            "controller:mpc_qp_status": [0.0] * 5,
        })
        result = metrics(columns, "convex_mpc")
        self.assertEqual(result["solver_timing_raw_csv_samples"], 5)
        self.assertEqual(result["solver_timing_active_csv_samples"], 3)
        self.assertEqual(
            result["solver_timing_scope"],
            "walking_phase_100hz_samples_of_200hz_internal_updates")
        self.assertAlmostEqual(result["solve_time_mean_s"], 0.003)
        self.assertAlmostEqual(result["solve_time_all_logged_samples_mean_s"],
                               0.0018)


class MpcComparisonProvenanceTests(unittest.TestCase):
    @staticmethod
    def shared_config():
        config = {key: None for key in KEY_EXPERIMENT_CONFIG_FIELDS}
        config.update({
            "level": "L4",
            "mode": "walk",
            "log_control_decomposition": True,
            "mpc_horizon_steps": 10,
            "duration": 0.04,
            "dt": 0.001,
            "mpc_dt": 0.005,
            "swing_time": 0.4,
            "step_height": 0.12,
            "stand_height": 1.08,
            "walk_start": 0.0,
            "vx": 0.1,
            "vy": 0.0,
            "wz": 0.0,
            "render_hz": 30.0,
            "log_hz": 100.0,
            "push_duration": 0.1,
            "push_force": [0.0, 50.0, 0.0],
            "u_weight": 1e-6,
            "state_weights": [1.0] * 12,
            "input_weights": [1.0] * 13,
            "terrain": {"type": "flat"},
            "nmpc": {"update_period_s": 0.02},
        })
        return config

    @staticmethod
    def write_columns(path, columns):
        fields = list(columns)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            writer.writerows(zip(*(columns[field] for field in fields)))

    @staticmethod
    def base_columns():
        count = 5
        return {
            "time_s": [0.0, 0.01, 0.02, 0.03, 0.04],
            "base_z_m": [1.08] * count,
            "roll_rad": [0.0] * count,
            "pitch_rad": [0.0] * count,
            "yaw_rad": [0.0] * count,
            "vx_world_m_s": [0.1] * count,
            "vy_world_m_s": [0.0] * count,
            "cmd_vx_body_m_s": [0.1] * count,
            "cmd_vy_body_m_s": [0.0] * count,
            "cmd_wz_rad_s": [0.0] * count,
        }

    def manifest(self, backend):
        config = self.shared_config()
        config.update({
            "config": f"Pro2/experiments/L4/mpc_comparison/{backend}.json",
            "mpc_backend": backend,
        })
        return {
            "status": "completed",
            "controller_library": "Pro2/isaac/build/libopenloong_controller.so",
            "controller_library_sha256": "same-library-hash",
            "upstream_commit": "test-commit",
            "isaac_version": "test-version",
            "robot_usd": "Pro2/cache/robot.usda",
            "feedback": "simulation_ground_truth",
            "physics_backend": "PhysX CPU",
            "config": config,
        }

    def test_metrics_include_manifest_provenance_and_csv_reconciliation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            convex = root / "convex"
            nmpc = root / "nmpc"
            output = root / "analysis"
            convex.mkdir()
            nmpc.mkdir()
            convex_columns = self.base_columns()
            convex_columns.update({
                "controller:mpc_solve_s": [0.001] * 5,
                "controller:mpc_qp_status": [0.0] * 5,
            })
            nmpc_columns = self.base_columns()
            nmpc_columns.update({
                "controller:external_mpc_update_count":
                    [0.0, 1.0, 1.0, 2.0, 2.0],
                "controller:external_mpc_solve_s":
                    [0.0, 0.01, 0.01, 0.03, 0.03],
                "controller:external_mpc_status":
                    [0.0, 0.0, 0.0, 1.0, 1.0],
                "controller:wrench_source_external":
                    [0.0, 1.0, 1.0, 0.0, 0.0],
            })
            self.write_columns(convex / "states.csv", convex_columns)
            self.write_columns(nmpc / "states.csv", nmpc_columns)
            convex_manifest = self.manifest("cpp_convex")
            nmpc_manifest = self.manifest("nmpc_rti")
            nmpc_manifest["external_nmpc"] = {
                "updates": 2,
                "successful_updates": 1,
                "failed_updates": 1,
                "deadline_misses": 1,
                "success_fraction": 0.5,
                "deadline_miss_fraction": 0.5,
            }
            (convex / "run.json").write_text(
                json.dumps(convex_manifest), encoding="utf-8")
            (nmpc / "run.json").write_text(
                json.dumps(nmpc_manifest), encoding="utf-8")

            analyze_mpc_comparison(
                convex / "states.csv", nmpc / "states.csv", output)
            result = json.loads((output / "metrics.json").read_text())
            checks = result["comparison_checks"]
            self.assertTrue(checks["controller_library_sha256_match"])
            self.assertTrue(checks["both_runs_completed"])
            self.assertTrue(checks["key_experiment_config_match"])
            self.assertEqual(checks["key_experiment_config_mismatches"], [])
            self.assertEqual(
                result["provenance"]["nmpc"]["status"], "completed")
            self.assertEqual(
                result["provenance"]["nmpc"]["external_nmpc"]["updates"], 2)
            nmpc_result = result["runs"][1]
            self.assertEqual(nmpc_result["external_manifest_updates"], 2)
            self.assertEqual(nmpc_result["external_csv_update_counter_last"], 2)
            self.assertEqual(nmpc_result["external_csv_failed_updates"], 1)
            self.assertEqual(nmpc_result["external_csv_deadline_misses"], 1)
            self.assertTrue(
                nmpc_result["external_manifest_statistics_match_csv"])

    def test_manifest_checks_expose_real_mismatches(self):
        convex = self.manifest("cpp_convex")
        nmpc = self.manifest("nmpc_rti")
        nmpc["status"] = "failed"
        nmpc["controller_library_sha256"] = "different-library-hash"
        nmpc["config"]["vx"] = 0.2
        checks = manifest_comparison_checks(convex, nmpc)
        self.assertFalse(checks["controller_library_sha256_match"])
        self.assertFalse(checks["both_runs_completed"])
        self.assertFalse(checks["key_experiment_config_match"])
        self.assertEqual(checks["key_experiment_config_mismatches"], ["vx"])


class PathAndTerrainAnalysisTests(unittest.TestCase):
    fields = (
        "time_s", "base_x_m", "base_y_m", "base_z_m", "roll_rad",
        "pitch_rad", "yaw_rad", "vx_world_m_s", "vy_world_m_s",
        "cmd_vx_body_m_s", "cmd_vy_body_m_s", "walking_command",
    )
    path_fields = (
        "path:cross_track_error_m", "path:path_distance_m",
        "path:heading_error_rad", "path:progress_m",
        "path:progress_fraction", "path:goal_distance_m",
        "path:reached_goal",
    )

    @staticmethod
    def write_csv(path, fields, rows):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            writer.writerows(rows)

    def base_rows(self):
        return [
            [0.0, 0.0, 0.0, 1.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.1, 1.0, 0.1, 1.07, 0.01, -0.01, 0.0, 0.1, 0.0, 0.1, 0.0, 1.0],
            [0.2, 2.0, 0.0, 1.06, 0.02, -0.02, 0.0, 0.1, 0.0, 0.1, 0.0, 1.0],
        ]

    def assert_pngs(self, directory, names):
        for name in names:
            with self.subTest(name=name):
                path = directory / name
                self.assertTrue(path.is_file())
                with Image.open(path) as image:
                    image.verify()

    def test_path_metrics_reconcile_and_pngs_are_valid(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / "run.json").write_text(json.dumps({
                "status": "completed",
                "config": {"path_tracking": {"mode": "closed_loop"}},
                "samples": 3,
            }), encoding="utf-8")
            self.write_csv(run / "reference_path.csv",
                           ("point_index", "x_m", "y_m"),
                           ((0, 0, 0), (1, 1, 0), (2, 2, 0)))
            rows = [
                self.base_rows()[0] + [0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0],
                self.base_rows()[1] + [0.1, 0.1, 0.2, 1.0, 0.5, 1.0, 0.0],
                self.base_rows()[2] + [0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 1.0],
            ]
            self.write_csv(run / "states.csv", self.fields + self.path_fields, rows)
            reported = {
                "sample_count": 2,
                "cross_track_rmse_m": math.sqrt(0.01 / 2),
                "maximum_path_distance_m": 0.1,
                "heading_error_rmse_rad": math.sqrt(0.04 / 2),
                "final_progress_m": 2.0,
                "path_length_m": 2.0,
                "completion_fraction": 1.0,
                "reached_goal": True,
                "final_goal_distance_m": 0.0,
            }
            (run / "path_metrics.json").write_text(json.dumps(reported),
                                                   encoding="utf-8")
            result = analyze_run(run)
            self.assertEqual(result["analysis_kind"], "path_tracking")
            self.assertTrue(result["path_metrics_reconciliation"]
                            ["all_compared_values_match"])
            self.assertEqual(result["walking_samples"], 2)
            self.assert_pngs(run / "analysis", result["figures"])

    def test_terrain_auto_detection_and_pngs_are_valid(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / "run.json").write_text(json.dumps({
                "status": "completed",
                "config": {"terrain": {"type": "slope", "angle_deg": 5}},
                "terrain": {"config": {"type": "slope", "angle_deg": 5}},
            }), encoding="utf-8")
            self.write_csv(run / "states.csv", self.fields, self.base_rows())
            result = analyze_run(run)
            self.assertEqual(result["analysis_kind"], "terrain")
            self.assertEqual(result["terrain_type"], "slope")
            self.assertAlmostEqual(
                result["terrain_x_end_m"], 0.8 + 2.5 * math.cos(math.radians(5)))
            self.assertEqual(result["completion_reason"], "进入未穿越")
            self.assert_pngs(run / "analysis", result["figures"])


class TerrainTraversalTests(unittest.TestCase):
    def test_config_bounds_for_all_supported_task_terrains(self):
        slope = terrain_bounds({
            "type": "slope", "start_x_m": 1.0, "length_m": 2.0,
            "angle_deg": 60.0, "width_m": 4.0,
        })
        self.assertAlmostEqual(slope["terrain_x_end_m"], 2.0)
        self.assertEqual((slope["terrain_y_min_m"], slope["terrain_y_max_m"]),
                         (-2.0, 2.0))

        course = terrain_bounds({
            "type": "slope_course", "start_x_m": 0.5, "width_m": 2.0,
            "segments": [
                {"angle_deg": 0.0, "length_m": 1.0},
                {"angle_deg": 60.0, "length_m": 2.0},
            ],
        })
        self.assertAlmostEqual(course["terrain_x_end_m"], 2.5)

        rough = terrain_bounds({
            "type": "rough_blocks", "start_x_m": 0.8, "rows": 14,
            "columns": 10, "tile_size_m": 0.2,
        })
        self.assertAlmostEqual(rough["terrain_x_end_m"], 3.6)
        self.assertEqual((rough["terrain_y_min_m"], rough["terrain_y_max_m"]),
                         (-1.0, 1.0))

        random = terrain_bounds({
            "type": "random_obstacles", "region_x_m": [1.0, 3.0],
            "region_y_m": [-0.5, 0.7],
        })
        self.assertEqual((random["terrain_x_start_m"], random["terrain_x_end_m"]),
                         (1.0, 3.0))
        self.assertEqual((random["terrain_y_min_m"], random["terrain_y_max_m"]),
                         (-0.5, 0.7))

        blind = terrain_bounds({
            "type": "blind_step", "start_x_m": 0.9, "length_m": 4.0,
            "width_m": 2.0,
        })
        self.assertAlmostEqual(blind["terrain_x_end_m"], 4.9)

    def test_serialized_plan_has_priority_over_config(self):
        result = terrain_bounds(
            {"type": "rough_blocks", "start_x_m": 0.8, "rows": 2,
             "columns": 2, "tile_size_m": 0.2},
            {"plan": [
                {"name": "Ground", "center_m": [0, 0, 0],
                 "size_m": [200, 200, 0.1]},
                {"name": "Rough_00_00", "center_m": [10, 2, 0],
                 "size_m": [2, 4, 0.1]},
            ]},
        )
        self.assertEqual(result["terrain_bounds_source"], "terrain.plan")
        self.assertEqual((result["terrain_x_start_m"], result["terrain_x_end_m"]),
                         (9.0, 11.0))
        self.assertEqual((result["terrain_y_min_m"], result["terrain_y_max_m"]),
                         (0.0, 4.0))

    def test_completed_traversal_requires_y_range(self):
        columns = {
            "time_s": [0.0, 1.0, 2.0, 3.0, 4.0],
            "base_x_m": [0.0, 1.0, 1.5, 5.0, 2.1],
            "base_y_m": [0.0, 0.0, 0.0, 2.0, 0.0],
        }
        bounds = {
            "terrain_x_start_m": 1.0, "terrain_x_end_m": 2.0,
            "terrain_y_min_m": -1.0, "terrain_y_max_m": 1.0,
        }
        result = terrain_traversal_metrics(columns, "completed", bounds)
        self.assertEqual(result["maximum_base_x_m"], 2.1)
        self.assertEqual(result["entry_time_s"], 1.0)
        self.assertEqual(result["exit_time_s"], 4.0)
        self.assertEqual(result["samples_inside"], 2)
        self.assertEqual(result["progress_fraction"], 1.0)
        self.assertTrue(result["entered_terrain"])
        self.assertTrue(result["traversed_terrain"])
        self.assertEqual(result["completion_reason"], "穿越")

    def test_incomplete_and_failed_reasons_are_distinct(self):
        bounds = {
            "terrain_x_start_m": 1.0, "terrain_x_end_m": 2.0,
            "terrain_y_min_m": -1.0, "terrain_y_max_m": 1.0,
        }
        partial = {"time_s": [0.0, 1.0], "base_x_m": [0.0, 1.5],
                   "base_y_m": [0.0, 0.0]}
        result = terrain_traversal_metrics(partial, "completed", bounds)
        self.assertTrue(result["entered_terrain"])
        self.assertFalse(result["traversed_terrain"])
        self.assertEqual(result["completion_reason"], "进入未穿越")

        before = {"time_s": [0.0, 1.0], "base_x_m": [0.0, 0.5],
                  "base_y_m": [0.0, 0.0]}
        not_entered = terrain_traversal_metrics(before, "completed", bounds)
        self.assertFalse(not_entered["entered_terrain"])
        self.assertEqual(not_entered["completion_reason"], "未进入")

        failed = terrain_traversal_metrics(partial, "failed", bounds)
        self.assertFalse(failed["entered_terrain"])
        self.assertFalse(failed["traversed_terrain"])
        self.assertIsNone(failed["entry_time_s"])
        self.assertEqual(failed["completion_reason"], "运行失败")

    def test_blind_step_front_proxy_is_base_only_and_y_bounded(self):
        bounds = {
            "terrain_x_start_m": 0.9, "terrain_x_end_m": 4.9,
            "terrain_y_min_m": -1.0, "terrain_y_max_m": 1.0,
        }
        columns = {
            "time_s": [0.0, 1.0, 2.0],
            "base_x_m": [0.0, 1.21, 2.0],
            "base_y_m": [0.0, 0.0, 1.5],
        }
        traversal = terrain_traversal_metrics(columns, "completed", bounds)
        proxy = blind_step_front_metrics(
            "blind_step", "completed", bounds, traversal)
        self.assertAlmostEqual(proxy["step_front_clearance_m"], 0.31)
        self.assertTrue(proxy["step_front_crossed_proxy"])
        self.assertFalse(traversal["traversed_terrain"])
        self.assertIn("base-only proxy", proxy["step_front_proxy_note"])
        self.assertIn("contact prim", proxy["step_front_proxy_note"])
        self.assertIn("不能等同于双脚", proxy["step_front_proxy_note"])

        outside_only = terrain_traversal_metrics({
            "time_s": [0.0, 1.0], "base_x_m": [0.0, 2.0],
            "base_y_m": [0.0, 1.5],
        }, "completed", bounds)
        outside_proxy = blind_step_front_metrics(
            "blind_step", "completed", bounds, outside_only)
        self.assertFalse(outside_proxy["step_front_crossed_proxy"])

        failed_proxy = blind_step_front_metrics(
            "blind_step", "failed", bounds, traversal)
        self.assertFalse(failed_proxy["step_front_crossed_proxy"])
        non_step = blind_step_front_metrics(
            "slope", "completed", bounds, traversal)
        self.assertIsNone(non_step["step_front_clearance_m"])
        self.assertIsNone(non_step["step_front_crossed_proxy"])
        self.assertIsNone(non_step["step_front_proxy_note"])

    def test_terrain_summary_serializes_step_proxy_and_blanks_other_terrains(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            summary_directory = root / "summary"

            def make_run(name, config_path):
                run = root / name
                run.mkdir()
                (run / "run.json").write_text(json.dumps({
                    "config": {"config": config_path},
                }), encoding="utf-8")
                return run

            def make_metrics(terrain_type):
                result = {field: None for field in TERRAIN_SUMMARY_FIELDS[5:]}
                result.update({
                    "run_status": "completed",
                    "terrain_type": terrain_type,
                    "terrain_bounds_source": "terrain.config",
                    "entered_terrain": True,
                    "traversed_terrain": False,
                    "completion_reason": "进入未穿越",
                })
                return result

            slope_run = make_run(
                "slope", "Pro2/experiments/L4/terrain/slope_05deg.json")
            step_run = make_run(
                "step", "Pro2/experiments/L4/terrain/blind_step_up.json")
            slope_metrics = make_metrics("slope")
            step_metrics = make_metrics("blind_step")
            step_metrics.update({
                "step_front_clearance_m": 0.31,
                "step_front_crossed_proxy": True,
                "step_front_proxy_note": STEP_FRONT_PROXY_NOTE,
            })
            with patch("tools.analyze_l4_path_terrain._project_relative",
                       side_effect=["Pro2/output/L4/slope", "Pro2/output/L4/step"]):
                update_terrain_summary(slope_run, slope_metrics, summary_directory)
                json_path, csv_path = update_terrain_summary(
                    step_run, step_metrics, summary_directory)

            summary = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["schema_version"], 2)
            by_name = {row["experiment"]: row
                       for row in summary["experiments"]}
            self.assertIsNone(by_name["slope"]["step_front_clearance_m"])
            self.assertIsNone(by_name["slope"]["step_front_crossed_proxy"])
            self.assertIsNone(by_name["slope"]["step_front_proxy_note"])
            self.assertTrue(by_name["step"]["step_front_crossed_proxy"])
            with csv_path.open(newline="", encoding="utf-8-sig") as stream:
                csv_rows = {row["experiment"]: row for row in csv.DictReader(stream)}
            self.assertEqual(csv_rows["slope"]["step_front_clearance_m"], "")
            self.assertEqual(csv_rows["slope"]["step_front_crossed_proxy"], "")
            self.assertEqual(csv_rows["step"]["step_front_crossed_proxy"], "true")


if __name__ == "__main__":
    unittest.main()
