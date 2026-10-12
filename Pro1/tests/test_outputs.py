"""Plot/data and paired-disturbance comparison checks without third-party code."""

import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from raa.outputs import compare_runs, load_run, save_results, _thin_time
from raa.paths import build_reference


def row(t, x=0.0, y=0.0, z=.3, *, dx=0.0, dy=0.0, dz=0.0, down=True, stroke=0):
    return {"time": t, "target_x": x, "target_y": y, "target_z": z,
            "actual_x": x + dx, "actual_y": y + dy, "actual_z": z + dz,
            "pen_down": down, "stroke_index": stroke, "adaptive_tau1": 1.234}


class OutputTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="raa-outputs-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def text_and_lines(self, path):
        svg = ET.parse(path).getroot()
        return ([element.text for element in svg.iter() if element.tag.endswith("text")],
                [element for element in svg.iter() if element.tag.endswith("polyline")])

    def assert_concise(self, path, allowed):
        text, lines = self.text_and_lines(path)
        for label in text:
            if label in allowed:
                continue
            try:
                float(label)
            except ValueError:
                self.fail(f"Unexpected non-axis/legend/title text in {path.name}: {label!r}")
        for phrase in ("RMSE", "samples", "Completed run", "Partial run", "Max tip", "recorded", "same simulation instant"):
            self.assertFalse(any(phrase in str(label) for label in text))
        return lines

    def test_plots_are_concise_and_original_metrics_extra_csv_fields_survive(self):
        rows = [row(0, .4, 0), row(.1, .5, .1, dx=.001)]
        metadata = {"completed": True, "controller": "rbf", "metrics": {"tip_rmse_m": .0007, "custom_metric": 42}}
        returned = save_results(self.root, rows, metadata)
        self.assertEqual(returned, metadata["metrics"])
        self.assertEqual(json.loads((self.root / "metrics.json").read_text()), metadata)
        loaded, _ = load_run(self.root)
        self.assertEqual(loaded[-1]["actual_x"], .501)
        self.assertEqual(float(loaded[-1]["adaptive_tau1"]), 1.234)
        for filename, allowed in {
            "drawing_result.svg": {"Trajectory tracking", "Target", "Actual", "X (mm)", "Y (mm)"},
            "target_trajectory.svg": {"Target trajectory", "Target", "X (mm)", "Y (mm)"},
            "actual_trajectory.svg": {"Actual trajectory", "Actual", "X (mm)", "Y (mm)"},
            "error_history.svg": {"Position tracking error", "X", "Y", "Z", "Time (s)", "Position error (mm)"},
        }.items():
            self.assert_concise(self.root / filename, allowed)

    def test_pen_lifts_and_stroke_changes_never_connect(self):
        rows = [row(0, 0), row(.1, .01), row(.2, .5, down=False),
                row(.3, .8, stroke=1), row(.4, .81, stroke=1),
                row(.5, .9, stroke=2), row(.6, .91, stroke=2)]
        save_results(self.root, rows, {"metrics": {}})
        _, lines = self.text_and_lines(self.root / "drawing_result.svg")
        self.assertEqual(len(lines), 6, "Three separate segments for each of target and actual")

    def test_force_fields_roundtrip_preserves_false_and_zero_pulse_index(self):
        rows = [row(0), row(.1)]
        for item, active, pulse, force in zip(rows, [False, True], [-1, 0], [0.0, 3.0]):
            item.update(disturbance_active=active, disturbance_pulse_index=pulse,
                        external_force_x_n=force, external_force_y_n=0.0,
                        external_force_z_n=-force, custom_extension="False")
        save_results(self.root, rows, {"metrics": {}})
        loaded, _ = load_run(self.root)
        self.assertIs(loaded[0]["disturbance_active"], False)
        self.assertIs(loaded[1]["disturbance_active"], True)
        self.assertEqual([item["disturbance_pulse_index"] for item in loaded], [-1, 0])
        self.assertEqual([item["external_force_x_n"] for item in loaded], [0.0, 3.0])
        self.assertEqual(loaded[1]["external_force_z_n"], -3.0)
        self.assertEqual(loaded[0]["custom_extension"], "False")

    def test_xy_uses_equal_axis_scaling(self):
        save_results(self.root, [row(0, 0, 0), row(1, .1, .1)], {"metrics": {}})
        _, lines = self.text_and_lines(self.root / "target_trajectory.svg")
        points = [tuple(map(float, point.split(','))) for point in lines[0].attrib["points"].split()]
        self.assertAlmostEqual(abs(points[1][0] - points[0][0]), abs(points[1][1] - points[0][1]), places=3)

    def test_target_only_reads_complete_reference_even_for_short_recording(self):
        reference = build_reference(width=.1, dt=.02)
        reference.save_csv(self.root / "reference_cartesian.csv")
        save_results(self.root, [row(0, down=False)], {"completed": False, "metrics": {}})
        _, lines = self.text_and_lines(self.root / "target_trajectory.svg")
        self.assertEqual(len(lines), 8)
        _, actual = self.text_and_lines(self.root / "actual_trajectory.svg")
        self.assertEqual(len(actual), 0)

    def test_empty_data_is_valid_without_explanatory_plot_text(self):
        save_results(self.root, [], {"completed": False, "metrics": {}})
        self.assert_concise(self.root / "drawing_result.svg", {"Trajectory tracking", "Target", "Actual", "X (mm)", "Y (mm)"})

    def pair(self, *, deviation=None, disturbance=None):
        times = [i / 10 for i in range(21)]
        baseline = [row(t, .4 + .01 * t, 0, dx=.001) for t in times]
        values = deviation if deviation is not None else [.003] * len(times)
        pushed = [row(t, .4 + .01 * t, 0, dx=.001 + values[i]) for i, t in enumerate(times)]
        normal, disturbed = self.root / "normal", self.root / "disturbed"
        save_results(normal, baseline, {"completed": True, "reference_duration_s": 2.0, "metrics": {}})
        metadata = {"completed": True, "reference_duration_s": 2.0, "metrics": {}}
        if disturbance:
            metadata.update(disturbance)
        save_results(disturbed, pushed, metadata)
        return normal, disturbed

    def test_comparison_csv_json_and_three_concise_figures(self):
        normal, disturbed = self.pair()
        destination = self.root / "comparison"
        metadata = compare_runs(normal, disturbed, destination)
        self.assertAlmostEqual(metadata["metrics"]["normal_tracking"]["tip_rmse_m"], .001)
        self.assertAlmostEqual(metadata["metrics"]["disturbed_tracking"]["tip_rmse_m"], .004)
        self.assertAlmostEqual(metadata["metrics"]["disturbance_deviation"]["tip_rmse_m"], .003)
        with (destination / "comparison.csv").open(newline="") as stream:
            paired = list(csv.DictReader(stream))
        self.assertEqual(len(paired), 21)
        self.assertAlmostEqual(float(paired[10]["deviation_x"]), .003)
        self.assertAlmostEqual(float(paired[10]["disturbed_error_x"]), .004)
        self.assertEqual(json.loads((destination / "comparison_metrics.json").read_text()), metadata)
        for filename, allowed in {
            "trajectory_comparison.svg": {"Trajectory comparison", "Target", "Normal", "Disturbed", "X (mm)", "Y (mm)"},
            "tracking_error_comparison.svg": {"Tracking error comparison", "Normal", "Disturbed", "Time (s)", "3D tracking error (mm)"},
            "disturbance_deviation.svg": {"Disturbance deviation", "X", "Y", "Z", "Time (s)", "Position difference (mm)"},
        }.items():
            self.assert_concise(destination / filename, allowed)

    def test_comparison_rejects_partial_misaligned_or_changed_target(self):
        for problem in ("partial", "time", "target", "pen", "coverage"):
            with self.subTest(problem=problem):
                normal, disturbed = self.pair()
                rows, metadata = load_run(disturbed)
                if problem == "partial": metadata["completed"] = False
                if problem == "time": rows[5]["time"] += .01
                if problem == "target": rows[5]["target_y"] += .001
                if problem == "pen": rows[5]["pen_down"] = False
                if problem == "coverage": metadata["reference_duration_s"] = 3
                save_results(disturbed, rows, metadata)
                destination = self.root / ("bad-" + problem)
                with self.assertRaises(ValueError): compare_runs(normal, disturbed, destination)
                self.assertFalse(destination.exists())

    def test_recovery_requires_continuous_hold_and_resets_after_spike(self):
        # Force ends at .5s. .7s is briefly below threshold, but .9s breaks it.
        deviation = [.01] * 21
        deviation[7] = deviation[8] = .001
        for i in range(10, 21): deviation[i] = .001
        normal, disturbed = self.pair(deviation=deviation, disturbance={
            "disturbance_windows": [{"start_s": .2, "end_s": .5, "force_world_n": [2, 0, 0]}],
            "recovery_threshold_m": .002, "recovery_hold_s": .5,
        })
        metadata = compare_runs(normal, disturbed, self.root / "recovery")
        window = metadata["recovery"]["windows"][0]
        self.assertAlmostEqual(window["recovery_time_s"], .5)
        self.assertAlmostEqual(window["recovery_started_at_s"], 1)
        self.assertAlmostEqual(window["recovery_confirmed_at_s"], 1.5)
        self.assertEqual(window["force_world_n"], [2, 0, 0])

    def test_not_recovered_is_null_instead_of_false_success(self):
        normal, disturbed = self.pair(disturbance={"disturbance_windows": [{"start_s": .2, "end_s": .5}]})
        metadata = compare_runs(normal, disturbed, self.root / "no-recovery")
        self.assertIsNone(metadata["recovery"]["windows"][0]["recovery_time_s"])

    def test_recovery_hold_cannot_cross_next_disturbance(self):
        deviations = [.01] * 21
        for i in range(6, 21): deviations[i] = .001
        normal, disturbed = self.pair(deviation=deviations, disturbance={"disturbance_windows": [
            {"start_s": .2, "end_s": .5}, {"start_s": .9, "end_s": 1.0}]})
        metadata = compare_runs(normal, disturbed, self.root / "multiple-forces")
        self.assertIsNone(metadata["recovery"]["windows"][0]["recovery_time_s"])
        self.assertAlmostEqual(metadata["recovery"]["windows"][1]["recovery_time_s"], 0)

    def test_downsampling_preserves_narrow_positive_and_negative_peaks(self):
        points = [(i * .001, 0) for i in range(10000)]
        points[4321] = (4.321, 12)
        points[4322] = (4.322, -7)
        thin = _thin_time(points)
        self.assertIn(points[0], thin)
        self.assertIn(points[-1], thin)
        self.assertIn(points[4321], thin)
        self.assertIn(points[4322], thin)


if __name__ == "__main__":
    unittest.main()
