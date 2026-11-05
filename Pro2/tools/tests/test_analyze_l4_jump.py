import csv
import json
from pathlib import Path
import tempfile
import unittest

from Pro2.tools.analyze_l4_jump import compute_acceptance, write_analysis


class JumpAnalysisTests(unittest.TestCase):
    fields = (
        "time_s", "base_x_m", "base_y_m", "base_z_m", "roll_rad",
        "pitch_rad", "vx_world_m_s", "vy_world_m_s", "vz_world_m_s",
        "force_x_n", "force_y_n", "force_z_n", "controller:mpc_enabled",
        "controller:mpc_qp_status", "controller:wbc_qp_status",
        "jump:decision_left_contact_force_z_n",
        "jump:decision_right_contact_force_z_n", "jump:height_reference_m",
        "jump:phase",
    )

    @staticmethod
    def _write_run(root: Path, *, second_flight=False, csv_rise=True):
        phases = [
            "prepare", "crouch", "thrust", "flight_up", "flight_down",
            "landing", "recovery", "complete", "complete", "complete",
            "complete", "complete",
        ]
        times = [index * 0.1 for index in range(len(phases))]
        rows = []
        for index, (time_s, phase) in enumerate(zip(times, phases)):
            in_flight = phase in {"flight_up", "flight_down"}
            contact = 0.0 if in_flight else 380.0
            if second_flight and index in (8, 9):
                contact = 0.0
            if phase in {"crouch", "thrust"}:
                height = 0.92
            elif phase == "flight_up":
                height = 0.95
            elif phase == "flight_down":
                height = 1.02 if csv_rise else 0.95
            elif phase == "landing":
                height = 0.94
            else:
                height = 1.08
            rows.append([
                time_s, 0.01 * index, 0.0, height,
                0.01, -0.01, 0.02 if phase == "complete" else 0.0,
                0.0, 0.01 if phase == "complete" else 0.0,
                0.0, 0.0, 0.0,
                1.0 if phase in {"thrust", "landing", "recovery"} else 0.0,
                0.0, 0.0, contact, contact, 1.08, phase,
            ])
        with (root / "states.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(JumpAnalysisTests.fields)
            writer.writerows(rows)
        phase_sequence = [
            {"phase": phase, "time_s": times[index]}
            for index, phase in enumerate(phases[:8])
        ]
        jump = {
            "phase_sequence": phase_sequence,
            "takeoff_time_s": 0.3,
            "touchdown_time_s": 0.5,
            "actual_rise_m": 0.08,
            "minimum_required_rise_m": 0.06,
            "flight_mpc_disabled": True,
            "external_force_assist": False,
            "completed_phase_sequence": True,
            "final_phase": "complete",
            "pass": True,
        }
        run = {
            "status": "completed",
            "config": {"jump": {"stand_height_m": 1.08,
                                  "start_time_s": 0.2,
                                  "target_height_m": 0.12,
                                  "landing_foot_advance_m": 0.20,
                                  "takeoff_force_threshold_n": 80.0,
                                  "landing_force_threshold_n": 300.0,
                                  "contact_confirm_duration_s": 0.001}},
        }
        (root / "jump_metrics.json").write_text(
            json.dumps(jump), encoding="utf-8")
        (root / "run.json").write_text(json.dumps(run), encoding="utf-8")

    def test_passes_complete_stable_jump_and_writes_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "vertical_jump_run"
            root.mkdir()
            self._write_run(root)
            result, _ = compute_acceptance(root)
            self.assertTrue(result["pass"])
            self.assertAlmostEqual(
                result["metrics"]["csv_actual_rise_m"], 0.07)
            written = write_analysis(root)
            self.assertTrue(written["pass"])
            for name in (
                "acceptance.json", "README.md", "height.png", "velocity.png",
                "contact_forces.png", "attitude.png", "planar_trajectory.png",
            ):
                self.assertTrue((root / "analysis" / name).is_file(), name)

    def test_detects_second_flight_and_never_overwrites_analysis(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "vertical_jump_run"
            root.mkdir()
            self._write_run(root, second_flight=True)
            result = write_analysis(root)
            checks = {row["name"]: row for row in result["checks"]}
            self.assertFalse(result["pass"])
            self.assertFalse(checks["no_second_flight_after_touchdown"]["passed"])
            with self.assertRaises(FileExistsError):
                write_analysis(root)

    def test_forged_json_rise_cannot_replace_raw_height_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "vertical_jump_run"
            root.mkdir()
            self._write_run(root, csv_rise=False)
            result, _ = compute_acceptance(root)
            checks = {row["name"]: row for row in result["checks"]}
            self.assertTrue(checks["builtin_jump_pass"]["passed"])
            self.assertEqual(
                result["metrics"]["jump_metrics_actual_rise_m"], 0.08)
            self.assertEqual(result["metrics"]["csv_actual_rise_m"], 0.0)
            self.assertFalse(checks["minimum_jump_rise"]["passed"])
            self.assertFalse(result["pass"])

    def test_rejects_summary_event_times_inconsistent_with_raw_forces(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "vertical_jump_run"
            root.mkdir()
            self._write_run(root)
            path = root / "jump_metrics.json"
            jump = json.loads(path.read_text(encoding="utf-8"))
            jump["takeoff_time_s"] = 0.0
            jump["touchdown_time_s"] = 0.1
            path.write_text(json.dumps(jump), encoding="utf-8")
            result, _ = compute_acceptance(root)
            checks = {row["name"]: row for row in result["checks"]}
            self.assertTrue(checks["takeoff_and_touchdown"]["passed"])
            self.assertFalse(checks["event_timing_consistency"]["passed"])
            self.assertFalse(result["pass"])

    def test_rejects_output_outside_run_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            root = base / "vertical_jump_run"
            root.mkdir()
            self._write_run(root)
            with self.assertRaises(ValueError):
                write_analysis(root, base / "foreign-analysis")


if __name__ == "__main__":
    unittest.main()
