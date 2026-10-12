"""Batch supervision tests with real CSV/MP4 structures, without Isaac/GPU."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zipfile


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_experiments as experiments
from raa.figures import png_size, svg_size, svg_to_png


def _box(name, payload):
    return struct.pack(">I4s", len(payload) + 8, name.encode()) + payload


def _synthetic_mp4(path, frames, width=960, height=720):
    """A minimal sample-table fixture; it is never presented as a decoded movie."""
    scale, delta = 30000, 1000
    duration = frames * delta
    duration_header = bytes(12) + struct.pack(">II", scale, duration)
    tkhd = bytes(12) + struct.pack(">I", 1) + struct.pack(">II", width << 16, height << 16)
    sample_header = bytearray(78)
    struct.pack_into(">HH", sample_header, 24, width, height)
    avc1 = _box("avc1", sample_header + _box("avcC", bytes((1, 66, 0, 30, 0xff))))
    stsd = _box("stsd", bytes(4) + struct.pack(">I", 1) + avc1)
    stsz = _box("stsz", bytes(4) + struct.pack(">II", 1, frames))
    stts = _box("stts", bytes(4) + struct.pack(">III", 1, frames, delta))
    stbl = _box("stbl", stsd + stsz + stts)
    mdia = _box("mdia", _box("mdhd", duration_header) + _box("hdlr", bytes(8) + b"vide") +
                _box("minf", stbl))
    moov = _box("moov", _box("mvhd", duration_header) + _box("trak", _box("tkhd", tkhd) + mdia))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_box("ftyp", b"isom\x00\x00\x00\x00isom") + moov + _box("mdat", bytes(frames)))


def _json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def _fixture(item):
    output, video = Path(item["output"]), Path(item["video"])
    output.mkdir(parents=True, exist_ok=True)
    dt = item["resolved_config"]["dt"]
    rows = []
    for index in range(3):
        error = (index + 1) / 1000
        rows.append({"time": index * dt, "target_x": 0.4, "target_y": 0, "target_z": 0.3,
                     "actual_x": 0.4 + error, "actual_y": 0, "actual_z": 0.3,
                     "pen_down": True, "stroke_index": 0, "disturbance_active": False,
                     "external_force_x_n": 0, "external_force_y_n": 0, "external_force_z_n": 0})
    with (output / "trajectory.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metrics = {"sample_count": 3, "drawing_sample_count": 3, "torque_saturation_fraction": 0}
    rms = math.sqrt(sum((row["actual_x"] - row["target_x"]) ** 2 for row in rows) / 3)
    maximum = max(row["actual_x"] - row["target_x"] for row in rows)
    for prefix in ("tip", "xy", "drawing_tip", "drawing_xy"):
        metrics[f"{prefix}_rmse_m"] = rms
        metrics[f"{prefix}_max_error_m"] = maximum
    _synthetic_mp4(video, 62)
    video_metadata = {"path": str(video), "completed": True, "frame_count": 62, "motion_frame_count": 2,
                      "hold_frame_count": 60, "fps": 30.0, "duration_s": 62 / 30, "resolution": [960, 720],
                      "dropped_frames": 0, "no_dropped_frames": True, "codec": "H.264",
                      "last_sim_time_s": 2 * dt, "file_size_bytes": video.stat().st_size}
    _json(video.with_suffix(".recording.json"), video_metadata)
    metadata = {"completed": True, "timeline_play_enabled": True, "controller": item["controller"],
                "external_disturbance_enabled": item["condition"] == "disturbed", "metrics": metrics,
                "physics_dt_s": dt, "reference_duration_s": 2 * dt,
                "physics_elapsed_s": 3 * dt * (1 + 4.7497e-8), "video": video_metadata}
    _json(output / "metrics.json", metadata)
    _json(output / "resolved_config.json", item["resolved_config"])
    _json(output / "controller_state.json", {})
    (output / "scene.usda").write_text("#usda 1.0\n")
    (output / "reference_cartesian.csv").write_text("reference-fixture\n")
    with zipfile.ZipFile(output / "reference_joints.npz", "w") as archive:
        for key in ("time", "q", "dq", "tip"):
            archive.writestr(f"{key}.npy", b"fixture")
    return metadata


class ExperimentTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="raa-experiment-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.args = experiments.parser().parse_args(["--output-root", str(self.root / "output"),
                                                    "--video-root", str(self.root / "video")])

    def test_default_eight_serial_commands_are_full_and_fair(self):
        plan = experiments.build_plan(self.args)
        self.assertEqual([item["id"] for item in plan], [f"{controller}_{condition}" for condition in
                         ("normal", "disturbed") for controller in ("pd", "gc-pd", "rbf", "fuzzy")])
        self.assertEqual(len({item["common_config_sha256"] for item in plan}), 1)
        for item in plan:
            self.assertTrue(item["resolved_config"]["headless"])
            self.assertEqual(item["resolved_config"]["max_steps"], 0)
            self.assertEqual(item["resolved_config"]["hold_seconds"], 0)
            self.assertEqual(Path(item["video"]).name, f"{item['id']}.mp4")
            self.assertIn("--disturbance" if item["condition"] == "disturbed" else "--no-disturbance", item["command"])
            self.assertFalse(any(str(arg).endswith(".sh") for arg in item["command"]))

    def test_independent_validation_accepts_physx_float32_clock_rounding(self):
        item = experiments.build_plan(self.args)[0]
        _fixture(item)
        validated = experiments.validate_run(item)
        self.assertEqual(validated["independent_validation"]["csv_sample_count"], 3)
        self.assertTrue(validated["independent_validation"]["time_aligned_metrics_recomputed"])
        self.assertEqual(validated["video_validation"]["frame_count"], 62)

    def test_extra_physics_step_false_metrics_and_bad_video_are_rejected(self):
        item = experiments.build_plan(self.args)[0]
        original = _fixture(item)
        path = Path(item["output"]) / "metrics.json"
        for mutate in (
            lambda value: value.update(physics_elapsed_s=.004),
            lambda value: value["metrics"].update(drawing_tip_rmse_m=.1),
            lambda value: value.update(completed=False),
            lambda value: value.update(timeline_play_enabled=False),
            lambda value: value["video"].update(completed=False),
        ):
            value = json.loads(json.dumps(original))
            mutate(value)
            _json(path, value)
            with self.assertRaises(ValueError):
                experiments.validate_run(item)
        _json(path, original)
        video = Path(item["video"])
        video.write_bytes(video.read_bytes()[:-1])
        with self.assertRaises(ValueError):
            experiments.validate_run(item)

    def test_resume_requires_all_hashes_same_sources_and_unchanged_artifacts(self):
        item = experiments.build_plan(self.args)[0]
        _fixture(item)
        record = {**item, "status": "completed", "exit_code": 0, "signature_sha256": "same-source-config",
                  "artifact_sha256": experiments.artifact_hashes(item)}
        self.assertTrue(experiments.can_resume(record, item, "same-source-config"))
        self.assertFalse(experiments.can_resume(record, item, "different-source-config"))
        removed = dict(record)
        removed["artifact_sha256"] = dict(record["artifact_sha256"])
        removed["artifact_sha256"].pop(str(Path(item["video"]).resolve()))
        self.assertFalse(experiments.can_resume(removed, item, "same-source-config"))
        reference = Path(item["output"]) / "reference_cartesian.csv"
        reference.write_text("different-reference\n")
        self.assertFalse(experiments.can_resume(record, item, "same-source-config"))

    def test_failed_child_stops_next_run_and_preserves_failure_log_manifest(self):
        with patch("raa.runtime.locate_isaac", return_value=self.root), \
                patch.object(experiments, "locate_chrome", return_value="chrome-fixture"), \
                patch.object(experiments, "source_fingerprint", return_value={"sha256": "fixture", "files": {}}), \
                patch.object(experiments.subprocess, "run", return_value=SimpleNamespace(returncode=7)) as child:
            self.assertEqual(experiments.run_batch(self.args), 1)
        self.assertEqual(child.call_count, 1)
        manifest = json.loads((self.args.output_root / "experiment_manifest.json").read_text())
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["runs"]["pd_normal"]["exit_code"], 7)
        self.assertEqual(manifest["runs"]["pd_normal"]["status"], "failed")
        self.assertTrue((self.args.output_root / "logs/pd_normal.log").is_file())
        self.assertTrue((self.args.output_root / "summary.csv").is_file())

    def test_plan_does_not_locate_or_launch_isaac_chrome_or_create_output(self):
        self.args.plan = True
        with patch("raa.runtime.locate_isaac", side_effect=AssertionError("no Isaac")), \
                patch.object(experiments, "locate_chrome", side_effect=AssertionError("no Chrome")), \
                patch.object(experiments.subprocess, "run", side_effect=AssertionError("no child")), \
                patch("builtins.print"):
            self.assertEqual(experiments.run_batch(self.args), 0)
        self.assertFalse(self.args.output_root.exists())

    def test_successful_pair_exports_comparison_and_resume_runs_no_child(self):
        self.args.controllers = ["pd"]
        items = experiments.build_plan(self.args)
        calls = []

        def child(command, **kwargs):
            item = next(item for item in items if item["command"] == command)
            calls.append(item["id"])
            _fixture(item)
            return SimpleNamespace(returncode=0)

        def render(directory, **kwargs):
            rendered = []
            for svg in Path(directory).glob("*.svg"):
                width, height = svg_size(svg)
                png = svg.with_suffix(".png")
                png.write_bytes(b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IHDR" + struct.pack(">II", width, height))
                rendered.append(png)
            return rendered

        with patch("raa.runtime.locate_isaac", return_value=self.root), \
                patch.object(experiments, "locate_chrome", return_value="chrome-fixture"), \
                patch.object(experiments, "source_fingerprint", return_value={"sha256": "fixture", "files": {}}), \
                patch.object(experiments, "export_directory", side_effect=render), \
                patch.object(experiments.subprocess, "run", side_effect=child):
            self.args.pictures_root = self.root / "pictures"
            self.assertEqual(experiments.run_batch(self.args), 0)
            self.args.resume = True
            self.assertEqual(experiments.run_batch(self.args), 0)
        self.assertEqual(calls, ["pd_normal", "pd_disturbed"])
        manifest = json.loads((self.args.output_root / "experiment_manifest.json").read_text())
        self.assertEqual(manifest["status"], "completed")
        self.assertEqual(manifest["comparisons"]["pd"]["status"], "completed")
        self.assertEqual(manifest["comparisons"]["pd"]["metadata"]["metrics"]["disturbance_deviation"]["tip_rmse_m"], 0)
        self.assertEqual(len(list((self.args.pictures_root / "comparisons/pd").glob("*.png"))), 3)
        with (self.args.output_root / "summary.csv").open() as stream:
            summary = list(csv.DictReader(stream))
        self.assertEqual(len(summary), 2)
        self.assertEqual(float(summary[0]["disturbance_deviation_drawing_tip_rmse_m"]), 0)


class FigureTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="raa-figure-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def test_svg_pixels_viewbox_and_invalid_sizes(self):
        source = self.root / "test.svg"
        for attributes, expected in (("width='800px' height='580'", (800, 580)),
                                     ("viewBox='0 0 700.1 500.2'", (701, 501)),
                                     ("width='100%' height='100%' viewBox='0 0 800 800'", (800, 800))):
            source.write_text(f"<svg {attributes}/>")
            self.assertEqual(svg_size(source), expected)
        for attributes in ("width='0' height='800'", "width='1cm' height='800'", "width='NaN' height='800'"):
            source.write_text(f"<svg {attributes}/>")
            with self.assertRaises(ValueError):
                svg_size(source)

    def test_conversion_uses_own_profile_native_size_and_keeps_svg(self):
        source = self.root / "test.svg"
        source.write_text("<svg width='800' height='580'/>")
        original = source.read_bytes()
        commands = []

        def chrome(command, **kwargs):
            commands.append(command)
            png = Path(next(arg.split("=", 1)[1] for arg in command if arg.startswith("--screenshot=")))
            png.write_bytes(b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IHDR" + struct.pack(">II", 800, 580))
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        with patch("raa.figures.locate_chrome", return_value="chrome"), patch("raa.figures.subprocess.run", side_effect=chrome):
            first = svg_to_png(source)
            second = svg_to_png(source)
        self.assertEqual(png_size(first), (800, 580))
        self.assertEqual(first, second)
        self.assertEqual(source.read_bytes(), original)
        self.assertIn("--disable-gpu", commands[0])
        self.assertIn("--window-size=800,580", commands[0])
        profiles = [next(arg for arg in command if arg.startswith("--user-data-dir=")) for command in commands]
        self.assertNotEqual(profiles[0], profiles[1])
        self.assertEqual(list(self.root.iterdir()), [source, first])


if __name__ == "__main__":
    unittest.main()
