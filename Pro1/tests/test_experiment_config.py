"""Exercise video/force configuration at the real CLI without Isaac or GPU."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


MAIN = Path(__file__).resolve().parents[1] / "main.py"


class ExperimentConfigTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="raa-experiment-config-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.cwd = self.root / "working"
        self.cwd.mkdir()
        self.config_path = self.root / "configs" / "experiment.json"
        self.config_path.parent.mkdir()

    def write_config(self, data):
        self.config_path.write_text(json.dumps(data), encoding="utf-8")
        return self.config_path

    def invoke(self, *arguments):
        # -S excludes all site packages; nonexistent Isaac makes accidental
        # startup fail before GPU access. show-config must bypass that lookup.
        return subprocess.run(
            [sys.executable, "-S", str(MAIN), "--isaac-root", str(self.root / "no-isaac"),
             *map(str, arguments), "--show-config"],
            cwd=self.cwd, text=True, capture_output=True, timeout=15,
        )

    def shown(self, data=None, *arguments):
        config_arguments = ("--config", self.write_config(data)) if data is not None else ()
        result = self.invoke(*config_arguments, *arguments)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        return json.loads(result.stdout)

    def rejected(self, data, field):
        result = self.invoke("--config", self.write_config(data))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn(field, result.stderr)
        self.assertEqual(result.stdout, "")

    @staticmethod
    def pulse(start=1, end=2, force=None):
        return {"start_s": start, "end_s": end,
                "force_world_n": [3, 0, 0] if force is None else force}

    def test_video_paths_follow_config_directory_or_explicit_cli_cwd(self):
        data = {"record_video": "../clips/example.mp4", "output": "../results"}
        config = self.shown(data)
        self.assertEqual(Path(config["record_video"]), self.root / "clips" / "example.mp4")
        self.assertEqual(Path(config["output"]), self.root / "results")
        overridden = self.shown(data, "--record-video", "custom/other.mp4")
        self.assertEqual(Path(overridden["record_video"]), self.cwd / "custom" / "other.mp4")
        self.assertFalse((self.root / "clips").exists())
        self.assertFalse((self.root / "results").exists())
        self.assertFalse((self.cwd / "custom").exists())

    def test_video_defaults_and_valid_overrides_only_print_configuration(self):
        default = self.shown()
        self.assertIsNone(default["record_video"])
        self.assertEqual((default["video_fps"], default["video_width"], default["video_height"]), (30, 960, 720))
        self.assertEqual(default["video_tail_seconds"], 2.0)
        config = self.shown({"record_video": "test.MP4", "video_fps": 24,
                             "video_width": 1280, "video_height": 720, "video_tail_seconds": 0},
                            "--video-fps", 30, "--video-width", 640, "--video-height", 480,
                            "--video-tail-seconds", 0.5)
        self.assertEqual((config["video_fps"], config["video_width"], config["video_height"]), (30, 640, 480))
        self.assertEqual(config["video_tail_seconds"], 0.5)
        self.assertFalse(Path(config["record_video"]).exists())

    def test_invalid_video_types_and_extensions_are_rejected_before_startup(self):
        cases = [
            ({"record_video": "test.avi"}, "record_video"),
            ({"record_video": True}, "record_video"),
            ({"record_video": ""}, "record_video"),
            ({"video_fps": True}, "video_fps"),
            ({"video_fps": 30.0}, "video_fps"),
            ({"video_fps": 0}, "video_fps"),
            ({"video_fps": float("inf")}, "video_fps"),
            ({"video_width": 959}, "video_width"),
            ({"video_height": 721}, "video_height"),
            ({"video_width": "960"}, "video_width"),
            ({"video_height": False}, "video_height"),
            ({"video_width": -2}, "video_width"),
            ({"video_tail_seconds": True}, "video_tail_seconds"),
            ({"video_tail_seconds": -1}, "video_tail_seconds"),
            ({"video_tail_seconds": float("nan")}, "video_tail_seconds"),
        ]
        for data, field in cases:
            with self.subTest(data=data):
                self.rejected(data, field)

    def test_force_toggles_preserve_pulses_gains_and_adaptive_settings(self):
        pulses = [self.pulse(1, 2), self.pulse(2, 3, [0, -3, 0])]
        data = {"controller": "fuzzy", "speed": .025,
                "control": {"kp": [100] * 7}, "adaptive": {"learning_rate": 12},
                "disturbance": {"enabled": False, "pulses": pulses}}
        baseline = self.shown(data)
        enabled = self.shown(data, "--disturbance")
        disabled = self.shown({**data, "disturbance": {"enabled": True, "pulses": pulses}}, "--no-disturbance")
        self.assertTrue(enabled["disturbance"]["enabled"])
        self.assertFalse(disabled["disturbance"]["enabled"])
        for result in (enabled, disabled):
            self.assertEqual(result["disturbance"]["pulses"], baseline["disturbance"]["pulses"])
            for field in ("controller", "speed", "control", "adaptive", "record_video", "video_fps"):
                self.assertEqual(result[field], baseline[field])
        self.assertTrue(self.shown({"disturbance": {"enabled": True}})["disturbance"]["enabled"])
        result = self.invoke("--disturbance", "--no-disturbance")
        self.assertEqual(result.returncode, 2)

    def test_force_sections_and_pulse_fields_are_strict(self):
        cases = [
            ({"disturbance": True}, "disturbance"),
            ({"disturbance": {"enabled": 1}}, "disturbance.enabled"),
            ({"disturbance": {"enable": True}}, "未知字段"),
            ({"disturbance": {"pulses": {}}}, "pulses"),
            ({"disturbance": {"pulses": [False]}}, "pulses"),
            ({"disturbance": {"pulses": [{"start_s": 1, "end_s": 2}]}}, "每个外力脉冲"),
            ({"disturbance": {"pulses": [{**self.pulse(), "extra": 1}]}}, "未知字段"),
            ({"disturbance": {"pulses": [self.pulse(force=[3, 0])]}}, "force_world_n"),
            ({"disturbance": {"pulses": [self.pulse(force=[3, True, 0])]}}, "force_world_n"),
            ({"disturbance": {"pulses": [self.pulse(force="3,0,0")]}}, "force_world_n"),
        ]
        for data, field in cases:
            with self.subTest(data=data):
                self.rejected(data, field)

    def test_force_times_and_components_must_be_finite_and_numeric(self):
        pulses = [self.pulse(start=True), self.pulse(start="1"), self.pulse(start=-1),
                  self.pulse(start=float("nan")), self.pulse(end=float("inf")),
                  self.pulse(force=[3, float("nan"), 0]),
                  self.pulse(force=[0, -float("inf"), 0])]
        for pulse in pulses:
            with self.subTest(pulse=pulse):
                field = "force_world_n" if not all(isinstance(v, (int, float)) and
                    v == v and abs(v) != float("inf") for v in pulse["force_world_n"]) else "pulse."
                self.rejected({"disturbance": {"enabled": True, "pulses": [pulse]}}, field)

    def test_pulse_endpoints_may_touch_but_order_and_overlap_are_rejected(self):
        valid = self.shown({"disturbance": {"enabled": True,
                    "pulses": [self.pulse(0, 1), self.pulse(1, 2)]}})
        self.assertEqual(len(valid["disturbance"]["pulses"]), 2)
        for pulses in ([self.pulse(2, 2)], [self.pulse(2, 1)],
                       [self.pulse(0, 2), self.pulse(1, 3)],
                       [self.pulse(3, 4), self.pulse(1, 2)]):
            with self.subTest(pulses=pulses):
                self.rejected({"disturbance": {"pulses": pulses}}, "互不重叠")


if __name__ == "__main__":
    unittest.main()
