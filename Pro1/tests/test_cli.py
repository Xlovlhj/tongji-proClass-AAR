"""Exercise the real terminal entry point in a standard-library-only process.

Python -S omits third-party site packages. All invocations stop at show-config,
preview or validation; preview also receives a nonexistent Isaac path, so an
accidental simulator startup fails before any GPU work could begin.
"""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]
MAIN = PROJECT / "main.py"


class CLITests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="raa-cli-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def invoke(self, *arguments):
        return subprocess.run(
            [sys.executable, "-S", str(MAIN), *map(str, arguments)],
            cwd=self.root, text=True, capture_output=True, timeout=15,
        )

    def shown_config(self, *arguments):
        result = self.invoke(*arguments, "--show-config")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        return json.loads(result.stdout)

    def write_config(self, data):
        destination = self.root / "configs" / "experiment.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(data), encoding="utf-8")
        return destination

    def test_show_four_presets_without_site_packages(self):
        for controller in ("pd", "gc-pd", "rbf", "fuzzy"):
            with self.subTest(controller=controller):
                config = self.shown_config("--config", controller)
                self.assertEqual(config["controller"], controller)
                self.assertIs(config["headless"], False)
                self.assertEqual(Path(config["output"]), PROJECT / "output" / controller)
                self.assertEqual(config["dt"], .001)
                self.assertEqual(len(config["control"]["kp"]), 7)

    def test_gui_and_headless_explicitly_override_configuration(self):
        filename = self.write_config({"controller": "pd", "headless": True})
        self.assertTrue(self.shown_config("--config", filename)["headless"])
        self.assertFalse(self.shown_config("--config", filename, "--gui")["headless"])
        self.assertTrue(self.shown_config("--config", "rbf", "--headless")["headless"])
        self.assertFalse(self.shown_config("--config", "rbf", "--gui")["headless"])

    def test_absent_cli_values_do_not_overwrite_file(self):
        filename = self.write_config({
            "controller": "fuzzy", "headless": True, "speed": .017,
            "dt": .002, "logo_width": .11, "center": [.42, -.03],
            "output": "../preserved-results", "adaptive": {"learning_rate": 12},
        })
        config = self.shown_config("--config", filename)
        self.assertEqual(config["controller"], "fuzzy")
        self.assertTrue(config["headless"])
        self.assertEqual(config["speed"], .017)
        self.assertEqual(config["dt"], .002)
        self.assertEqual(config["logo_width"], .11)
        self.assertEqual(config["center"], [.42, -.03])
        self.assertEqual(config["adaptive"]["learning_rate"], 12)
        self.assertEqual(Path(config["output"]), self.root / "preserved-results")
        self.assertFalse((self.root / "preserved-results").exists(), "show-config must not create output directories")

    def test_explicit_cli_geometry_and_paths_win(self):
        config = self.shown_config("--config", "rbf", "--speed", .02,
                                   "--center", .4, .03, "--output", "cli-results")
        self.assertEqual(config["speed"], .02)
        self.assertEqual(config["center"], [.4, .03])
        self.assertEqual(Path(config["output"]), self.root / "cli-results")
        self.assertEqual(config["adaptive"]["rbf_width"], .65)

    def test_invalid_config_returns_two_with_clear_error(self):
        for data, expected in [({"speeed": .04}, "未知字段"),
                               ({"headless": 1}, "headless"),
                               ({"dt": 0}, "dt"),
                               ({"controller": "fuzzy", "adaptive": {"rbf_width": .65}}, "不支持")]:
            with self.subTest(data=data):
                result = self.invoke("--config", self.write_config(data), "--show-config")
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn(expected, result.stderr)
                self.assertEqual(result.stdout, "")

    def test_missing_config_returns_two(self):
        result = self.invoke("--config", self.root / "missing.json", "--show-config")
        self.assertEqual(result.returncode, 2)
        self.assertIn("找不到配置文件", result.stderr)

    def test_preview_writes_svg_without_isaac_or_site_packages(self):
        output = self.root / "preview"
        result = self.invoke("--config", "fuzzy", "--preview", "--output", output,
                             "--isaac-root", self.root / "nonexistent-isaac")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("共 8 笔", result.stdout)
        self.assertEqual(result.stderr, "")
        self.assertEqual({p.name for p in output.iterdir()}, {"logo_preview.svg"})
        root = ET.parse(output / "logo_preview.svg").getroot()
        polylines = [element for element in root.iter() if element.tag.endswith("polyline")]
        self.assertEqual(len(polylines), 8)

    def test_default_show_config_preserves_baseline(self):
        config = self.shown_config()
        self.assertEqual(config["controller"], "gc-pd")
        self.assertFalse(config["headless"])
        self.assertEqual(Path(config["output"]), PROJECT / "output")
        self.assertEqual(config["adaptive"], {})

    def test_show_config_rejects_cross_field_adaptive_parameters(self):
        cases = [({"controller": "rbf", "dt": .02}, "dt"),
                 ({"controller": "fuzzy", "adaptive": {"leakage": 2000}}, "leakage"),
                 ({"controller": "rbf", "adaptive": {"rbf_width": 1e-8}}, "rbf_width"),
                 ({"controller": "fuzzy", "control": {"torque_limits": [1] * 7}}, "compensation_limits")]
        for data, expected in cases:
            with self.subTest(data=data):
                result = self.invoke("--config", self.write_config(data), "--show-config")
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn(expected, result.stderr)


if __name__ == "__main__":
    unittest.main()
