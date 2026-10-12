"""Meaningful configuration regression checks; no Isaac/NumPy imports needed."""

from argparse import ArgumentParser, Namespace, SUPPRESS
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from raa.config import CONFIG_DIR, load_config, merge_config, validate_config


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="raa-config-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def write_config(self, data, filename="experiment.json"):
        destination = self.root / "configs" / filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(data), encoding="utf-8")
        return destination

    def test_four_presets_choose_gui_and_separate_outputs(self):
        for alias in ("pd", "gc-pd", "rbf", "fuzzy"):
            with self.subTest(alias=alias):
                config = load_config(alias)
                self.assertEqual(config["controller"], alias)
                self.assertIs(config["headless"], False)
                self.assertEqual(config["output"], CONFIG_DIR.parent / "output" / alias)
                self.assertEqual(config["dt"], 0.001)
                self.assertTrue(Path(config["config_path"]).is_file())
        self.assertEqual(load_config("gc_pd"), load_config("gc-pd"))

    def test_file_paths_resolve_relative_to_config_not_cwd(self):
        filename = self.write_config({"controller": "pd", "output": "../recordings", "isaac_root": "../local-isaac"})
        config = load_config(filename)
        self.assertEqual(config["output"], self.root / "recordings")
        self.assertEqual(config["isaac_root"], self.root / "local-isaac")
        self.assertEqual(config["config_path"], str(filename.resolve()))
        self.assertFalse(config["output"].exists(), "Loading should not create result directories")

    def test_explicit_cli_overrides_preserve_unmentioned_file_values(self):
        parser = ArgumentParser(argument_default=SUPPRESS)
        parser.add_argument("--speed", type=float)
        parser.add_argument("--headless", action="store_true", default=SUPPRESS)
        explicit = parser.parse_args(["--speed", "0.02"])
        config = load_config("rbf")
        result = merge_config({"controller": "gc-pd", "speed": .04, "headless": True}, config, explicit)
        self.assertIsInstance(result, Namespace)
        self.assertEqual(result.controller, "rbf")
        self.assertEqual(result.speed, .02)
        self.assertFalse(result.headless)
        self.assertEqual(result.adaptive["rbf_width"], .65)
        self.assertEqual(result.config_path, config["config_path"])

    def test_nested_gains_merge_and_scalar_expansion_without_mutation(self):
        defaults = {"controller": "gc-pd", "control": {"kp": 10, "kd": 3}}
        config = {"control": {"kp": [20] * 7}}
        result = merge_config(defaults, config, {"control": {"kd": 4}})
        self.assertEqual(result.control["kp"], [20.] * 7)
        self.assertEqual(result.control["kd"], [4.] * 7)
        self.assertEqual(defaults["control"]["kd"], 3)
        self.assertEqual(config["control"]["kp"], [20] * 7)

    def test_cli_relative_output_uses_cwd(self):
        result = merge_config({}, load_config("fuzzy"), {"output": "overridden-results"})
        self.assertEqual(result.output, Path.cwd() / "overridden-results")

    def test_invalid_boolean_numbers_nonfinite_values_and_shapes(self):
        invalid = [
            {"speed": True}, {"headless": 0}, {"max_steps": True}, {"max_steps": 1.5},
            {"hold_seconds": -1}, {"dt": 0}, {"speed": float("inf")}, {"speed": 10 ** 400},
            {"draw_z": float("nan")}, {"center": [0, True]}, {"center": [0, 0, 0]},
            {"control": {"kp": [1] * 6}}, {"control": {"kd": True}},
            {"control": {"torque_limits": [1, 1, 1, 1, 1, 1, 0]}},
            {"adaptive": {"learning_rate": True}}, {"adaptive": {"leakage": -0.01}},
            {"adaptive": {"centers_per_axis": 2.5}}, {"adaptive": {"centers_per_axis": 10}},
            {"adaptive": {"compensation_limits": 2}}, {"output": ""},
        ]
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(ValueError):
                validate_config(data)

    def test_unknown_and_controller_incompatible_fields_rejected(self):
        invalid = [
            {"speeed": .04}, {"control": {"kpp": 100}}, {"adaptive": {"learn_rate": 30}},
            {"controller": "rbf", "adaptive": {"error_scale": .02}},
            {"controller": "fuzzy", "adaptive": {"rbf_width": .65}},
            {"controller": "pd", "adaptive": {"learning_rate": 30}},
            {"controller": "pid"}, {"controller": 1},
        ]
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(ValueError):
                load_config(self.write_config(data))

    def test_missing_file_and_nonobject_json_rejected(self):
        with self.assertRaises(FileNotFoundError):
            load_config(self.root / "missing.json")
        with self.assertRaises(ValueError):
            load_config(self.write_config([1, 2, 3]))

    def test_duplicate_keys_do_not_silently_overwrite_tuning(self):
        filename = self.write_config({})
        filename.write_text('{"control":{"kp":20,"kp":30}}', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "重复字段"):
            load_config(filename)

    def test_no_config_preserves_defaults(self):
        result = merge_config({"controller": "gc-pd", "output": self.root / "output"}, load_config(None))
        self.assertEqual(result.controller, "gc-pd")
        self.assertEqual(result.control, {})
        self.assertEqual(result.adaptive, {})
        self.assertIsNone(result.config_path)

    def test_adaptive_cross_field_constraints_match_runtime_factory(self):
        invalid = [
            {"controller": "rbf", "dt": .011},
            {"controller": "fuzzy", "dt": .001, "adaptive": {"leakage": 1001}},
            {"controller": "rbf", "adaptive": {"rbf_width": 1e-7}},
            {"controller": "fuzzy", "adaptive": {"compensation_limits": [6, 6, 6, 6, 2, 2, 13]}},
            {"controller": "rbf", "control": {"torque_limits": [1] * 7}},
        ]
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(ValueError):
                merge_config({}, data)
        valid = merge_config({}, {"controller": "rbf", "dt": .01,
                                   "adaptive": {"leakage": 100, "rbf_width": 1e-6,
                                                "compensation_limits": [1] * 7},
                                   "control": {"torque_limits": [1] * 7}})
        self.assertEqual(valid.dt, .01)

    def test_partial_adaptive_fields_use_final_defaults_during_merge(self):
        partial = validate_config({"controller": "fuzzy", "adaptive": {"leakage": 2000}})
        with self.assertRaisesRegex(ValueError, "leakage"):
            merge_config({"dt": .001}, partial)
        self.assertEqual(merge_config({"dt": .0001}, partial).adaptive["leakage"], 2000)


if __name__ == "__main__":
    unittest.main()
