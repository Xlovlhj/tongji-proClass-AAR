import argparse
import contextlib
import io
import sys
import unittest
from unittest.mock import patch

from Pro2 import visualize_tasks


class VisualizeTasksTests(unittest.TestCase):
    def test_accepts_l4_jump_as_positional_task(self):
        with patch.object(sys, "argv", ["visualize_tasks.py", "l4-jump", "--dry-run"]):
            args = visualize_tasks.arguments()
        self.assertEqual(args.task, "l4-jump")
        self.assertTrue(args.dry_run)

    def test_jump_dry_run_uses_config_analyzer_and_isolated_l4_output(self):
        args = argparse.Namespace(
            isaac_root="/opt/isaacsim",
            render_hz=30.0,
            record_video=False,
            dry_run=True,
            no_open=True,
        )
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            output = visualize_tasks.run_jump(args, "20260102-030405-000006_test")
        printed = stream.getvalue()
        self.assertEqual(
            visualize_tasks.relative(output),
            "Pro2/output/L4/03_terrain_and_jump/"
            "visual_vertical_jump_20260102-030405-000006_test",
        )
        self.assertIn("Pro2/experiments/L4/jump/vertical_jump.json", printed)
        self.assertIn("--log-control-decomposition", printed)
        self.assertIn("Pro2/tools/analyze_l4_jump.py --run", printed)
        self.assertIn(str(output.relative_to(visualize_tasks.PROJECT_ROOT)), printed)


if __name__ == "__main__":
    unittest.main()
