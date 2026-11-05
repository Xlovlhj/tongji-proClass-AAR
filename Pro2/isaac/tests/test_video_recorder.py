"""Failure recordings preserve real duration and never claim experiment success."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from isaac.simulation import PhysicalInstabilityError, ViewportVideoRecorder


class VideoRecorderTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.args = SimpleNamespace(record_video=True, video_fps=10, duration=1.0,
                                    output=Path(self.directory.name))
        self.recorder = ViewportVideoRecorder(self.args, viewport=object())
        self.encode = Mock(side_effect=self._encode)
        self.patcher = patch.dict(sys.modules, {
            "video_encoding": SimpleNamespace(encode_image_file_sequence=self.encode)})
        self.patcher.start()
        self.addCleanup(self.patcher.stop)

    def _encode(self, pattern, start, fps, destination, remove_frames):
        Path(destination).write_bytes(b"test-encoder-output")
        return True

    def test_partial_recording_is_rejected_unless_explicit(self):
        self.recorder.frame_count = 3
        with self.assertRaisesRegex(RuntimeError, "视频帧数不完整"):
            self.recorder.finish()
        self.encode.assert_not_called()
        self.assertFalse(self.recorder.manifest()["video_saved"])

    def test_partial_recording_keeps_actual_length_and_failure_reason(self):
        self.recorder.frame_count = 3
        reason = "t=0.294s：机器人失稳"
        self.recorder.finish(allow_partial=True, termination_reason=reason)
        result = self.recorder.manifest()
        self.assertTrue(result["video_saved"])
        self.assertFalse(result["video_complete"])
        self.assertEqual(result["video_frames"], 3)
        self.assertEqual(result["video_duration_s"], 0.3)
        self.assertEqual(result["video_planned_frames"], 10)
        self.assertEqual(result["video_planned_duration_s"], 1.0)
        self.assertEqual(result["video_termination_reason"], reason)
        self.encode.assert_called_once_with(
            str(self.recorder.frames_dir / "frame.%06d.png"), 0, 10,
            str(self.recorder.path), True)

    def test_partial_recording_requires_frames_and_reason(self):
        with self.assertRaisesRegex(RuntimeError, "没有已完成的视频帧"):
            self.recorder.finish(allow_partial=True, termination_reason="instability")
        self.recorder.frame_count = 3
        with self.assertRaisesRegex(ValueError, "提前终止原因"):
            self.recorder.finish(allow_partial=True)
        self.encode.assert_not_called()

    def test_normal_recording_remains_complete(self):
        self.recorder.frame_count = 10
        self.recorder.finish()
        result = self.recorder.manifest()
        self.assertTrue(result["video_complete"])
        self.assertEqual(result["video_termination_reason"], "completed")

    def test_encoder_failure_is_not_saved(self):
        self.recorder.frame_count = 3
        self.encode.side_effect = None
        self.encode.return_value = False
        with self.assertRaisesRegex(RuntimeError, "编码器执行失败"):
            self.recorder.finish(allow_partial=True, termination_reason="instability")
        self.assertFalse(self.recorder.manifest()["video_saved"])
        self.assertFalse(self.recorder.manifest()["video_complete"])

    def test_only_physical_stop_has_typed_failure_and_original_state(self):
        failure = PhysicalInstabilityError(8.994, [1.0, 0.2, 0.59], 0.7, 0.4)
        self.assertIsInstance(failure, RuntimeError)
        self.assertNotIsInstance(RuntimeError("capture failed"), PhysicalInstabilityError)
        self.assertEqual(failure.state["time_s"], 8.994)
        self.assertEqual(failure.state["base_position_m"], [1.0, 0.2, 0.59])
        self.assertIn("z=0.590m", str(failure))


if __name__ == "__main__":
    unittest.main()
