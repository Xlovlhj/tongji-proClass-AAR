"""Verify frame ordering, native-buffer ownership and aborts without Isaac/GPU."""

import ctypes
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raa.recording import VideoRecorder, _copy_buffer, _rgba_bytes


class FakeEncoder:
    def __init__(self):
        self.frames = []
        self.finalized = 0
        self.reject_frame = False

    def start_encoding(self, filename, fps, total_frames, overwrite):
        self.start_arguments = (fps, total_frames, overwrite)
        Path(filename).write_bytes(b"fake-mp4")
        return True

    def encode_next_frame_from_buffer(self, buffer, width, height):
        if self.reject_frame:
            return False
        self.frames.append((bytes(buffer), width, height))
        return True

    def finalize_encoding(self):
        self.finalized += 1


class RecordingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="raa-recording-test-")
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "test.mp4"
        self.encoder = FakeEncoder()
        self.pending = []
        self.render_count = 0
        self.settings = {}

        def schedule(viewport, callback, is_hdr):
            self.assertFalse(is_hdr)
            self.pending.append(callback)
            # Capture's own future can report done before callback delivery.
            return SimpleNamespace(done=True)

        modules = {name: ModuleType(name) for name in (
            "isaacsim", "isaacsim.core", "isaacsim.core.utils",
            "isaacsim.core.utils.extensions", "video_encoding", "omni",
            "omni.kit", "omni.kit.viewport", "omni.kit.viewport.utility",
            "carb", "carb.settings")}
        modules["isaacsim.core.utils.extensions"].enable_extension = lambda name: name == "omni.videoencoding"
        modules["video_encoding"].get_video_encoding_interface = lambda: self.encoder
        modules["omni.kit.viewport.utility"].capture_viewport_to_buffer = schedule
        modules["carb.settings"].get_settings = lambda: SimpleNamespace(
            get=self.settings.get, set=self.settings.__setitem__)
        modules["carb"].settings = modules["carb.settings"]
        context = patch.dict(sys.modules, modules)
        context.start()
        self.addCleanup(context.stop)
        self.sync_report = {
            "patched_samples": 1, "idr_samples": 1, "p_samples": 1,
            "sample_count": 2, "media_sha256_before": "same",
            "media_sha256_after": "same", "media_payload_unchanged": True,
            "metadata_bytes_changed": 4,
        }
        repair = patch("raa.video_validation.repair_native_mp4_sync_flags",
                       return_value=self.sync_report, create=True)
        self.repair = repair.start()
        self.addCleanup(repair.stop)
        self.recorder = VideoRecorder(SimpleNamespace(), self.path, fps=30, width=2, height=2)
        self.addCleanup(self.recorder.abort, "test cleanup")

    def render(self):
        self.render_count += 1
        # Delayed GPU readback: only the second held-state render delivers it.
        if self.pending and self.render_count % 2 == 0:
            callback = self.pending.pop(0)
            callback(bytes([self.render_count] * 16), 16, 2, 2, "RGBA8_UNORM")

    def test_capture_waits_for_actual_readback_and_preserves_frame_order(self):
        self.recorder.capture(self.render, 0.0)
        self.recorder.capture(self.render, 1 / 30)
        self.assertEqual(self.render_count, 4)
        self.assertEqual([frame[0][0] for frame in self.encoder.frames], [2, 4])
        metadata = self.recorder.finish()
        self.assertEqual(self.encoder.start_arguments, (30.0, 0, True))
        self.assertEqual(metadata["frame_count"], 2)
        self.assertEqual(metadata["motion_frame_count"], 2)
        self.assertEqual(metadata["capture_render_updates"], 4)
        self.assertAlmostEqual(metadata["duration_s"], 2 / 30)
        self.assertTrue(metadata["no_dropped_frames"])
        self.repair.assert_called_once_with(self.path)
        self.assertEqual(metadata["mp4_sync_flags"], self.sync_report)
        self.assertEqual(self.recorder.finish(), metadata)
        self.repair.assert_called_once_with(self.path)
        self.recorder.abort("finally after finish")
        self.assertTrue(self.path.exists())
        self.assertEqual(self.encoder.finalized, 1)

    def test_sync_repair_failure_deletes_finalized_video_and_leaves_abort_marker(self):
        self.recorder.capture(self.render, 0.0)
        self.repair.side_effect = ValueError("Native sync flags cannot be verified")
        with self.assertRaisesRegex(ValueError, "cannot be verified"):
            self.recorder.finish()
        self.repair.assert_called_once_with(self.path)
        self.assertEqual(self.encoder.finalized, 1)
        self.assertFalse(self.path.exists())
        self.assertFalse(self.path.with_suffix(".recording.json").exists())
        marker = json.loads(self.path.with_suffix(".aborted.json").read_text())
        self.assertFalse(marker["completed"])
        self.assertIn("cannot be verified", marker["reason"])
        with self.assertRaisesRegex(RuntimeError, "aborted recorder"):
            self.recorder.finish()
        self.recorder.abort("already cleaned")
        self.assertEqual(self.encoder.finalized, 1)
        self.repair.assert_called_once_with(self.path)

    def test_final_hold_does_not_invent_motion_time_or_render(self):
        self.recorder.capture(self.render, 0.2)
        self.recorder.hold_final(2.0)
        metadata = self.recorder.finish()
        self.assertEqual(metadata["motion_frame_count"], 1)
        self.assertEqual(metadata["hold_frame_count"], 60)
        self.assertEqual(metadata["frame_count"], 61)
        self.assertEqual(metadata["hold_seconds"], 2.0)
        self.assertEqual(metadata["source_duration_s"], 0.2)
        self.assertEqual(self.recorder.last_sim_time_s, 0.2)
        self.assertEqual(self.render_count, 2)
        self.assertTrue(all(frame == self.encoder.frames[0] for frame in self.encoder.frames))

    def test_rejected_frame_removes_unfinished_video_and_marks_abort(self):
        self.encoder.reject_frame = True
        with self.assertRaisesRegex(RuntimeError, "rejected"):
            self.recorder.capture(self.render, 0.0)
        self.assertFalse(self.path.exists())
        marker = json.loads(self.path.with_suffix(".aborted.json").read_text())
        self.assertFalse(marker["completed"])
        self.assertEqual(marker["frame_count"], 0)
        self.assertEqual(self.encoder.finalized, 1)
        self.recorder.abort()
        self.assertEqual(self.encoder.finalized, 1)

    def test_readback_timeout_never_records_or_keeps_an_incomplete_mp4(self):
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.recorder.capture(lambda: None, 0.0)
        self.assertEqual(len(self.encoder.frames), 0)
        self.assertFalse(self.path.exists())

    def test_motion_times_must_increase(self):
        self.assertIsNone(self.recorder.last_sim_time_s)
        self.recorder.capture(self.render, 0.0)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self.recorder.capture(self.render, 0.0)
        self.assertEqual(len(self.encoder.frames), 1)

    def test_known_total_count_includes_hold_and_reaches_native_encoder(self):
        self.recorder.abort("replace unknown-count fixture")
        self.recorder = VideoRecorder(SimpleNamespace(), self.path, fps=30, width=2, height=2,
                                      expected_frames=61)
        self.recorder.capture(self.render, 0.0)
        self.recorder.hold_final(2.0)
        metadata = self.recorder.finish()
        self.assertEqual(self.encoder.start_arguments, (30.0, 61, True))
        self.assertEqual(metadata["expected_frame_count"], 61)
        self.assertTrue(metadata["frame_count_matches_expected"])

    def test_missing_known_frames_abort_and_remove_short_video(self):
        self.recorder.abort("replace unknown-count fixture")
        self.recorder = VideoRecorder(SimpleNamespace(), self.path, width=2, height=2,
                                      expected_frames=2)
        self.recorder.capture(self.render, 0.0)
        with self.assertRaisesRegex(RuntimeError, "expected exactly 2"):
            self.recorder.finish()
        self.assertFalse(self.path.exists())
        marker = json.loads(self.path.with_suffix(".aborted.json").read_text())
        self.assertFalse(marker["frame_count_matches_expected"])

    def test_resolution_change_aborts_instead_of_corrupting_stream(self):
        def changed_resolution():
            self.pending.pop(0)(bytes(32), 32, 4, 2, "RGBA8_UNORM")
        with self.assertRaisesRegex(RuntimeError, "resolution changed"):
            self.recorder.capture(changed_resolution, 0.0)
        self.assertFalse(self.path.exists())

    def test_padded_bgra_scanlines_become_packed_rgba(self):
        pixel = bytes([30, 20, 10, 255])
        padded = (pixel * 2 + bytes(4)) * 2
        self.assertEqual(_rgba_bytes(padded, 24, 2, 2, "BGRA8_SRGB"),
                         bytes([10, 20, 30, 255]) * 4)

    def test_native_capsule_is_copied_before_buffer_expires(self):
        storage = (ctypes.c_ubyte * 4)(1, 2, 3, 4)
        create = ctypes.pythonapi.PyCapsule_New
        create.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p]
        create.restype = ctypes.py_object
        capsule = create(ctypes.addressof(storage), None, None)
        copied = _copy_buffer(capsule, 4)
        storage[0] = 99
        self.assertEqual(copied, bytes([1, 2, 3, 4]))


if __name__ == "__main__":
    unittest.main()
