"""Stream Isaac viewport frames to the locally bundled H.264 encoder.

The caller owns the simulation clock and requests a frame at k / fps. Capture
waits for GPU readback and encoding before returning, rendering the same held
physical state while it waits. No physics stepping or controller update lives
in this module. Isaac imports are delayed until a recorder is constructed.
"""

from __future__ import annotations

import ctypes
import json
import math
from pathlib import Path
import threading
import time
from typing import Callable


def _copy_buffer(buffer, size: int) -> bytes:
    """Copy while the renderer callback still owns its native allocation."""
    try:
        view = memoryview(buffer)
    except TypeError:
        get_name = ctypes.pythonapi.PyCapsule_GetName
        get_name.argtypes = [ctypes.py_object]
        get_name.restype = ctypes.c_char_p
        get_pointer = ctypes.pythonapi.PyCapsule_GetPointer
        get_pointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
        get_pointer.restype = ctypes.c_void_p
        pointer = get_pointer(buffer, get_name(buffer))
        if not pointer:
            raise RuntimeError("Viewport returned an empty capture capsule")
        return ctypes.string_at(pointer, size)
    else:
        copied = view.tobytes()
        if len(copied) < size:
            raise RuntimeError("Viewport buffer is smaller than its reported size")
        return copied[:size]


def _rgba_bytes(buffer, size: int, width: int, height: int, byte_format) -> bytes:
    """Pack an RGBA8/BGRA8 capture, including any padded scanlines."""
    name = getattr(byte_format, "name", str(byte_format).rsplit(".", 1)[-1])
    if name not in {"RGBA8_UNORM", "RGBA8_SRGB", "BGRA8_UNORM", "BGRA8_SRGB"}:
        raise RuntimeError(f"Unsupported viewport capture format: {byte_format}")
    row_size = width * 4
    expected = row_size * height
    if size < expected or size % height:
        raise RuntimeError(f"Invalid capture buffer size {size} for {width}x{height} RGBA8")
    data = _copy_buffer(buffer, size)
    stride = size // height
    if stride != row_size:
        data = b"".join(data[y * stride:y * stride + row_size] for y in range(height))
    if name.startswith("BGRA"):
        packed = bytearray(data)
        red, blue = packed[0::4], packed[2::4]
        packed[0::4], packed[2::4] = blue, red
        data = bytes(packed)
    return data


class VideoRecorder:
    """One synchronous native-encoder session for an Isaac viewport.

    ``capture`` must receive a renderer that holds physical time fixed (for
    example World.render). Call it at the desired simulation-time frame grid;
    slower readback extends wall time, rather than dropping or duplicating
    frames. The encoder has one process-wide session, so use recorders serially.
    """

    def __init__(self, viewport, path, fps=30, width=960, height=720, expected_frames: int | None = None):
        if viewport is None:
            raise ValueError("Video recording requires an available viewport")
        if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0:
            raise ValueError("Video fps must be a finite positive number")
        for name, value in (("width", width), ("height", height)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0 or value % 2:
                raise ValueError(f"Video {name} must be a positive even integer")
        if (expected_frames is not None and (isinstance(expected_frames, bool)
                or not isinstance(expected_frames, int) or expected_frames <= 0)):
            raise ValueError("expected_frames must be a positive integer or None")
        self.path = Path(path).expanduser().resolve()
        if self.path.suffix.lower() != ".mp4":
            raise ValueError("Video output must have an .mp4 extension")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.viewport = viewport
        self.fps, self.width, self.height = float(fps), width, height
        self.expected_frames = expected_frames
        self.frame_count = 0
        self.motion_frame_count = 0
        self.hold_frame_count = 0
        self._last_rgba = None
        self._times: list[float] = []
        self._render_updates = 0
        self._pending = False
        self._closed = False
        self._metadata = None
        self._settings = None
        self._saved_settings = {}
        self._encoder_started = False

        from isaacsim.core.utils.extensions import enable_extension
        if not enable_extension("omni.videoencoding"):
            raise RuntimeError("Could not enable the bundled omni.videoencoding extension")
        from video_encoding import get_video_encoding_interface
        from omni.kit.viewport.utility import capture_viewport_to_buffer
        import carb.settings

        self._schedule_capture = capture_viewport_to_buffer
        self._encoder = get_video_encoding_interface()
        if self._encoder is None:
            raise RuntimeError("The native video encoding interface is unavailable")
        self._settings = carb.settings.get_settings()
        settings = {
            "/exts/omni.videoencoding/maxEncodeWidth": width,
            "/exts/omni.videoencoding/maxEncodeHeight": height,
            "/exts/omni.videoencoding/bitrate": 8_000_000,
            "/exts/omni.videoencoding/profile": "H264_PROFILE_HIGH",
        }
        for name, value in settings.items():
            self._saved_settings[name] = self._settings.get(name)
            self._settings.set(name, value)
        viewport.fill_frame = False
        viewport.resolution_scale = 1.0
        viewport.resolution = (width, height)
        try:
            # A known count makes native MP4 headers carry finite durations.
            # Zero is the documented streaming/unknown-length fallback.
            if not self._encoder.start_encoding(str(self.path), self.fps, expected_frames or 0, True):
                raise RuntimeError("Native H.264 encoder could not start recording")
            self._encoder_started = True
            self.path.with_suffix(".aborted.json").unlink(missing_ok=True)
        except Exception:
            self._restore_settings()
            raise

    @property
    def last_sim_time_s(self) -> float | None:
        """Last captured motion time; final hold never changes it."""
        return self._times[-1] if self._times else None

    def capture(self, render_callable: Callable[[], None], sim_time_s: float) -> None:
        """Encode exactly one requested frame; no state advances while waiting."""
        if self._closed or self._pending:
            raise RuntimeError("Recorder is closed or another capture is pending")
        if (isinstance(sim_time_s, bool) or not isinstance(sim_time_s, (int, float))
                or not math.isfinite(sim_time_s) or sim_time_s < 0):
            raise ValueError("Capture simulation time must be finite and nonnegative")
        sim_time_s = float(sim_time_s)
        if self._times and sim_time_s <= self._times[-1]:
            raise ValueError("Capture simulation times must be strictly increasing")
        done = threading.Event()
        failure: list[Exception] = []
        self._pending = True

        def on_capture(buffer, buffer_size, width, height, byte_format):
            try:
                if self._closed:
                    raise RuntimeError("Recording was aborted before readback completed")
                if (width, height) != (self.width, self.height):
                    raise RuntimeError(f"Capture resolution changed to {width}x{height}; "
                                       f"expected {self.width}x{self.height}")
                rgba = _rgba_bytes(buffer, int(buffer_size), width, height, byte_format)
                if not self._encoder.encode_next_frame_from_buffer(rgba, width, height):
                    raise RuntimeError("Native encoder rejected a viewport frame")
                self._last_rgba = rgba
            except Exception as exc:
                failure.append(exc)
            finally:
                # The delegate's own future completes when readback is scheduled;
                # this latch instead means the callback AND encoding completed.
                done.set()

        try:
            helper = self._schedule_capture(self.viewport, on_capture, is_hdr=False)
            if helper is None:
                raise RuntimeError("Viewport refused to schedule a frame capture")
            deadline = time.monotonic() + 30.0
            updates = 0
            while not done.is_set():
                if updates >= 240 or time.monotonic() >= deadline:
                    raise RuntimeError("Viewport frame readback timed out")
                render_callable()
                updates += 1
                self._render_updates += 1
            if failure:
                raise failure[0]
            self._times.append(sim_time_s)
            self.frame_count += 1
            self.motion_frame_count += 1
        except Exception as exc:
            self.abort(str(exc))
            raise
        finally:
            self._pending = False

    def hold_final(self, seconds: float) -> None:
        """Repeat the final image for viewing, preserving its source sim time."""
        if self._closed or self._pending or self._last_rgba is None:
            raise RuntimeError("Final hold requires an open recorder with a completed frame")
        if (isinstance(seconds, bool) or not isinstance(seconds, (int, float))
                or not math.isfinite(seconds) or seconds < 0):
            raise ValueError("Final hold seconds must be finite and nonnegative")
        try:
            for _ in range(round(self.fps * seconds)):
                if not self._encoder.encode_next_frame_from_buffer(self._last_rgba, self.width, self.height):
                    raise RuntimeError("Native encoder rejected a final hold frame")
                self.hold_frame_count += 1
                self.frame_count += 1
        except Exception as exc:
            self.abort(str(exc))
            raise

    def _summary(self, completed: bool) -> dict:
        return {
            "path": str(self.path), "completed": completed,
            "frame_count": self.frame_count, "fps": self.fps,
            "expected_frame_count": self.expected_frames,
            "frame_count_matches_expected": (self.frame_count == self.expected_frames
                if self.expected_frames is not None else None),
            "duration_s": self.frame_count / self.fps,
            "motion_frame_count": self.motion_frame_count,
            "hold_frame_count": self.hold_frame_count,
            "hold_seconds": self.hold_frame_count / self.fps,
            "resolution": [self.width, self.height], "codec": "H.264",
            "container": "MP4", "encoder": "Isaac omni.videoencoding",
            "dropped_frames": 0, "duplicated_motion_frames": 0,
            "repeated_final_frames": self.hold_frame_count,
            "no_dropped_frames": bool(completed),
            "first_sim_time_s": self._times[0] if self._times else None,
            "last_sim_time_s": self._times[-1] if self._times else None,
            "source_duration_s": self._times[-1] if self._times else 0.0,
            "simulation_time_span_s": self._times[-1] - self._times[0] if self._times else 0.0,
            "capture_render_updates": self._render_updates,
            "frame_timing": "one held physical state per requested simulation-time sample",
        }

    def _restore_settings(self) -> None:
        if self._settings is not None:
            for name, value in self._saved_settings.items():
                if value is not None:
                    self._settings.set(name, value)

    def finish(self) -> dict:
        """Flush, repair native sync flags without re-encoding, and save metadata."""
        if self._closed:
            if self._metadata and self._metadata["completed"]:
                return dict(self._metadata)
            raise RuntimeError("An aborted recorder cannot be finished")
        if self._pending or self.frame_count == 0:
            self.abort("No completed frames or capture still pending")
            raise RuntimeError("Cannot finish a video without completed frames")
        if self.expected_frames is not None and self.frame_count != self.expected_frames:
            reason = (f"Recorded {self.frame_count} frames; expected exactly "
                      f"{self.expected_frames}")
            self.abort(reason)
            raise RuntimeError(reason)
        try:
            self._encoder.finalize_encoding()
            self._encoder_started = False
            if not self.path.is_file() or self.path.stat().st_size == 0:
                raise RuntimeError("Native encoder did not produce a nonempty MP4 file")
            # Native fragmented MP4s incorrectly mark predictive frames as
            # independently decodable. Correct only container sync flags so
            # playback can seek to genuine H.264 IDR frames; payload stays exact.
            from .video_validation import repair_native_mp4_sync_flags
            sync_report = repair_native_mp4_sync_flags(self.path)
            self._metadata = self._summary(True)
            self._metadata["mp4_sync_flags"] = sync_report
            self._metadata["file_size_bytes"] = self.path.stat().st_size
            self.path.with_suffix(".recording.json").write_text(
                json.dumps(self._metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                encoding="utf-8")
            self._closed = True
            return dict(self._metadata)
        except Exception as exc:
            self.abort(str(exc))
            raise
        finally:
            self._restore_settings()

    def abort(self, reason: str = "Recording aborted") -> dict:
        """Close native resources and remove the unfinished MP4, leaving a marker."""
        if self._closed:
            return dict(self._metadata or self._summary(False))
        errors = []
        try:
            if self._encoder_started:
                self._encoder.finalize_encoding()
        except Exception as exc:
            errors.append(str(exc))
        finally:
            self._encoder_started = False
            self._closed = True
            self._restore_settings()
        self.path.unlink(missing_ok=True)
        self.path.with_suffix(".recording.json").unlink(missing_ok=True)
        self._metadata = self._summary(False)
        self._metadata["reason"] = str(reason)
        if errors:
            self._metadata["cleanup_errors"] = errors
        self.path.with_suffix(".aborted.json").write_text(
            json.dumps(self._metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8")
        return dict(self._metadata)
