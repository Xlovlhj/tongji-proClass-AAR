"""Container math and browser-result plumbing, without GPU/video generation."""

import base64
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raa.video_validation import inspect_mp4, save_video_frame, repair_native_mp4_sync_flags, _CDP, _wait_browser_result


def box(kind, payload=b"", *, large=False):
    if large:
        return struct.pack(">I4sQ", 1, kind.encode(), len(payload) + 16) + payload
    return struct.pack(">I4s", len(payload) + 8, kind.encode()) + payload


def full(version=0, flags=0):
    return bytes([version]) + flags.to_bytes(3, "big")


def duration_box(kind, ticks, *, version=0):
    payload = full(version) + bytes(16 if version else 8)
    payload += struct.pack(">IQ" if version else ">II", 30, ticks)
    return box(kind, payload)


def movie(*, count=3, ticks=None, fragmented=False, default_duration=1,
          version=0, codec="avc1", large=False, timing_count=None):
    ticks = count if ticks is None else ticks
    header = box("tkhd", full() + bytes(8) + struct.pack(">I", 1) + bytes(60)
                 + struct.pack(">II", 2 << 16, 2 << 16))
    handler = box("hdlr", full() + bytes(4) + b"vide" + bytes(12))
    sample = bytearray(78)
    struct.pack_into(">HH", sample, 24, 2, 2)
    sample += box("avcC", bytes([1, 100, 0, 31, 255]) + bytes(2))
    description = box("stsd", full() + struct.pack(">I", 1) + box(codec, sample))
    static_count = 0 if fragmented else count
    sizes = box("stsz", full() + struct.pack(">II", 0, static_count) + struct.pack(">I", 1) * static_count)
    timed = static_count if timing_count is None else timing_count
    timing = box("stts", full() + struct.pack(">I", int(timed > 0))
                 + (struct.pack(">II", timed, 1) if timed else b""))
    table = box("minf", box("stbl", description + sizes + timing))
    track = box("trak", header + box("mdia", duration_box("mdhd", ticks, version=version) + handler + table))
    extension = box("mvex", box("trex", full() + struct.pack(">IIIII", 1, 1, default_duration, 0, 0))) if fragmented else b""
    return box("ftyp", b"isom" + bytes(4) + b"isomavc1") + box("moov", duration_box("mvhd", ticks, version=version) + track + extension, large=large)


def fragment(start, count=1, *, explicit_duration=True):
    header = box("tfhd", full(flags=0x20000) + struct.pack(">I", 1))
    decode_time = box("tfdt", full(1) + struct.pack(">Q", start))
    flags = 0x300 if explicit_duration else 0
    run = box("trun", full(flags=flags) + struct.pack(">I", count)
              + (struct.pack(">II", 1, 1) * count if explicit_duration else b""))
    return box("moof", box("mfhd", full() + struct.pack(">I", start + 1))
               + box("traf", header + decode_time + run)) + box("mdat", b"x" * count)


def native_fragment(start, *, idr=False, bad_nal=False, count=1):
    nal = bytes([0x65 if idr else 0x41, 0x80])
    encoded = struct.pack(">I", 999 if bad_nal else len(nal)) + nal
    header = box("tfhd", full(flags=0x20000) + struct.pack(">I", 1))
    decode_time = box("tfdt", full(1) + struct.pack(">Q", start))
    def make(offset):
        run = box("trun", full(flags=0x305) + struct.pack(">IiIII", count, offset, 0x02000000, 1, len(encoded)))
        return box("moof", box("mfhd", full() + struct.pack(">I", start + 1))
                   + box("traf", header + decode_time + run))
    fragment = make(0)
    return make(len(fragment) + 8) + box("mdat", encoded)


def png():
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 2, 2, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(bytes(14))) + chunk(b"IEND", b"")


def server_frame(data, opcode=1, final=True):
    length = len(data)
    marker = length if length < 126 else 126 if length < 65536 else 127
    return bytes([(128 if final else 0) | opcode, marker]) + (
        b"" if length < 126 else struct.pack(">H" if length < 65536 else ">Q", length)) + data


class VideoValidationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="raa-video-inspection-test-")
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "sample.mp4"

    def write(self, data, *, sidecar=False):
        self.path.write_bytes(data)
        if sidecar:
            self.path.with_suffix(".recording.json").write_text(json.dumps({
                "completed": True, "frame_count": 3, "duration_s": .1, "fps": 30,
                "resolution": [2, 2], "codec": "H.264", "file_size_bytes": len(data)}))
        return inspect_mp4(self.path)

    def test_classic_stsz_timing_resolution_codec_and_sidecar(self):
        report = self.write(movie() + box("mdat", b"xxx"), sidecar=True)
        self.assertTrue(report["validated"])
        self.assertTrue(report["recording_consistent"])
        self.assertFalse(report["browser_verified"])
        self.assertEqual(report["frame_count"], 3)
        self.assertEqual(report["frame_count_source"], "stsz")
        self.assertEqual(report["codec_string"], "avc1.64001f")
        self.assertEqual(report["resolution"], [2, 2])
        self.assertAlmostEqual(report["duration_s"], .1)
        self.assertEqual(report["fps"], 30)

    def test_fragmented_native_unknown_header_uses_real_trun_timeline(self):
        data = movie(fragmented=True, ticks=0xffffffff) + b"".join(fragment(i) for i in range(3))
        report = self.write(data, sidecar=True)
        self.assertTrue(report["validated"])
        self.assertTrue(report["recording_consistent"])
        self.assertEqual(report["frame_count"], 3)
        self.assertEqual(report["frame_count_source"], "trun")
        self.assertIsNone(report["movie_duration_s"])
        self.assertIsNone(report["media_duration_s"])
        self.assertAlmostEqual(report["duration_s"], .1)
        self.assertEqual(len(report["warnings"]), 2)

    def test_fragmented_fixed_header_and_default_trex_duration(self):
        data = movie(fragmented=True) + fragment(0, 3, explicit_duration=False)
        report = self.write(data, sidecar=True)
        self.assertTrue(report["validated"])
        self.assertEqual(report["movie_duration_s"], .1)
        self.assertEqual(report["warnings"], [])
        self.assertEqual(report["tracks"][0]["fragment_count"], 1)

    def test_version_one_header_and_large_box_sizes(self):
        report = self.write(movie(version=1, large=True) + box("mdat", b"xxx", large=True))
        self.assertTrue(report["validated"])
        self.assertAlmostEqual(report["movie_duration_s"], .1)

    def test_sidecar_mismatch_is_reported(self):
        self.write(movie() + box("mdat", b"xxx"), sidecar=True)
        sidecar = self.path.with_suffix(".recording.json")
        state = json.loads(sidecar.read_text())
        state["frame_count"] = 4
        state["resolution"] = [3, 2]
        state["completed"] = False
        sidecar.write_text(json.dumps(state))
        report = inspect_mp4(self.path)
        self.assertFalse(report["validated"])
        self.assertFalse(report["recording_consistent"])
        self.assertEqual(len(report["recording_differences"]), 3)

    def test_native_sync_repair_uses_nal_idr_and_preserves_all_media_bytes(self):
        data = movie(fragmented=True) + native_fragment(0, idr=True) + native_fragment(1) + native_fragment(2)
        self.write(data, sidecar=True)
        report = repair_native_mp4_sync_flags(self.path)
        self.assertEqual(report["sample_count"], 3)
        self.assertEqual(report["idr_samples"], 1)
        self.assertEqual(report["p_samples"], 2)
        self.assertEqual(report["patched_samples"], 2)
        self.assertEqual(report["metadata_bytes_changed"], 4)
        self.assertTrue(report["media_payload_unchanged"])
        self.assertEqual(report["media_sha256_before"], report["media_sha256_after"])
        before_idr = struct.pack(">I", 2) + b"\x65\x80"
        before_p = struct.pack(">I", 2) + b"\x41\x80"
        self.assertEqual(report["media_sha256_before"], hashlib.sha256(before_idr + before_p * 2).hexdigest())
        self.assertTrue(inspect_mp4(self.path)["validated"])
        repaired = self.path.read_bytes()
        second = repair_native_mp4_sync_flags(self.path)
        self.assertEqual(second["patched_samples"], 0)
        self.assertEqual(second["metadata_bytes_changed"], 0)
        self.assertEqual(self.path.read_bytes(), repaired)

    def test_native_repair_refuses_bad_formats_before_mutating_any_bytes(self):
        valid = movie(fragmented=True) + native_fragment(0, idr=True) + native_fragment(1)
        for data in (valid + native_fragment(2, bad_nal=True),
                     valid + box("sidx"), valid + box("mfra"),
                     movie() + box("mdat", b"xxx"),
                     movie(fragmented=True) + native_fragment(0),
                     movie(fragmented=True) + native_fragment(0, idr=True, count=2)):
            with self.subTest(length=len(data)):
                self.path.write_bytes(data)
                with self.assertRaises(ValueError):
                    repair_native_mp4_sync_flags(self.path)
                self.assertEqual(self.path.read_bytes(), data)

    def test_disagreeing_headers_overlap_and_codec_are_rejected(self):
        report = self.write(movie(fragmented=True, ticks=30) + fragment(0) + fragment(0))
        self.assertFalse(report["validated"])
        self.assertTrue(any("overlap" in error for error in report["errors"]))
        self.assertTrue(any("timeline disagree" in error for error in report["errors"]))
        report = self.write(movie(codec="mp4v") + box("mdat", b"xxx"))
        self.assertFalse(report["validated"])
        self.assertIn("Expected H.264 avc1", report["errors"][0])

    def test_truncated_boxes_and_missing_fragment_duration_raise(self):
        for data in (b"abc", movie()[:-1], movie(timing_count=2),
                     movie(fragmented=True, default_duration=0) + fragment(0, explicit_duration=False)):
            with self.subTest(length=len(data)):
                self.path.write_bytes(data)
                with self.assertRaises(ValueError):
                    inspect_mp4(self.path)

    def browser_state(self, **changes):
        state = {"status": "decoded", "requested_time_s": .05, "current_time_s": .05,
                 "frame_media_time_s": .05,
                 "duration_s": .1, "resolution": [2, 2], "ready_state": 4,
                 "playback_started": True, "can_play_h264": "probably",
                 "data_url": "data:image/png;base64," + base64.b64encode(png()).decode()}
        state.update(changes)
        return state

    def test_browser_result_exports_decoded_png_and_requests_cpu_only_chrome(self):
        self.write(movie() + box("mdat", b"xxx"))
        output = self.path.with_suffix(".png")
        with patch("raa.video_validation._wait_browser_result", return_value=self.browser_state()) as run:
            report = save_video_frame(self.path, .05, output, chrome_binary="chrome")
        self.assertTrue(report["browser_verified"])
        self.assertEqual(output.read_bytes(), png())
        command = run.call_args.args[0]
        self.assertIn("--headless", command)
        self.assertIn("--disable-gpu", command)
        self.assertIn("--no-sandbox", command)
        self.assertIn("--remote-debugging-port=0", command)
        self.assertNotIn("--dump-dom", command)
        self.assertTrue(command[-1].startswith("file://"))

    def test_browser_wrong_duration_or_seek_cannot_be_claimed_verified(self):
        self.write(movie() + box("mdat", b"xxx"))
        output = self.path.with_suffix(".png")
        for changes in ({"duration_s": 143165576.5}, {"current_time_s": 20},
                        {"frame_media_time_s": 2.0},
                        {"playback_started": False}, {"resolution": [3, 2]},
                        {"status": "error", "message": "media error 4"}):
            with self.subTest(changes=changes):
                with patch("raa.video_validation._wait_browser_result", return_value=self.browser_state(**changes)):
                    with self.assertRaises(RuntimeError):
                        save_video_frame(self.path, .05, output, chrome_binary="chrome")
                self.assertFalse(output.exists())

    def test_seek_time_must_be_finite_nonnegative_and_in_media_timeline(self):
        self.write(movie() + box("mdat", b"xxx"))
        for time in (True, -1, float("inf"), float("nan"), .1):
            with self.subTest(time=time), self.assertRaises(ValueError):
                save_video_frame(self.path, time, self.path.with_suffix(".png"))

    def test_cdp_masks_client_frames_handles_ping_events_and_large_fragmented_reply(self):
        reply = json.dumps({"id": 1, "result": {"value": "x" * 66000}}).encode()
        traffic = (server_frame(json.dumps({"method": "event"}).encode())
                   + server_frame(b"ping", opcode=9)
                   + server_frame(reply[:100], final=False)
                   + server_frame(reply[100:], opcode=0))

        class Connection:
            def __init__(self):
                self.received = b""
                self.sent = []
                self.closed = False

            def sendall(self, data):
                self.sent.append(data)
                if data.startswith(b"GET"):
                    key = next(line.split(b": ")[1] for line in data.split(b"\r\n") if line.startswith(b"Sec-WebSocket-Key:"))
                    accept = base64.b64encode(hashlib.sha1(key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest())
                    self.received = b"HTTP/1.1 101 Switching Protocols\r\nSec-WebSocket-Accept: " + accept + b"\r\n\r\n" + traffic

            def recv(self, size):
                value, self.received = self.received[:size], self.received[size:]
                return value

            def close(self):
                self.closed = True

        connection = Connection()
        with patch("raa.video_validation.socket.create_connection", return_value=connection):
            client = _CDP("ws://127.0.0.1:1234/devtools/page/1", 1)
            result = client.call("Runtime.evaluate", {"expression": "x" * 130})
            client.close()
        self.assertEqual(result["value"], "x" * 66000)
        request = connection.sent[1]
        self.assertEqual(request[1] & 128, 128)
        self.assertEqual(request[1] & 127, 126)
        length = int.from_bytes(request[2:4], "big")
        mask, data = request[4:8], request[8:]
        decoded = bytes(value ^ mask[index % 4] for index, value in enumerate(data))
        self.assertEqual(len(decoded), length)
        self.assertEqual(json.loads(decoded)["method"], "Runtime.evaluate")
        self.assertEqual(connection.sent[2][0] & 15, 10, "Reply to server ping is a pong")
        self.assertTrue(connection.closed)

    def test_real_time_browser_poll_waits_past_starting_state(self):
        profile = self.path.parent / "profile"
        profile.mkdir()
        (profile / "DevToolsActivePort").write_text("1234\n/devtools/browser/1\n")
        target = [{"type": "page", "webSocketDebuggerUrl": "ws://127.0.0.1:1234/devtools/page/1"}]
        decoded = self.browser_state()
        with patch("raa.video_validation.subprocess.Popen") as launch, \
             patch("raa.video_validation.urlopen", return_value=io.BytesIO(json.dumps(target).encode())), \
             patch("raa.video_validation._CDP") as socket_client, \
             patch("raa.video_validation.time.sleep") as pause:
            launch.return_value.poll.return_value = None
            client = socket_client.return_value
            client.call.side_effect = [{"result": {"value": {"status": "starting"}}},
                                       {"result": {"value": decoded}}, {}]
            self.assertEqual(_wait_browser_result(["chrome"], profile, 1), decoded)
            pause.assert_called_once_with(.1)
            self.assertEqual(client.call.call_args_list[-1].args[0], "Browser.close")
            client.close.assert_called_once()

    def test_cdp_debugging_connection_is_local(self):
        with self.assertRaises(ValueError):
            _CDP("ws://remote.example:1234/devtools/page/1", 1)


if __name__ == "__main__":
    unittest.main()
