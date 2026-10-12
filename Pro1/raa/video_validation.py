"""Inspect real MP4 tracks and decode a selected frame with CPU-only Chrome.

This module never imports Isaac, NumPy, a video encoder, or a graphics engine.
MP4 inspection seeks over mdat payloads; frame export decodes the supplied movie
in a browser and reads its canvas pixels. No substitute animation is generated.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import tempfile
import time
from typing import BinaryIO
from urllib.parse import urlsplit
from urllib.request import urlopen


@dataclass(frozen=True)
class _Box:
    type: str
    start: int
    size: int
    payload: int

    @property
    def end(self):
        return self.start + self.size


def _boxes(stream: BinaryIO, start: int, end: int):
    position = start
    while position < end:
        if end - position < 8:
            raise ValueError(f"Truncated MP4 box header at byte {position}")
        stream.seek(position)
        size, kind = struct.unpack(">I4s", stream.read(8))
        header = 8
        if size == 1:
            if end - position < 16:
                raise ValueError("Truncated 64-bit MP4 box size")
            size = struct.unpack(">Q", stream.read(8))[0]
            header = 16
        elif size == 0:
            size = end - position
        if size < header or position + size > end:
            raise ValueError(f"Invalid MP4 box size at byte {position}")
        yield _Box(kind.decode("ascii", errors="replace"), position, size, position + header)
        position += size


def _children(stream, box):
    return list(_boxes(stream, box.payload, box.end))


def _find(boxes, name, *, required=True):
    matches = [box for box in boxes if box.type == name]
    if not matches:
        if required:
            raise ValueError(f"Required MP4 box {name} is missing")
        return None
    return matches[0]


def _read(stream, box, offset, count):
    if offset < 0 or box.payload + offset + count > box.end:
        raise ValueError(f"Truncated MP4 {box.type} data")
    stream.seek(box.payload + offset)
    data = stream.read(count)
    if len(data) != count:
        raise ValueError(f"Truncated MP4 {box.type} data")
    return data


def _duration(stream, box):
    version = _read(stream, box, 0, 1)[0]
    if version == 0:
        timescale, duration = struct.unpack(">II", _read(stream, box, 12, 8))
    elif version == 1:
        timescale, duration = struct.unpack(">IQ", _read(stream, box, 20, 12))
    else:
        raise ValueError(f"Unsupported {box.type} version {version}")
    if timescale <= 0:
        raise ValueError(f"Invalid {box.type} timescale")
    unknown = (1 << (32 if version == 0 else 64)) - 1
    return timescale, None if duration == unknown else duration / timescale


def _track(stream, box):
    children = _children(stream, box)
    header = _find(children, "tkhd")
    version = _read(stream, header, 0, 1)[0]
    track_id = struct.unpack(">I", _read(stream, header, 12 if version == 0 else 20, 4))[0]
    width, height = struct.unpack(">II", _read(stream, header, header.end - header.payload - 8, 8))
    media = _find(children, "mdia")
    media_children = _children(stream, media)
    handler = _read(stream, _find(media_children, "hdlr"), 8, 4).decode("ascii", errors="replace")
    timescale, duration = _duration(stream, _find(media_children, "mdhd"))
    result = {"track_id": track_id, "handler": handler, "timescale": timescale,
              "duration_s": duration, "media_header_duration_s": duration,
              "display_resolution": [width / 65536, height / 65536]}
    if handler != "vide":
        return result
    minimum = _find(media_children, "minf")
    table = _find(_children(stream, minimum), "stbl")
    entries = _children(stream, table)
    sizes = _find(entries, "stsz")
    sample_size, count = struct.unpack(">II", _read(stream, sizes, 4, 8))
    if sample_size == 0 and sizes.end - sizes.payload < 12 + count * 4:
        raise ValueError("Truncated stsz sample size table")
    result.update(frame_count=count, stsz_frame_count=count, frame_count_source="stsz")
    description = _find(entries, "stsd")
    entry_count = struct.unpack(">I", _read(stream, description, 4, 4))[0]
    descriptions = list(_boxes(stream, description.payload + 8, description.end))
    if entry_count != len(descriptions) or not descriptions:
        raise ValueError("Invalid stsd sample description count")
    codecs = []
    for sample in descriptions:
        codec = sample.type
        sample_width, sample_height = struct.unpack(">HH", _read(stream, sample, 24, 4))
        codecs.append(codec)
        if "codec" not in result:
            result.update(codec=codec, resolution=[sample_width, sample_height])
        if codec in {"avc1", "avc3"}:
            extras = list(_boxes(stream, sample.payload + 78, sample.end))
            configuration = _find(extras, "avcC")
            data = _read(stream, configuration, 0, 5)
            if data[0] != 1:
                raise ValueError("Unsupported avcC configuration version")
            if "codec_string" not in result:
                result["codec_string"] = f"{codec}.{data[1]:02x}{data[2]:02x}{data[3]:02x}"
                result["nal_length_size"] = (data[4] & 3) + 1
    result["sample_codecs"] = codecs
    timing = _find(entries, "stts", required=False)
    if timing:
        timing_count = struct.unpack(">I", _read(stream, timing, 4, 4))[0]
        ticks = 0
        timed_samples = 0
        for i in range(timing_count):
            samples, delta = struct.unpack(">II", _read(stream, timing, 8 + i * 8, 8))
            timed_samples += samples
            ticks += samples * delta
        if timed_samples != count:
            raise ValueError("stts and stsz frame counts disagree")
        result["sample_duration_s"] = ticks / timescale
    return result


def _fragment_defaults(stream, movie_children):
    extension = _find(movie_children, "mvex", required=False)
    defaults = {}
    if extension:
        for box in _children(stream, extension):
            if box.type == "trex":
                track_id, _, duration, size, _ = struct.unpack(">IIIII", _read(stream, box, 4, 20))
                defaults[track_id] = {"duration": duration, "size": size}
    return defaults


def _fragment_samples(stream, top, defaults, tracks):
    """Read fMP4 decode timing and counts without reading encoded mdat bytes."""
    by_id = {track["track_id"]: track for track in tracks}
    totals = {}
    for movie_fragment in top:
        if movie_fragment.type != "moof":
            continue
        for fragment in _children(stream, movie_fragment):
            if fragment.type != "traf":
                continue
            boxes = _children(stream, fragment)
            header = _find(boxes, "tfhd")
            flags = int.from_bytes(_read(stream, header, 1, 3), "big")
            track_id = struct.unpack(">I", _read(stream, header, 4, 4))[0]
            if track_id not in by_id:
                raise ValueError(f"Fragment references missing track {track_id}")
            default = dict(defaults.get(track_id, {"duration": 0, "size": 0}))
            cursor = 8
            for mask, length, key in ((1, 8, None), (2, 4, None), (8, 4, "duration"),
                                      (16, 4, "size"), (32, 4, None)):
                if flags & mask:
                    data = _read(stream, header, cursor, length)
                    if key:
                        default[key] = int.from_bytes(data, "big")
                    cursor += length
            total = totals.setdefault(track_id, {"frame_count": 0, "ticks": 0,
                "end_ticks": 0, "first_ticks": None, "fragment_count": 0,
                "gaps_ticks": 0, "overlaps_ticks": 0})
            decode_time = _find(boxes, "tfdt", required=False)
            start = total["end_ticks"]
            if decode_time:
                version = _read(stream, decode_time, 0, 1)[0]
                if version not in (0, 1):
                    raise ValueError(f"Unsupported tfdt version {version}")
                start = int.from_bytes(_read(stream, decode_time, 4, 8 if version else 4), "big")
            if total["fragment_count"]:
                delta = start - total["end_ticks"]
                total["gaps_ticks"] += max(delta, 0)
                total["overlaps_ticks"] += max(-delta, 0)
            else:
                total["first_ticks"] = start
            end = start
            total["fragment_count"] += 1
            for run in boxes:
                if run.type != "trun":
                    continue
                run_flags = int.from_bytes(_read(stream, run, 1, 3), "big")
                count = struct.unpack(">I", _read(stream, run, 4, 4))[0]
                cursor = 8
                if run_flags & 1:
                    _read(stream, run, cursor, 4)
                    cursor += 4
                if run_flags & 4:
                    _read(stream, run, cursor, 4)
                    cursor += 4
                fields = [mask for mask in (256, 512, 1024, 2048) if run_flags & mask]
                if cursor + count * 4 * len(fields) > run.end - run.payload:
                    raise ValueError("Truncated trun sample table")
                if 256 not in fields:
                    if count and default["duration"] <= 0:
                        raise ValueError("Fragment sample duration is missing or zero")
                    total["ticks"] += count * default["duration"]
                    end += count * default["duration"]
                    total["frame_count"] += count
                    continue
                for _ in range(count):
                    duration = default["duration"]
                    for mask in fields:
                        value = int.from_bytes(_read(stream, run, cursor, 4), "big")
                        cursor += 4
                        if mask == 256:
                            duration = value
                    if duration <= 0:
                        raise ValueError("Fragment sample duration is missing or zero")
                    total["ticks"] += duration
                    end += duration
                total["frame_count"] += count
            total["end_ticks"] = end
    for track_id, total in totals.items():
        track = by_id[track_id]
        scale = track["timescale"]
        initial_duration = track.get("sample_duration_s", 0)
        track.update(fragment_frame_count=total["frame_count"], fragment_count=total["fragment_count"],
            frame_count=track.get("stsz_frame_count", 0) + total["frame_count"],
            frame_count_source="stsz+trun" if track.get("stsz_frame_count") else "trun",
            sample_duration_s=initial_duration + total["ticks"] / scale,
            fragment_start_s=total["first_ticks"] / scale,
            fragment_end_s=total["end_ticks"] / scale,
            fragment_gaps_s=total["gaps_ticks"] / scale,
            fragment_overlaps_s=total["overlaps_ticks"] / scale,
            duration_s=max(initial_duration, total["end_ticks"] / scale))


def inspect_mp4(path: str | Path) -> dict:
    """Read H.264/avc1, resolution and classic or fragmented MP4 sample timing.

    When a same-name .recording.json exists, ``recording_consistent`` reports
    checks against frame_count/fps/duration/resolution/file_size_bytes. Missing
    sidecars are reported as None, never claimed as a successful consistency
    check. Malformed boxes raise ValueError; content mismatches appear in errors.
    """
    path = Path(path).expanduser().resolve()
    size = path.stat().st_size
    with path.open("rb") as stream:
        top = list(_boxes(stream, 0, size))
        movie = _find(top, "moov")
        children = _children(stream, movie)
        movie_scale, movie_duration = _duration(stream, _find(children, "mvhd"))
        tracks = [_track(stream, box) for box in children if box.type == "trak"]
        _fragment_samples(stream, top, _fragment_defaults(stream, children), tracks)
    video = next((track for track in tracks if track["handler"] == "vide"), None)
    if video is None:
        raise ValueError("MP4 contains no video track")
    errors = []
    warnings = []
    if video["codec"] != "avc1":
        errors.append(f"Expected H.264 avc1, found {video['codec']}")
    if video["frame_count"] <= 0:
        errors.append("Video contains no stsz/trun frames")
    if min(video["resolution"]) <= 0:
        errors.append("Video resolution must be positive")
    tick = 1 / video["timescale"]
    sample_duration = video.get("sample_duration_s", video["duration_s"]) or 0
    timeline_duration = video.get("fragment_end_s", sample_duration)
    if timeline_duration <= 0 or sample_duration <= 0:
        errors.append("Video sample duration must be positive")
    video["fps"] = video["frame_count"] / sample_duration if sample_duration > 0 else 0
    header_duration = video["media_header_duration_s"]
    if header_duration is None:
        warnings.append("mdhd duration is unknown; browser duration requires actual decoding verification")
    elif abs(timeline_duration - header_duration) > tick + 1e-8:
        errors.append("mdhd duration and sample timeline disagree")
    if movie_duration is None:
        warnings.append("mvhd duration is unknown; using sample timeline for inspection")
    elif abs(timeline_duration - movie_duration) > max(tick, 1 / movie_scale) + 1e-8:
        errors.append("mvhd duration and video sample timeline disagree")
    if video.get("fragment_overlaps_s", 0) > 0:
        errors.append("Fragment decode timestamps overlap")
    if video.get("fragment_gaps_s", 0) > 0:
        warnings.append("Fragment decode timeline has gaps")
    differences = []
    sidecar = path.with_suffix(".recording.json")
    recording = None
    if sidecar.is_file():
        recording = json.loads(sidecar.read_text(encoding="utf-8"))
        if not isinstance(recording, dict):
            raise ValueError("Recording sidecar must be a JSON object")
        if recording.get("completed") is not True:
            differences.append("Recording is not marked completed")
        for name, actual in (("frame_count", video["frame_count"]), ("resolution", video["resolution"]),
                             ("file_size_bytes", size)):
            if name in recording and recording[name] != actual:
                differences.append(f"{name}: sidecar={recording[name]!r}, MP4={actual!r}")
        for name, actual, tolerance in (("duration_s", timeline_duration, tick + 1e-8),
                                        ("fps", video["fps"], max(1e-6, video["fps"] * tick / max(sample_duration, tick)))):
            if name in recording:
                try:
                    value = float(recording[name])
                except (TypeError, ValueError, OverflowError):
                    value = math.nan
                if not math.isfinite(value) or abs(value - actual) > tolerance:
                    differences.append(f"{name}: sidecar={recording[name]!r}, MP4={actual!r}")
        if recording.get("codec") not in (None, "H.264", "h264", "avc1"):
            differences.append(f"Sidecar codec {recording['codec']!r} does not describe H.264")
        errors.extend(differences)
    return {"path": str(path), "container": "MP4", "file_size_bytes": size,
            "codec": video["codec"], "codec_string": video.get("codec_string"),
            "resolution": video["resolution"], "frame_count": video["frame_count"],
            "frame_count_source": video["frame_count_source"],
            "duration_s": timeline_duration, "sample_duration_s": sample_duration,
            "media_duration_s": header_duration, "movie_duration_s": movie_duration,
            "movie_timescale": movie_scale,
            "fps": video["fps"], "tracks": tracks,
            "recording_sidecar": str(sidecar) if recording is not None else None,
            "recording_consistent": not differences if recording is not None else None,
            "recording_differences": differences, "validated": not errors,
            "browser_verified": False, "errors": errors, "warnings": warnings}


def _media_digest(stream, top):
    digest = hashlib.sha256()
    size = 0
    for box in top:
        if box.type != "mdat":
            continue
        stream.seek(box.payload)
        remaining = box.end - box.payload
        size += remaining
        while remaining:
            data = stream.read(min(1_048_576, remaining))
            if not data:
                raise ValueError("Truncated MP4 media payload")
            digest.update(data)
            remaining -= len(data)
    return digest.hexdigest(), size


def repair_native_mp4_sync_flags(path: str | Path) -> dict:
    """Correct native one-sample fMP4 flags using actual H.264 IDR NAL units.

    Some native recordings label every P frame as an independent sync frame.
    Browser seeking then starts without the P frame's references and can show
    a stale or damaged frame. This function changes only the four metadata
    bytes of trun.first_sample_flags; all mdat bytes are hashed before/after.
    It validates every fragment before editing and refuses formats with sidx
    or mfra/tfra indexes that would require additional index repair.
    """
    path = Path(path).expanduser().resolve()
    with path.open("r+b") as stream:
        top = list(_boxes(stream, 0, path.stat().st_size))
        if any(box.type in {"sidx", "mfra", "tfra"} for box in top):
            raise ValueError("Native sync repair requires an MP4 without random-access indexes")
        movie = _find(top, "moov")
        movie_children = _children(stream, movie)
        tracks = [_track(stream, box) for box in movie_children if box.type == "trak"]
        if len(tracks) != 1 or tracks[0]["handler"] != "vide" or tracks[0].get("codec") != "avc1":
            raise ValueError("Native sync repair requires one avc1 video track")
        track = tracks[0]
        if track["stsz_frame_count"]:
            raise ValueError("Native sync repair requires samples only in fragments")
        prefix_size = track["nal_length_size"]
        if prefix_size not in (1, 2, 4):
            raise ValueError("Unsupported H.264 NAL length prefix")
        patches = []
        idr = 0
        predicted = 0
        samples = 0
        for index, fragment in enumerate(top):
            if fragment.type != "moof":
                continue
            tracks_in_fragment = [box for box in _children(stream, fragment) if box.type == "traf"]
            if len(tracks_in_fragment) != 1:
                raise ValueError("Native sync repair requires one traf per fragment")
            children = _children(stream, tracks_in_fragment[0])
            header = _find(children, "tfhd")
            header_flags = int.from_bytes(_read(stream, header, 1, 3), "big")
            track_id = struct.unpack(">I", _read(stream, header, 4, 4))[0]
            if track_id != track["track_id"] or not header_flags & 0x20000 or header_flags & 1:
                raise ValueError("Native fragment must use the video track and moof-relative offsets")
            runs = [box for box in children if box.type == "trun"]
            if len(runs) != 1:
                raise ValueError("Native sync repair requires one trun per fragment")
            run = runs[0]
            flags = int.from_bytes(_read(stream, run, 1, 3), "big")
            count = struct.unpack(">I", _read(stream, run, 4, 4))[0]
            if count != 1 or flags & 0x400 or flags & 0x205 != 0x205:
                raise ValueError("Native fragment requires one sample with data offset, size and first-sample flags")
            offset = struct.unpack(">i", _read(stream, run, 8, 4))[0]
            flag_offset = run.payload + 12
            old_flags = _read(stream, run, 12, 4)
            sample_size_offset = 16 + (4 if flags & 0x100 else 0)
            sample_size = struct.unpack(">I", _read(stream, run, sample_size_offset, 4))[0]
            if index + 1 >= len(top) or top[index + 1].type != "mdat":
                raise ValueError("Native fragment must be followed by its mdat")
            media = top[index + 1]
            start = fragment.start + offset
            if start != media.payload or sample_size != media.end - media.payload:
                raise ValueError("Native sample offset/size does not cover its mdat")
            stream.seek(start)
            encoded = stream.read(sample_size)
            cursor = 0
            types = set()
            while cursor < len(encoded):
                if cursor + prefix_size > len(encoded):
                    raise ValueError("Truncated H.264 NAL length prefix")
                length = int.from_bytes(encoded[cursor:cursor + prefix_size], "big")
                cursor += prefix_size
                if length <= 0 or cursor + length > len(encoded) or encoded[cursor] & 128:
                    raise ValueError("Invalid H.264 NAL payload")
                types.add(encoded[cursor] & 31)
                cursor += length
            if not types.intersection({1, 5}) or types.intersection({2, 3, 4}) or {1, 5}.issubset(types):
                raise ValueError("Native sample must contain either IDR or non-IDR H.264 slices")
            independent = 5 in types
            if samples == 0 and not independent:
                raise ValueError("Native video must start with an independent H.264 IDR frame")
            idr += int(independent)
            predicted += int(not independent)
            samples += 1
            correct_flags = struct.pack(">I", 0x02000000 if independent else 0x01010000)
            if old_flags != correct_flags:
                patches.append((flag_offset, old_flags, correct_flags))
        if not samples:
            raise ValueError("Native sync repair found no fragmented samples")
        if idr == 0:
            raise ValueError("Native video has no independent H.264 IDR frame")
        before, media_size = _media_digest(stream, top)
        for offset, _, value in patches:
            stream.seek(offset)
            stream.write(value)
        stream.flush()
        after, after_size = _media_digest(stream, top)
        if before != after or media_size != after_size:
            raise RuntimeError("MP4 sync repair changed encoded media bytes")
    return {"path": str(path), "sample_count": samples, "patched_samples": len(patches),
            "idr_samples": idr, "p_samples": predicted,
            "metadata_bytes_changed": sum(a != b for _, old, new in patches for a, b in zip(old, new)),
            "media_bytes": media_size, "media_sha256_before": before, "media_sha256_after": after,
            "media_payload_unchanged": True, "random_access_indexes": False}


class _CDP:
    """Small RFC6455 client for the local Chrome DevTools JSON endpoint."""

    def __init__(self, url, timeout):
        address = urlsplit(url)
        if address.scheme != "ws" or address.hostname not in ("127.0.0.1", "localhost"):
            raise ValueError("Chrome debugging WebSocket must be local")
        self.socket = socket.create_connection((address.hostname, address.port), timeout=timeout)
        self.buffer = b""
        self.identifier = 0
        key = base64.b64encode(os.urandom(16)).decode()
        path = address.path + ("?" + address.query if address.query else "")
        request = (f"GET {path} HTTP/1.1\r\nHost: {address.hostname}:{address.port}\r\n"
                   "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                   f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        try:
            self.socket.sendall(request.encode())
            while b"\r\n\r\n" not in self.buffer:
                incoming = self.socket.recv(4096)
                if not incoming:
                    raise RuntimeError("Chrome closed its debugging handshake")
                self.buffer += incoming
                if len(self.buffer) > 65536:
                    raise RuntimeError("Chrome debugging handshake is too large")
            header, self.buffer = self.buffer.split(b"\r\n\r\n", 1)
            lines = header.decode("ascii").split("\r\n")
            fields = {name.strip().lower(): value.strip() for name, value in
                      (line.split(":", 1) for line in lines[1:] if ":" in line)}
            accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
            if " 101 " not in lines[0] or fields.get("sec-websocket-accept") != accept:
                raise RuntimeError("Chrome refused the debugging WebSocket handshake")
        except Exception:
            self.socket.close()
            raise

    def close(self):
        self.socket.close()

    def _exact(self, count):
        while len(self.buffer) < count:
            incoming = self.socket.recv(max(4096, count - len(self.buffer)))
            if not incoming:
                raise RuntimeError("Chrome closed its debugging connection")
            self.buffer += incoming
        value, self.buffer = self.buffer[:count], self.buffer[count:]
        return value

    def _send(self, opcode, payload):
        length = len(payload)
        header = bytes([0x80 | opcode, 0x80 | min(length, 126 if length < 65536 else 127)])
        if length >= 126:
            header += struct.pack(">H" if length < 65536 else ">Q", length)
        mask = os.urandom(4)
        masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
        self.socket.sendall(header + mask + masked)

    def _message(self):
        chunks = []
        while True:
            first, second = self._exact(2)
            length = second & 127
            if length in (126, 127):
                length = int.from_bytes(self._exact(2 if length == 126 else 8), "big")
            if length > 100_000_000:
                raise RuntimeError("Chrome debugging message is too large")
            mask = self._exact(4) if second & 128 else None
            data = self._exact(length)
            if mask:
                data = bytes(value ^ mask[index % 4] for index, value in enumerate(data))
            opcode = first & 15
            if opcode == 8:
                raise RuntimeError("Chrome closed its debugging connection")
            if opcode == 9:
                self._send(10, data)
                continue
            if opcode == 10:
                continue
            if opcode not in (0, 1):
                raise RuntimeError("Unexpected Chrome debugging WebSocket data")
            chunks.append(data)
            if first & 128:
                return json.loads(b"".join(chunks))

    def call(self, method, params=None):
        self.identifier += 1
        self._send(1, json.dumps({"id": self.identifier, "method": method, "params": params or {}}).encode())
        while True:
            result = self._message()
            if result.get("id") == self.identifier:
                if "error" in result:
                    raise RuntimeError(f"Chrome {method} failed: {result['error']}")
                return result.get("result", {})


def _wait_browser_result(command, profile, timeout_s):
    """Keep Chrome running until actual media IO/playback/seek has completed."""
    deadline = time.monotonic() + timeout_s
    client = None
    with tempfile.TemporaryFile(mode="w+b") as log:
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=log)
        try:
            port_file = profile / "DevToolsActivePort"
            while not port_file.is_file():
                if process.poll() is not None or time.monotonic() >= deadline:
                    log.seek(0)
                    raise RuntimeError("Chrome debugging startup failed: " + log.read().decode(errors="replace")[-1000:])
                time.sleep(.05)
            port = int(port_file.read_text().splitlines()[0])
            while time.monotonic() < deadline:
                with urlopen(f"http://127.0.0.1:{port}/json/list", timeout=min(3, max(.1, deadline - time.monotonic()))) as response:
                    targets = json.load(response)
                page = next((target for target in targets if target.get("type") == "page"), None)
                if page:
                    client = _CDP(page["webSocketDebuggerUrl"], min(5, max(.1, deadline - time.monotonic())))
                    break
                time.sleep(.05)
            if client is None:
                raise RuntimeError("Chrome did not expose a video page before timeout")
            expression = "JSON.parse(document.getElementById('raa-video-result')?.textContent || '{\"status\":\"loading\"}')"
            last_state = {"status": "loading"}
            while time.monotonic() < deadline:
                result = client.call("Runtime.evaluate", {"expression": expression, "returnByValue": True})
                if "exceptionDetails" in result:
                    raise RuntimeError(f"Chrome video status evaluation failed: {result['exceptionDetails']}")
                last_state = result.get("result", {}).get("value", {"status": "loading"})
                if last_state.get("status") in ("decoded", "error"):
                    return last_state
                time.sleep(.1)
            raise RuntimeError(f"Chrome timed out waiting for actual video decode: {last_state}")
        finally:
            if client:
                try:
                    client.call("Browser.close")
                except (OSError, RuntimeError):
                    pass
                client.close()
            if process.poll() is None:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)


def save_video_frame(path: str | Path, time_s: float, output_png: str | Path, *,
                     chrome_binary: str | None = None, timeout_s: float = 45) -> dict:
    """Seek and decode an actual MP4 frame with local CPU-only Chrome.

    Chrome reads a temporary local HTML video/canvas page. A local DevTools
    connection polls real time until playback and seeking publish decoded PNG
    pixels in the DOM, then closes Chrome. Files stay in a temporary profile.
    This verifies browser H.264 decoding beyond MP4 header inspection. The
    caller must execute Chrome in an environment whose permissions allow it.
    """
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    if isinstance(time_s, bool) or not isinstance(time_s, (int, float)) or not math.isfinite(time_s) or time_s < 0:
        raise ValueError("time_s must be finite and nonnegative")
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("timeout_s must be finite and positive")
    inspection = inspect_mp4(path)
    if not inspection["validated"]:
        raise ValueError("MP4 validation failed: " + "; ".join(inspection["errors"]))
    if time_s >= inspection["duration_s"]:
        raise ValueError("time_s must lie inside the MP4 sample timeline")
    chrome = chrome_binary or shutil.which("google-chrome") or shutil.which("chromium")
    if not chrome:
        raise FileNotFoundError("Chrome/Chromium is required for actual video frame decoding")
    source = json.dumps(path.as_uri())
    html = '''<!doctype html><html><body>
<video id="video" preload="auto" muted playsinline style="width:640px"></video>
<canvas id="canvas"></canvas><pre id="raa-video-result">{"status":"starting"}</pre>
<script>
const video=document.getElementById("video"), canvas=document.getElementById("canvas"), result=document.getElementById("raa-video-result");
const publish=data=>result.textContent=JSON.stringify(data);
const once=(name)=>new Promise((resolve,reject)=>{video.addEventListener(name,resolve,{once:true});video.addEventListener("error",()=>reject(new Error("media error "+video.error?.code)),{once:true});});
const frame=()=>new Promise(resolve=>video.requestVideoFrameCallback((now,metadata)=>resolve(metadata)));
(async()=>{try {
 const metadata=once("loadedmetadata");video.src=SOURCE;video.load();await metadata;
 if(!Number.isFinite(video.duration)||REQUESTED>=video.duration) throw new Error("Requested time lies outside video duration");
 if(video.readyState<2) await once("loadeddata");
 if(!video.requestVideoFrameCallback)throw new Error("Browser lacks presented-frame timestamp verification");
 const first=frame();await video.play();let displayed=await first;video.pause();
 if(Math.abs(video.currentTime-REQUESTED)>1e-8){
  publish({status:"seeking",requested_time_s:REQUESTED,frame_media_time_s:displayed.mediaTime});
  const presented=frame(),seek=once("seeked");video.currentTime=REQUESTED;await seek;displayed=await presented;
 }
 if(video.readyState<2) await once("loadeddata");
 if(Math.abs(displayed.mediaTime-REQUESTED)>FRAMETOL)throw new Error("Displayed frame "+displayed.mediaTime+" differs from requested "+REQUESTED);
 canvas.width=video.videoWidth;canvas.height=video.videoHeight;
 if(!canvas.width||!canvas.height)throw new Error("Decoded frame has no dimensions");
 canvas.getContext("2d").drawImage(video,0,0);
 publish({status:"decoded",requested_time_s:REQUESTED,current_time_s:video.currentTime,duration_s:video.duration,
 frame_media_time_s:displayed.mediaTime,presented_frames:displayed.presentedFrames,
 resolution:[video.videoWidth,video.videoHeight],ready_state:video.readyState,playback_started:true,
 can_play_h264:video.canPlayType(CODECTYPE),data_url:canvas.toDataURL("image/png")});
}catch(error){publish({status:"error",message:String(error),ready_state:video.readyState,duration_s:Number.isFinite(video.duration)?video.duration:String(video.duration)});}})();
</script></body></html>'''.replace("SOURCE", source).replace("REQUESTED", json.dumps(float(time_s))).replace(
        "CODECTYPE", json.dumps(f'video/mp4; codecs="{inspection["codec_string"]}"')).replace(
        "FRAMETOL", repr(1 / inspection["fps"] + 1e-6))
    with tempfile.TemporaryDirectory(prefix="raa-video-frame-") as directory:
        directory = Path(directory)
        page = directory / "decode.html"
        page.write_text(html, encoding="utf-8")
        command = [str(chrome), "--headless", "--disable-gpu", "--disable-gpu-compositing", "--no-sandbox",
                   "--disable-dev-shm-usage", "--allow-file-access-from-files", "--no-first-run",
                   "--no-default-browser-check", "--autoplay-policy=no-user-gesture-required",
                   "--remote-debugging-address=127.0.0.1", "--remote-debugging-port=0",
                   f"--user-data-dir={directory / 'profile'}", page.as_uri()]
        state = _wait_browser_result(command, directory / "profile", timeout_s)
    if state.get("status") != "decoded":
        raise RuntimeError(f"Chrome failed to decode video: {state}")
    duration = state.get("duration_s")
    current = state.get("current_time_s")
    tolerance = max(1 / track["timescale"] for track in inspection["tracks"] if track["handler"] == "vide") + 1e-8
    if not isinstance(duration, (float, int)) or not math.isfinite(duration) or abs(duration - inspection["duration_s"]) > tolerance:
        raise RuntimeError(f"Browser duration {duration!r}s disagrees with MP4 sample timeline {inspection['duration_s']}s")
    if not isinstance(current, (float, int)) or not math.isfinite(current) or abs(current - time_s) > tolerance:
        raise RuntimeError(f"Browser seek returned {current!r}s instead of requested {time_s}s")
    frame_time = state.get("frame_media_time_s")
    if not isinstance(frame_time, (float, int)) or not math.isfinite(frame_time) or abs(frame_time - time_s) > 1 / inspection["fps"] + 1e-6:
        raise RuntimeError(f"Displayed video frame {frame_time!r}s differs from requested {time_s}s")
    if state.get("resolution") != inspection["resolution"]:
        raise RuntimeError("Browser decoded video dimensions disagree with MP4 track")
    if state.get("playback_started") is not True or not state.get("can_play_h264"):
        raise RuntimeError("Browser could not start H.264 playback")
    data_url = state.pop("data_url", "")
    if not data_url.startswith("data:image/png;base64,"):
        raise RuntimeError("Decoded canvas did not return a PNG")
    image = base64.b64decode(data_url.split(",", 1)[1], validate=True)
    if len(image) < 33 or image[:8] != b"\x89PNG\r\n\x1a\n" or image[12:16] != b"IHDR":
        raise RuntimeError("Decoded canvas PNG header is invalid")
    dimensions = list(struct.unpack(">II", image[16:24]))
    if dimensions != state.get("resolution"):
        raise RuntimeError("Canvas PNG resolution and decoded video dimensions disagree")
    output = Path(output_png).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(image)
    return {**state, "output_png": str(output), "file_size_bytes": len(image), "browser_verified": True,
            "frame_time_verified": True,
            "mp4_duration_s": inspection["duration_s"], "mp4_frame_count": inspection["frame_count"],
            "decoder": "Chrome HTML video/canvas; GPU disabled"}
