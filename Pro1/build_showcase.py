"""Make small README GIF previews from the eight original Isaac recordings.

Run: python3 Pro1/build_showcase.py
Uses an existing FFmpeg executable; does not install packages or run simulation.
Original MP4 files are checked by SHA-256 before and after conversion.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile

from raa.video_validation import inspect_mp4


ROOT = Path(__file__).resolve().parent
CONTROLLERS = ("pd", "gc-pd", "rbf", "fuzzy")
CONDITIONS = ("normal", "disturbed")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def find_ffmpeg(explicit: str | None) -> Path:
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"FFmpeg does not exist: {candidate}")
        return candidate
    available = shutil.which("ffmpeg")
    if available:
        return Path(available).resolve()
    candidates = []
    spec = importlib.util.find_spec("imageio_ffmpeg")
    if spec and spec.origin:
        candidates.extend(Path(spec.origin).parent.glob("binaries/ffmpeg*"))
    for prefix in {Path(sys.prefix), Path(sys.base_prefix)}:
        candidates.extend(prefix.glob("lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg*"))
    for installation in (Path.home() / "miniconda3", Path.home() / "anaconda3"):
        candidates.extend(installation.glob("lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg*"))
        candidates.extend(installation.glob("envs/*/lib/python*/site-packages/imageio_ffmpeg/binaries/ffmpeg*"))
    for candidate in sorted(set(candidates)):
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError("No existing FFmpeg found. Supply --ffmpeg /path/to/ffmpeg.")


def inspect_gif(path: Path) -> dict:
    """Read image count, dimensions, loop flag and actual GIF centisecond delays."""
    data = path.read_bytes()
    if data[:6] not in (b"GIF87a", b"GIF89a") or len(data) < 14:
        raise ValueError(f"Invalid GIF header: {path}")
    width, height, packed = struct.unpack_from("<HHB", data, 6)
    offset = 13 + (3 * (2 ** ((packed & 7) + 1)) if packed & 128 else 0)
    frame_count, total_delay, pending_delay = 0, 0, 0
    loop = None

    def sub_blocks(start):
        chunks = []
        while True:
            if start >= len(data):
                raise ValueError("Truncated GIF sub-block")
            size = data[start]
            start += 1
            if size == 0:
                return start, b"".join(chunks)
            if start + size > len(data):
                raise ValueError("Truncated GIF sub-block payload")
            chunks.append(data[start:start + size])
            start += size

    trailer_seen = False
    while offset < len(data):
        marker = data[offset]
        offset += 1
        if marker == 0x3B:
            trailer_seen = True
            break
        if marker == 0x21:
            if offset >= len(data):
                raise ValueError("Truncated GIF extension")
            label = data[offset]
            offset, payload = sub_blocks(offset + 1)
            if label == 0xF9:
                if len(payload) != 4:
                    raise ValueError("Invalid GIF graphics control extension")
                pending_delay = struct.unpack_from("<H", payload, 1)[0]
            elif label == 0xFF and payload.startswith(b"NETSCAPE2.0\x01"):
                loop = struct.unpack_from("<H", payload, 12)[0]
        elif marker == 0x2C:
            if offset + 9 > len(data):
                raise ValueError("Truncated GIF image descriptor")
            left, top, image_width, image_height, image_packed = struct.unpack_from("<HHHHB", data, offset)
            if left + image_width > width or top + image_height > height:
                raise ValueError("GIF frame exceeds its canvas")
            offset += 9
            if image_packed & 128:
                offset += 3 * (2 ** ((image_packed & 7) + 1))
            offset, _ = sub_blocks(offset + 1)  # Skip LZW minimum code size.
            frame_count += 1
            total_delay += pending_delay
            pending_delay = 0
        else:
            raise ValueError(f"Unexpected GIF block 0x{marker:02x}")
    if not trailer_seen or frame_count == 0:
        raise ValueError("GIF has no complete image sequence")
    return {"resolution": [width, height], "frame_count": frame_count,
            "duration_s": total_delay / 100.0, "loop_count": loop,
            "loop_forever": loop == 0}


def decoded_frame_count(ffmpeg: Path, gif: Path) -> int:
    command = [str(ffmpeg), "-hide_banner", "-loglevel", "error", "-threads", "2",
               "-i", str(gif), "-vsync", "0", "-f", "null", "-", "-progress", "pipe:1"]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    counts = re.findall(r"^frame=(\d+)\s*$", result.stdout, re.MULTILINE)
    if not counts:
        raise ValueError(f"FFmpeg did not report decoded GIF frames: {gif}")
    return int(counts[-1])


def make_preview(ffmpeg: Path, source: Path, output_dir: Path, options) -> dict:
    source_hash = sha256(source)
    source_info = inspect_mp4(source)
    desired_duration = source_info["duration_s"] / options.speed
    output = output_dir / f"{source.stem}.gif"
    filters = (f"setpts=PTS/{options.speed},fps={options.fps},"
               f"scale={options.width}:{options.height}:flags=lanczos,split[a][b];"
               f"[a]palettegen=max_colors={options.max_colors}:stats_mode=diff[p];"
               "[b][p]paletteuse=dither=bayer:bayer_scale=3")
    with tempfile.TemporaryDirectory(prefix=f".{source.stem}-", dir=output_dir) as temporary:
        partial = Path(temporary) / output.name
        command = [str(ffmpeg), "-hide_banner", "-loglevel", "error", "-y",
                   "-threads", "2", "-filter_complex_threads", "1", "-i", str(source),
                   "-an", "-filter_complex", filters, "-loop", "0", str(partial)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        preview_info = inspect_gif(partial)
        decoded_frames = decoded_frame_count(ffmpeg, partial)
        expected_frames = round(desired_duration * options.fps)
        if abs(preview_info["frame_count"] - expected_frames) > 1:
            raise ValueError(f"Unexpected GIF frame count for {source.name}: {preview_info}")
        if decoded_frames != preview_info["frame_count"]:
            raise ValueError(f"GIF decode count differs from container count: {source.name}")
        if abs(preview_info["duration_s"] - desired_duration) > 2 / options.fps + 0.01:
            raise ValueError(f"GIF duration differs from requested speed: {source.name}")
        if preview_info["resolution"] != [options.width, options.height] or not preview_info["loop_forever"]:
            raise ValueError(f"Invalid GIF dimensions or looping: {source.name}")
        if partial.stat().st_size >= 10 * 1024 * 1024:
            raise ValueError(f"GIF exceeds 10 MiB: {source.name}; reduce --max-colors or --width")
        if sha256(source) != source_hash:
            raise ValueError(f"Original MP4 changed during conversion: {source}")
        partial.replace(output)
    return {"source": f"../{source.name}", "source_sha256": source_hash,
            "source_size_bytes": source.stat().st_size,
            "source_duration_s": source_info["duration_s"],
            "source_frame_count": source_info["frame_count"],
            "original_source_unchanged": True, "preview": output.name,
            "preview_sha256": sha256(output), "preview_size_bytes": output.stat().st_size,
            "speed_multiplier": options.speed, "requested_preview_fps": options.fps,
            "max_colors": options.max_colors, "dither": "bayer", "bayer_scale": 3,
            "expected_preview_duration_s": desired_duration,
            "decoded_frame_count": decoded_frames, "decode_verified": True, **preview_info}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ffmpeg", help="Path to an already-installed FFmpeg executable")
    parser.add_argument("--speed", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--max-colors", type=int, default=128)
    parser.add_argument("--jobs", type=int, choices=(1, 2), default=2)
    options = parser.parse_args()
    if not math.isfinite(options.speed) or options.speed <= 0 or not 1 <= options.fps <= 50:
        parser.error("--speed must be positive and finite; --fps must be between 1 and 50")
    if options.width <= 0 or options.height <= 0 or not 4 <= options.max_colors <= 256:
        parser.error("Dimensions must be positive; --max-colors must be between 4 and 256")
    ffmpeg = find_ffmpeg(options.ffmpeg)
    output_dir = ROOT / "video" / "preview"
    sources = [ROOT / "video" / f"{controller}_{condition}.mp4"
               for condition in CONDITIONS for controller in CONTROLLERS]
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError(f"Original recording is missing: {source}")
    output_dir.mkdir(parents=True, exist_ok=True)
    results = {}
    with ThreadPoolExecutor(max_workers=options.jobs) as pool:
        pending = {pool.submit(make_preview, ffmpeg, source, output_dir, options): source for source in sources}
        for future in as_completed(pending):
            source = pending[future]
            result = future.result()
            results[source.stem] = result
            print(f"{result['preview']}: {result['preview_size_bytes'] / 1024**2:.2f} MiB, "
                  f"{result['frame_count']} frames, {result['duration_s']:.2f} s; decode verified", flush=True)
    manifest = {"completed": True, "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "source": "Eight complete Isaac Sim viewport recordings",
                "preview_only": True, "text_overlays": False,
                "ffmpeg_version": subprocess.run([str(ffmpeg), "-version"], check=True,
                                                  capture_output=True, text=True).stdout.splitlines()[0],
                "settings": {"speed_multiplier": options.speed, "fps": options.fps,
                             "resolution": [options.width, options.height], "max_colors": options.max_colors},
                "original_videos_unchanged": all(item["original_source_unchanged"] for item in results.values()),
                "previews": [results[source.stem] for source in sources]}
    manifest_path = output_dir / "preview_manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {manifest_path.relative_to(ROOT)}", flush=True)


if __name__ == "__main__":
    main()
