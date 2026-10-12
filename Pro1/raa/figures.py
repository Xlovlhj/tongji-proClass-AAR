"""Export existing scientific SVG figures to PNG with a CPU-only Chrome process.

Each conversion uses a temporary Chrome profile. The source SVG remains the
editable, resolution-independent figure; PNG has the SVG's explicit pixel size.
No Isaac, NumPy, plotting package or browser session is needed.
"""

from __future__ import annotations

import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def locate_chrome(explicit: str | Path | None = None) -> str:
    if explicit is not None:
        candidate = shutil.which(str(explicit))
        if candidate:
            return candidate
        raise FileNotFoundError(f"找不到 Chrome 可执行程序：{explicit}")
    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
        candidate = shutil.which(name)
        if candidate:
            return candidate
    raise FileNotFoundError("SVG 转 PNG 需要已有 Chrome；可以使用 --chrome 指定它的路径。")


def svg_size(path: str | Path) -> tuple[int, int]:
    """Read explicit SVG width/height (pixels), falling back to its viewBox."""
    root = ET.parse(path).getroot()
    if root.tag.rsplit("}", 1)[-1] != "svg":
        raise ValueError(f"不是 SVG 图形：{path}")
    view_box = root.get("viewBox", "").replace(",", " ").split()
    fallback = None
    if len(view_box) == 4:
        try:
            fallback = float(view_box[2]), float(view_box[3])
        except ValueError:
            pass
    dimensions = []
    for axis, name in enumerate(("width", "height")):
        raw = root.get(name)
        if raw is None or raw.endswith("%"):
            if fallback is None:
                raise ValueError(f"SVG 缺少可用的 {name} / viewBox：{path}")
            value = fallback[axis]
        else:
            if not re.fullmatch(r"\s*(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?(?:px)?\s*", raw):
                raise ValueError(f"SVG 的 {name} 需要像素数值：{raw}")
            value = float(raw.strip().removesuffix("px"))
        if not math.isfinite(value) or not 0 < value <= 16384:
            raise ValueError(f"SVG 的 {name} 必须在 (0, 16384] 像素之间")
        dimensions.append(math.ceil(value))
    return tuple(dimensions)


def png_size(path: str | Path) -> tuple[int, int]:
    with Path(path).open("rb") as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError(f"不是有效的 PNG 图形：{path}")
    return struct.unpack(">II", header[16:24])


def svg_to_png(source: str | Path, destination: str | Path | None = None, *,
               chrome: str | Path | None = None, timeout_s: float = 60) -> Path:
    """Render an SVG at its native size and atomically replace the PNG output."""
    source = Path(source).expanduser().resolve(strict=True)
    destination = source.with_suffix(".png") if destination is None else Path(destination).expanduser().resolve()
    if source == destination:
        raise ValueError("PNG 输出路径必须与源 SVG 路径不同")
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("Chrome 超时时间必须是有限正数")
    width, height = svg_size(source)
    browser = locate_chrome(chrome)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # A temporary file on the destination filesystem allows atomic replacement.
    with tempfile.TemporaryDirectory(prefix="raa-figure-", dir=destination.parent) as temporary:
        temporary = Path(temporary)
        rendered = temporary / "rendered.png"
        command = [browser, "--headless=new", "--disable-gpu", "--no-sandbox",
                   "--disable-dev-shm-usage", "--disable-background-networking",
                   "--disable-component-update", "--disable-extensions", "--disable-sync",
                   "--no-first-run", "--no-default-browser-check", "--hide-scrollbars",
                   "--force-device-scale-factor=1", f"--user-data-dir={temporary / 'profile'}",
                   f"--window-size={width},{height}", f"--screenshot={rendered}", source.as_uri()]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, timeout=timeout_s, check=False)
        if result.returncode or not rendered.is_file():
            detail = (result.stderr or result.stdout).strip()[-2000:]
            raise RuntimeError(f"Chrome 导出失败（退出码 {result.returncode}）：{source}\n{detail}")
        actual = png_size(rendered)
        if actual != (width, height):
            raise RuntimeError(f"PNG 尺寸 {actual} 与 SVG 尺寸 {(width, height)} 不一致：{source}")
        rendered.replace(destination)
    return destination


def export_directory(directory: str | Path, *, chrome: str | Path | None = None) -> list[Path]:
    """Convert each top-level SVG in one run/comparison directory, serially."""
    browser = locate_chrome(chrome)
    return [svg_to_png(source, chrome=browser) for source in sorted(Path(directory).glob("*.svg"))]
