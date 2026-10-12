"""ChatGPT blossom outlines and a deterministic, lifted Cartesian reference.

Only the Python standard library is needed.  ``generate_logo_strokes`` reads the
vendored SVG instead of relying on an image service or approximating the logo.
The drawing is the *outline* of the filled mark: eight closed contours, with a
lift between contours.  It is not a filled raster drawing or a centreline logo.
All positions returned by ``build_reference`` are in metres, time in seconds.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from html import escape
from math import acos, atan2, ceil, cos, hypot, isfinite, pi, sin, sqrt
from pathlib import Path
import csv
import re
import xml.etree.ElementTree as ET


Point2 = tuple[float, float]
Point3 = tuple[float, float, float]
ASSET_PATH = Path(__file__).resolve().parents[1] / "assets" / "chatgpt_logo.svg"
LOGO_SOURCE = {
    "name": "OpenAI / ChatGPT blossom symbol (2025 design)",
    "author": "OpenAI",
    "description_page": "https://commons.wikimedia.org/wiki/File:OpenAI_logo_2025_(symbol).svg",
    "download_url": "https://upload.wikimedia.org/wikipedia/commons/6/66/OpenAI_logo_2025_%28symbol%29.svg",
    "asset_revision": "2026-08-24 cropped SVG, fetched 2026-10-10",
    "brand_guidelines": "https://openai.com/brand/",
    "copyright_note": "Wikimedia Commons labels this geometric mark PD-textlogo (public domain).",
    "trademark_note": "OpenAI and ChatGPT marks belong to OpenAI. This independent course demonstration is not endorsed by OpenAI.",
    "drawing_method": "Trace all eight SVG boundary subpaths; preserve aspect ratio; lift between subpaths.",
}


@dataclass(frozen=True)
class CartesianSample:
    time: float
    position: Point3
    pen_down: bool
    stroke_index: int


@dataclass(frozen=True)
class CartesianReference:
    samples: list[CartesianSample]
    strokes: list[list[Point3]]
    dt: float
    source: dict[str, str]

    @property
    def duration(self) -> float:
        return self.samples[-1].time if self.samples else 0.0

    @property
    def positions(self) -> list[Point3]:
        return [sample.position for sample in self.samples]

    @property
    def times(self) -> list[float]:
        return [sample.time for sample in self.samples]

    @property
    def pen_down(self) -> list[bool]:
        return [sample.pen_down for sample in self.samples]

    @property
    def stroke_ids(self) -> list[int]:
        return [sample.stroke_index for sample in self.samples]

    @property
    def drawing_length(self) -> float:
        return sum(sum(_distance(a, b) for a, b in zip(s, s[1:])) for s in self.strokes)

    def save_csv(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "x_m", "y_m", "z_m", "pen_down", "stroke_index"])
            for sample in self.samples:
                writer.writerow([sample.time, *sample.position, int(sample.pen_down), sample.stroke_index])
        return destination


def _distance(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    return sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _arc_points(start: Point2, args: list[float], relative: bool, tolerance: float) -> list[Point2]:
    """SVG endpoint-parameterized ellipse, per the W3C implementation notes."""
    rx, ry, rotation, large, sweep, end_x, end_y = args
    end = (end_x + start[0], end_y + start[1]) if relative else (end_x, end_y)
    rx, ry = abs(rx), abs(ry)
    if rx == 0 or ry == 0 or _distance(start, end) < 1e-12:
        return [end]
    phi = rotation * pi / 180
    cp, sp = cos(phi), sin(phi)
    dx, dy = (start[0] - end[0]) / 2, (start[1] - end[1]) / 2
    xp, yp = cp * dx + sp * dy, -sp * dx + cp * dy
    correction = xp * xp / (rx * rx) + yp * yp / (ry * ry)
    if correction > 1:
        rx *= sqrt(correction)
        ry *= sqrt(correction)
    numerator = max(0.0, rx * rx * ry * ry - rx * rx * yp * yp - ry * ry * xp * xp)
    denominator = rx * rx * yp * yp + ry * ry * xp * xp
    coefficient = sqrt(numerator / denominator) * (-1 if bool(large) == bool(sweep) else 1)
    cxp, cyp = coefficient * rx * yp / ry, -coefficient * ry * xp / rx
    cx = cp * cxp - sp * cyp + (start[0] + end[0]) / 2
    cy = sp * cxp + cp * cyp + (start[1] + end[1]) / 2
    ux, uy = (xp - cxp) / rx, (yp - cyp) / ry
    vx, vy = (-xp - cxp) / rx, (-yp - cyp) / ry
    angle = atan2(uy, ux)
    delta = atan2(ux * vy - uy * vx, ux * vx + uy * vy)
    if not sweep and delta > 0:
        delta -= 2 * pi
    elif sweep and delta < 0:
        delta += 2 * pi
    count = max(2, ceil(abs(delta) * max(rx, ry) / tolerance))
    result = []
    for index in range(1, count + 1):
        theta = angle + delta * index / count
        result.append((cx + cp * rx * cos(theta) - sp * ry * sin(theta),
                       cy + sp * rx * cos(theta) + cp * ry * sin(theta)))
    result[-1] = end
    return result


def _flatten_svg_path(data: str, tolerance: float) -> list[list[Point2]]:
    """Flatten the M/L/H/V/Q/C/A/Z commands in the bundled SVG.

    Implicit repeated commands and relative coordinates are supported.  Fail
    explicitly on unsupported commands rather than silently drawing bad paths.
    """
    tokens = re.findall(r"[A-Za-z]|[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", data)
    sizes = {"M": 2, "L": 2, "H": 1, "V": 1, "Q": 4, "C": 6, "A": 7}
    contours: list[list[Point2]] = []
    points: list[Point2] = []
    cursor = (0.0, 0.0)
    origin = cursor
    index = 0
    command = ""
    while index < len(tokens):
        if tokens[index].isalpha():
            command = tokens[index]
            index += 1
        op, relative = command.upper(), command.islower()
        if op == "Z":
            if points and _distance(points[-1], origin) > 1e-12:
                points.append(origin)
            if points:
                contours.append(points)
            points = []
            cursor = origin
            command = ""
            continue
        if op not in sizes:
            raise ValueError(f"Unsupported SVG path command {command!r}")
        size = sizes[op]
        values = tokens[index:index + size]
        if len(values) != size or any(value.isalpha() for value in values):
            raise ValueError(f"Incomplete SVG command {command!r}")
        args = [float(value) for value in values]
        index += size

        def point(x: float, y: float) -> Point2:
            return (x + cursor[0], y + cursor[1]) if relative else (x, y)

        if op == "M":
            if points:
                contours.append(points)
            cursor = point(*args)
            origin = cursor
            points = [cursor]
            command = "l" if relative else "L"
        elif op in {"L", "H", "V"}:
            endpoint = (point(*args) if op == "L" else
                        ((args[0] + cursor[0] if relative else args[0]), cursor[1]) if op == "H" else
                        (cursor[0], args[0] + cursor[1] if relative else args[0]))
            points.append(endpoint)
            cursor = endpoint
        elif op in {"Q", "C"}:
            controls = [cursor] + [point(*args[j:j + 2]) for j in range(0, size, 2)]
            count = max(2, ceil(sum(_distance(a, b) for a, b in zip(controls, controls[1:])) / tolerance))
            for j in range(1, count + 1):
                t = j / count
                if op == "Q":
                    weights = ((1 - t) ** 2, 2 * (1 - t) * t, t * t)
                else:
                    weights = ((1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t * t, t ** 3)
                points.append(tuple(sum(weight * p[k] for weight, p in zip(weights, controls)) for k in (0, 1)))
            cursor = controls[-1]
        elif op == "A":
            points.extend(_arc_points(cursor, args, relative, tolerance))
            cursor = points[-1]
    if points:
        contours.append(points)
    return contours


def generate_logo_strokes(resolution: float = 0.0015) -> list[list[Point2]]:
    """Return eight centered boundary contours, largest dimension equal to one.

    Y points upwards (the SVG has Y downwards).  ``resolution`` is approximately
    the maximum curved-segment chord length in normalized coordinates.  The
    SVG is local, so generating the logo never needs network access.
    """
    if not isfinite(resolution) or resolution <= 0 or resolution > 0.05:
        raise ValueError("resolution must be finite and in (0, 0.05]")
    root = ET.parse(ASSET_PATH).getroot()
    view_box = [float(value) for value in root.attrib["viewBox"].split()]
    nominal_size = max(view_box[2:])
    strokes = []
    for element in root.iter():
        if element.tag.rsplit("}", 1)[-1] == "path":
            strokes.extend(_flatten_svg_path(element.attrib["d"], resolution * nominal_size))
    if not strokes:
        raise ValueError("The logo SVG contains no paths")
    xs = [point[0] for stroke in strokes for point in stroke]
    ys = [point[1] for stroke in strokes for point in stroke]
    center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
    scale = max(max(xs) - min(xs), max(ys) - min(ys))
    normalized = []
    for stroke in strokes:
        clean = []
        for x, y in stroke:
            point = ((x - center[0]) / scale, (center[1] - y) / scale)
            if not clean or _distance(clean[-1], point) > 1e-10:
                clean.append(point)
        if len(clean) > 1:
            # SVG fill closes subpaths implicitly even when their data omits Z.
            if _distance(clean[0], clean[-1]) > 1e-10:
                clean.append(clean[0])
            normalized.append(clean)
    return normalized


def _corner_sections(points: list[Point3], corner_angle: float = 0.30) -> list[list[Point3]]:
    """Stop at true SVG corners while retaining continuous rounded portions."""
    cuts = [0]
    for index in range(1, len(points) - 1):
        a = tuple(x - y for x, y in zip(points[index], points[index - 1]))
        b = tuple(x - y for x, y in zip(points[index + 1], points[index]))
        norm = _distance(points[index], points[index - 1]) * _distance(points[index + 1], points[index])
        if norm > 1e-15 and acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / norm))) > corner_angle:
            cuts.append(index)
    cuts.append(len(points) - 1)
    return [points[a:b + 1] for a, b in zip(cuts, cuts[1:]) if b > a]


def build_reference(
    width: float = 0.18,
    center_xy: Point2 = (0.45, 0.0),
    draw_z: float = 0.45,
    lift_height: float = 0.025,
    speed: float = 0.04,
    dt: float = 1 / 120,
    height: float | None = None,
    travel_speed: float | None = None,
) -> CartesianReference:
    """Build a uniform-dt, deterministic draw/lift/travel/lower reference.

    Width/height specify a bounding box; aspect ratio is preserved.  ``speed``
    caps scalar drawing speed, not average speed.  Each rounded section uses
    quintic arc-length timing (zero endpoint velocity and acceleration); sharp
    corners are explicit stops.  Curves are tightly sampled SVG polylines.
    ``draw_z`` is the virtual ink plane; physical contact force is not imposed.
    """
    scalars = [width, draw_z, lift_height, speed, dt, *center_xy]
    if height is not None:
        scalars.append(height)
    if travel_speed is not None:
        scalars.append(travel_speed)
    if not all(isfinite(value) for value in scalars):
        raise ValueError("All trajectory parameters must be finite")
    if width <= 0 or speed <= 0 or dt <= 0 or lift_height <= 0 or (height is not None and height <= 0):
        raise ValueError("width, height, speed, dt and lift_height must be positive")
    travel_speed = travel_speed if travel_speed is not None else speed * 1.5
    if travel_speed <= 0:
        raise ValueError("travel_speed must be positive")
    logo = generate_logo_strokes()
    xspan = max(x for s in logo for x, _ in s) - min(x for s in logo for x, _ in s)
    yspan = max(y for s in logo for _, y in s) - min(y for s in logo for _, y in s)
    scale = width / xspan
    if height is not None:
        scale = min(scale, height / yspan)
    strokes = [[(center_xy[0] + x * scale, center_xy[1] + y * scale, draw_z) for x, y in stroke] for stroke in logo]
    first = strokes[0][0]
    samples = [CartesianSample(0.0, (first[0], first[1], draw_z + lift_height), False, -1)]

    def append_motion(route: list[Point3], velocity: float, pen_down: bool, stroke_index: int, final_down: bool | None = None) -> None:
        distances = [0.0]
        for a, b in zip(route, route[1:]):
            distances.append(distances[-1] + _distance(a, b))
        length = distances[-1]
        if length <= 1e-12:
            return
        # max derivative of 10u^3-15u^4+6u^5 is 1.875.
        steps = max(4, ceil(max(0.12, 1.875 * length / velocity) / dt))
        initial_index = len(samples) - 1
        for j in range(1, steps + 1):
            u = j / steps
            distance = (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5) * length
            index = min(len(route) - 2, max(0, bisect_right(distances, distance) - 1))
            span = distances[index + 1] - distances[index]
            fraction = (distance - distances[index]) / span if span > 0 else 0.0
            position = tuple(a + fraction * (b - a) for a, b in zip(route[index], route[index + 1]))
            state = final_down if j == steps and final_down is not None else pen_down
            samples.append(CartesianSample((initial_index + j) * dt, position, state, stroke_index))

    for stroke_index, stroke in enumerate(strokes):
        start = stroke[0]
        elevated = (start[0], start[1], draw_z + lift_height)
        append_motion([samples[-1].position, elevated], travel_speed, False, stroke_index)
        append_motion([elevated, start], travel_speed, False, stroke_index, final_down=True)
        for section in _corner_sections(stroke):
            append_motion(section, speed, True, stroke_index)
        end = stroke[-1]
        append_motion([end, (end[0], end[1], draw_z + lift_height)], travel_speed, False, stroke_index)
    return CartesianReference(samples, strokes, dt, LOGO_SOURCE.copy())


def save_preview_svg(path: str | Path, strokes: list[list[tuple[float, ...]]] | None = None, title: str = "robot arm artist · raa-v0") -> Path:
    """Save a dependency-free SVG preview of drawing paths (travel omitted)."""
    if strokes is None:
        strokes = generate_logo_strokes()
    points = [point for stroke in strokes for point in stroke]
    if not points:
        raise ValueError("No strokes to preview")
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    center = ((max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2)
    elements = []
    for index, stroke in enumerate(strokes):
        coordinates = " ".join(f"{320 + 480 * (p[0] - center[0]) / span:.3f},{344 - 480 * (p[1] - center[1]) / span:.3f}" for p in stroke)
        elements.append(f'<polyline id="stroke-{index}" points="{coordinates}"/>')
    svg = '\n'.join([
        '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="640" viewBox="0 0 640 640">',
        f'<title>{escape(title)} — ChatGPT blossom outline</title>',
        f'<desc>{escape(LOGO_SOURCE["description_page"])}. Eight closed contours with pen lifts. Educational drawing target; OpenAI mark.</desc>',
        '<rect width="640" height="640" fill="white"/>',
        f'<text x="320" y="40" text-anchor="middle" font-family="sans-serif" font-size="22" fill="#111">{escape(title)}</text>',
        '<g fill="none" stroke="#111" stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round">',
        *elements, '</g>', '</svg>',
    ])
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(svg + '\n', encoding="utf-8")
    return destination
