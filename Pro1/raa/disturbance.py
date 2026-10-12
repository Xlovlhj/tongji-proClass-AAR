"""Deterministic world-frame forces applied physically at the virtual pen tip.

The reference path is never altered. All controllers use the same schedule
computed from the reference's pen-down intervals, independent of actual motion.
Forces are in newtons, positions in metres, moments in newton metres.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, Sequence

import numpy as np


def _vector3(values: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite values")
    return result


@dataclass(frozen=True)
class ForcePulse:
    start_s: float
    end_s: float
    force_world: tuple[float, float, float]
    label: str = "tip force"

    def __post_init__(self) -> None:
        start, end = float(self.start_s), float(self.end_s)
        if not isfinite(start) or not isfinite(end) or start < 0 or end <= start:
            raise ValueError("force pulse must have finite 0 <= start_s < end_s")
        force = _vector3(self.force_world, "force_world")
        if not np.any(force):
            raise ValueError("a force pulse must have nonzero force")
        if np.any(np.abs(force) > np.finfo(np.float32).max):
            raise ValueError("force must be representable by the PhysX float32 API")
        object.__setattr__(self, "start_s", start)
        object.__setattr__(self, "end_s", end)
        object.__setattr__(self, "force_world", tuple(float(v) for v in force))


DEFAULT_PULSES = (
    ForcePulse(20.0, 22.0, (3.0, 0.0, 0.0), "+X push"),
    ForcePulse(55.0, 57.0, (0.0, -3.0, 0.0), "-Y push"),
    ForcePulse(90.0, 92.0, (2.0, 2.0, 0.0), "+X +Y push"),
)


@dataclass(frozen=True)
class ForceSample:
    time_s: float
    force_world: np.ndarray
    active: bool
    pulse_index: int


class DisturbanceSchedule:
    """Non-overlapping half-open pulses, start <= t < end.

    Disabled schedules still expose their windows in metadata but always
    return a zero force. Explicit per-step sampling avoids accumulated clocks.
    """

    def __init__(self, pulses: Sequence[ForcePulse] = DEFAULT_PULSES, *, enabled: bool = True) -> None:
        if not isinstance(enabled, (bool, np.bool_)):
            raise ValueError("disturbance enabled must be boolean")
        self.enabled = bool(enabled)
        supplied = tuple(pulses)
        if not all(isinstance(pulse, ForcePulse) for pulse in supplied):
            raise TypeError("pulses must be ForcePulse objects")
        self.pulses = tuple(sorted(supplied, key=lambda pulse: pulse.start_s))
        for left, right in zip(self.pulses, self.pulses[1:]):
            if left.end_s > right.start_s:
                raise ValueError("force pulses must not overlap")

    def sample(self, time_s: float) -> ForceSample:
        t = float(time_s)
        if not isfinite(t) or t < 0:
            raise ValueError("disturbance time must be finite and nonnegative")
        if self.enabled:
            for index, pulse in enumerate(self.pulses):
                if pulse.start_s <= t < pulse.end_s:
                    return ForceSample(t, np.asarray(pulse.force_world, dtype=np.float64), True, index)
        return ForceSample(t, np.zeros(3, dtype=np.float64), False, -1)

    def sample_step(self, step: int, dt: float) -> ForceSample:
        if isinstance(step, bool) or not isinstance(step, (int, np.integer)) or step < 0:
            raise ValueError("step must be a nonnegative integer")
        dt = float(dt)
        if not isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        return self.sample(int(step) * dt)

    def metadata(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "frame": "world",
            "force_units": "N",
            "position_units": "m",
            "application_body": "panda_hand",
            "application_point": "actual virtual pen tip in world coordinates",
            "interval_convention": "start_s <= time < end_s",
            "reference_modified": False,
            "pulses": [{"start_s": p.start_s, "end_s": p.end_s,
                "force_world_n": list(p.force_world), "label": p.label} for p in self.pulses],
        }


def pen_down_intervals(reference) -> list[tuple[float, float, int]]:
    """Conservative continuous ink intervals (both adjacent nodes pen down).

    The last pen-down node is the end, not the following pen-up node. Separate
    stroke IDs never merge, even when their boolean flags touch in a fixture.
    """
    times = np.asarray(reference.times, dtype=np.float64)
    down = np.asarray(reference.pen_down)
    stroke = np.asarray(reference.stroke_ids)
    if (times.ndim != 1 or len(times) < 2 or not np.all(np.isfinite(times))
            or np.any(np.diff(times) <= 0) or times[0] < 0
            or down.shape != times.shape or stroke.shape != times.shape):
        raise ValueError("reference times, pen-down flags and stroke IDs must be aligned")
    if not np.all(np.isin(down, (False, True))):
        raise ValueError("reference pen-down flags must be boolean")
    if not np.all(np.isfinite(stroke)) or not np.all(stroke == np.floor(stroke)):
        raise ValueError("reference stroke IDs must be finite integers")
    down = down.astype(bool)
    intervals = []
    start = None
    for index in range(len(times)):
        if down[index]:
            if start is None:
                start = index
            elif stroke[index] != stroke[start]:
                if times[index - 1] > times[start]:
                    intervals.append((float(times[start]), float(times[index - 1]), int(stroke[start])))
                start = index
        elif start is not None:
            if times[index - 1] > times[start]:
                intervals.append((float(times[start]), float(times[index - 1]), int(stroke[start])))
            start = None
    if start is not None and times[-1] > times[start]:
        intervals.append((float(times[start]), float(times[-1]), int(stroke[start])))
    return intervals


def build_default_schedule(reference=None, *, enabled: bool = True) -> DisturbanceSchedule:
    """Fit three 2-second windows nearest 20/55/90 seconds inside ink strokes.

    This function is deterministic and independent of controller/error data.
    If a reference cannot fit three full pulses, fail instead of silently
    applying force during pen lifts or dropping requested disturbances.
    """
    if reference is None:
        return DisturbanceSchedule(DEFAULT_PULSES, enabled=enabled)
    intervals = pen_down_intervals(reference)
    aligned = []
    previous_end = 0.0
    for pulse in DEFAULT_PULSES:
        duration = pulse.end_s - pulse.start_s
        candidates = []
        for start, end, _ in intervals:
            first = max(start, previous_end)
            latest = end - duration
            if latest < first:
                continue
            chosen = min(max(pulse.start_s, first), latest)
            candidates.append((abs(chosen - pulse.start_s), chosen, end))
        if not candidates:
            raise ValueError("reference cannot fit three 2-second pen-down disturbance windows")
        _, chosen, interval_end = min(candidates)
        end = min(chosen + duration, interval_end)
        aligned.append(ForcePulse(chosen, end, pulse.force_world, pulse.label))
        previous_end = end
    return DisturbanceSchedule(aligned, enabled=enabled)


def world_tip_position(
    hand_position_world: Sequence[float],
    hand_quaternion_wxyz: Sequence[float],
    tip_offset_local: Sequence[float] = (0.0, 0.0, 0.13),
) -> np.ndarray:
    """Map the hand-frame offset to a real world-space application point."""
    position = _vector3(hand_position_world, "hand_position_world")
    offset = _vector3(tip_offset_local, "tip_offset_local")
    quaternion = np.asarray(hand_quaternion_wxyz, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("hand_quaternion_wxyz must contain four finite values")
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("hand quaternion must have nonzero finite norm")
    w, x, y, z = quaternion / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])
    return position + rotation @ offset


def force_moment_world(
    origin_world: Sequence[float], application_point_world: Sequence[float], force_world: Sequence[float],
) -> np.ndarray:
    """Moment about the supplied origin: (point-origin) x force.

    Use the actual centre of mass for its net moment. This is a diagnostic;
    do not also pass it as a torque when applying the same force at the tip.
    """
    origin = _vector3(origin_world, "origin_world")
    point = _vector3(application_point_world, "application_point_world")
    force = _vector3(force_world, "force_world")
    return np.cross(point - origin, force)


def apply_tip_force(hand_rigid, sample: ForceSample, actual_tip_world: Sequence[float]) -> None:
    """Apply to PhysX immediately before world.step, including explicit zeros.

    Verified against local Isaac Sim 6.0.1 experimental RigidPrim. The API
    keyword is ``local_frame=False``; forces and positions are then both world
    values. PhysX handles the lever-arm moment, so torques stays None. Forces
    are newtons (not impulses): do not multiply by the time step.
    """
    force = _vector3(sample.force_world, "force_world")
    point = _vector3(actual_tip_world, "actual_tip_world")
    if np.any(np.abs(force) > np.finfo(np.float32).max) or np.any(np.abs(point) > np.finfo(np.float32).max):
        raise ValueError("force and position must fit PhysX float32 values")
    hand_rigid.apply_forces_and_torques_at_pos(
        forces=force.reshape(1, 3), torques=None,
        positions=point.reshape(1, 3), local_frame=False,
    )
