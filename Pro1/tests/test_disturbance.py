"""Repeatable force timing, pen-down alignment and physical application contract."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from raa.disturbance import (DEFAULT_PULSES, DisturbanceSchedule, ForcePulse,
    apply_tip_force, build_default_schedule, force_moment_world,
    pen_down_intervals, world_tip_position)
from raa.paths import build_reference


class DisturbanceTests(unittest.TestCase):
    def test_half_open_boundaries_and_disabled_control_trial(self):
        schedule = DisturbanceSchedule()
        for index, pulse in enumerate(DEFAULT_PULSES):
            self.assertFalse(schedule.sample(np.nextafter(pulse.start_s, 0)).active)
            at_start = schedule.sample(pulse.start_s)
            self.assertTrue(at_start.active)
            self.assertEqual(at_start.pulse_index, index)
            np.testing.assert_array_equal(at_start.force_world, pulse.force_world)
            self.assertTrue(schedule.sample(np.nextafter(pulse.end_s, 0)).active)
            self.assertFalse(schedule.sample(pulse.end_s).active)
        disabled = DisturbanceSchedule(enabled=False)
        for time in (0, 20, 21, 55, 56, 90, 91, 120):
            sample = disabled.sample(time)
            self.assertFalse(sample.active)
            self.assertEqual(sample.pulse_index, -1)
            np.testing.assert_array_equal(sample.force_world, np.zeros(3))

    def test_integer_step_schedule_has_exact_duration_and_force_impulse(self):
        schedule = DisturbanceSchedule()
        impulse = np.zeros(3)
        active_counts = np.zeros(3, dtype=int)
        # Three physical 2-second forces at 1 kHz: impulses are F*duration.
        for step in range(93001):
            sample = schedule.sample_step(step, 0.001)
            impulse += sample.force_world * 0.001
            if sample.active:
                active_counts[sample.pulse_index] += 1
        np.testing.assert_array_equal(active_counts, [2000, 2000, 2000])
        np.testing.assert_allclose(impulse, [10, -2, 0], atol=1e-10)

    def test_default_logo_uses_identical_pen_down_windows_without_mutation(self):
        reference = build_reference(width=0.16, draw_z=0.305, dt=1 / 60)
        original_times = reference.times.copy()
        original_positions = reference.positions.copy()
        original_flags = reference.pen_down.copy()
        times = np.asarray(reference.times)
        flags = np.asarray(reference.pen_down)
        enabled = build_default_schedule(reference)
        disabled = build_default_schedule(reference, enabled=False)
        self.assertEqual(enabled.pulses, disabled.pulses)
        self.assertEqual(enabled.pulses, DEFAULT_PULSES)
        for pulse in enabled.pulses:
            active_times = np.arange(int(pulse.start_s * 1000), int(pulse.end_s * 1000)) * 0.001
            indices = np.searchsorted(times, active_times, side="right") - 1
            self.assertTrue(np.all(flags[indices] & flags[indices + 1]))
        self.assertEqual(reference.times, original_times)
        self.assertEqual(reference.positions, original_positions)
        self.assertEqual(reference.pen_down, original_flags)
        json.dumps(enabled.metadata(), allow_nan=False)

    def test_alignment_moves_force_out_of_lifts_using_reference_only(self):
        times = np.arange(0, 101, 0.5)
        down = np.zeros(len(times), dtype=bool)
        stroke = np.full(len(times), -1)
        for sid, (start, end) in enumerate(((24, 27), (58, 61), (95, 98))):
            mask = (times >= start) & (times <= end)
            down[mask] = True
            stroke[mask] = sid
        reference = SimpleNamespace(times=times, pen_down=down, stroke_ids=stroke)
        schedule = build_default_schedule(reference)
        self.assertEqual([(p.start_s, p.end_s) for p in schedule.pulses], [(24, 26), (58, 60), (95, 97)])
        repeated = build_default_schedule(reference)
        self.assertEqual(schedule.pulses, repeated.pulses)
        for pulse, (_, _, sid) in zip(schedule.pulses, pen_down_intervals(reference)):
            sample = schedule.sample((pulse.start_s + pulse.end_s) / 2)
            self.assertTrue(sample.active)
            self.assertGreaterEqual(sid, 0)

    def test_stroke_ids_split_intervals_and_short_paths_fail_explicitly(self):
        reference = SimpleNamespace(times=np.arange(9), pen_down=np.ones(9, dtype=bool),
            stroke_ids=np.array([0, 0, 1, 1, 2, 2, 3, 3, 3]))
        self.assertEqual(pen_down_intervals(reference), [(0, 1, 0), (2, 3, 1), (4, 5, 2), (6, 8, 3)])
        with self.assertRaises(ValueError):
            build_default_schedule(reference)
        reference.times[3] = reference.times[2]
        with self.assertRaises(ValueError):
            pen_down_intervals(reference)

    def test_tip_transform_and_lever_arm_moment(self):
        origin = np.array([1., 2., 3.])
        tip = world_tip_position(origin, [0, 0, 1, 0])
        np.testing.assert_allclose(tip, [1, 2, 2.87])
        # A +X force at a tip below the hand produces -Y moment.
        np.testing.assert_allclose(force_moment_world(origin, tip, [3, 0, 0]), [0, -0.39, 0], atol=1e-14)
        np.testing.assert_allclose(force_moment_world(origin, tip, [0, -3, 0]), [-0.39, 0, 0], atol=1e-14)
        # Quaternion sign and normalization describe the same physical frame.
        np.testing.assert_allclose(world_tip_position(origin, [0, 0, -2, 0]), tip)
        custom = world_tip_position(origin, [1, 0, 0, 0], (0.02, -0.01, 0.1))
        np.testing.assert_allclose(custom, [1.02, 1.99, 3.1])

    def test_application_uses_absolute_world_point_and_newtons_without_double_moment(self):
        class RecordedRigid:
            def __init__(self):
                self.calls = []
            def apply_forces_and_torques_at_pos(self, **kwargs):
                self.calls.append(kwargs)
        rigid = RecordedRigid()
        schedule = DisturbanceSchedule()
        point = np.array([0.45, 0.02, 0.305])
        apply_tip_force(rigid, schedule.sample(20.5), point)
        call = rigid.calls[-1]
        np.testing.assert_array_equal(call["forces"], [[3, 0, 0]])
        np.testing.assert_array_equal(call["positions"], point[None, :])
        self.assertIsNone(call["torques"])
        self.assertIs(call["local_frame"], False)
        apply_tip_force(rigid, schedule.sample(22), point)
        np.testing.assert_array_equal(rigid.calls[-1]["forces"], np.zeros((1, 3)))
        self.assertEqual(len(rigid.calls), 2)

    def test_invalid_timing_force_and_overlapping_pulses_are_rejected(self):
        for start, end in ((-1, 2), (2, 2), (3, 2), (np.nan, 2), (0, np.inf)):
            with self.assertRaises(ValueError):
                ForcePulse(start, end, (1, 0, 0))
        for force in ((0, 0, 0), (1, 2), (np.nan, 0, 0), (np.inf, 0, 0), (1e100, 0, 0)):
            with self.assertRaises(ValueError):
                ForcePulse(1, 2, force)
        with self.assertRaises(ValueError):
            DisturbanceSchedule([ForcePulse(0, 2, (1, 0, 0)), ForcePulse(1, 3, (0, 1, 0))])
        with self.assertRaises(TypeError):
            DisturbanceSchedule(["bad pulse"])
        with self.assertRaises(ValueError):
            DisturbanceSchedule(enabled="false")
        schedule = DisturbanceSchedule()
        for time in (-1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                schedule.sample(time)
        for step, dt in ((-1, 0.001), (0.5, 0.001), (True, 0.001), (0, 0), (0, np.nan)):
            with self.assertRaises(ValueError):
                schedule.sample_step(step, dt)
        with self.assertRaises(ValueError):
            world_tip_position([0, 0, 0], [0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
