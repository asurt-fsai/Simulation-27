"""Clock alignment tests independent of ROS, CARLA, and the active simulator."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from asurt_velodyne_sim.clock_alignment import ClockOffsetCalibrator


class ClockAlignmentTests(unittest.TestCase):
    def test_median_rejects_outlier_and_preserves_capture_stamp(self):
        sync = ClockOffsetCalibrator()
        expected = 1795186301.412895
        jitter = [(i % 7 - 3) * 0.007 for i in range(21)]
        jitter[3] = 0.50
        for i, deviation in enumerate(jitter):
            sim_time = 500.0 + i * 0.1
            locked = sync.observe(
                expected + sim_time + deviation, sim_time, i * 0.1, i + 100
            )
            self.assertEqual(locked, i == 20)
            if i < 20:
                self.assertIsNone(sync.offset)
        self.assertEqual(len(sync.samples), 21)
        self.assertAlmostEqual(sync.window_seconds, 2.0)
        self.assertAlmostEqual(sync.offset, expected, delta=0.008)
        self.assertAlmostEqual(sync.offset + 503.25, expected + 503.25,
                               delta=0.008)
        self.assertFalse(sync.due(999.0))  # frozen, zero future snapshot reads
        self.assertFalse(sync.observe(expected + 1000, 1000, 999, 900))
        self.assertEqual(len(sync.samples), 21)

    def test_spacing_duplicate_frames_and_nonfinite_pairs(self):
        sync = ClockOffsetCalibrator()
        self.assertTrue(sync.due(0.0))
        self.assertFalse(sync.observe(100.0, 1.0, 0.0, 42))
        self.assertFalse(sync.due(0.05))
        self.assertFalse(sync.observe(100.5, 1.5, 0.05, 43))
        self.assertFalse(sync.observe(101.0, 2.0, 0.11, 42))
        self.assertFalse(sync.due(0.12))
        self.assertFalse(sync.observe(float("nan"), 2.0, 0.22, 43))
        self.assertFalse(sync.observe(101.0, 2.0, 0.33, 43))
        self.assertEqual(len(sync.samples), 2)
        self.assertIsNone(sync.offset)

    def test_failed_snapshot_attempts_are_throttled(self):
        sync = ClockOffsetCalibrator()
        self.assertTrue(sync.due(0.0))
        sync.defer(0.0)
        self.assertFalse(sync.due(0.05))
        self.assertTrue(sync.due(0.10))

    def test_bounded_partial_window_finalizes_with_15_samples(self):
        sync = ClockOffsetCalibrator()
        for i in range(16):
            now = i * 0.21
            locked = sync.observe(1000.0 + 10 + now, 10 + now, now, i)
        self.assertTrue(locked)
        self.assertEqual(len(sync.samples), 16)
        self.assertAlmostEqual(sync.offset, 1000.0)
        self.assertGreaterEqual(sync.window_seconds, 3.0)


if __name__ == "__main__":
    unittest.main()
