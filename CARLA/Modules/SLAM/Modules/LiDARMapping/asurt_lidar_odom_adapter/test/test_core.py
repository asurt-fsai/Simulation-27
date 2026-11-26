"""Deterministic SE2 positives/negatives; no CARLA actor truth, launches or TF."""
import math
import unittest
import numpy as np

from asurt_lidar_odom_adapter.core import Pair, Pose, TrajectoryCalibration, wrap


def series(n=86, *, mirror=False, scale=1., reverse_yaw=False,
           only_straight=False):
    rotation = .34
    R = np.array([[math.cos(-rotation), -math.sin(-rotation)],
                  [math.sin(-rotation), math.cos(-rotation)]])
    pairs = []
    for i in range(n):
        heading = .2 if only_straight else .2 + .042 * i
        if only_straight:
            bx, by = .36 * i, 0.
        else:
            bx = 8. * (math.sin(heading) - math.sin(.2))
            by = 8. * (math.cos(.2) - math.cos(heading))
        raw = scale * (R @ np.array([bx, by]) + [12., -4.])
        if mirror:
            raw[1] *= -1.
        yaw = wrap(heading - rotation)
        if reverse_yaw:
            yaw = -yaw
        pairs.append(Pair(1. + i * .1, Pose(*raw, yaw),
                          Pose(bx, by, heading)))
    return pairs


class CalibrationTests(unittest.TestCase):
    def run_pairs(self, samples):
        adapter = TrajectoryCalibration()
        outputs = [adapter.feed(p) for p in samples]
        return adapter, [x for x in outputs if x is not None]

    def test_proper_se2_gates_until_real_excitation_and_heldout(self):
        adapter, outputs = self.run_pairs(series())
        self.assertGreater(len(outputs), 4)
        self.assertIsNotNone(adapter.lock)
        self.assertAlmostEqual(adapter.lock.scale_ratio, 1., places=6)
        self.assertAlmostEqual(adapter.lock.angle, .34, places=5)
        self.assertAlmostEqual(outputs[-1].pose.x, series()[-1].bridge.x, places=5)
        self.assertAlmostEqual(outputs[-1].pose.y, series()[-1].bridge.y, places=5)
        self.assertEqual(adapter.state, 'validated_LeGO_derived')
        self.assertGreaterEqual(outputs[-1].var_xyyaw[0], .025)

    def test_reflection_rejected_despite_consistent_translation_scale(self):
        adapter, outputs = self.run_pairs(series(mirror=True))
        self.assertFalse(outputs)
        self.assertIsNone(adapter.lock)
        self.assertIn(adapter.state, (
            'reflection_detected', 'train_position_residual',
            'quaternion_yaw_inconsistent'))

    def test_misoriented_or_reversed_yaw_rejected(self):
        adapter, outputs = self.run_pairs(series(reverse_yaw=True))
        self.assertFalse(outputs)
        self.assertEqual(adapter.state, 'quaternion_yaw_inconsistent')

    def test_wrong_scale_rejected_without_scale_fitting(self):
        adapter, outputs = self.run_pairs(series(scale=.7))
        self.assertFalse(outputs)
        self.assertEqual(adapter.state, 'scale_mismatch')

    def test_straight_line_cannot_claim_heading_calibration(self):
        adapter, outputs = self.run_pairs(series(only_straight=True))
        self.assertFalse(outputs)
        self.assertIsNone(adapter.lock)
        self.assertEqual(adapter.state, 'insufficient_translation_or_turn')

    def test_teleport_discards_lock_and_output_until_recalibration(self):
        adapter, outputs = self.run_pairs(series())
        self.assertTrue(outputs)
        a = series()[-1]
        jumped = Pair(a.t + .1,
                      Pose(a.lidar.x + 55., a.lidar.y, a.lidar.yaw),
                      Pose(a.bridge.x + 55., a.bridge.y, a.bridge.yaw))
        self.assertIsNone(adapter.feed(jumped))
        self.assertIsNone(adapter.lock)
        self.assertEqual(adapter.state, 'collecting_correspondences')
        self.assertGreater(adapter.resets, 0)

    def test_time_rewind_clears_calibration(self):
        adapter, outputs = self.run_pairs(series())
        self.assertTrue(outputs)
        self.assertIsNone(adapter.feed(series(1)[0]))
        self.assertIsNone(adapter.lock)
        self.assertGreater(adapter.resets, 0)

    def test_after_lock_lidar_variation_is_not_bridge_republication(self):
        adapter, outputs = self.run_pairs(series())
        last = series()[-1]
        next_t = last.t + .1
        changed = Pair(next_t,
                       Pose(last.lidar.x + .22, last.lidar.y, last.lidar.yaw),
                       last.bridge)
        out = adapter.feed(changed)
        self.assertIsNotNone(out)
        self.assertGreater(math.hypot(
            out.pose.x - outputs[-1].pose.x,
            out.pose.y - outputs[-1].pose.y), .20)
        self.assertEqual(adapter.state, 'validated_LeGO_derived')

    def test_postlock_large_drift_revokes_validation(self):
        adapter, outputs = self.run_pairs(series())
        self.assertTrue(outputs)
        last = series()[-1]
        mismatch = Pair(last.t + .1,
                        Pose(last.lidar.x + 1.5, last.lidar.y, last.lidar.yaw),
                        last.bridge)
        self.assertIsNone(adapter.feed(mismatch))
        self.assertIsNone(adapter.lock)
        self.assertEqual(adapter.state, 'postlock_bridge_disagreement')

    def test_bounded_memory_and_parameter_guard(self):
        a = TrajectoryCalibration(min_pairs=30, max_pairs=48)
        for p in series():
            a.feed(p)
        self.assertLessEqual(len(a.samples), 48)
        with self.assertRaises(ValueError):
            TrajectoryCalibration(max_pairs=500)


if __name__ == '__main__':
    unittest.main()
