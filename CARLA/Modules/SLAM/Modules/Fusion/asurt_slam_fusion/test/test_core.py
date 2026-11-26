import math
import unittest

import numpy as np

from asurt_slam_fusion.core import (
    Cone, LidarPoseCorrector, Pose2, between, compose, fuse_cones,
    rebase_point, transform_local, wrap,
)


class FusionMathTests(unittest.TestCase):
    def test_one_to_one_colour_and_lidar_precision(self):
        camera = [Cone(5.6, 1., .2, 0, .9, 13),
                  Cone(5.0, -2., .2, 1, .9, 14)]
        lidar = [Cone(5.0, -1.98, .18, 4, .7),
                 Cone(5.0, 1.04, .18, 4, .85),
                 Cone(5.04, 1.03, .20, 4, .7)]
        result, counts = fuse_cones(camera, lidar)
        self.assertEqual(counts['matched'], 2)
        self.assertEqual(counts['unmatched_lidar'], 1)
        self.assertEqual([d.kind for d in result], [0, 1])
        self.assertEqual([d.identifier for d in result], [13, 14])
        self.assertLess(abs(result[0].x - 5.0), abs(result[0].x - 5.6))

    def test_no_false_colour_match_or_far_pair(self):
        result, info = fuse_cones([Cone(2., 0., 0., 0, .8)],
                                  [Cone(2., 0., 0., 1, .9)])
        self.assertEqual(result, [])
        self.assertEqual(info['matched'], 0)
        result, _ = fuse_cones([Cone(2., 0., 0., 0, .8)],
                               [Cone(5., 0., 0., 4, .9)])
        self.assertFalse(result)

    def test_invalid_measurement_and_bounds(self):
        camera = [Cone(3., .5, .2, 0, .8)] * 90
        lidar = [Cone(3.1, .5, .2, 4, .8)] * 90
        fused, counts = fuse_cones(camera, lidar)
        self.assertEqual(counts['camera'], 64)
        self.assertEqual(counts['matched'], 64)
        bad, _ = fuse_cones([Cone(float('nan'), 1., 0., 0, .9)], lidar)
        self.assertEqual(bad, [])

    def test_pose_geometry_and_ego_compensation(self):
        a = Pose2(3., -2., .45)
        b = Pose2(5., 1., -.2)
        reconstructed = compose(a, between(a, b))
        self.assertTrue(np.allclose((reconstructed.x, reconstructed.y),
                                    (b.x, b.y), atol=1e-9))
        self.assertAlmostEqual(wrap(reconstructed.yaw-b.yaw), 0., places=9)
        transformed = rebase_point((4., 0., .1), Pose2(1., 0., 0.),
                                   Pose2(3., 0., 0.))
        self.assertTrue(np.allclose(transformed, (2., 0., .1)))

    def test_estimator_requires_map_and_lidar_motion(self):
        e = LidarPoseCorrector()
        points = [Cone(3., -1., 0., 0, .9),
                  Cone(4., 1.7, 0., 1, .9),
                  Cone(7., -.6, 0., 0, .9),
                  Cone(8., 2.3, 0., 1, .9)]
        current, reason = e.correct(points, .0)
        self.assertIsNone(current)
        self.assertIn('lidar_odom', reason)
        self.assertEqual(e.integrate(Pose2(0., 0., 0.), .1,
                                     Pose2(0., 0., 0.)), 'anchored')
        result, reason = e.correct(points, .1)
        self.assertIsNone(result)
        self.assertIn('map_matches', reason)
        e.learn(points, Pose2(0., 0., 0.), .1)
        e.learn(points, Pose2(0., 0., 0.), .2)
        self.assertEqual(e.integrate(Pose2(.25, 0., 0.), .3), 'integrated')
        result, reason = e.correct(points, .3)
        self.assertEqual(reason, 'corrected')
        self.assertIsNotNone(result)
        pose, cov = result
        self.assertGreater(pose.x, 0.)
        self.assertLess(pose.x, .25)
        self.assertLess(e.last_residual, 1e-5)
        self.assertTrue(np.all(cov.diagonal() > 0))

    def test_outlier_odom_and_map_ambiguity_are_gated(self):
        e = LidarPoseCorrector()
        line = [Cone(2.+i, 0., .1, i%2, .8) for i in range(4)]
        e.learn(line, Pose2(0., 0., 0.), 1.)
        e.learn(line, Pose2(0., 0., 0.), 2.)
        self.assertEqual(e.integrate(Pose2(0., 0., 0.), 2.0,
                                     Pose2(0., 0., 0.)), 'anchored')
        self.assertEqual(e.integrate(Pose2(8., 0., 0.), 2.1),
                         'lidar_odometry_jump')
        result, _ = e.correct(line, 2.1)
        self.assertIsNone(result)
        e.reset()
        self.assertFalse(e.anchors)
        self.assertIsNone(e.prediction)

    def test_anchor_memory_cap_and_expiration(self):
        e = LidarPoseCorrector(max_anchors=3, map_ttl_s=2.)
        points = [Cone(3. + 2*i, float(i), 0., i%2, .8)
                  for i in range(10)]
        e.learn(points, Pose2(0., 0., 0.), 1.)
        self.assertEqual(len(e.anchors), 3)
        e.learn([], Pose2(0., 0., 0.), 4.)
        self.assertFalse(e.anchors)

    def test_rotation_correction_is_bounded_and_directional(self):
        e = LidarPoseCorrector()
        points = [Cone(x, y, .1, kind, .9) for x, y, kind in
                  ((4., -2., 0), (7., 2., 1), (8., -1., 0), (11., 4., 1))]
        for t in (1., 1.1):
            e.learn(points, Pose2(0., 0., 0.), t)
        self.assertEqual(e.integrate(Pose2(0., 0., 0.), 1.1,
                                     Pose2(0., 0., 0.)), 'anchored')
        self.assertEqual(e.integrate(Pose2(0., 0., .06), 1.2), 'integrated')
        result, state = e.correct(points, 1.2)
        self.assertEqual(state, 'corrected')
        self.assertIsNotNone(result)
        self.assertLess(result[0].yaw, .06)
        self.assertGreater(result[0].yaw, 0.)


if __name__ == '__main__':
    unittest.main()
