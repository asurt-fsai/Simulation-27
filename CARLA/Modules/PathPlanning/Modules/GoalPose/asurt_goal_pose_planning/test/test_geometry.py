"""Dubins analytical endpoint, turning radius, goal-frame and bounds tests."""
import math
import random
import unittest

from asurt_goal_pose_planning.geometry import (
    Pose2, angle, dubins_path, local_goal_to_map,
)


class DubinsTests(unittest.TestCase):
    def test_straight_waypoints_in_map(self):
        points, length, word = dubins_path(
            Pose2(1., 2., 0.), Pose2(9., 2., 0.), 3.26)
        self.assertEqual(points[0], Pose2(1., 2., 0.))
        self.assertEqual(points[-1], Pose2(9., 2., 0.))
        self.assertAlmostEqual(length, 8., places=5)
        self.assertTrue(all(abs(p.y-2.) < 1e-5 for p in points))
        self.assertLessEqual(len(points), 600)

    def test_all_forward_dubins_families_and_exact_pose(self):
        rng = random.Random(20261123)
        observed = set()
        for _ in range(250):
            start = Pose2(0., 0., rng.uniform(-math.pi, math.pi))
            goal = Pose2(rng.uniform(-20, 20), rng.uniform(-20, 20),
                         rng.uniform(-math.pi, math.pi))
            points, length, word = dubins_path(start, goal, 3.26)
            self.assertGreater(len(points), 2)
            self.assertEqual(points[0], start)
            self.assertLess(math.dist((points[-1].x, points[-1].y),
                                      (goal.x, goal.y)), 1e-4)
            self.assertLess(abs(angle(points[-1].yaw-goal.yaw)), 1e-4)
            self.assertLessEqual(length, 100.)
            observed.add(word)
            for a, b in zip(points, points[1:]):
                ds = math.hypot(b.x-a.x, b.y-a.y)
                self.assertLess(ds, .31)
                self.assertLessEqual(abs(angle(b.yaw-a.yaw)),
                                     ds/3.26+.002)
        self.assertEqual(observed, {"LSL", "LSR", "RSL", "RSR", "LRL", "RLR"})

    def test_base_link_goal_converted_once_including_heading(self):
        source = Pose2(12., -4., math.pi/2.)
        mapped = local_goal_to_map(Pose2(5., -2., -.25), source)
        self.assertAlmostEqual(mapped.x, 14.)
        self.assertAlmostEqual(mapped.y, 1.)
        self.assertAlmostEqual(mapped.yaw, math.pi/2-.25)

    def test_constraints_reject_impossible_or_corrupt_inputs(self):
        self.assertEqual(dubins_path(Pose2(0, 0, 0), Pose2(12, 0, 0),
                                     3.26, max_length=3.)[0], ())
        self.assertEqual(dubins_path(Pose2(0, 0, 0), Pose2(12, 0, 0),
                                     3.26, max_points=3)[0], ())
        with self.assertRaises(ValueError):
            dubins_path(Pose2(0, 0, 0), Pose2(float("nan"), 0, 0), 3.26)
        with self.assertRaises(ValueError):
            dubins_path(Pose2(0, 0, 0), Pose2(3, 0, 0), 0.)


if __name__ == "__main__":
    unittest.main()
