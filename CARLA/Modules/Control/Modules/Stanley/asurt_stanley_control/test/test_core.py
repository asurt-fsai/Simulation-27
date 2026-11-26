import math
import unittest

from asurt_stanley_control.core import (
    MAX_TRACK_PATH_POINTS, Pose2, Settings, curvature, project_front_axle,
    speed_ramp, track_command,
)


class StanleyTest(unittest.TestCase):
    def test_straight_high_speed_command_can_exceed_pure_pursuit(self):
        path = [(x*.5, 0.) for x in range(201)]
        steer, speed, report = track_command(path, Pose2(2., 0., 0.), 15.)
        self.assertAlmostEqual(steer, 0., places=5)
        self.assertGreater(speed, 20.)
        self.assertLessEqual(speed, 100./3.6)
        self.assertAlmostEqual(report["cross_track_error_m"], 0.)

    def test_correct_left_and_right_steering(self):
        path = [(x*.5, 0.) for x in range(160)]
        left, _, detail = track_command(path, Pose2(4., -0.8, 0.), 8.)
        right, _, _ = track_command(path, Pose2(4., .8, 0.), 8.)
        self.assertGreater(left, 0.)
        self.assertLess(right, 0.)
        self.assertAlmostEqual(left, -right)
        self.assertGreater(detail["cross_track_error_m"], 0.)

    def test_decelerate_before_goal(self):
        path = [(i*.25, 0.) for i in range(21)]
        _, speed, report = track_command(path, Pose2(2., 0., 0.), 2.)
        self.assertLess(speed, 6.)
        self.assertLess(report["remaining_path_m"], 2.)

    def test_corner_speed_below_straight(self):
        straight = [(i*.4, 0.) for i in range(160)]
        curve = [(10*math.sin(t/10.), 10*(1.-math.cos(t/10.)))
                 for t in [i*.3 for i in range(130)]]
        _, fast, _ = track_command(straight, Pose2(0., 0., 0.), 3.)
        _, slow, _ = track_command(curve, Pose2(0., 0., 0.), 3.)
        self.assertLess(slow, fast)

    def test_invalid_route_and_cross_track_refused(self):
        with self.assertRaises(ValueError):
            track_command([(0, 0), (1, 0)], Pose2(0, 0, 0), 1.)
        with self.assertRaises(ValueError):
            track_command([(0, 0), (1, 0), (2, 0)],
                          Pose2(0, 8, 0), 1.)
        with self.assertRaises(ValueError):
            Settings(target_speed_mps=50.).validate()

    def test_autonomous_approaches_tight_bend_and_stops_before_it(self):
        path = [(float(i), 0.) for i in range(13)] + [(12., float(i)) for i in range(1, 12)]
        with self.assertRaisesRegex(ValueError, "path_curvature_exceeds_steering"):
            track_command(path, Pose2(2., 0., 0.), 0.)
        _, approaching, details = track_command(path, Pose2(2., 0., 0.), 0.,
                                                stop_before_tight_bend=True)
        self.assertGreater(approaching, 0.)
        self.assertLess(approaching, 10.)
        self.assertAlmostEqual(details["distance_to_tight_bend_m"], 8.425)
        _, braking, _ = track_command(path, Pose2(7., 0., 0.), 0., stop_before_tight_bend=True)
        self.assertGreater(braking, 0.)
        self.assertLess(braking, approaching)
        _, stopped, _ = track_command(path, Pose2(9., 0., 0.), 0., stop_before_tight_bend=True)
        self.assertEqual(stopped, 0.)

    def test_autonomous_can_follow_a_new_path_or_ignore_a_bend_behind_it(self):
        x, y = 12., 3.
        pose = Pose2(x, y, math.pi/2)
        new_path = [(x, float(i)) for i in range(10)]
        old_path = [(float(i), 0.) for i in range(12)] + new_path
        _, expected_speed, _ = track_command(new_path, pose, 1., stop_before_tight_bend=True)
        _, speed, details = track_command(old_path, pose, 1., stop_before_tight_bend=True)
        self.assertGreater(speed, 0.)
        self.assertAlmostEqual(speed, expected_speed)
        self.assertIsNone(details["distance_to_tight_bend_m"])

    def test_speed_ramp(self):
        self.assertAlmostEqual(speed_ramp(0., 20., 4., .1), .4)
        self.assertEqual(speed_ramp(18., 8., 4., .1), 8.)
        self.assertEqual(speed_ramp(18., 0., 4., .1), 0.)

    def test_full_circuit_path_is_accepted_and_bounded(self):
        path = [(50*math.cos(2*math.pi*i/1800),
                 50*math.sin(2*math.pi*i/1800)) for i in range(1800)]
        steer, speed, report = track_command(
            path, Pose2(50., 0., math.pi/2), 3.)
        self.assertGreater(speed, 0.)
        self.assertLessEqual(abs(steer), math.radians(16.))
        self.assertLess(report["max_route_curvature"], .03)
        with self.assertRaisesRegex(ValueError, "path_size"):
            track_command([(i*.1, 0.)
                           for i in range(MAX_TRACK_PATH_POINTS+1)],
                          Pose2(0., 0., 0.), 3.)


if __name__ == "__main__":
    unittest.main()
