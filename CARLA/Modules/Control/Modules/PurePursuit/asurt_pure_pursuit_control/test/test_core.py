"""Pure Python geometry tests; these never touch CARLA or ROS."""
import math
import unittest

from asurt_pure_pursuit_control.core import (
    Settings, VehiclePose, current_local_path, local_to_map,
    lookahead_distance, map_to_local, ramp_speed, steering_and_speed,
    target_on_path,
)


class TestPurePursuit(unittest.TestCase):
    def test_legacy_fixed_and_adaptive_lookahead(self):
        cfg = Settings()
        self.assertEqual(lookahead_distance(0., cfg), 1.8)
        self.assertAlmostEqual(lookahead_distance(2., cfg), 2.9)
        self.assertEqual(lookahead_distance(100., cfg), 4.)

    def test_straight_target_and_steer(self):
        cfg = Settings()
        target = target_on_path([(0, 0), (5, 0), (12, 0)], 2.)
        self.assertEqual(target, (2., 0.))
        steer, speed, curvature = steering_and_speed(target, cfg)
        self.assertEqual((steer, speed, curvature), (0., .8, 0.))

    def test_left_and_right_have_ros_sign(self):
        cfg = Settings()
        left = target_on_path([(0, 0), (5, 2), (10, 4)], 2.)
        right = target_on_path([(0, 0), (5, -2), (10, -4)], 2.)
        l_steer, l_speed, _ = steering_and_speed(left, cfg)
        r_steer, r_speed, _ = steering_and_speed(right, cfg)
        self.assertGreater(l_steer, 0.)
        self.assertAlmostEqual(l_steer, -r_steer)
        self.assertLess(l_speed, cfg.target_speed_mps)
        self.assertAlmostEqual(l_speed, r_speed)

    def test_continuous_target_for_car_already_on_map_path(self):
        now = VehiclePose(6., 0., 0.)
        local = current_local_path([(0., 0.), (5., 0.), (10., 0.),
                                    (15., 0.)], "map", now)
        target = target_on_path(local, 2.)
        self.assertAlmostEqual(target[0], 2.)
        self.assertAlmostEqual(target[1], 0.)

    def test_source_timestamp_reprojection_for_moving_car(self):
        then = VehiclePose(3., 2., math.pi/2)
        now = VehiclePose(3., 3., math.pi/2)
        local = current_local_path([(2., 0.), (4., 0.), (8., 1.)],
                                   "base_link", now, then)
        self.assertAlmostEqual(local[0][0], 1.)
        self.assertAlmostEqual(local[0][1], 0.)
        self.assertAlmostEqual(local[-1][0], 7.)
        self.assertAlmostEqual(local[-1][1], 1.)
        self.assertAlmostEqual(map_to_local(local_to_map((2., 1.), then),
                                            then)[0], 2.)

    def test_missing_origin_or_wrong_frame_is_not_guessed(self):
        with self.assertRaises(ValueError):
            current_local_path([(1., 0.)], "base_link",
                               VehiclePose(0., 0., 0.))
        with self.assertRaises(ValueError):
            current_local_path([(1., 0.)], "zed_left_camera",
                               VehiclePose(0., 0., 0.))

    def test_short_nan_and_behind_paths_not_driven(self):
        self.assertIsNone(target_on_path([(1., 0.)], 1.))
        self.assertIsNone(target_on_path([(0., 0.), (float("nan"), 0.)], 1.))
        self.assertIsNone(target_on_path([(-5., 0.), (-1., 0.)], 2.))
        with self.assertRaises(ValueError):
            steering_and_speed((-.1, 0.), Settings())

    def test_steer_saturation_and_speed_limits(self):
        cfg = Settings()
        steer, speed, curvature = steering_and_speed((.2, 1.8), cfg)
        self.assertAlmostEqual(steer, cfg.max_steer_rad)
        self.assertLessEqual(speed, cfg.target_speed_mps)
        self.assertGreaterEqual(speed, cfg.min_speed_mps)
        self.assertGreater(curvature, 0.)

    def test_speed_ramp_and_immediate_stop(self):
        self.assertAlmostEqual(ramp_speed(0., 1.0, .8, .1), .08)
        self.assertEqual(ramp_speed(.4, 0., .8, .1), 0.)
        self.assertEqual(ramp_speed(1., .4, .8, .1), .4)

    def test_reject_inconsistent_config(self):
        with self.assertRaises(ValueError):
            Settings(wheelbase_m=-1.).validate()
        with self.assertRaises(ValueError):
            Settings(max_steer_rad=float("nan")).validate()
        with self.assertRaises(ValueError):
            Settings(min_speed_mps=2.).validate()


if __name__ == "__main__":
    unittest.main()
