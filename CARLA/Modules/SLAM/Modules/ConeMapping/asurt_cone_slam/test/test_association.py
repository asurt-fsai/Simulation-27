import math
import unittest

from asurt_cone_slam.association import ConeMap, transform_xy


class ConeMappingTests(unittest.TestCase):
    def test_repeated_landmarks_confirm_and_associate_once(self):
        mapper = ConeMap(minimum_observations=3)
        for timestamp in (1., 2., 3.):
            mapper.update([(2., 5., 0, .9, 5.4), (2.5, 9., 1, .8, 9.3)], timestamp)
        self.assertEqual(len(mapper.landmarks), 2)
        self.assertEqual(len(mapper.confirmed()), 2)
        self.assertEqual([m.observations for m in mapper.confirmed()], [3, 3])

    def test_color_mismatch_and_far_outlier_remains_tentative(self):
        mapper = ConeMap(minimum_observations=2)
        mapper.update([(2., 5., 0, .9, 5.4)], 1.)
        mapper.update([(2., 5., 0, .9, 5.4)], 1.5)
        mapper.update([(2.1, 5., 1, .9, 5.5), (400., -999., 0, .7, 4.)], 2.)
        self.assertEqual(len(mapper.landmarks), 3)
        self.assertEqual([m.kind for m in mapper.confirmed()], [0])

    def test_nonfinite_and_invalid_class_rejected(self):
        mapper = ConeMap()
        mapper.update([(math.nan, 3., 0, .8, 3.), (1., 2., 0, .8, 99.),
                       (1., 2., 4, .8, 3.), (0., 0., 0, .8, 0.),
                       (1., 2., 0, .8, 3.)], 1.)
        self.assertEqual(len(mapper.landmarks), 1)

    def test_silverstone_flatasphalt_reset(self):
        mapper = ConeMap(minimum_observations=2)
        mapper.update([(-51., -4., 0, .8, 3.)], 1.)
        mapper.update([(-51., -4., 0, .8, 3.)], 2.)
        self.assertEqual(len(mapper.confirmed()), 1)
        mapper.reset()
        mapper.update([(11., 26., 1, .8, 3.)], 1.)
        self.assertEqual(len(mapper.confirmed()), 0)
        self.assertEqual(mapper.landmarks[0].kind, 1)

    def test_tentatives_pruned_but_confirmed_kept(self):
        mapper = ConeMap(minimum_observations=3, max_tentative_missed=3)
        for t in (1., 2., 3.):
            mapper.update([(1., 1., 0, .8, 1.)], t)
        mapper.update([(8., 5., 1, .8, 9.)], 4.)
        for t in (5., 6., 7., 8.):
            mapper.update([], t)
        self.assertEqual([m.kind for m in mapper.landmarks], [0])

    def test_quaternion_transform(self):
        xy = transform_xy((2., 0., 0.), (11., 26., .5),
                          (0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4)))
        self.assertAlmostEqual(xy[0], 11.)
        self.assertAlmostEqual(xy[1], 28.)
        self.assertAlmostEqual(xy[2], .5)


if __name__ == "__main__":
    unittest.main()
