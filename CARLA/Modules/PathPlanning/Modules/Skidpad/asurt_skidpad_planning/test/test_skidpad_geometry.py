"""Synthetic two-circle/partial-arc rejection and reset-independent geometry."""
import math
import unittest

from asurt_skidpad_planning.geometry import SkidpadConfig, skidpad_path


def skidpad_cones(loops=2, count=20):
    radius = 9.125
    all_cones = []
    for index, side in enumerate((-1, 1)[:loops]):
        cx, cy = 15., side*radius
        for kind, r in ((0, radius + (1.5 if index == 0 else -1.5)),
                        (1, radius + (-1.5 if index == 0 else 1.5))):
            for i in range(count):
                theta = 2*math.pi*i/count
                all_cones.append((cx+r*math.cos(theta), cy+r*math.sin(theta),
                                  kind, .92))
    return all_cones


class SkidpadGeometryTests(unittest.TestCase):
    def test_two_observed_circles_produce_four_laps(self):
        result = skidpad_path(skidpad_cones(), (0., 0., 0.))
        self.assertEqual(result.reason, 'ok', result.reason)
        self.assertEqual(result.pairs, 2)
        self.assertEqual(len(result.points), 290)
        self.assertAlmostEqual(result.points[1][0], 15., delta=1.)

    def test_one_circle_cannot_establish_eight(self):
        self.assertNotEqual(skidpad_path(skidpad_cones(loops=1),
                                          (0., 0., 0.)).reason, 'ok')

    def test_empty_and_sparse_are_refused(self):
        self.assertFalse(skidpad_path([], (0., 0., 0.)).points)
        self.assertFalse(skidpad_path(skidpad_cones(count=2),
                                      (0., 0., 0.)).points)

    def test_pose_far_from_entry_is_refused(self):
        result = skidpad_path(skidpad_cones(), (0., 20., 0.))
        self.assertNotEqual(result.reason, 'ok')

    def test_colorless_or_low_confidence_arcs_refused(self):
        raw = [(x, y, 4, p) for x, y, _, p in skidpad_cones()]
        self.assertNotEqual(skidpad_path(raw, (0., 0., 0.)).reason, 'ok')
        raw = [(x, y, t, .05) for x, y, t, _ in skidpad_cones()]
        self.assertNotEqual(skidpad_path(raw, (0., 0., 0.)).reason, 'ok')

    def test_invalid_radius_refused(self):
        cfg = SkidpadConfig(radius_m=100.)
        self.assertEqual(skidpad_path(skidpad_cones(), (0., 0., 0.), cfg).reason,
                         'invalid_config')


if __name__ == '__main__':
    unittest.main()
