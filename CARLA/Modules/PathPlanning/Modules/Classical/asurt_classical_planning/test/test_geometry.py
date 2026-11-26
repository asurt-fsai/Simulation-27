"""Synthetic track geometry tests; never connect to live CARLA."""

import math
import unittest

from asurt_classical_planning.geometry import Config, centerline, in_map


def corridor(n=6, lateral=2.0):
    return ([(2.0+i*2, lateral, 0, .9) for i in range(n)]
            + [(2.0+i*2, -lateral, 1, .9) for i in range(n)])


def peak_curvature(points):
    def magnitude(a, b, c):
        twice_area = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
        return 2*twice_area/(math.dist(a, b)*math.dist(b, c)*math.dist(a, c))

    return max(magnitude(*triple) for triple in zip(points, points[1:], points[2:]))


class GeometryTests(unittest.TestCase):
    def test_straight_corridor(self):
        result = centerline(corridor(), (0., 0., 0.))
        self.assertEqual(result.reason, 'ok')
        self.assertEqual(result.pairs, 6)
        self.assertGreaterEqual(len(result.points), 16)
        self.assertTrue(all(abs(y) < 1e-6 for _, y in result.points))
        self.assertAlmostEqual(result.points[-1][0], 12., places=6)

    def test_rotated_and_translated_map(self):
        p = (101., -40., math.pi/2)
        raw = corridor()
        world = in_map(raw, p)
        result = centerline(world, p)
        self.assertEqual(result.reason, 'ok')
        self.assertAlmostEqual(result.points[0][0], 101., places=6)
        self.assertAlmostEqual(result.points[-1][1], -28., places=6)

    def test_reversed_colours_are_rejected(self):
        wrong = [(x, y, 1 if kind == 0 else 0, conf) for x, y, kind, conf in corridor()]
        self.assertNotEqual(centerline(wrong, (0., 0., 0.)).reason, 'ok')

    def test_one_boundary_never_generates_a_path(self):
        result = centerline(corridor()[:6], (0., 0., 0.))
        self.assertEqual(result.reason, 'need_two_cones_each_side')
        self.assertFalse(result.points)

    def test_missing_and_unknown_detections(self):
        sparse = [(2., 2., 0, 1.), (2., -2., 1, 1.),
                  (4., 0., 4, 1.), (5., 2., 0, .02)]
        self.assertNotEqual(centerline(sparse, (0., 0., 0.)).reason, 'ok')

    def test_reject_non_track_and_remote_gates(self):
        narrow = [(x, y*.1, kind, p) for x, y, kind, p in corridor()]
        self.assertNotEqual(centerline(narrow, (0., 0., 0.)).reason, 'ok')
        distant = [(x+30, y, kind, p) for x, y, kind, p in corridor()]
        self.assertNotEqual(centerline(distant, (0., 0., 0.)).reason, 'ok')

    def test_deterministic_bounds_and_invalid_pose(self):
        raw = corridor(80)
        cfg = Config(max_cones=12)
        a = centerline(raw, (0., 0., 0.), cfg)
        b = centerline(raw, (0., 0., 0.), cfg)
        self.assertEqual(a, b)
        self.assertNotEqual(centerline(raw, (float('nan'), 0., 0.)).reason, 'ok')

    def test_turn_horizon_and_no_extrapolation(self):
        points = corridor(n=4)
        result = centerline(points, (0., 0., 0.), Config(lookahead_m=6.1))
        self.assertEqual(result.reason, 'ok')
        self.assertLessEqual(result.points[-1][0], 6.1)

    def test_right_bend_with_both_boundaries_laterally_right(self):
        # After the first gate, a curving corridor can be wholly to the right
        # of the vehicle's initial heading, while maintaining blue-left order.
        raw = [(3., 2., 0, .9), (6., 0., 0, .9), (9., -3., 0, .9),
               (3., -2., 1, .9), (6., -4., 1, .9), (9., -7., 1, .9)]
        result = centerline(raw, (0., 0., 0.))
        self.assertEqual(result.reason, 'ok')
        self.assertEqual(result.pairs, 3)
        self.assertLess(result.points[-1][1], -4.0)

    def test_remote_parallel_lane_is_not_plannable(self):
        raw = [(3., 8., 0, .9), (6., 8., 0, .9),
               (3., 4., 1, .9), (6., 4., 1, .9)]
        self.assertNotEqual(centerline(raw, (0., 0., 0.)).reason, 'ok')

    def test_gradual_bend_has_no_curvature_spikes_at_paired_gates(self):
        gates = [(1., -.1), (4.5, -.3), (8., -.85), (13., -2.55), (16.5, -4.4)]
        cones = [(x, y+offset, kind, .9) for x, y in gates
                 for offset, kind in [(2., 0), (-2., 1)]]
        for spacing in (.25, .5):
            with self.subTest(spacing=spacing):
                result = centerline(cones, (0., 0., 0.), Config(sample_spacing_m=spacing))
                self.assertEqual(result.reason, 'ok')
                self.assertEqual(result.pairs, len(gates))
                self.assertLess(math.dist(result.points[0], gates[0]), 1e-7)
                self.assertLess(math.dist(result.points[-1], gates[-1]), 1e-7)
                self.assertLess(peak_curvature(result.points), .08)
                for start, end in zip(result.points, result.points[1:]):
                    self.assertLessEqual(math.dist(start, end), spacing+1e-7)

    def test_rounding_preserves_curvature_of_a_genuinely_tight_bend(self):
        gates = [(1., 0.), (2., -.5), (3., 0.)]
        cones = [(x, y+offset, kind, .9) for x, y in gates
                 for offset, kind in [(2., 0), (-2., 1)]]
        result = centerline(cones, (0., 0., 0.))
        self.assertEqual(result.reason, 'ok')
        self.assertGreater(peak_curvature(result.points), .182)


if __name__ == '__main__':
    unittest.main()
