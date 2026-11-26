"""Independent geometry and model contract tests; no simulator or actor truth."""
import math
import unittest
from dataclasses import replace

from asurt_deep_learning_planning.core import (
    Limits, check_prediction, paired_gates, prepare_cones,
    raw_prediction_points, preview_age_allowed,
)


def cones():
    return [(x, y, kind, .96)
            for x in (2., 4., 6., 8., 10.)
            for kind, y in ((0, 2.), (1, -2.))]


def valid_output():
    # Model format x_right, y_forward. Conversion yields REP103 (x, 0).
    return [(0., 1.5 + .5*i) for i in range(15)]


class TestLegacyPlanning(unittest.TestCase):
    def test_late_but_finite_raw_model_result_can_be_viewed_not_controlled(self):
        request_stamp = 10_000_000_000
        age_limit = 3_000_000_000
        self.assertTrue(preview_age_allowed(12_000_000_000, request_stamp,
                                            age_limit, True))
        self.assertFalse(preview_age_allowed(12_000_000_000, request_stamp,
                                             250_000_000, True))
        self.assertFalse(preview_age_allowed(12_000_000_000, request_stamp,
                                             age_limit, False))
        self.assertFalse(preview_age_allowed(14_000_000_000, request_stamp,
                                             age_limit, True))
        self.assertFalse(preview_age_allowed(9_000_000_000, request_stamp,
                                             age_limit, True))

    def test_unvalidated_prediction_preserves_all_fifteen_exact_model_points(self):
        # The model is *predictive*: waypoints beyond the visible cones are
        # neither omitted nor clipped. This topic is visualization-only.
        predictions = [(2.5 + i * .1, 2. + i * 7.) for i in range(15)]
        expected = tuple((forward, -right) for right, forward in predictions)
        self.assertEqual(raw_prediction_points(predictions), expected)
        _, observations, _ = prepare_cones(cones())
        self.assertEqual(check_prediction(predictions, observations)[1],
                         'invalid_waypoint_progress')
        self.assertEqual(len(raw_prediction_points(predictions)), 15)

    def test_raw_prediction_drops_bad_shape_or_nonfinite_without_partial_path(self):
        valid = [(0., 1. + i) for i in range(15)]
        self.assertEqual(raw_prediction_points(valid[:-1]), ())
        self.assertEqual(raw_prediction_points(valid[:2] + [(float('nan'), 3.)]
                                               + valid[3:]), ())
        self.assertEqual(raw_prediction_points(valid[:4] + [(0., float('inf'))]
                                               + valid[5:]), ())
        self.assertEqual(raw_prediction_points(None), ())

    def test_original_input_shape_coordinate_transform_and_padding(self):
        raw = cones()[:6]
        rows, selected, reason = prepare_cones(raw)
        self.assertEqual(reason, 'ok')
        self.assertEqual(len(rows), 10)
        self.assertEqual(rows[0], (-2., 2., 1., 0.))
        self.assertEqual(rows[1], (2., 2., 0., 1.))
        self.assertEqual(rows[6:], ((0., 0., 0., 0.),)*4)
        self.assertEqual(len(selected), 6)

    def test_color_order_is_deterministic_and_orange_not_aliased(self):
        observations = cones()
        extra = [(2., 0., 2, 1.), (4., 0., 3, 1.), (3., 0., 99, 1.)]
        first = prepare_cones(observations + extra)
        second = prepare_cones(list(reversed(observations + extra)))
        self.assertEqual(first, second)
        self.assertEqual(first[2], 'ok')
        self.assertEqual(len(first[1]), 10)
        self.assertTrue(all(c[2] in (0, 1) for c in first[1]))

    def test_invalid_or_one_sided_input_never_invokes_model(self):
        for input_cones in (
            [], [(2., 2., 0, 1.)], cones()[::2],
            [(2., 0., 2, 1.), (4., 0., 3, 1.)],
            [(float('nan'), 2., 0, 1.), (float('inf'), 2., 1, 1.)],
        ):
            self.assertIsNone(prepare_cones(input_cones)[0])

    def test_valid_prediction_in_bounded_corridor(self):
        rows, observed, reason = prepare_cones(cones())
        self.assertEqual(reason, 'ok')
        gates, reason = paired_gates(observed)
        self.assertEqual(reason, 'ok')
        self.assertEqual(len(gates), 5)
        points, reason, count = check_prediction(valid_output(), observed)
        self.assertEqual(reason, 'ok')
        self.assertEqual(count, 5)
        self.assertEqual(points[0], (1.5, -0.))
        self.assertEqual(points[-1], (8.5, -0.))

    def test_reject_nan_bad_shape_reverse_and_curb_crossing(self):
        _, observed, _ = prepare_cones(cones())
        variants = (
            (valid_output()[:-1], 'wrong_output_shape'),
            (valid_output()[:5] + [(float('nan'), 5.)] + valid_output()[6:],
             'nonfinite_output'),
            ([(0., 4.)] + valid_output()[1:], 'invalid_waypoint_progress'),
            ([(2.5, x) for _, x in valid_output()], 'path_outside_cone_boundaries'),
        )
        for output, expected in variants:
            with self.subTest(expected=expected):
                self.assertEqual(check_prediction(output, observed)[1], expected)

    def test_keeps_overlap_and_interpolates_both_limits(self):
        observed = [(x, y, kind, .96) for x in (3.5, 6., 9.5)
                    for kind, y in ((0, 2.), (1, -2.))]
        prediction = [(-.1 * x, x) for x in (.2 + i for i in range(15))]
        raw = raw_prediction_points(prediction)
        points, reason, count = check_prediction(prediction, observed)
        self.assertEqual((reason, count), ('ok', 3))
        self.assertEqual(points[0][0], 1.)
        self.assertAlmostEqual(points[0][1], .1)
        self.assertEqual(points[-1][0], 9.5)
        self.assertAlmostEqual(points[-1][1], .95)
        self.assertLess(len(points), len(raw))
        self.assertEqual(raw_prediction_points(prediction), raw)

    def test_rejects_short_overlap_and_gate_crossing_between_waypoints(self):
        observed = [(x, y, kind, .96) for x in (2., 5.)
                    for kind, y in ((0, 2.), (1, -2.))]
        prediction = [(0., 4.8 + .5 * i) for i in range(15)]
        self.assertEqual(check_prediction(prediction, observed)[1], 'path_outside_observed_extent')
        observed = [(x, y, kind, .96) for x in (2., 4., 6.)
                    for kind, y in ((0, .1 if x == 4. else 2.), (1, -2.5))]
        prediction = [(.2, 1.75 + .5 * i) for i in range(15)]
        self.assertEqual(check_prediction(prediction, observed)[1], 'path_outside_cone_boundaries')

    def test_gates_deny_narrow_and_wrong_side_tracks(self):
        for wrong in (
            [(x, .3 if kind == 0 else -.3, kind, .9) for x, _, kind, _ in cones()],
            [(x, -2. if kind == 0 else 2., kind, .9) for x, _, kind, _ in cones()],
        ):
            self.assertNotEqual(paired_gates(wrong)[1], 'ok')

    def test_first_gate_and_coverage(self):
        _, observed, _ = prepare_cones(cones())
        self.assertEqual(paired_gates(observed, replace(Limits(), max_first_gate_m=1.))[1],
                         'first_gate_too_far')
        self.assertEqual(check_prediction(valid_output(), observed,
                         replace(Limits(), corridor_margin_m=2.2))[1],
                         'path_outside_cone_boundaries')

    def test_rejects_bad_fov_and_low_confidence(self):
        bad = [(x, y, kind, .1) for x, y, kind, _ in cones()]
        self.assertIsNone(prepare_cones(bad)[0])
        sideways = [(2., 9., kind, .95) for kind in (0, 1, 0, 1)]
        self.assertIsNone(prepare_cones(sideways)[0])


if __name__ == '__main__':
    unittest.main()
