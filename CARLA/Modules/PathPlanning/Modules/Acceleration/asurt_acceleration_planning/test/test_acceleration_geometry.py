"""Acceleration-specific synthetic geometry and refusal cases."""
import unittest

from asurt_acceleration_planning.geometry import AccelerationConfig, acceleration_path


class AccelerationTests(unittest.TestCase):
    def test_straight_gate_corridor(self):
        raw = ([(2+i*2., 2., 0, .9) for i in range(6)]
               + [(2+i*2., -2., 1, .9) for i in range(6)])
        result = acceleration_path(raw, (0., 0., 0.))
        self.assertEqual(result.reason, 'ok')
        self.assertGreater(result.points[-1][0], 12.)
        self.assertLessEqual(result.points[-1][0], 14.01)
        self.assertTrue(all(abs(y) < 1e-6 for _, y in result.points))

    def test_reject_curving_centerline(self):
        raw = [(3., 2., 0, .95), (6., 0., 0, .95), (9., -3., 0, .95),
               (3., -2., 1, .95), (6., -4., 1, .95), (9., -7., 1, .95)]
        self.assertEqual(acceleration_path(raw, (0., 0., 0.)).reason,
                         'non_straight_acceleration_heading')

    def test_no_missing_side_fallback(self):
        raw = [(2., 2., 0, .95), (4., 2., 0, .95)]
        self.assertNotEqual(acceleration_path(raw, (0., 0., 0.)).reason, 'ok')

    def test_no_unobserved_long_extrapolation(self):
        raw = ([(2+i*2., 2., 0, .9) for i in range(6)]
               + [(2+i*2., -2., 1, .9) for i in range(6)])
        c = AccelerationConfig(prediction_extension_m=90.)
        self.assertEqual(acceleration_path(raw, (0., 0., 0.), c).reason, 'invalid_config')


if __name__ == '__main__':
    unittest.main()
