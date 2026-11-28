import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from asurt_velodyne_sim.scan import BankFrameAssembler, RevolutionAccumulator, decode_carla_lidar


class VelodyneScanTests(unittest.TestCase):
    def test_delayed_bank_preserves_both_complete_sectors_in_frame_order(self):
        frames = BankFrameAssembler(2)
        first = np.array([[1, 2, 3, .5]], dtype=np.float32)
        second = np.array([[4, 5, 6, .6]], dtype=np.float32)
        self.assertEqual(frames.add(60, 2.0, 0, first), [])
        self.assertEqual(frames.add(61, 2.033, 0, second), [])
        # Banks with zero returns are still valid deliveries.
        self.assertEqual(frames.add(61, 2.033, 1, second[:0]), [])
        ready = frames.add(60, 2.0, 1, second)
        self.assertEqual([stamp for stamp, _ in ready], [2.0, 2.033])
        np.testing.assert_array_equal(ready[0][1], np.concatenate((first, second)))
        np.testing.assert_array_equal(ready[1][1], second)
        self.assertEqual(frames.dropped_frames, 0)

    def test_missing_bank_expires_and_cannot_replay_a_sector(self):
        frames = BankFrameAssembler(2, max_lag_frames=2)
        points = np.zeros((1, 4), dtype=np.float32)
        self.assertEqual(frames.add(60, 2.0, 0, points), [])
        self.assertEqual(frames.add(61, 2.033, 0, points), [])
        self.assertEqual(frames.add(61, 2.033, 1, points), [])
        ready = frames.add(62, 2.066, 0, points)
        self.assertEqual([stamp for stamp, _ in ready], [2.033])
        self.assertEqual(frames.dropped_frames, 1)
        self.assertEqual(frames.add(60, 2.0, 1, points), [])
        self.assertEqual(frames.late_banks, 1)

    def test_raw_carla_coordinates_become_ros_flu_and_keep_intensity(self):
        raw = np.array([[3.0, 2.0, -0.5, 0.25],
                        [8.0, -1.0, 1.2, 0.9]], dtype="<f4")
        converted = decode_carla_lidar(raw.tobytes())
        np.testing.assert_allclose(converted, [[3, -2, -.5, .25],
                                               [8, 1, 1.2, .9]])
        self.assertEqual(converted.dtype, np.float32)
        np.testing.assert_array_equal(np.frombuffer(raw.tobytes(), dtype="<f4").reshape(-1, 4), raw)

    def test_reject_malformed_packet(self):
        with self.assertRaises(ValueError):
            decode_carla_lidar(b"abc")

    def test_accumulates_segments_from_multiple_world_ticks(self):
        accumulator = RevolutionAccumulator(10.0)
        packet = np.ones((5, 4), dtype=np.float32)
        self.assertIsNone(accumulator.add(10.0, packet))
        self.assertIsNone(accumulator.add(10.05, packet))
        finished = accumulator.add(10.1, packet)
        self.assertAlmostEqual(finished[0], 10.1)
        self.assertEqual(finished[1].shape, (15, 4))
        self.assertEqual(len(accumulator.packets), 0)
        self.assertIsNone(accumulator.add(10.13, packet))

    def test_no_old_packets_replayed(self):
        accumulator = RevolutionAccumulator(10.0)
        packet = np.ones((2, 4), dtype=np.float32)
        accumulator.add(1, packet)
        accumulator.add(0.99, packet)
        self.assertEqual(accumulator.out_of_order, 1)
        result = accumulator.add(1.1, packet)
        self.assertEqual(len(result[1]), 4)

    def test_ten_hz_scans_preserve_phase_with_thirty_hz_world_ticks(self):
        accumulator = RevolutionAccumulator(10.0)
        packet = np.ones((3, 4), dtype=np.float32)
        output = [accumulator.add(4.0 + i / 30.0, packet) for i in range(301)]
        output = [item for item in output if item is not None]
        self.assertGreaterEqual(len(output), 98)
        self.assertLessEqual(len(output), 100)
        self.assertEqual(sum(len(item[1]) for item in output),
                         (301 - len(accumulator.packets)) * 3)
        self.assertTrue(all(b[0] > a[0] for a, b in zip(output, output[1:])))

    def test_lost_full_scan_discards_incomplete_segments(self):
        accumulator = RevolutionAccumulator(10.0)
        packet = np.ones((2, 4), dtype=np.float32)
        self.assertIsNone(accumulator.add(0.0, packet))
        self.assertIsNone(accumulator.add(0.22, packet))
        self.assertEqual(accumulator.dropped_packets, 1)
        self.assertEqual(len(accumulator.packets), 1)
        self.assertIsNotNone(accumulator.add(0.32, packet))

    def test_bounded_if_world_fps_much_higher_than_lidar_rate(self):
        accumulator = RevolutionAccumulator(1.0, max_packets=3)
        packet = np.ones((2, 4), dtype=np.float32)
        for i in range(6):
            self.assertIsNone(accumulator.add(i * .1, packet))
        self.assertEqual(len(accumulator.packets), 3)
        self.assertEqual(accumulator.dropped_packets, 3)

    def test_invalid_data_rejected(self):
        with self.assertRaises(ValueError):
            RevolutionAccumulator(0)
        with self.assertRaises(ValueError):
            RevolutionAccumulator().add(1, np.ones((3, 3), dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
