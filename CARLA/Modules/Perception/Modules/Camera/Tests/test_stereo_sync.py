import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / "Adapters/CARLA/Camera/asurt_zed2i_sim"))
from asurt_zed2i_sim.stereo_sync import StereoSynchronizer


class StereoSyncTests(unittest.TestCase):
    def test_adjacent_camera_frames_match_without_matching_frame_numbers(self):
        sync = StereoSynchronizer(max_skew_s=0.05)
        for i in range(12):
            # Simulated 30 Hz world, each camera 10 Hz but shifted 1 tick.
            sync.add("left", 3*i, 0.099*i, f"L{i}")
            sync.add("right", 3*i+1, 0.099*i+0.033, f"R{i}")
            left, right, skew = sync.pop_latest()
            self.assertEqual((left[1], right[1]), (f"L{i}", f"R{i}"))
            self.assertAlmostEqual(skew, 0.033)

    def test_large_skew_never_creates_fictitious_stereo_pair(self):
        sync = StereoSynchronizer(max_skew_s=0.05)
        sync.add("left", 10, 0.33, "left")
        sync.add("right", 14, 0.47, "right")
        self.assertIsNone(sync.pop_latest())

    def test_one_to_one_and_old_pairs_never_reused(self):
        sync = StereoSynchronizer()
        for frame in (3, 6, 9):
            sync.add("left", frame, frame * 0.033, f"L{frame}")
            sync.add("right", frame + 1, (frame + 1) * 0.033, f"R{frame+1}")
        left, right, _ = sync.pop_latest()
        self.assertEqual((left[0], right[0]), (9, 10))
        self.assertIsNone(sync.pop_latest())
        sync.add("left", 6, 0.198, "stale")
        self.assertEqual(sync.dropped_late, 1)

    def test_single_missing_eye_has_bounded_memory(self):
        sync = StereoSynchronizer(max_pending_per_eye=5)
        for frame in range(30):
            sync.add("left", frame, frame * 0.033, "img")
        self.assertEqual(len(sync.frames["left"]), 5)
        self.assertEqual(sync.dropped_unpaired, 25)

    def test_exact_frame_pair_still_works(self):
        sync = StereoSynchronizer()
        sync.add("left", 100, 3.3, "L")
        sync.add("right", 100, 3.3, "R")
        self.assertEqual(sync.pop_latest()[2], 0.0)


if __name__ == "__main__":
    unittest.main()
