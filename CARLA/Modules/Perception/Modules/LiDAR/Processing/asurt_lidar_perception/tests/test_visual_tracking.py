"""Behavior tests for RViz-only LiDAR marker stability."""

import unittest

from asurt_lidar_perception.visual_tracking import LidarVisualTracker


def observation(x, y=0.2, z=-0.1, score=0.9):
    return ((x, y, z), score)


class VisualTrackerTests(unittest.TestCase):
    def test_requires_confirmation_and_tracks_real_movement(self):
        visual = LidarVisualTracker()
        self.assertEqual(visual.update([observation(5)], "velodyne", now=1.0), [])
        confirmed = visual.update([observation(5.1)], "velodyne", now=1.1)
        self.assertEqual(len(confirmed), 1)
        ident = confirmed[0].id
        moved = visual.update([observation(5.6)], "velodyne", now=1.2)
        self.assertEqual([c.id for c in moved], [ident])
        self.assertGreater(moved[0].center[0], confirmed[0].center[0])

    def test_short_misses_hold_then_expire_without_ghosts(self):
        visual = LidarVisualTracker()
        visual.update([observation(5)], "velodyne", now=1.0)
        confirmed = visual.update([observation(5)], "velodyne", now=1.1)
        first_id = confirmed[0].id
        self.assertEqual([c.id for c in visual.update([], "velodyne", now=1.25)],
                         [first_id])
        self.assertEqual([c.id for c in visual.update([], "velodyne", now=1.4)],
                         [first_id])
        self.assertEqual(visual.update([], "velodyne", now=1.5), [])
        self.assertFalse(visual.tracks)

    def test_earlier_uncorroborated_candidate_never_becomes_a_ghost(self):
        visual = LidarVisualTracker()
        self.assertEqual(visual.update([observation(4)], "sensor", now=2), [])
        self.assertEqual(visual.update([], "sensor", now=2.1), [])
        self.assertEqual(visual.update([], "sensor", now=2.2), [])

    def test_reversed_order_keeps_ids_and_associations_unique(self):
        visual = LidarVisualTracker()
        initial = [observation(3), observation(8)]
        visual.update(initial, "sensor", now=1)
        before = visual.update(initial, "sensor", now=1.1)
        after = visual.update(list(reversed(initial)), "sensor", now=1.2)
        self.assertEqual([(v.id, round(v.center[0])) for v in before],
                         [(v.id, round(v.center[0])) for v in after])

    def test_frame_change_invalidates_previous_coordinates(self):
        visual = LidarVisualTracker()
        visual.update([observation(5)], "lidar_old", now=1)
        confirmed = visual.update([observation(5)], "lidar_old", now=1.1)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(visual.update([observation(5)], "lidar_new", now=1.2), [])
        again = visual.update([observation(5)], "lidar_new", now=1.3)
        self.assertNotEqual(again[0].id, confirmed[0].id)


if __name__ == "__main__":
    unittest.main()
