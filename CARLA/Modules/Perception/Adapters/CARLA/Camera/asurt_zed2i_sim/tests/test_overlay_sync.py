"""Regression coverage for asynchronous camera overlay selection."""

import unittest

from asurt_zed2i_sim.overlay_sync import next_overlay_frame


class TestOverlaySync(unittest.TestCase):
    def test_capture_outpacing_inference_never_publishes_unannotated_frame(self):
        # YOLO is still analyzing frame 101 while the camera has reached 104.
        raw = (104, "raw104", 12.4, 5.4)
        self.assertIsNone(next_overlay_frame(-1, raw, None, True))

        complete = (101, "analyzed101", "stamp101", 5.1, ("cone101",))
        self.assertIs(next_overlay_frame(-1, raw, complete, True), complete)
        self.assertIsNone(next_overlay_frame(101, raw, complete, True))

        # New raw frames cannot blank the completed overlay while inference runs.
        newer_raw = (108, "raw108", 12.8, 5.8)
        self.assertIsNone(next_overlay_frame(101, newer_raw, complete, True))
        next_complete = (106, "analyzed106", "stamp106", 5.6, ("cone106",))
        self.assertIs(next_overlay_frame(101, newer_raw, next_complete, True), next_complete)

    def test_completed_empty_detection_is_visible_and_old_result_is_ignored(self):
        empty = (110, "analyzed110", "stamp110", 6.0, ())
        self.assertIs(next_overlay_frame(109, (112, "raw", 13.0, 6.2), empty, True), empty)
        self.assertIsNone(next_overlay_frame(110, None, empty, True))
        self.assertIsNone(next_overlay_frame(111, None, empty, True))

    def test_detection_disabled_keeps_independent_live_camera(self):
        raw = (120, "raw120", 14.0, 7.0)
        stale_detection = (101, "analyzed101", "stamp101", 5.1, ())
        self.assertIs(next_overlay_frame(119, raw, stale_detection, False), raw)
        self.assertIsNone(next_overlay_frame(120, raw, stale_detection, False))


if __name__ == "__main__":
    unittest.main()
