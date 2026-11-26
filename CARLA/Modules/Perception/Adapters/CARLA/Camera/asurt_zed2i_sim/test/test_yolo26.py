"""CPU-only tests for the FSOCO YOLO26 adapter and independent stereo geometry."""
import unittest

import cv2
import numpy as np

from asurt_zed2i_sim.detection import (
    CONE_LABEL_IDS, Candidate, Cone, associate_candidates, nominal_cone_height,
)
from asurt_zed2i_sim.optics import Calibration
from asurt_zed2i_sim.tracking import ConeTracker
from asurt_zed2i_sim.yolo26 import (
    Yolo26Stereo, decode_detections, right_epipolar_match,
)


class TestYolo26(unittest.TestCase):
    def test_letterbox_is_rgb_nchw_and_keeps_original_aspect(self):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        image[:, :, :] = (2, 40, 200)
        blob = Yolo26Stereo.preprocess(image)
        self.assertEqual(blob.shape, (1, 3, 640, 640))
        self.assertEqual(blob.dtype, np.float32)
        self.assertAlmostEqual(float(blob[0, 0, 200, 320]), 200 / 255)
        self.assertAlmostEqual(float(blob[0, 2, 200, 320]), 2 / 255)
        self.assertAlmostEqual(float(blob[0, 0, 20, 320]), 114 / 255)

    def test_decode_letterbox_and_fsoco_classes(self):
        names = {0: "yellow", 1: "blue", 2: "orange",
                 3: "large_orange", 4: "unknown"}
        rows = np.array([
            [100, 190, 140, 230, .83, 1],
            [160, 190, 180, 240, .67, 3],
            [210, 190, 230, 230, .91, 4],
            [280, 190, 300, 235, .18, 0],
        ], dtype=np.float32)
        actual = decode_detections(rows, names, (360, 640, 3), .30)
        self.assertEqual([x.color for x in actual], ["blue", "large_orange"])
        self.assertEqual(CONE_LABEL_IDS[actual[1].color], 3)
        self.assertEqual(CONE_LABEL_IDS["orange"], 2)
        self.assertEqual((actual[0].x, actual[0].y, actual[0].w, actual[0].h),
                         (100, 50, 40, 40))
        self.assertAlmostEqual(actual[0].confidence, 83, delta=.01)

    def test_large_orange_stereo_uses_505mm_height_and_retains_type(self):
        cal = Calibration()
        small = Candidate("orange", 250, 180, 12, 28, 336, 87)
        large = Candidate("large_orange", 250, 180, 12, 28, 336, 87)
        right_hsv = Candidate("orange", 242, 180, 12, 28, 336)
        small_cones = associate_candidates([small], [right_hsv], cal)
        large_cones = associate_candidates([large], [right_hsv], cal)
        self.assertEqual([c.color for c in large_cones], ["large_orange"])
        self.assertEqual([c.color for c in small_cones], ["orange"])
        self.assertAlmostEqual(nominal_cone_height(large.color), .505)
        self.assertGreater(large_cones[0].x, small_cones[0].x)
        tracker = ConeTracker(min_hits=1)
        tracked = tracker.update([
            Cone("orange", 85., 4.0, -1.0, 0.0, (120, 180, 12, 28)),
            Cone("large_orange", 90., 4.0, 1.0, 0.0, (250, 180, 12, 28)),
        ])
        self.assertEqual({c.color for c in tracked}, {"orange", "large_orange"})

    def test_epipolar_template_match_uses_right_eye_pixels(self):
        rng = np.random.RandomState(7)
        left = np.zeros((360, 640), dtype=np.uint8)
        right = np.zeros_like(left)
        patch = rng.randint(10, 240, size=(28, 12)).astype(np.uint8)
        left[180:208, 250:262] = patch
        right[180:208, 242:254] = patch
        c = Candidate("blue", 250, 180, 12, 28, 336, 83)
        matched = right_epipolar_match(c, left, right, .12)
        self.assertIsNotNone(matched)
        self.assertEqual((matched.x, matched.y), (242, 180))
        cones = associate_candidates([c], [matched], Calibration())
        self.assertEqual(len(cones), 1)
        self.assertEqual(cones[0].box, (250, 180, 12, 28))
        self.assertAlmostEqual(cones[0].confidence, 83)
        self.assertTrue(1 < cones[0].x < 10)

    def test_large_orange_template_fallback_retains_large_class(self):
        rng = np.random.RandomState(21)
        left = np.zeros((360, 640), dtype=np.uint8)
        right = np.zeros_like(left)
        patch = rng.randint(10, 235, size=(30, 13)).astype(np.uint8)
        left[177:207, 250:263] = patch
        right[177:207, 243:256] = patch
        c = Candidate("large_orange", 250, 177, 13, 30, 390, 88)
        matched = right_epipolar_match(c, left, right, .12)
        self.assertIsNotNone(matched)
        self.assertEqual(matched.color, "large_orange")
        result = associate_candidates([c], [matched], Calibration())
        self.assertEqual([cone.color for cone in result], ["large_orange"])

    def test_no_spurious_stereo_match_without_right_evidence(self):
        image = np.zeros((360, 640), dtype=np.uint8)
        left = image.copy()
        left[170:200, 250:262] = cv2.randn(np.empty((30, 12), np.uint8), 120, 45)
        c = Candidate("yellow", 250, 170, 12, 30, 360, 80)
        self.assertIsNone(right_epipolar_match(c, left, image, .12))


if __name__ == "__main__":
    unittest.main()
