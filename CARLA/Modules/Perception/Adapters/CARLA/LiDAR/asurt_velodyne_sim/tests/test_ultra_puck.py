import math
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from asurt_velodyne_sim.ultra_puck import VLP32C_BEAMS, beam_banks, rotate_to_sensor


class UltraPuckTest(unittest.TestCase):
    def test_banks_reconstruct_all_nominal_angles(self):
        banks = beam_banks()
        self.assertEqual(len(banks), 14)
        reconstructed = sorted((angle, yaw) for yaw, elevations in banks
                               for angle in np.linspace(min(elevations), max(elevations), len(elevations)))
        np.testing.assert_allclose(reconstructed, sorted(VLP32C_BEAMS), rtol=0, atol=1e-9)
        elevations = sorted(angle for angle, _ in VLP32C_BEAMS)
        self.assertEqual((len(set(elevations)), elevations[0], elevations[-1]), (32, -25, 15))
        self.assertEqual({yaw for yaw, _ in banks}, {-4.2, -1.4, 1.4, 4.2})
        self.assertAlmostEqual(min(np.diff(elevations)), 0.333, places=3)

    def test_bank_yaw_is_removed_in_ros_frame(self):
        points = np.array([[1., 0., .25, .7]], dtype=np.float32)
        rotate_to_sensor(points, 4.2)
        angle = math.radians(4.2)
        np.testing.assert_allclose(points, [[math.cos(angle), -math.sin(angle), .25, .7]])

    def test_default_model_config_matches_calibration(self):
        params = yaml.safe_load((PACKAGE / "config/vlp32c.yaml").read_text())[
            "asurt_velodyne_sim_node"]["ros__parameters"]
        self.assertEqual((params["model"], params["channels"], params["range_m"]), ("vlp32c", 32, 200))
        self.assertEqual((params["rotation_hz"], params["points_per_second"]), (10, 600000))
