"""Check both sensor config files against the measured FSAI Blender/URDF poses.

The CARLA actor origin coincides with Blender's original world origin, while
the URDF base_link was shifted. This is backed by the CARLA wheel-local X
coordinates matching original Blender wheel X values (2026-11-20 audit).
"""
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


def find_asurt_root():
    """Resolve the canonical ASURT tree after the 2026-11 folder migration."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "Simulation/Gazebo/Source/fsai_simulation").is_dir() and (
            candidate / "Simulation/Systems/Perception/Adapters/CARLA/Camera/asurt_zed2i_sim"
        ).is_dir():
            return candidate
    raise RuntimeError("Cannot find the ASURT project root from the sensor tests")


ROOT = find_asurt_root()
MANIFEST = ROOT / "Simulation/Gazebo/Source/fsai_simulation/fsai_description/urdf/fsaiurdf_manifest.json"
URDF = ROOT / "Simulation/Gazebo/Source/fsai_simulation/fsai_description/urdf/fsaiurdf.urdf"
CAMERA_SRC = ROOT / "Simulation/Systems/Perception/Adapters/CARLA/Camera/asurt_zed2i_sim"
LIDAR_SRC = ROOT / "Simulation/Systems/Perception/Adapters/CARLA/LiDAR/asurt_velodyne_sim"
CAMERA_YAML = CAMERA_SRC / "config/zed2i.yaml"
LIDAR_YAML = LIDAR_SRC / "config/vlp16.yaml"
sys.path.insert(0, str(CAMERA_SRC))
from asurt_zed2i_sim.optics import Calibration


def params(path, node_name):
    return yaml.safe_load(path.read_text())[node_name]["ros__parameters"]


class SourceAlignedMountTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = json.loads(MANIFEST.read_text())["geometry"]
        cls.joints = {
            joint.attrib["name"]: [
                float(v) for v in joint.find("origin").attrib["xyz"].split()
            ]
            for joint in ET.parse(URDF).getroot().findall("joint")
            if joint.find("origin") is not None
        }

    def assert_mount(self, name, p, original_key, joint):
        source_pose = self.geometry[original_key]
        urdf_pose = self.joints[joint]
        urdf_origin = self.geometry["base_origin_world_m"]
        # Both YAML profiles use ROS y-left and original Blender origin.
        for i, key in enumerate(("mount_x_m", "mount_y_m", "mount_z_m")):
            self.assertAlmostEqual(p[key], source_pose[i], delta=0.00005)
            self.assertAlmostEqual(
                p[key] - urdf_origin[i], urdf_pose[i], delta=0.00005
            )

    def test_stereo_housing_is_measured_not_previous_provisional_pose(self):
        self.assert_mount(
            "stereo", params(CAMERA_YAML, "zed2i_stereo_node"),
            "camera_object_origin_world_m", "camera_joint"
        )

    def test_lidar_housing_is_measured_not_previous_provisional_pose(self):
        self.assert_mount(
            "lidar", params(LIDAR_YAML, "asurt_velodyne_sim_node"),
            "lidar_object_origin_world_m", "lidar_joint"
        )

    def test_two_mounts_are_separate_and_on_correct_vehicle(self):
        camera = params(CAMERA_YAML, "zed2i_stereo_node")
        lidar = params(LIDAR_YAML, "asurt_velodyne_sim_node")
        self.assertAlmostEqual(camera["mount_x_m"] - lidar["mount_x_m"],
                               0.0806479752, delta=0.00005)
        self.assertGreater(camera["mount_z_m"], lidar["mount_z_m"])
        self.assertEqual(camera["role_name"], lidar["role_name"])
        self.assertEqual(camera["port"], lidar["port"])
        self.assertEqual(lidar["frame_id"], "velodyne")
        self.assertEqual(lidar["base_frame_id"], "base_link")

    def test_zed_fallback_calibration_agrees_with_checked_in_mount(self):
        """A parameter-less ZED launch must not revert to an old CARLA mount."""
        camera = params(CAMERA_YAML, "zed2i_stereo_node")
        reference = Calibration()
        for yaml_key, default in (
            ("mount_x_m", reference.mount_x),
            ("mount_y_m", reference.mount_y),
            ("mount_z_m", reference.mount_z),
            ("baseline_m", reference.baseline_m),
            ("horizontal_fov_deg", reference.hfov_deg),
        ):
            self.assertAlmostEqual(camera[yaml_key], default, places=7)
        self.assertEqual(camera["image_width"], reference.width)
        self.assertEqual(camera["image_height"], reference.height)


if __name__ == "__main__":
    unittest.main()
