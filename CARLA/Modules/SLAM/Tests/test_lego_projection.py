#!/usr/bin/env python3
"""Check LeGO projection of synthetic VLP-16 and VLP-32C sweeps."""
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import rclpy
import yaml


ROOT = Path(__file__).resolve().parents[4]
SLAM = ROOT / "Simulation/Systems/SLAM"
BINARY = (SLAM / "Workspace/slam_integration/install/lego_loam_sr"
          / "lib/lego_loam_sr/lego_loam_sr")
CONFIG_DIR = SLAM / "Modules/LiDARMapping/LeGO-LOAM/config"
GTSAM_LIB = (SLAM / "ThirdParty/gtsam-prefix/opt/ros/humble"
             / "lib/x86_64-linux-gnu")


def synthetic_sweep(elevations):
    fields = [PointField(name=name, offset=4 * i,
                         datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    points = []
    for azimuth in range(0, 360, 2):
        theta = math.radians(azimuth)
        for beam in elevations:
            elevation = math.radians(beam)
            distance = 9.0 + 0.3 * math.sin(theta * 3)
            points.append((distance * math.cos(elevation) * math.cos(theta),
                           distance * math.cos(elevation) * math.sin(theta),
                           distance * math.sin(elevation), 0.6))
    header = Header()
    header.frame_id = 'velodyne'
    header.stamp.sec = 100
    return point_cloud2.create_cloud(header, fields, points)


class LeGOProjection(unittest.TestCase):
    def test_shared_vlp16_pointcloud_projection(self):
        self.check_projection("vlp16", [-15.0 + 2.0 * row for row in range(16)], 12)

    def test_all_vlp32c_beams_project_to_distinct_rows(self):
        config = yaml.safe_load((CONFIG_DIR / "asurt_vlp32c.yaml").read_text())
        elevations = config["image_projection"]["ros__parameters"]["laser"]["beam_elevations_deg"]
        self.assertEqual(len(elevations), 32)
        self.check_projection("vlp32c", elevations, 32)

    def check_projection(self, model, elevations, min_rows):
        scratch = Path(tempfile.mkdtemp(
            prefix='lego-project-', dir=SLAM / 'Tests'))
        executable = scratch / 'lego_projection_test_bin'
        shutil.copy2(BINARY, executable)
        env = dict(os.environ, ROS_DOMAIN_ID='187')
        env['LD_LIBRARY_PATH'] = str(GTSAM_LIB) + ':' + env.get('LD_LIBRARY_PATH', '')
        rclpy.init(args=None, domain_id=187)
        node = rclpy.create_node('asurt_lego_projection_test')
        publisher = node.create_publisher(PointCloud2, '/velodyne_points', 10)
        received = []
        subscriber = node.create_subscription(
            PointCloud2, '/slam/lidar/full_cloud_projected',
            lambda msg: received.append(msg), 10)
        process = subprocess.Popen(
            [str(executable), '--ros-args', '--params-file', str(CONFIG_DIR / f"asurt_{model}.yaml"),
             '--log-level', 'warn'], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 14.0
            while (publisher.get_subscription_count() < 1 or
                   node.count_publishers('/slam/lidar/full_cloud_projected') < 1):
                self.assertIsNone(process.poll(), 'LOAM exited before graph discovery')
                self.assertLess(time.monotonic(), deadline,
                                f'LOAM {model} input/output publishers not discovered')
                rclpy.spin_once(node, timeout_sec=.1)
            sweep = synthetic_sweep(elevations)
            empty = PointCloud2()
            empty.header = sweep.header
            empty.fields = sweep.fields
            empty.point_step = sweep.point_step
            empty.height = 1
            empty.width = 0
            empty.row_step = 0
            for _ in range(3):
                publisher.publish(empty)
                rclpy.spin_once(node, timeout_sec=.08)
            time.sleep(.25)
            self.assertIsNone(process.poll(),
                              f'Empty {model} sweep crashed LeGO projection')
            while not received and time.monotonic() < deadline:
                self.assertIsNone(process.poll(), f'LOAM exited processing {model} scan')
                publisher.publish(sweep)
                rclpy.spin_once(node, timeout_sec=.2)
            self.assertTrue(received, 'No point cloud output from projection')
            self.assertEqual(received[0].header.frame_id, 'velodyne')
            self.assertGreater(received[0].width * received[0].height, 0)
            valid = list(point_cloud2.read_points(
                received[0], field_names=('x', 'y', 'z', 'intensity'),
                skip_nans=True))
            self.assertGreater(len(valid), 800,
                               'Projection produced a grid with too few valid returns')
            used_rows = {int(point[3]) for point in valid}
            self.assertGreaterEqual(len(used_rows), min_rows,
                                    f'Expected returns across {model} elevation channels')
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_subscription(subscriber)
            node.destroy_publisher(publisher)
            node.destroy_node()
            rclpy.shutdown()
            if process.stderr:
                process.stderr.close()
            shutil.rmtree(scratch)


if __name__ == '__main__':
    unittest.main()
