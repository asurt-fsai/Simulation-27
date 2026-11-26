#!/usr/bin/env python3
"""Exercise LeGO's CARLA map-reset guard on a separate ROS domain.

This starts only the passive SLAM binary. It never connects to CARLA,
spawns sensors, changes the live clock, or sends driving controls.
"""
import os
from pathlib import Path
import signal
import shutil
import subprocess
import tempfile
import time
import unittest

from nav_msgs.msg import Odometry
import rclpy


ROOT = Path(__file__).resolve().parents[4]
BINARY = (ROOT / "Simulation/Systems/SLAM/Workspace/slam_integration"
          / "install/lego_loam_sr/lib/lego_loam_sr/lego_loam_sr")
CONFIG = (ROOT / "Simulation/Systems/SLAM/Modules/LiDARMapping"
          / "LeGO-LOAM/config/asurt_vlp16.yaml")
GTSAM_LIB = (ROOT / "Simulation/Systems/SLAM/ThirdParty/gtsam-prefix"
             / "opt/ros/humble/lib/x86_64-linux-gnu")


def odometry(stamp, x=0.0, y=0.0):
    message = Odometry()
    message.header.frame_id = "map"
    message.child_frame_id = "base_link"
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    return message


class LeGOReset(unittest.TestCase):
    def test_clock_rewind_terminates_stale_pose_graph(self):
        env = dict(os.environ, ROS_DOMAIN_ID="186")
        env["LD_LIBRARY_PATH"] = str(GTSAM_LIB) + ":" + env.get("LD_LIBRARY_PATH", "")
        # The live System Manager correctly SIGINTs installed executables of
        # modules switched OFF, even on a different ROS domain. A temporary
        # test-only binary prevents the manager from terminating this test.
        scratch = Path(tempfile.mkdtemp(
            prefix="lego-reset-", dir=ROOT / "Simulation/Systems/SLAM/Tests"))
        executable = scratch / "lego_reset_test_bin"
        shutil.copy2(BINARY, executable)
        rclpy.init(args=None, domain_id=186)
        node = rclpy.create_node("asurt_lego_reset_test")
        publisher = node.create_publisher(Odometry, "/odom", 10)
        process = subprocess.Popen(
            [str(executable), "--ros-args", "--params-file", str(CONFIG),
             "--log-level", "warn"], env=env, stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 22.0
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                self.assertIsNone(
                    process.poll(), "LeGO exited during initialization: " +
                    (process.stderr.read()[-2000:] if process.poll() is not None else ""))
                time.sleep(0.1)
            self.assertGreater(publisher.get_subscription_count(), 0,
                               "No independent LeGO /odom reset subscription")

            for i in range(8):
                publisher.publish(odometry(101.0 + i * .1))
                time.sleep(.1)
            self.assertIsNone(process.poll(), "LeGO exited before discontinuity")

            until = time.monotonic() + 8.0
            while process.poll() is None and time.monotonic() < until:
                publisher.publish(odometry(1.0))
                time.sleep(.1)
            self.assertEqual(process.poll(), 0,
                             "LeGO did not cleanly discard map after rewind")
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=4)
            node.destroy_publisher(publisher)
            node.destroy_node()
            rclpy.shutdown()
            if process.stderr is not None:
                process.stderr.close()
            shutil.rmtree(scratch)


if __name__ == "__main__":
    unittest.main()
