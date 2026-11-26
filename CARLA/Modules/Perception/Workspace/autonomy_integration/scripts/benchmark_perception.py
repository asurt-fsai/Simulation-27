#!/usr/bin/env python3
"""Read-only live ROS perception benchmark; never ticks or alters CARLA.

Usage after sourcing Humble and both ASURT overlays:
  /usr/bin/python3 scripts/benchmark_perception.py --seconds 20
"""
import argparse
from collections import defaultdict
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class Benchmark(Node):
    def __init__(self):
        super().__init__('asurt_perception_benchmark')
        self.sample = defaultdict(list)
        self.clock_s = None
        self.clock_first = None
        self.clock_count = 0
        self.clock_backward = 0
        self.clock_last = None
        self.subs = []
        self.subs.append(self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data))
        self.observe('/velodyne_points', PointCloud2)
        self.observe('/filtered_points', PointCloud2)
        try:
            from zed_msgs.msg import ObjectsStamped
            self.observe('/zed/zed_node/obj_det/objects', ObjectsStamped)
        except ImportError:
            pass
        try:
            from asurt_msgs.msg import LandmarkArray
            self.observe('/perception_landmarks', LandmarkArray)
            self.observe('/lidar/cones', LandmarkArray)
        except ImportError:
            pass

    def on_clock(self, msg):
        t = stamp_s(msg.clock)
        if self.clock_last is not None and t < self.clock_last:
            self.clock_backward += 1
        if self.clock_first is None:
            self.clock_first = t
        self.clock_s = t
        self.clock_last = t
        self.clock_count += 1

    def observe(self, topic, cls):
        self.subs.append(self.create_subscription(
            cls, topic, lambda msg, name=topic: self.on_msg(name, msg),
            qos_profile_sensor_data))

    def on_msg(self, topic, msg):
        header = getattr(msg, 'header', None)
        stamp = stamp_s(header.stamp) if header is not None else None
        now = time.monotonic()
        npoints = getattr(msg, 'width', None)
        if npoints is None:
            for name in ('objects', 'landmarks', 'landmark_array'):
                value = getattr(msg, name, None)
                if value is not None:
                    npoints = len(value)
                    break
        age = self.clock_s - stamp if stamp is not None and self.clock_s is not None else None
        self.sample[topic].append((now, stamp, npoints, age, getattr(header, 'frame_id', None)))

    def report(self, wall_seconds):
        clock_elapsed = (self.clock_last - self.clock_first) if self.clock_count > 1 else None
        result = {
            'wall_seconds': round(wall_seconds, 3),
            'clock_messages': self.clock_count,
            'clock_backwards': self.clock_backward,
            'simulated_seconds': round(clock_elapsed, 3) if clock_elapsed is not None else None,
            'realtime_factor': round(clock_elapsed / wall_seconds, 3) if clock_elapsed is not None else None,
            'topics': {},
        }
        for topic in ('/velodyne_points', '/filtered_points', '/zed/zed_node/obj_det/objects', '/perception_landmarks', '/lidar/cones'):
            records = self.sample[topic]
            gaps = [b[0] - a[0] for a, b in zip(records, records[1:])]
            stamps = [r[1] for r in records if r[1] is not None]
            sizes = [r[2] for r in records if r[2] is not None]
            ages = [r[3] for r in records if r[3] is not None]
            frames = sorted(set(r[4] for r in records if r[4]))
            result['topics'][topic] = {
                'messages': len(records),
                'wall_hz': round(len(records) / wall_seconds, 2),
                'sim_hz': round(len(records) / clock_elapsed, 2) if clock_elapsed and clock_elapsed > 0 else None,
                'max_gap_wall_s': round(max(gaps), 3) if gaps else None,
                'mean_gap_wall_s': round(statistics.mean(gaps), 3) if gaps else None,
                'stamp_nonmonotonic': sum(b <= a for a, b in zip(stamps, stamps[1:])),
                'mean_clock_minus_stamp_s': round(statistics.mean(ages), 3) if ages else None,
                'max_clock_minus_stamp_s': round(max(ages), 3) if ages else None,
                'mean_count': round(statistics.mean(sizes), 1) if sizes else None,
                'frames': frames,
            }
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if not 5 <= args.seconds <= 180:
        parser.error('--seconds must be in [5, 180]')
    rclpy.init()
    node = Benchmark()
    try:
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report(time.monotonic() - start)
        encoded = json.dumps(report, indent=2)
        print(encoded)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(encoded + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
