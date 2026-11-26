#!/usr/bin/env python3
"""Read-only accuracy audit against the existing physical CARLA cone actors.

The simulated sensors and production perception nodes NEVER receive actor
locations from this tool. Use only with a stationary hero: this tool cannot
interpolate vehicle poses from an earlier measurement timestamp. World cone
geometry is a reference for OFFLINE regression scoring, not perception data.

Run after sourcing ROS 2 Humble and the ASURT overlays:
  /usr/bin/python3 scripts/evaluate_perception_geometry.py --seconds 10
"""
import argparse
from collections import defaultdict
import json
import math
import os
import statistics
import sys
import time

import numpy as np
import rclpy
from asurt_msgs.msg import LandmarkArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from course_cones import course_cone_color


def load_carla():
    try:
        import carla
    except ImportError:
        sys.path.append(os.environ.get(
            'ASURT_CARLA_SITE',
            '/home/hassan/ASURT/.venvs/asurt-native/lib/python3.10/site-packages'))
        import carla
    return carla


def metrics(values):
    if not values:
        return {'samples': 0}
    values = [float(v) for v in values]
    return {
        'samples': len(values),
        'mean_xy_error_m': round(statistics.mean(values), 4),
        'median_xy_error_m': round(statistics.median(values), 4),
        'p90_xy_error_m': round(float(np.percentile(values, 90)), 4),
        'max_xy_error_m': round(max(values), 4),
        'fraction_within_0_15m': round(sum(v <= .15 for v in values) / len(values), 3),
        'fraction_within_0_50m': round(sum(v <= .5 for v in values) / len(values), 3),
    }


class GeometryAudit(Node):
    def __init__(self, scene):
        super().__init__('asurt_offline_geometry_audit')
        self.scene = scene
        self.samples = defaultdict(list)
        self.owned_subscriptions = []
        for topic, label in (('/lidar/cones', 'lidar'),
                             ('/perception_landmarks', 'stereo')):
            self.owned_subscriptions.append(self.create_subscription(
                LandmarkArray, topic,
                lambda msg, name=label: self.receive(name, msg),
                qos_profile_sensor_data))

    def receive(self, name, msg):
        expected_frame = 'velodyne' if name == 'lidar' else 'base_link'
        if msg.header.frame_id != expected_frame:
            self.get_logger().warning(
                f'Unexpected {name} frame {msg.header.frame_id!r}')
            return
        if len(self.samples[name]) >= 12:
            return
        self.samples[name].append(msg)

    def score(self):
        truth = np.array([a.get_location() for a in self.scene['cones']],
                         dtype=object)
        cone_xy = np.array([(a.x, a.y) for a in truth], dtype=np.float64)
        results = {}
        for name, messages in self.samples.items():
            sensor = self.scene['lidar'] if name == 'lidar' else self.scene['hero']
            T = np.asarray(sensor.get_transform().get_matrix(), dtype=float)
            dist_all, near, far = [], [], []
            colors = {'correct': 0, 'incorrect': 0}
            reports = []
            for msg in messages:
                for lm in msg.landmarks:
                    ros_pos = lm.position
                    # ROS +Y left to CARLA +Y right in the source frame.
                    p = np.array([ros_pos.x, -ros_pos.y, ros_pos.z, 1.])
                    xy = (T @ p)[:2]
                    distance = np.linalg.norm(cone_xy - xy, axis=1)
                    j = int(np.argmin(distance))
                    d = float(distance[j])
                    dist_all.append(d)
                    (near if ros_pos.x <= 7.0 else far).append(d)
                    cone = self.scene['cones'][j]
                    if name == 'stereo':
                        color = course_cone_color(cone)
                        expected = 0 if color == 'blue' else 1
                        colors['correct' if lm.type == expected else 'incorrect'] += 1
                    if len(reports) < 8:
                        reports.append({
                            'predicted_forward_m': round(ros_pos.x, 3),
                            'nearest_cone_id': cone.id,
                            'nearest_cone_color': color,
                            'xy_error_m': round(d, 4),
                        })
            results[name] = {
                'topic_messages': len(messages),
                'all_detections': metrics(dist_all),
                'forward_up_to_7m': metrics(near),
                'forward_above_7m': metrics(far),
                'stereo_color_matches_nearest': colors if name == 'stereo' else None,
                'first_detections': reports,
            }
        return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if not 3 <= args.seconds <= 60:
        parser.error('--seconds must be in [3, 60]')
    carla = load_carla()
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(6)
    world = client.get_world()
    actors = world.get_actors()
    heroes = [a for a in actors if a.attributes.get('role_name') == 'hero'
              and a.type_id == 'vehicle.vehicle.asurt_fsai']
    lidars = [a for a in actors if a.type_id == 'sensor.lidar.ray_cast']
    cones = [a for a in actors if course_cone_color(a)]
    if len(heroes) != 1 or len(lidars) != 1 or len(cones) != 100:
        raise RuntimeError('Expected one hero, one LiDAR, and 100 existing cones')
    scene = {'hero': heroes[0], 'lidar': lidars[0], 'cones': cones}
    def speed():
        v = heroes[0].get_velocity()
        return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
    if speed() > 0.05:
        raise RuntimeError('Vehicle is moving; stationary evaluation is invalid')
    rclpy.init()
    node = GeometryAudit(scene)
    try:
        start = time.monotonic()
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.2)
            if len(node.samples['lidar']) >= 12 and len(node.samples['stereo']) >= 12:
                break
        if speed() > 0.05:
            raise RuntimeError('Hero moved during observation; invalidate audit')
        report = {
            'note': 'OFFLINE nearest-cone XY scoring; never consumed by perception. Stationary only. Repeated scans are correlated.',
            'actor_cones': len(cones),
            'hero_id': heroes[0].id,
            'lidar_id': lidars[0].id,
            'results': node.score(),
        }
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
