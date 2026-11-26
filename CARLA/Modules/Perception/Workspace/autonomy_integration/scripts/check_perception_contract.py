#!/usr/bin/env python3
"""Read-only ROS 2 + CARLA sensor-interface contract audit.

The checker never creates CARLA actors, publishes commands, or ticks the world.
It validates the existing hero and observer mounts and samples the live topics.
Run with sourced Humble, old ASURT message overlay and integration overlay.
"""

import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile,
    ReliabilityPolicy, qos_profile_sensor_data,
)
from asurt_msgs.msg import LandmarkArray
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from zed_msgs.msg import ObjectsStamped
from course_cones import course_cone_color


EXPECTED_TOPICS = {
    '/velodyne_points': (PointCloud2, 'velodyne'),
    '/filtered_points': (PointCloud2, 'velodyne'),
    '/lidar/cones': (LandmarkArray, 'velodyne'),
    '/zed/zed_node/obj_det/objects': (ObjectsStamped, 'base_link'),
    '/perception_landmarks': (LandmarkArray, 'base_link'),
}


def carla_module():
    try:
        import carla
    except ImportError:
        sys.path.append(os.environ.get(
            'ASURT_CARLA_SITE',
            '/home/hassan/ASURT/.venvs/asurt-native/lib/python3.10/site-packages',
        ))
        import carla
    return carla


def as_xyz(transform):
    loc = transform.location
    return [float(loc.x), float(loc.y), float(loc.z)]


def validate_actors(world, error):
    actors = list(world.get_actors())
    heroes = [a for a in actors if a.type_id == 'vehicle.vehicle.asurt_fsai'
              and a.attributes.get('role_name') == 'hero']
    cone_colors = [color for actor in actors if (color := course_cone_color(actor))]
    lidar = [a for a in actors if a.type_id.startswith('sensor.lidar.')]
    eyes = {side: [a for a in actors if a.type_id == 'sensor.camera.rgb'
                   and a.attributes.get('role_name') == f'asurt_zed2i_sim_{side}']
            for side in ('left', 'right')}
    if len(heroes) != 1:
        error(f'Expected one FSAI hero, found {len(heroes)}')
    if len(cone_colors) != 100:
        error(f'Expected 100 physical cones, found {len(cone_colors)}')
    if len(lidar) != 1:
        error(f'Expected one ray-cast LiDAR, found {len(lidar)}')
    if len(eyes['left']) != 1 or len(eyes['right']) != 1:
        error(f'Expected one camera per eye, found {[(s, len(v)) for s, v in eyes.items()]}')
    settings = world.get_settings()
    if not settings.synchronous_mode:
        error('CARLA world is not in synchronous mode')
    if not heroes[0:1]:
        return {'hero_count': len(heroes), 'cone_count': len(cone_colors),
                'lidar_count': len(lidar), 'eye_counts': {s: len(v) for s, v in eyes.items()}}
    hero = heroes[0]
    world_from_hero = np.asarray(hero.get_transform().get_inverse_matrix())
    expected = {
        'velodyne': ([0.37553552, -0.01003163, 0.69655180], lidar),
        'zed_left': ([0.45618349, -0.08084175, 0.73042095], eyes['left']),
        'zed_right': ([0.45618349, 0.03915825, 0.73042095], eyes['right']),
    }
    mounts = {}
    for name, (reference, found) in expected.items():
        if len(found) != 1:
            continue
        sensor = found[0]
        transform = sensor.get_transform()
        point = np.r_[as_xyz(transform), [1.0]]
        local = (world_from_hero @ point)[:3].astype(float).tolist()
        discrepancy = float(np.linalg.norm(np.subtract(local, reference)))
        mounts[name] = {'actor_id': sensor.id, 'local_carla_xyz': [round(v, 5) for v in local],
                        'deviation_m': round(discrepancy, 5),
                        'parent': sensor.parent.id if sensor.parent else None}
        if sensor.parent is None or sensor.parent.id != hero.id:
            error(f'{name} does not attach to the existing hero')
        if discrepancy > 0.005:
            error(f'{name} mounting error is {discrepancy:.3f}m')
    return {
        'hero_id': hero.id,
        'cone_count': len(cone_colors),
        'blue_count': cone_colors.count('blue'),
        'yellow_count': cone_colors.count('yellow'),
        'lidar_count': len(lidar),
        'synchronous_mode': settings.synchronous_mode,
        'fixed_delta_s': settings.fixed_delta_seconds,
        'mounts': mounts,
    }


def stamp_seconds(t):
    return float(t.sec) + float(t.nanosec) * 1e-9


class TopicAudit(Node):
    def __init__(self):
        super().__init__('asurt_perception_contract_audit')
        self.data = defaultdict(list)
        self.subscriptions_owned = []
        for topic, (message_type, _) in EXPECTED_TOPICS.items():
            self.subscriptions_owned.append(self.create_subscription(
                message_type, topic,
                lambda m, t=topic: self.on_message(t, m),
                qos_profile_sensor_data))
        self.clock = []
        self.subscriptions_owned.append(self.create_subscription(
            Clock, '/clock', self.on_clock, qos_profile_sensor_data))
        self.transforms = defaultdict(set)
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_owned.append(self.create_subscription(
            TFMessage, '/tf_static', self.on_tf, transient))

    def on_clock(self, msg):
        self.clock.append(stamp_seconds(msg.clock))
        if len(self.clock) > 300:
            self.clock.pop(0)

    def on_message(self, topic, msg):
        if len(self.data[topic]) >= 30:
            return
        hdr = msg.header
        sample = {'frame': hdr.frame_id, 'stamp': stamp_seconds(hdr.stamp)}
        if isinstance(msg, PointCloud2):
            sample['fields'] = [f.name for f in msg.fields]
            sample['points'] = msg.width * msg.height
        elif isinstance(msg, LandmarkArray):
            sample['landmarks'] = len(msg.landmarks)
        self.data[topic].append(sample)

    def on_tf(self, msg):
        for tr in msg.transforms:
            t = tr.transform.translation
            r = tr.transform.rotation
            self.transforms[tr.child_frame_id].add((
                tr.header.frame_id,
                round(t.x, 5), round(t.y, 5), round(t.z, 5),
                round(r.x, 5), round(r.y, 5), round(r.z, 5), round(r.w, 5),
            ))

    def report(self, error):
        result = {}
        for topic, (message_type, expected_frame) in EXPECTED_TOPICS.items():
            samples = self.data[topic]
            publishers = self.count_publishers(topic)
            if publishers != 1:
                error(f'{topic}: expected exactly one publisher, found {publishers}')
            if not samples:
                error(f'{topic}: no samples received')
            wrong_frames = sorted(set(s['frame'] for s in samples if s['frame'] != expected_frame))
            if wrong_frames:
                error(f'{topic}: incorrect frames {wrong_frames}')
            stamps = [s['stamp'] for s in samples]
            if any(b <= a for a, b in zip(stamps, stamps[1:])):
                error(f'{topic}: non-increasing sensor timestamps')
            if message_type is PointCloud2 and samples:
                fields = samples[0]['fields']
                if topic == '/velodyne_points' and fields[:4] != ['x','y','z','intensity']:
                    error(f'{topic}: missing x/y/z/intensity fields: {fields}')
                if not {'x', 'y', 'z'} <= set(fields):
                    error(f'{topic}: missing XYZ fields: {fields}')
            result[topic] = {
                'publishers': publishers, 'messages': len(samples),
                'frame': samples[0]['frame'] if samples else None,
                'first': samples[0] if samples else None,
            }
        for topic in ('/clock', '/odom'):
            publishers = self.count_publishers(topic)
            if publishers != 1:
                error(f'{topic}: expected one publisher, found {publishers}')
            result[topic] = {'publishers': publishers}
        if len(self.clock) < 4:
            error('Too few /clock samples')
        elif any(b < a for a, b in zip(self.clock, self.clock[1:])):
            error('/clock time moved backwards')
        expected_tf = {
            'velodyne': ('base_link', .37553552, .01003163, .6965518),
            'zed_left_camera_frame': ('base_link', .45618349, .08084175, .73042095),
            'zed_right_camera_frame': ('base_link', .45618349, -.03915825, .73042095),
        }
        tf_result = {}
        for child, expected in expected_tf.items():
            variants = self.transforms[child]
            if len(variants) != 1:
                error(f'TF child {child}: expected one static transform, found {len(variants)}')
            else:
                actual = next(iter(variants))
                if actual[0] != expected[0] or np.linalg.norm(
                        np.array(actual[1:4]) - np.array(expected[1:])) > .005:
                    error(f'TF child {child}: incorrect transform {actual}')
            tf_result[child] = [list(v) for v in sorted(variants)]
        return {'topics': result, 'tf_static': tf_result}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--output', type=str, default='')
    args = parser.parse_args()
    if not 4 <= args.seconds <= 60:
        parser.error('--seconds must be between 4 and 60')
    failures = []
    client = carla_module().Client('127.0.0.1', 2000)
    client.set_timeout(6)
    world = client.get_world()
    actor_result = validate_actors(world, failures.append)
    rclpy.init()
    audit = TopicAudit()
    try:
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(audit, timeout_sec=.2)
        report = {
            'elapsed_wall_s': round(time.monotonic() - start, 2),
            'actors': actor_result,
            'interface': audit.report(failures.append),
            'passed': not failures,
            'failures': failures,
        }
        encoded = json.dumps(report, indent=2)
        print(encoded)
        if args.output:
            Path(args.output).write_text(encoded + '\n', encoding='utf-8')
        if failures:
            raise SystemExit(1)
    finally:
        audit.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
