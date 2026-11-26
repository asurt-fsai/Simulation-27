#!/usr/bin/env python3
"""Read-only inventory of cone and bridge-odom geometry; no ROS publishers."""
import json
import math
import time
from heapq import nsmallest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from asurt_msgs.msg import LandmarkArray
from nav_msgs.msg import Odometry

from asurt_classical_planning.geometry import Config, centerline, in_map


def pose_from_odom(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return (float(p.x), float(p.y), yaw)


def message_time(message):
    stamp = message.header.stamp
    return round(stamp.sec + stamp.nanosec / 1e9, 3)


def inspect(seconds=5.):
    rclpy.init()
    node = rclpy.create_node('asurt_classical_planning_input_probe')
    latest = {}
    node.create_subscription(Odometry, '/odom',
                             lambda m: latest.__setitem__('odom', m), qos_profile_sensor_data)
    node.create_subscription(LandmarkArray, '/map/global_cones',
                             lambda m: latest.__setitem__('map', m), 10)
    node.create_subscription(LandmarkArray, '/perception_landmarks',
                             lambda m: latest.__setitem__('perception', m), 10)
    try:
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.15)
        summary = {}
        odom = latest.get('odom')
        pose = pose_from_odom(odom) if odom and odom.header.frame_id == 'map' else None
        summary['odom'] = {
            'received': bool(odom), 'frame': odom.header.frame_id if odom else None,
            'planar_pose': tuple(round(v, 3) for v in pose) if pose else None,
            'stamp_s': message_time(odom) if odom else None,
        }
        for source in ('map', 'perception'):
            msg = latest.get(source)
            if msg is None:
                summary[source] = {'received': False}
                continue
            raw = [(float(l.position.x), float(l.position.y), int(l.type),
                    float(l.probability)) for l in msg.landmarks]
            info = {
                'received': True, 'frame': msg.header.frame_id,
                'stamp_s': message_time(msg),
                'count': len(raw),
                'blue': sum(l[2] == 0 for l in raw),
                'yellow': sum(l[2] == 1 for l in raw),
                'confidence_range': (
                    [round(min(l[3] for l in raw), 3),
                     round(max(l[3] for l in raw), 3)] if raw else None),
            }
            if len(raw) <= 24:
                info['observed_xy_type'] = [
                    [round(x, 2), round(y, 2), color] for x, y, color, _ in raw]
            if pose and msg.header.frame_id == ('map' if source == 'map' else 'base_link'):
                # Same geometrical algorithm, without any active planner node.
                origin = pose[:2] if source == 'map' else (0., 0.)
                cones = []
                for color in (0, 1):
                    eligible = (entry for entry in raw[:2048]
                                if entry[2] == color and entry[3] >= .25
                                and all(math.isfinite(v) for v in entry))
                    cones.extend(nsmallest(64, eligible,
                                           key=lambda e: (e[0]-origin[0])**2
                                           + (e[1]-origin[1])**2))
                if source == 'perception':
                    cones = in_map(cones, pose)
                proposal = centerline(cones, pose, Config())
                info['sample_preview'] = {'reason': proposal.reason,
                                          'pairs': proposal.pairs,
                                          'points': len(proposal.points)}
                if source == 'perception':
                    info['odom_stamp_delta_s'] = round(
                        message_time(odom)-message_time(msg), 3)
            summary[source] = info
        print(json.dumps(summary, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    inspect()
