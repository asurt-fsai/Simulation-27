#!/usr/bin/env python3
"""Isolated ROS2 acceptance for acceleration and skidpad planning outputs.

Run with ROS_DOMAIN_ID=194, installed workspace sourced, and --mode. No live
CARLA connection, command publishers, or System Manager state changes.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from asurt_msgs.msg import Landmark, LandmarkArray
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def message_stamp(t):
    sec = int(t)
    nano = int((t-sec)*1e9)
    return sec, nano


def cone_array(t, mode, valid=True):
    msg = LandmarkArray()
    msg.header.frame_id = 'map' if mode == 'skidpad' else 'base_link'
    msg.header.stamp.sec, msg.header.stamp.nanosec = message_stamp(t)
    if not valid:
        return msg
    if mode == 'acceleration':
        points = [(float(2+2*i), y, kind) for kind, y in
                  ((Landmark.BLUE_CONE, 2.), (Landmark.YELLOW_CONE, -2.))
                  for i in range(6)]
    else:
        radius = 9.125
        points = []
        for idx, side in enumerate((-1., 1.)):
            for kind, rr in ((Landmark.BLUE_CONE, radius + (1.5 if idx == 0 else -1.5)),
                             (Landmark.YELLOW_CONE, radius + (-1.5 if idx == 0 else 1.5))):
                for i in range(18):
                    theta = i*math.pi/9
                    points.append((15+rr*math.cos(theta),
                                   side*radius+rr*math.sin(theta), kind))
    for x, y, kind in points:
        l = Landmark()
        l.position.x, l.position.y, l.type, l.probability = x, y, kind, .95
        msg.landmarks.append(l)
    return msg


def exercise(mode):
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0'):
        raise RuntimeError('Refusing to run on live default ROS domain')
    module = 'asurt_%s_planning.node' % mode
    proc = subprocess.Popen(
        [sys.executable, '-m', module, '--ros-args', '-p', 'use_sim_time:=true',
         '-p', 'input_source:=' + ('map' if mode == 'skidpad' else 'perception'),
         '-r', '/odom:=/mission_test/odom',
         '-r', '/map/global_cones:=/mission_test/map',
         '-r', '/perception_landmarks:=/mission_test/cones'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    rclpy.init()
    node = rclpy.create_node('mission_ros_contract_' + mode)
    clock = node.create_publisher(Clock, '/clock', 10)
    odom = node.create_publisher(Odometry, '/mission_test/odom', 10)
    local = node.create_publisher(LandmarkArray, '/mission_test/cones', 10)
    mapped = node.create_publisher(LandmarkArray, '/mission_test/map', 10)
    paths, statuses = [], []
    node.create_subscription(Path, '/planning/' + mode + '/path',
                             lambda p: paths.append(p), 10)
    node.create_subscription(String, '/planning/' + mode + '/status',
                             lambda m: statuses.append(json.loads(m.data)), 10)
    sim_time = 20.

    def feed(valid=True):
        nonlocal sim_time
        sim_time += .05
        t = Clock()
        t.clock.sec, t.clock.nanosec = message_stamp(sim_time)
        clock.publish(t)
        pose = Odometry()
        pose.header.stamp = t.clock
        pose.header.frame_id = 'map'
        pose.child_frame_id = 'base_link'
        pose.pose.pose.orientation.w = 1.
        odom.publish(pose)
        arr = cone_array(sim_time, mode, valid)
        (mapped if mode == 'skidpad' else local).publish(arr)
        rclpy.spin_once(node, timeout_sec=.08)
        if proc.poll() is not None:
            raise RuntimeError('Planner exited while testing: ' + mode)

    try:
        deadline = time.monotonic()+16
        while time.monotonic() < deadline:
            feed()
            if any(p.poses and p.header.frame_id == 'map' for p in paths) and any(
                    s['state'] == 'preview' and s.get('mode') == mode
                    and not s['control_ready'] for s in statuses):
                break
        else:
            raise AssertionError(mode + ' failed to publish a map-frame path')
        expected = 290 if mode == 'skidpad' else 20
        last = next(p for p in reversed(paths) if p.poses)
        assert len(last.poses) >= expected, (mode, len(last.poses), expected)
        paths.clear()
        deadline = time.monotonic()+7
        while time.monotonic() < deadline:
            feed(valid=False)
            if any(not p.poses for p in paths) and statuses[-1]['state'] == 'unavailable':
                break
        else:
            raise AssertionError(mode + ' retained an invalid previous path')
        print('PASS', mode, 'path and invalid-data clear; latest status:', statuses[-1]['reason'])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            _, stderr = proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            _, stderr = proc.communicate(timeout=3)
        if proc.returncode not in (-15, 0):
            print('Planner stderr:', stderr.decode(errors='replace')[-1500:])


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--mode', choices=('acceleration', 'skidpad'), required=True)
    exercise(p.parse_args().mode)
