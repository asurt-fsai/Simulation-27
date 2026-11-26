#!/usr/bin/env python3
"""Isolated ROS-domain planner contract test, no CARLA or live publishers."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from asurt_msgs.msg import Landmark, LandmarkArray
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


PLANNING = Path(__file__).resolve().parents[1]
EXECUTABLE = (PLANNING / 'Workspace/planning_integration/install'
              / 'asurt_classical_planning/lib/asurt_classical_planning/classical_planner_node')


def stamp(seconds):
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds-sec)*1e9)))


def odometry(t):
    msg = Odometry()
    msg.header.stamp = stamp(t)
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.
    return msg


def cones(t, two_sides=True, frame='base_link', count=5):
    msg = LandmarkArray()
    msg.header.stamp = stamp(t)
    msg.header.frame_id = frame
    # Deliberately group by colour: naive truncation of a large map could
    # remove the yellow side entirely and fabricate missing-data behavior.
    for color, side in ((Landmark.BLUE_CONE, 2.), (Landmark.YELLOW_CONE, -2.)):
        for i in range(count):
            if not two_sides and color == Landmark.YELLOW_CONE:
                continue
            landmark = Landmark()
            landmark.type = color
            landmark.position.x = 2. + i*2.
            landmark.position.y = side
            landmark.probability = .98
            msg.landmarks.append(landmark)
    return msg


def exercise():
    if not EXECUTABLE.exists():
        raise RuntimeError('build the asurt_classical_planning package first')
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('set an isolated ROS_DOMAIN_ID, e.g. 193; refusing live domain')
    # Run the source entrypoint in this test domain while the production
    # planner module stays off in the live System Manager.
    p = subprocess.Popen([
        sys.executable, '-m', 'asurt_classical_planning.node', '--ros-args',
        '-p', 'use_sim_time:=true',
        '-p', 'input_source:=auto',
        '-r', '/odom:=/planning_test/odom',
        '-r', '/perception_landmarks:=/planning_test/cones',
        '-r', '/map/global_cones:=/planning_test/map',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    rclpy.init()
    node = rclpy.create_node('classical_planning_contract_test')
    clk = node.create_publisher(Clock, '/clock', 10)
    odom_pub = node.create_publisher(Odometry, '/planning_test/odom', 10)
    cone_pub = node.create_publisher(LandmarkArray, '/planning_test/cones', 10)
    map_pub = node.create_publisher(LandmarkArray, '/planning_test/map', 10)
    paths = []
    statuses = []
    node.create_subscription(RosPath, '/planning/classical/path',
                             lambda m: paths.append(len(m.poses)), 10)
    node.create_subscription(String, '/planning/classical/status',
                             lambda m: statuses.append(json.loads(m.data)), 10)

    try:
        current_time = 20.
        def publish(two_sides=True, fresh=True, map_data=False):
            nonlocal current_time
            current_time += .05
            clock = Clock()
            clock.clock = stamp(current_time)
            clk.publish(clock)
            if fresh:
                odom_pub.publish(odometry(current_time))
                cone_pub.publish(cones(current_time, two_sides=two_sides))
                if map_data:
                    map_pub.publish(cones(current_time, two_sides=two_sides,
                                          frame='map', count=100))
            rclpy.spin_once(node, timeout_sec=.07)
            if p.poll() is not None:
                raise RuntimeError('planner executable exited prematurely')

        until = time.monotonic() + 10.
        while time.monotonic() < until:
            publish(map_data=True)
            if any(n >= 3 for n in paths) and any(
                    s['state'] == 'preview' and s['source'] == 'map'
                    and not s['control_ready'] for s in statuses):
                break
        else:
            raise AssertionError('expected map-frame path from SLAM cone map and /odom')

        # Let the test SLAM map expire; auto must use fresh local Perception.
        until = time.monotonic() + 8.
        while time.monotonic() < until:
            publish()
            if any(s['state'] == 'preview' and s['source'] == 'perception'
                   and not s['control_ready'] for s in statuses):
                break
        else:
            raise AssertionError('no Perception fallback after map went stale')

        # Now remove the yellow boundary. The last RViz path must be deleted.
        paths.clear()
        until = time.monotonic() + 5.
        while time.monotonic() < until:
            publish(two_sides=False)
            if 0 in paths and any(
                    s['state'] == 'unavailable'
                    and s['reason'] == 'need_two_cones_each_side' for s in statuses):
                break
        else:
            raise AssertionError('stale path remained after one-sided input')

        # Re-establish a path, then rewind /clock without fresh landmarks.
        paths.clear()
        until = time.monotonic() + 5.
        while time.monotonic() < until:
            publish()
            if any(n >= 3 for n in paths):
                break
        else:
            raise AssertionError('planner did not recover on fresh paired cones')
        paths.clear()
        current_time = 1.
        until = time.monotonic() + 5.
        while time.monotonic() < until:
            publish(fresh=False)
            if 0 in paths and statuses and statuses[-1]['state'] == 'unavailable':
                break
        else:
            raise AssertionError('clock rewind did not clear old path')
        print('PASS isolated ROS contract: SLAM map priority, Perception fallback, '
              'one-sided clearing, clock rewind')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        p.terminate()
        try:
            _, err = p.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
            _, err = p.communicate(timeout=3)
        if p.returncode not in (-15, 0):
            print('Planner stderr:', err.decode(errors='replace')[-2000:])


if __name__ == '__main__':
    exercise()
