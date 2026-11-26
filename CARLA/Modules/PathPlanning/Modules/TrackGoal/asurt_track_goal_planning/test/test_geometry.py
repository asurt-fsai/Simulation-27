"""Mapped track planning contracts with no CARLA or ROS process."""
import json
import math
from pathlib import Path
import unittest

from asurt_track_goal_planning.geometry import (
    Config, Pose2, _max_curvature, plan_track, route_clearance,
)


def track_cones(number=13, gap=4., half_width=2.):
    return tuple((gap*i, y, kind, 1.) for i in range(number)
                 for kind, y in ((0, half_width), (1, -half_width)))


def curved_track(radius, number=34, step=.08):
    """An oriented left bend with blue inside / yellow outside."""
    cones = []
    for i in range(number):
        angle = i*step
        x = radius*math.sin(angle)
        y = radius*(1-math.cos(angle))
        nx, ny = -math.sin(angle), math.cos(angle)
        cones.extend(((x+2*nx, y+2*ny, 0, 1.),
                      (x-2*nx, y-2*ny, 1, 1.)))
    end = (number-4)*step
    return (tuple(cones), Pose2(0, 0, 0),
            Pose2(radius*math.sin(end), radius*(1-math.cos(end)), end))


def circular_track(radius):
    count = round(2*math.pi*radius/2.5)
    cones = []
    for i in range(count):
        angle = 2*math.pi*i/count
        x, y = radius*math.cos(angle), radius*math.sin(angle)
        nx, ny = -math.cos(angle), -math.sin(angle)
        cones.extend(((x+2*nx, y+2*ny, 0, 1.),
                      (x-2*nx, y-2*ny, 1, 1.)))
    last = 2*math.pi*(count-1)/count
    return (tuple(cones), Pose2(radius, 0, math.pi/2),
            Pose2(radius*math.cos(last), radius*math.sin(last),
                  last+math.pi/2))


class TrackGeometryTest(unittest.TestCase):
    def test_straight_track_reaches_manual_goal(self):
        cones = track_cones()
        result = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0), cones)
        self.assertEqual(result.reason, "ok")
        self.assertGreater(len(result.points), 50)
        self.assertLess(abs(result.points[-1][0]-36.), 1e-5)
        self.assertTrue(all(abs(y) < .15 for _, y in result.points))
        self.assertGreaterEqual(result.minimum_clearance_m, .72)

    def test_single_orange_cone_in_track_requires_detour(self):
        cones = track_cones() + ((14., 0., 2, 1.),)
        result = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0), cones)
        self.assertEqual(result.reason, "ok")
        self.assertGreater(result.lateral_changes, 0)
        self.assertGreaterEqual(route_clearance(result.points, cones), .72)
        self.assertGreater(max(abs(y) for x, y in result.points if 12<x<17), .75)
        self.assertLessEqual(_max_curvature(result.points), Config().max_path_curvature_m_inv)

    def test_curved_track_uses_nearby_gates_and_driveable_tangents(self):
        # Previously rejected even 30 m radius bends due to skipping the
        # first five gates and imposing sudden end-of-lane heading changes.
        for radius in (10., 12., 20., 30.):
            with self.subTest(radius=radius):
                cones, start, goal = curved_track(radius)
                result = plan_track(start, goal, cones)
                self.assertEqual(result.reason, "ok")
                self.assertGreater(result.gates_used, 10)
                self.assertLessEqual(_max_curvature(result.points), .18)
                self.assertGreaterEqual(route_clearance(result.points, cones), .72)

    def test_goal_behind_vehicle_follows_full_mapped_circuit(self):
        # The old chord-based cone crop and initial-terminal shortcut either
        # dropped the rear half of the map or returned only a single gate.
        for radius in (20., 30., 55.):
            with self.subTest(radius=radius):
                cones, start, goal = circular_track(radius)
                result = plan_track(start, goal, cones)
                self.assertEqual(result.reason, "ok")
                self.assertGreater(result.route_length_m, 5*radius)
                self.assertGreater(result.gates_used, len(cones)//2-5)
                self.assertEqual(result.points[0], (start.x, start.y))
                self.assertEqual(result.points[-1], (goal.x, goal.y))
                self.assertLessEqual(_max_curvature(result.points), .18)
                self.assertLessEqual(len(result.points), Config().max_path_points)

    def test_open_track_cannot_invent_full_lap_for_goal_behind(self):
        cones = track_cones(number=21)
        result = plan_track(Pose2(1, 0, 0), Pose2(-2, 0, 0), cones)
        self.assertFalse(result.points)

    def test_map_over_budget_is_rejected_instead_of_silently_truncated(self):
        cones = track_cones(number=13)
        result = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0),
                            cones, Config(max_cones=12))
        self.assertEqual(result.reason, "mapped_track_exceeds_cone_budget")

    def test_confirmed_static_map_can_retain_aged_low_covariance_scores(self):
        cones = tuple((x, y, kind, .09)
                      for x, y, kind, _ in track_cones(number=22))
        start, goal = Pose2(1., 0., 0.), Pose2(72., 0., 0.)
        ordinary = plan_track(start, goal, cones)
        self.assertFalse(ordinary.points)
        confirmed = plan_track(start, goal, cones,
                               Config(confirmed_map=True))
        self.assertEqual(confirmed.reason, "ok")
        self.assertGreater(len(confirmed.points), 100)
        self.assertGreaterEqual(confirmed.minimum_clearance_m, .72)

    def test_observed_slam_map_goal_produces_feasible_track_path(self):
        fixture = Path(__file__).with_name("fixtures") / "mapped_cone_circuit_20261123.json"
        snapshot = json.loads(fixture.read_text())
        cones = tuple(tuple(row) for row in snapshot["cones"])
        self.assertEqual(len(cones), 127)
        vehicle = Pose2(*snapshot["start"])
        goal = Pose2(*snapshot["goal"])
        # The mapper confirms features before publication, yet covariance
        # decay puts almost all of their raw scores below the old threshold.
        rejected = plan_track(vehicle, goal, cones)
        self.assertFalse(rejected.points)
        accepted = plan_track(vehicle, goal, cones,
                              Config(confirmed_map=True))
        self.assertEqual(accepted.reason, "ok")
        self.assertGreater(len(accepted.points), 80)
        self.assertGreater(accepted.route_length_m, 25)
        self.assertLess(_max_curvature(accepted.points), .18)
        self.assertGreaterEqual(accepted.minimum_clearance_m, .72)

    def test_indexed_clearance_matches_bruteforce(self):
        cones = track_cones(number=16)
        points = [(1+i*.31, .35*math.sin(i*.25)) for i in range(151)]
        brute = min(math.dist((x, y), (a[0], a[1])) if a == b else
                    __import__('asurt_track_goal_planning.geometry', fromlist=['segment_distance']).segment_distance((x, y), a, b)
                    for x, y, _, _ in cones for a, b in zip(points, points[1:]))
        self.assertAlmostEqual(route_clearance(points, cones), brute, places=10)

    def test_blocked_track_has_no_driveable_path(self):
        cones = track_cones()+tuple((14., y, 2, 1.) for y in (-1., 0., 1.))
        result = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0), cones)
        self.assertFalse(result.points)
        self.assertEqual(result.reason, "no_obstacle_clear_track_connection")

    def test_refuse_unmapped_goal_and_one_sided_track(self):
        cones = track_cones()
        self.assertFalse(plan_track(Pose2(1, 0, 0),
                                    Pose2(35, 25, 0), cones).points)
        self.assertFalse(plan_track(Pose2(1, 0, 0),
                                    Pose2(35, 0, 0),
                                    [c for c in cones if c[2] == 0]).points)
        self.assertFalse(plan_track(Pose2(1, 0, 0),
                                    Pose2(35, 0, 0), ()).points)

    def test_refuse_reversed_vehicle_and_uncertain_map(self):
        cones = track_cones()
        self.assertFalse(plan_track(Pose2(1, 0, math.pi),
                                    Pose2(36, 0, 0), cones).points)
        low_confidence = tuple((x, y, k, .05) for x, y, k, _ in cones)
        self.assertFalse(plan_track(Pose2(1, 0, 0),
                                    Pose2(36, 0, 0), low_confidence).points)

    def test_deterministic_bounded_output(self):
        cones = track_cones()
        one = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0), cones)
        two = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0), cones)
        self.assertEqual(one, two)
        self.assertLessEqual(len(one.points), Config().max_path_points)

    def test_long_mapped_straight_can_support_full_speed_braking_distance(self):
        cones = track_cones(number=49)
        result = plan_track(Pose2(1, 0, 0), Pose2(170, 0, 0), cones)
        self.assertEqual(result.reason, "ok")
        self.assertGreater(result.route_length_m, 160)
        self.assertLessEqual(len(result.points), Config().max_path_points)

    def test_large_old_slam_map_does_not_hide_nearby_cones(self):
        ancient = tuple((-1000. - 4*i, y, k, 1.)
                        for i in range(150) for k, y in ((0, 2.), (1, -2.)))
        near = track_cones()
        result = plan_track(Pose2(1, 0, 0), Pose2(36, 0, 0),
                            ancient+near)
        self.assertEqual(result.reason, "ok")


if __name__ == "__main__":
    unittest.main()
