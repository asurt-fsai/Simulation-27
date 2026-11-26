"""Test lifecycle ordering and failure isolation without a running CARLA."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from module_control import definitions, module_plan

source_launchers = Path(__file__).resolve().parents[2] / "Simulator/Launchers"
sys.path.insert(0, str(source_launchers if source_launchers.is_dir()
                       else Path("/mnt/e/ASURT/Simulation/CARLA/Launchers")))
from managed_modules import ManagedModules


class Child:
    def __init__(self, pid, name):
        self.pid = pid
        self.name = name
        self.code = None

    def poll(self):
        return self.code


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.entries = definitions({})
        self.current = {"bridge": Child(41, "bridge")}
        self.events = []
        self.manager = ManagedModules(self.entries, self.current, self.start, self.stop)

    def start(self, name, command):
        self.events.append(("start", name))
        self.assertEqual(command[0], "bash")
        self.assertTrue(Path(command[1]).is_file())
        self.current[name] = Child(100 + len(self.events), name)

    def stop(self, child):
        self.events.append(("stop", child.name))

    def apply(self, **overrides):
        modules, _, enabled = module_plan(self.entries, overrides)
        self.manager.reconfigure(modules)
        self.manager.reconcile(enabled)

    def test_defaults_have_no_experimental_processes(self):
        self.apply()
        self.assertEqual(self.events, [("start", name) for name in
            ("zed2i", "velodyne", "lidar_cones", "zed_converter")])
        self.assertIn("bridge", self.current)

    def test_dependencies_stop_in_reverse_and_independently_restart(self):
        self.apply(slam_cone_map=True, slam_fusion=True)
        self.events.clear()
        self.apply(slam_cone_map=True, slam_fusion=True, velodyne=False)
        self.assertEqual(self.events, [("stop", name) for name in
            ("slam_fusion", "lidar_cones", "velodyne")])
        self.assertIn("slam_cone_map", self.current)
        self.events.clear()
        self.apply(slam_cone_map=True, slam_fusion=True)
        self.assertEqual(self.events, [("start", name) for name in
            ("velodyne", "lidar_cones", "slam_fusion")])

    def test_failed_module_retries_without_tearing_down_bridge(self):
        with patch("managed_modules.time.monotonic", side_effect=(1, 2, 3, 32)):
            self.apply(slam_camera=True)
            self.current["slam_camera"].code = 1
            self.events.clear()
            self.apply(slam_camera=True)
            self.assertEqual(self.events, [("stop", "slam_camera")])
            self.assertIn("Exited 1", self.manager.snapshot()["slam_camera"]["error"])
            self.apply(slam_camera=True)
            self.assertNotIn("slam_camera", self.current)
            self.apply(slam_camera=True)
            self.assertEqual(self.events[-1], ("start", "slam_camera"))
            self.assertIsNone(self.manager.snapshot()["slam_camera"]["error"])
            self.assertIsNone(self.current["bridge"].poll())

    def test_local_replacement_restarts_dependents_without_restarting_bridge(self):
        self.apply(slam_cone_map=True, planning_classical=True)
        new = definitions({"slam_cone_map": {"binary": "/tmp/team/cone_mapper_node"}})
        self.events.clear()
        self.manager.reconfigure(new)
        self.assertEqual(self.events, [("stop", "planning_classical"), ("stop", "slam_cone_map")])
        self.entries = new
        self.apply(slam_cone_map=True, planning_classical=True)
        self.assertEqual(self.events[-2:], [("start", "slam_cone_map"), ("start", "planning_classical")])
        self.assertIn("bridge", self.current)

    def test_fake_replaces_real_map_provider_and_restarts_existing_consumers(self):
        self.apply(slam_cone_map=True, planning_classical=True)
        self.events.clear()
        self.apply(slam_fake=True, slam_cone_map=True, planning_classical=True)
        self.assertEqual(self.events, [
            ("stop", "planning_classical"), ("stop", "slam_cone_map"),
            ("start", "slam_fake"), ("start", "planning_classical"),
        ])
        self.events.clear()
        self.apply(slam_cone_map=True, planning_classical=True)
        self.assertEqual(self.events, [
            ("stop", "planning_classical"), ("stop", "slam_fake"),
            ("start", "slam_cone_map"), ("start", "planning_classical"),
        ])

    def test_fake_map_planner_runs_without_perception(self):
        self.apply(slam_fake=True, planning_track_goal=True, control_stanley=True,
                   zed2i=False, velodyne=False)
        self.assertIn("slam_fake", self.current)
        self.assertIn("planning_track_goal", self.current)
        self.assertIn("control_stanley", self.current)
        self.assertNotIn("slam_cone_map", self.current)
        self.assertNotIn("zed_converter", self.current)

    def test_switch_to_fake_slam_stops_fake_map_before_starting_next_publisher(self):
        switches = dict(slam_cone_map=True, planning_classical=True, zed2i=False, velodyne=False)
        self.apply(slam_fake=True, **switches)
        self.events.clear()
        self.apply(slam_fake=True, slam_fake_slam=True, **switches)
        self.assertEqual(self.events, [
            ("stop", "planning_classical"), ("stop", "slam_fake"),
            ("start", "slam_fake_slam"), ("start", "planning_classical"),
        ])
        self.assertNotIn("slam_cone_map", self.current)
        self.assertNotIn("zed_converter", self.current)
