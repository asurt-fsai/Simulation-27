"""Module enablement tests without touching the running simulator."""
import os
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest
from unittest import mock

import module_control as mod
import slam_settings


class ModuleSwitchTests(unittest.TestCase):
    def test_defaults_and_dependency_order(self):
        modules = mod.definitions({})
        perception = [m for m in modules if m.group == "Perception"]
        slam = [m for m in modules if m.group == "SLAM"]
        self.assertEqual([m.name for m in perception],
                         ["zed2i", "velodyne", "lidar_cones", "zed_converter"])
        self.assertTrue(all(m.default_enabled for m in perception))
        self.assertEqual(len(slam), 5)
        self.assertEqual({module.label for module in slam if module.name in mod.FAKE_PROVIDERS},
                         {"FakeMAP", "FakeSLAM"})
        self.assertFalse(any(m.default_enabled for m in slam))

    def test_fusion_requires_both_sources(self):
        modules = mod.definitions({"slam_fusion": {"binary": "/tmp/local_fusion"}})
        for dep in ("zed_converter", "lidar_cones"):
            desired, effective = mod.effective_states(modules, {"slam_fusion": True, dep: False})
            self.assertTrue(desired["slam_fusion"])
            self.assertFalse(effective["slam_fusion"])
        _, effective = mod.effective_states(modules, {"slam_fusion": True})
        self.assertTrue(effective["slam_fusion"])

    def test_persisted_switch_blocks_only_related_dependents(self):
        with tempfile.TemporaryDirectory() as directory, \
             mock.patch.object(mod, "CONFIG", Path(directory)), \
             mock.patch.object(mod, "STATE", Path(directory) / "modules.json"), \
             mock.patch.object(mod, "SOURCES", Path(directory) / "module-sources.json"), \
             mock.patch.object(mod, "STATE_LOCK", Path(directory) / "modules.lock"):
            desired, effective = mod.save_switches({"velodyne": False})
            self.assertFalse(effective["velodyne"])
            self.assertFalse(effective["lidar_cones"])
            self.assertTrue(effective["zed2i"])
            self.assertTrue(effective["zed_converter"])
            self.assertTrue(desired["lidar_cones"])
            reloaded = mod.saved_overrides()
            self.assertIs(reloaded["velodyne"], False)
            _, effective = mod.save_switches({"velodyne": True})
            self.assertTrue(effective["lidar_cones"])
            mod.save_switches({"slam_fake": True})
            desired, effective = mod.save_switches({"slam_fake_slam": True})
            self.assertTrue(effective["slam_fake_slam"])
            self.assertFalse(desired["slam_fake"])
            self.assertFalse(mod.saved_overrides()["slam_fake"])
            mod.save_switches({"control_fs26_simple": True})
            desired, effective = mod.save_switches({"control_fs26_adaptive": True})
            self.assertTrue(effective["control_fs26_adaptive"])
            self.assertFalse(desired["control_fs26_simple"])
            self.assertFalse(effective["planning_goal_pose"])

    def test_only_fs26_control_is_bundled_and_retired_local_selection_survives(self):
        controls = [module.name for module in mod.definitions({}) if module.group == "Control"]
        self.assertEqual(controls, ["control_fs26_simple", "control_fs26_adaptive"])
        entries = mod.definitions({"control_stanley_auto": {"binary": "/tmp/my_stanley",
                                                           "setup": "/tmp/local_setup.bash"}})
        old = next(module for module in entries if module.name == "control_stanley_auto")
        self.assertTrue(old.local)
        self.assertEqual(old.binary, "/tmp/my_stanley")
        self.assertEqual(old.setup, "/tmp/local_setup.bash")
        entries = mod.definitions({"control_stanley": {"binary": "/tmp/my_track_stanley"}})
        self.assertIn("planning_track_goal", {module.name for module in entries})
        self.assertIn("control_stanley", {module.name for module in entries})

    def test_status_explains_disabled_and_transitive_prerequisites(self):
        entries = mod.definitions({})
        plan = mod.module_plan(entries, {"planning_classical": True, "slam_cone_map": True,
                                         "zed_converter": False})
        with mock.patch.object(mod, "module_plan", return_value=plan):
            status = {row["id"]: row for row in mod.live_states()["modules"]}
        self.assertEqual(status["slam_cone_map"]["prerequisites"], ["ZED Legacy Converter"])
        self.assertIn("FS-System-26 Cone Mapper", status["planning_classical"]["status"])
        self.assertIn("ZED Legacy Converter (disabled)", status["planning_classical"]["status"])

    def test_autonomous_planner_topics_skip_manual_and_raw_preview(self):
        entries = mod.definitions({"team_planner": {"binary": "/tmp/team/planner", "label": "Team planner",
                                                    "group": "Planning", "topic": "/team/path"}})
        with mock.patch.object(mod, "definitions", return_value=entries):
            topics = mod.autonomous_planners()
        self.assertIn("/team/path", topics)
        self.assertIn("/planning/deep_learning/path", topics)
        self.assertNotIn("/planning/deep_learning/raw_path", topics)
        self.assertNotIn("/planning/goal_pose/path", topics)
        self.assertNotIn("/planning/track_goal/path", topics)

    def test_autonomous_planner_preference_persists_and_can_return_to_automatic(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(mod, "CONFIG", Path(directory)):
            self.assertEqual(mod.autonomous_planner(), "")
            mod.save_autonomous_planner("/planning/classical/path")
            self.assertEqual(mod.autonomous_planner(), "/planning/classical/path")
            mod.save_autonomous_planner("/planning/goal_pose/path")
            self.assertEqual(mod.autonomous_planner(), "/planning/goal_pose/path")
            with self.assertRaises(ValueError):
                mod.save_autonomous_planner("/planning/track_goal/path")
            mod.save_autonomous_planner("")
            self.assertEqual(mod.autonomous_planner(), "")

    def test_noise_settings_persist_independently_and_reject_invalid_values(self):
        with tempfile.TemporaryDirectory() as directory, \
             mock.patch.dict("os.environ", {"XDG_CONFIG_HOME": directory}):
            self.assertEqual(slam_settings.read_settings(), {"fake_map": 0., "fake_slam": 0.})
            saved = {"fake_map": .05, "fake_slam": .12}
            slam_settings.save_settings(saved)
            self.assertEqual(slam_settings.read_settings(), saved)
            for invalid in (-.1, float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    slam_settings.save_settings(dict(saved, fake_slam=invalid))
            self.assertEqual(slam_settings.read_settings(), saved)

    def test_new_unconfigured_module_defaults_off(self):
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "launch.sh"
            script.write_text("#!/bin/bash\n")
            registry = Path(directory) / "modules.conf"
            registry.write_text(
                "future|root/tools/future/node|launch.sh|/future/output|-|"
                "Path Planning|Planner|off\n"
            )
            with mock.patch.object(mod, "REGISTRY", registry), \
                 mock.patch.object(mod, "ROOT", Path(directory)), \
                 mock.patch.object(mod, "STATE", Path(directory) / "state.json"):
                entries = mod.definitions({})
                desired, effective = mod.effective_states(entries)
            self.assertEqual(entries[0].label, "Planner")
            self.assertFalse(desired["future"])
            self.assertFalse(effective["future"])

    def test_process_match_requires_exact_entrypoint(self):
        # No signal is sent; just inspect running ROS console scripts.
        zed = mod.definitions()[0]
        for pid in mod.node_pids(zed):
            argv = (Path("/proc") / str(pid) / "cmdline").read_bytes().split(b"\0")
            self.assertIn(zed.binary.encode(), argv[:2])

    def test_status_follows_a_supervised_local_launcher_after_exec(self):
        entries = mod.definitions({"local_mpc": {"binary": "/tmp/asurt-test-mpc-launch.sh",
                                                "label": "MPC", "group": "Control",
                                                "topic": "/mpc/predicted_path"}})
        local = next(module for module in entries if module.name == "local_mpc")
        with subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"],
                              start_new_session=True) as process:
            try:
                self.assertEqual(mod.node_pids(local, group=process.pid, supervisor=os.getpid()), [process.pid])
                self.assertEqual(mod.node_pids(local, group=process.pid, supervisor=os.getppid()), [])
                self.assertEqual(mod.node_pids(local), [])
            finally:
                process.terminate()


if __name__ == "__main__":
    unittest.main()
