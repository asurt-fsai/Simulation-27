"""Verify aligned map views, independent mission layers, passive launchers."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from build_planning_profiles import (configure, configure_control,
                                     configure_goal_pose, configure_track_control, flatten)


HERE = Path(__file__).resolve().parent
CONFIG = HERE / "config"
MODES = ("classical", "acceleration", "skidpad", "deep_learning")
MAP_URDF = HERE.parents[1] / "Simulation/Systems/SLAM/Visualization/fsai_vehicle_map_ground_preview.urdf"
LOCAL_URDF = MAP_URDF.with_name("fsai_vehicle_preview.urdf")


def cfg(filename):
    return yaml.safe_load((CONFIG / filename).read_text())


class PlanningViewTests(unittest.TestCase):
    def test_high_speed_track_rviz_has_mapped_cones_and_stanley_topics(self):
        expected = configure_track_control(cfg("planning_classical.rviz"))
        actual = cfg("control_track_mode.rviz")
        self.assertEqual(actual, expected)
        displays = actual["Visualization Manager"]["Displays"]
        self.assertTrue(next(g for g in displays
                             if g["Name"].startswith("SLAM cones |"))["Enabled"])
        self.assertFalse(next(g for g in displays
                              if g["Name"].startswith("Perception cones |"))["Enabled"])
        group = next(g for g in displays
                     if g["Name"].startswith("Track Control |"))
        self.assertEqual(
            {x["Topic"]["Value"] for x in group["Displays"]},
            {"/planning/track_goal/path", "/planning/track_goal/goal_marker",
             "/control/stanley/target", "/control/stanley/driven_path",
             "/control/stanley/diagnostic_marker"})
        self.assertTrue(any(x.get("Class") == "rviz_default_plugins/SetGoal"
                            and x.get("Topic") == "/goal_pose"
                            for x in actual["Visualization Manager"]["Tools"]))

    def test_dedicated_control_view_has_goal_and_real_motion_comparison(self):
        expected = configure_control(cfg("planning_classical.rviz"))
        actual = cfg("control_goal_pose.rviz")
        self.assertEqual(actual, expected)
        self.assertEqual(actual["Visualization Manager"]["Global Options"]["Fixed Frame"],
                         "map")
        manager = actual["Visualization Manager"]
        self.assertTrue(any(tool.get("Class") == "rviz_default_plugins/SetGoal"
                            and tool.get("Topic") == "/goal_pose"
                            for tool in manager["Tools"]))
        group = next(g for g in manager["Displays"]
                     if g["Name"].startswith("Control | goal"))
        topics = {item["Topic"]["Value"] for item in group["Displays"]}
        self.assertEqual(topics, {
            "/planning/goal_pose/path", "/planning/goal_pose/goal_marker",
            "/control/pure_pursuit/target", "/control/pure_pursuit/driven_path",
            "/control/pure_pursuit/diagnostic_marker",
        })

    def test_goal_pose_profile_has_click_tool_route_and_target(self):
        base = cfg("planning_classical.rviz")
        actual = cfg("planning_goal_pose_mode.rviz")
        self.assertEqual(actual, configure_goal_pose(base))
        manager = actual["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        self.assertTrue(any(tool.get("Class") == "rviz_default_plugins/SetGoal"
                            and tool.get("Topic") == "/goal_pose"
                            for tool in manager["Tools"]))
        groups = manager["Displays"]
        self.assertFalse(next(g for g in groups if g["Name"].startswith("SLAM"))["Enabled"])
        group = next(g for g in groups if g["Name"].startswith("Goal Pose"))
        self.assertTrue(group["Enabled"])
        self.assertEqual([x["Topic"]["Value"] for x in group["Displays"]],
                         ["/planning/goal_pose/path", "/planning/goal_pose/goal_marker",
                          "/control/pure_pursuit/target"])

    def test_combined_and_zed_views_default_to_cylinders_and_allow_converter_comparison(self):
        # /perception_landmarks is a custom LandmarkArray, not an RViz display
        # type. Its authoritative visualization sibling is /perception_markers,
        # published by the same ZED legacy converter in base_link.
        for filename in ("perception_combined.rviz", "zed2i_live.rviz"):
            with self.subTest(filename=filename):
                manager = cfg(filename)["Visualization Manager"]
                self.assertEqual(manager["Global Options"]["Fixed Frame"], "base_link")
                markers = [d for d in manager["Displays"]
                           if d["Class"] == "rviz_default_plugins/MarkerArray"]
                by_topic = {d["Topic"]["Value"]: d for d in markers}
                self.assertIn("/perception_markers", by_topic)
                self.assertFalse(by_topic["/perception_markers"]["Enabled"])
                self.assertEqual(by_topic["/perception_markers"]["Topic"]["Reliability Policy"],
                                 "Reliable")
                self.assertIn("/zed/zed_node/cone_markers", by_topic)
                self.assertTrue(by_topic["/zed/zed_node/cone_markers"]["Enabled"],
                                "Use original correctly sized cylinder markers by default")
                self.assertIn("Converter spheres (optional)",
                              by_topic["/perception_markers"]["Name"])

    def test_all_views_have_ground_aligned_vehicle_and_map_grid(self):
        for filename in ("planning_classical.rviz", "slam_mapping.rviz",
                         "slam_diagnostics.rviz"):
            with self.subTest(filename=filename):
                manager = cfg(filename)["Visualization Manager"]
                self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
                displays = list(flatten(manager["Displays"]))
                grid = next(d for d in displays if d["Class"] == "rviz_default_plugins/Grid")
                model = next(d for d in displays if d["Class"] == "rviz_default_plugins/RobotModel")
                self.assertEqual(grid["Reference Frame"], "map")
                self.assertEqual(grid["Plane"], "XY")
                self.assertAlmostEqual(grid["Offset"]["Z"], -.02)
                self.assertEqual(Path(model["Description File"]), Path(
                    "/home/hassan/ASURT/Simulation/Systems/SLAM/Visualization/fsai_vehicle_map_ground_preview.urdf"))

        # Combined Perception uses local base_link coordinates; it must keep
        # its unshifted vehicle preview, with no arbitrary map-ground offset.
        combined = cfg("perception_combined.rviz")["Visualization Manager"]
        self.assertEqual(combined["Global Options"]["Fixed Frame"], "base_link")
        model = next(d for d in combined["Displays"]
                     if d["Class"] == "rviz_default_plugins/RobotModel")
        self.assertEqual(Path(model["Description File"]), LOCAL_URDF)

    def test_wheel_contact_offset_is_grounded_in_flat_asphalt_bridge_pose(self):
        original = ET.parse(LOCAL_URDF).getroot()
        grounded = ET.parse(MAP_URDF).getroot()
        originals = {part.attrib["name"]: part.find("origin") for part in
                     original.findall(".//visual")}
        adjusted = {part.attrib["name"]: part.find("origin") for part in
                    grounded.findall(".//visual")}
        self.assertEqual(set(originals), set(adjusted))
        for name, start in originals.items():
            before = [float(x) for x in start.attrib["xyz"].split()]
            after = [float(x) for x in adjusted[name].attrib["xyz"].split()]
            self.assertAlmostEqual(after[2], before[2]-.5, places=6, msg=name)
            self.assertEqual(after[:2], before[:2])
        # Bridge map z≈0.503, front wheel radius=0.2565, wheel origin
        # after correction z≈-0.2521: contact near the FlatAsphalt z=0 plane.
        front_z = float(adjusted["front_left_wheel"].attrib["xyz"].split()[2])
        self.assertAlmostEqual(.5031 + front_z - .2565, 0, delta=.015)

    def test_planning_groups_are_independently_toggleable(self):
        base = cfg("planning_classical.rviz")
        manager = base["Visualization Manager"]
        self.assertEqual(manager["Global Options"]["Fixed Frame"], "map")
        groups = manager["Displays"]
        self.assertEqual(len(groups), 7)
        self.assertTrue(all(g["Class"] == "rviz_common/Group" for g in groups))
        for mode in MODES:
            group = next(g for g in groups
                         if g["Name"].lower().startswith(mode.replace("_", " ")))
            self.assertTrue(group["Enabled"])
            items = {d["Topic"]["Value"]: d for d in group["Displays"]}
            if mode == "deep_learning":
                self.assertFalse(items["/planning/deep_learning/path"]["Enabled"])
                self.assertTrue(items["/planning/deep_learning/raw_path"]["Enabled"])
                self.assertIn("RAW LSTM", items["/planning/deep_learning/raw_path"]["Name"])
            else:
                self.assertTrue(items["/planning/"+mode+"/path"]["Enabled"])
            if mode != "deep_learning":
                self.assertFalse(items["/planning/"+mode+"/markers"]["Enabled"])
            else:
                self.assertNotIn("/planning/deep_learning/markers", items)
        flat = {d.get("Topic", {}).get("Value"): d
                for d in flatten(groups)}
        self.assertNotIn("/zed/zed_node/cone_markers", flat)
        self.assertIn("/map/global_cones_markers", flat)
        self.assertFalse(flat["/odom"]["Enabled"])

    def test_mode_presets_single_visible_path_and_idempotent_generation(self):
        base = cfg("planning_classical.rviz")
        self.assertEqual(configure(base, "overview"), base)
        for active in MODES:
            filename = "planning_"+active+"_mode.rviz"
            preset = cfg(filename)
            self.assertEqual(preset, configure(base, active))
            groups = preset["Visualization Manager"]["Displays"]
            for mode in MODES:
                group = next(g for g in groups
                             if g["Name"].lower().startswith(mode.replace("_", " ")))
                self.assertEqual(group["Enabled"], mode == active)
            self.assertTrue(groups[0]["Enabled"])
            self.assertTrue(groups[1]["Enabled"])
            if active == "deep_learning":
                self.assertEqual(len(groups), 9)
                self.assertEqual(preset["Visualization Manager"]["Global Options"]
                                 ["Fixed Frame"], "base_link")
                live = groups[1]
                self.assertIn("Perception | LIVE", live["Name"])
                camera = live["Displays"][0]
                self.assertEqual(camera["Topic"]["Value"], "/zed/zed_node/cone_markers")
                self.assertTrue(camera["Enabled"])
                self.assertIn("cylinders", camera["Name"])
                self.assertFalse(groups[2]["Enabled"], "SLAM map must default OFF")
                self.assertEqual(groups[2]["Displays"][0]["Topic"]["Value"],
                                 "/map/global_cones_markers")
                local_grid, local_robot = groups[0]["Displays"]
                self.assertEqual(local_grid["Reference Frame"], "base_link")
                self.assertEqual(local_robot["Description File"], str(LOCAL_URDF))
                self.assertTrue(next(x for x in groups if
                                     x["Name"].startswith("Deep Learning"))["Enabled"])
                control = next(g for g in groups if
                               g["Name"].startswith("Control | Pure Pursuit"))
                self.assertTrue(control["Enabled"])
                self.assertEqual(control["Displays"][0]["Class"],
                                 "rviz_default_plugins/Marker")
                self.assertEqual(control["Displays"][0]["Topic"]["Value"],
                                 "/control/pure_pursuit/target")
            else:
                self.assertEqual(len(groups), 7)
                self.assertEqual(preset["Visualization Manager"]["Global Options"]
                                 ["Fixed Frame"], "map")
            self.assertFalse(groups[-1]["Enabled"])

    def test_no_publishing_tools_or_planner_activations(self):
        manager = cfg("planning_classical.rviz")["Visualization Manager"]
        self.assertTrue(all(tool["Class"] in {
            "rviz_default_plugins/Interact", "rviz_default_plugins/MoveCamera",
            "rviz_default_plugins/Select", "rviz_default_plugins/FocusCamera",
        } for tool in manager["Tools"]))
        script = (HERE / "launch_planning.sh").read_text()
        self.assertNotIn("manage.py", script)
        self.assertNotIn("--enable", script)
        self.assertIn('exec rviz2 -d "$config"', script)
        for mode in MODES:
            self.assertIn(mode, script)


if __name__ == "__main__":
    unittest.main()
