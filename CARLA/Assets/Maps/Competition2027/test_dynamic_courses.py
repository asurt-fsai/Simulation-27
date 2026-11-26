"""Pure FSUK 2027 geometric and ASURT importer/planner contract tests.

There are no CARLA API calls, actor spawns, world ticks or ROS processes.
"""
import csv
from collections import Counter
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
# Simulation/CARLA/Maps may be a symlink to Assets/Maps; resolve by the
# repository's canonical importer instead of guessing the number of parents.
ROOT = next(p for p in (HERE, *HERE.parents)
            if (p / "tools/Track Importer/import_cone_track.py").is_file())
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "Simulation/Systems/PathPlanning/Modules/Classical/asurt_classical_planning"))
sys.path.insert(0, str(ROOT / "Simulation/Systems/PathPlanning/Modules/Acceleration/asurt_acceleration_planning"))
sys.path.insert(0, str(ROOT / "Simulation/Systems/PathPlanning/Modules/Skidpad/asurt_skidpad_planning"))

from generate_dynamic_courses import acceleration, skidpad
from asurt_acceleration_planning.geometry import acceleration_path
from asurt_skidpad_planning.geometry import skidpad_path

IMPORTER = ROOT / "tools/Track Importer/import_cone_track.py"
SPEC = importlib.util.spec_from_file_location("asurt_import_for_competition_tests", IMPORTER)
importer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = importer
SPEC.loader.exec_module(importer)


class CourseTests(unittest.TestCase):
    def test_acceleration_75m_3m_min_and_5m_cone_interval(self):
        cones, route, layout = acceleration()
        measured = layout["geometry"]
        self.assertEqual(measured["timed_distance_m"], 75.)
        self.assertGreaterEqual(measured["selected_driving_lane_width_m"], 3.)
        self.assertEqual(measured["finish_line_x_m"], 75.)
        self.assertEqual(measured["stop_distance_after_finish_m"], 50.)
        self.assertEqual(len(cones), 67)
        self.assertEqual(Counter(c["tag"] for c in cones),
                         Counter({"blue": 16, "yellow": 16,
                                  "orange": 27, "orange_big": 8}))
        for tag, sign in (("blue", -1), ("yellow", 1)):
            stations = sorted(c["x"] for c in cones if c["tag"] == tag
                              and c["role"] == "timed_boundary")
            self.assertEqual(stations, list(range(0, 76, 5)))
            ys = {c["y"] for c in cones if c["tag"] == tag}
            self.assertEqual(ys, {sign * 1.9})
        self.assertEqual(layout["staging"]["frontmost_vehicle_point_target_m"][0], -.30)
        self.assertEqual({(c["x"],c["y"]) for c in cones
                          if c["tag"]=="orange_big"},
                         {(-.75,-3.5),(-.75,3.5),(.75,-3.5),(.75,3.5),
                          (74.25,-3.5),(74.25,3.5),(75.75,-3.5),(75.75,3.5)})
        self.assertEqual({c["x"] for c in cones if c["role"]=="entry_lane"},
                         {-5.,-2.5})
        self.assertEqual(route[-1][1:], (120., 0.))
        self.assertEqual(layout["cone_handbook"]["entry_and_exit"], "orange (small)")
        self.assertEqual(layout["cone_handbook"]["2021_handbook_illustrated_stop_area_m"], 100.)
        self.assertEqual(layout["cone_handbook"]["2027_rule_driverless_stop_limit_m"], 50.)
        barrier = sorted(c["y"] for c in cones if
                         c["x"] == 125. and c["tag"]=="orange")
        self.assertEqual(barrier, [-1.9, -1., 0., 1., 1.9])
        self.assertTrue(all(b-a <= 1.0 for a,b in zip(barrier,barrier[1:])))

    def test_skidpad_diameters_counts_and_timing(self):
        cones, path, layout = skidpad()
        geom = layout["geometry"]
        self.assertEqual(geom["centre_separation_m"], 18.25)
        self.assertEqual(geom["inner_circle_diameter_m"], 15.25)
        self.assertEqual(geom["outer_circle_diameter_m"], 21.25)
        self.assertEqual(geom["driving_corridor_width_m"], 3.0)
        self.assertEqual(geom["inner_cones_per_circle"], 16)
        self.assertEqual(geom["outer_cones_per_circle"], 13)
        self.assertEqual(layout["staging"]["distance_before_start_finish_m"], 15.)
        self.assertEqual(layout["stop"]["required_stop_distance_after_timing_line_m"], 25.)
        ring_counts = Counter((c["circle"], c["ring"]) for c in cones if
                              c["role"] == "circle_boundary")
        self.assertEqual(ring_counts, Counter({
            ("right", "inner"): 16, ("right", "outer"): 13,
            ("left", "inner"): 16, ("left", "outer"): 13,
        }))
        self.assertEqual(len(cones), 81)  # 58 rings + 4 ordered gate + 15 small + 4 big
        self.assertEqual(path[0], ("entry", 0., 0.))
        self.assertEqual(path[-1], ("exit", 37.5, 0.))
        self.assertEqual(Counter(c["tag"] for c in cones),
                         Counter({"blue": 31, "yellow": 31,
                                  "orange": 15, "orange_big": 4}))
        self.assertEqual({(c["x"],c["y"]) for c in cones
                          if c["tag"]=="orange_big"},
                         {(14.25,-3.5),(14.25,3.5),(15.75,-3.5),(15.75,3.5)})
        self.assertEqual({c["x"] for c in cones if c["role"]=="entry_lane"
                          and c["tag"]=="orange"}, {-5., -2.5})
        self.assertEqual({c["x"] for c in cones if c["role"]=="exit_stopping_lane"},
                         {25., 30., 35.})
        barrier = sorted(c["y"] for c in cones if c["x"]==40. and c["tag"]=="orange")
        self.assertEqual(barrier, [-1.65, -1., 0., 1., 1.65])
        self.assertTrue(all(b-a<=1.0 for a,b in zip(barrier,barrier[1:])))
        for c in cones:
            if c["role"] != "circle_boundary":
                continue
            cx, cy = geom["circle_centres_m"][c["circle"]]
            radius = math.hypot(c["x"]-cx, c["y"]-cy)
            expected = geom["inner_cone_centre_radius_m"] if c["ring"] == "inner" else geom["outer_cone_centre_radius_m"]
            self.assertAlmostEqual(radius, expected, delta=2e-6)

    def test_reference_routes_stay_clear_of_all_cones(self):
        for name, func, min_clearance in (
            ("acceleration", acceleration, 1.85),
            ("skidpad", skidpad, 1.60),
        ):
            cones, path, _ = func()
            centres = [(c["x"], c["y"]) for c in cones]
            samples = [(x, y) for _, x, y in path]
            with self.subTest(mode=name):
                # Collision-like point clearance is conservative and checks
                # other circle cones as well as the ring being followed.
                smallest = min(math.dist(c, p) for c in centres for p in samples)
                self.assertGreaterEqual(smallest, min_clearance)
                nearest_pair = min(math.dist(a, b) for a, b in itertools.combinations(centres, 2))
                # Stop-area end barriers deliberately have <=1m spacing.
                self.assertGreater(nearest_pair, .60)

    def test_importer_loads_both_formats_and_preserves_start_heading(self):
        for name, first_gate in (("acceleration", 0.), ("skidpad", 0.)):
            base = HERE / name / (name + "_2027_cones")
            csv_cones = importer.load_cones(base.with_suffix(".csv"))
            json_cones = importer.load_cones(base.with_suffix(".json"))
            self.assertEqual(csv_cones, json_cones)
            self.assertFalse(importer.suggested_invert_y(csv_cones))
            x, y, yaw, width = importer.track_start(csv_cones)
            self.assertEqual((x, y, yaw), (first_gate, 0., 0.))
            self.assertAlmostEqual(width, 3.8 if name == "acceleration" else 3.3)
            self.assertGreaterEqual(len(csv_cones), 2)

    def test_reference_path_and_metadata_are_consistent(self):
        for name, builder in (("acceleration", acceleration), ("skidpad", skidpad)):
            cones, path, layout = builder()
            source = HERE / name
            saved_layout = json.loads((source / (name+"_2027_layout.json")).read_text())
            self.assertEqual(saved_layout, layout)
            with (source / (name+"_2027_reference_path.csv")).open() as stream:
                saved = list(csv.DictReader(stream))
            self.assertEqual(len(saved), len(path))
            self.assertEqual(saved[0]["stage"], path[0][0])
            svg = source / (name+"_2027_preview.svg")
            self.assertEqual(ET.parse(svg).getroot().tag.split("}")[-1], "svg")

    def test_planning_strategies_accept_rules_based_layout(self):
        # The CARLA bridge flips the Y sign when converting to ROS REP-103.
        for name, builder, strategy, start in (
            ("acceleration", acceleration, acceleration_path, (-5., 0., 0.)),
            ("skidpad", skidpad, skidpad_path, (0., 0., 0.)),
        ):
            cones, _, _ = builder()
            ros_cones = [(c["x"], -c["y"],
                          0 if c["tag"] == "blue" else 1, .95)
                         for c in cones if c["tag"] in ("blue", "yellow")]
            answer = strategy(ros_cones, start)
            with self.subTest(name=name):
                self.assertEqual(answer.reason, "ok")
                self.assertGreaterEqual(answer.pairs, 2)
                self.assertGreater(len(answer.points), 10)


if __name__ == "__main__":
    unittest.main()
