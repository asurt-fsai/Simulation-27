"""Handbook DE6.3: both small and large orange cone variants survive import.

Pure offline tests. Never connect to the live CARLA server or spawn actors.
"""

import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock
from types import SimpleNamespace

IMPORTER_PATH = Path(__file__).resolve().parents[1] / "import_cone_track.py"
SPEC = importlib.util.spec_from_file_location("asurt_track_orange_import", IMPORTER_PATH)
importer = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = importer
SPEC.loader.exec_module(importer)


class OrangeConeContractTests(unittest.TestCase):
    def setUp(self):
        work = tempfile.TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.root = Path(work.name)
        self.records = [
            ("blue", 0., -1.9),
            ("yellow", 0., 1.9),
            ("blue", 5., -1.9),
            ("yellow", 5., 1.9),
            ("orange", -5., -1.9),
            ("orange", -5., 1.9),
            ("orange_big", -.75, -3.5),
            ("orange_big", .75, 3.5),
        ]

    def test_csv_json_structures_preserve_both_oranges_and_start_gate(self):
        csv_path = self.root / "cones.csv"
        with csv_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("tag", "x", "y", "z"))
            for tag, x, y in self.records:
                writer.writerow((tag, x, y, 0))
        json_objects = self.root / "objects.json"
        json_objects.write_text(json.dumps({"cones": [
            {"tag": c, "x": x, "y": y, "z": 0.0}
            for c, x, y in self.records]}))
        json_arrays = self.root / "arrays.json"
        json_arrays.write_text(json.dumps({
            "x": [x for _, x, _ in self.records],
            "y": [y for _, _, y in self.records],
            "color": [c for c, _, _ in self.records],
        }))
        json_grouped = self.root / "grouped.json"
        json_grouped.write_text(json.dumps({
            c: [[x, y] for tag, x, y in self.records if tag == c]
            for c in ("blue", "yellow", "orange", "orange_big")
        }))
        expected = importer.load_cones(csv_path)
        self.assertEqual(len(expected), 8)
        self.assertEqual({c.color for c in expected},
                         {"blue", "yellow", "orange", "orange_big"})
        self.assertEqual(expected, importer.load_cones(json_objects))
        self.assertEqual(expected, importer.load_cones(json_arrays))
        grouped = importer.load_cones(json_grouped)
        self.assertCountEqual(grouped, expected)
        self.assertEqual(importer.track_start(expected), (0., 0., 0., 3.8))
        self.assertFalse(importer.suggested_invert_y(expected))

    def test_explicit_custom_blueprints_for_both_orange_sizes(self):
        # Fake blueprint library verifies the importer chooses distinct
        # physical meshes, rather than silently mapping both orange sizes
        # onto the same neutral prop.
        bps = {}
        for color in ("blue", "yellow", "orange", "orange_big"):
            type_id = importer.CUSTOM_CONE_PROPS[color]
            blueprint = Mock()
            blueprint.id = type_id
            blueprint.has_attribute.return_value = True
            blueprint.get_attribute.return_value.is_modifiable = True
            bps[type_id] = blueprint
        library = Mock()
        library.find.side_effect = lambda name: bps[name]
        world = Mock()
        world.get_blueprint_library.return_value = library
        selected = importer.choose_blueprints(
            importer.load_cones(self._write_minimal()),
            SimpleNamespace(blue_blueprint=None, yellow_blueprint=None,
                            orange_blueprint=None, orange_big_blueprint=None,
                            allow_neutral_props=False), world)
        self.assertEqual(selected["orange"].id,
                         "static.prop.asurtorangecone")
        self.assertEqual(selected["orange_big"].id,
                         "static.prop.asurtbigorangecone")
        self.assertNotEqual(selected["orange"].id, selected["orange_big"].id)

    def _write_minimal(self):
        path = self.root / "minimal.json"
        path.write_text(json.dumps([{"tag": c, "position": [x, y]}
                                   for c, x, y in self.records]))
        return path


if __name__ == "__main__":
    unittest.main()
