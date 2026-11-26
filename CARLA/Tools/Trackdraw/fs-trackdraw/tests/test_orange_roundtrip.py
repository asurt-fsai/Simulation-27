"""Competition cone colors survive both editable Trackdraw file formats."""
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from trackdraw.exporter import (
    export_manual_cones_csv, export_manual_cones_json,
    export_track_csv, export_track_json, transform_points_to_export_frame,
)
from trackdraw.importer import load_track_csv, load_track_json


class OrangeConeRoundTripTests(unittest.TestCase):
    def test_full_track_csv_and_json_retain_orange_type_position_and_boundary_fit(self):
        centerline = [(100.0, 100.0), (120.0, 100.0), (140.0, 100.0)]
        blue = [(110.0, 110.0), (130.0, 110.0), (130.0, 130.0), (110.0, 130.0)]
        yellow = [(110.0, 150.0), (130.0, 150.0), (130.0, 170.0), (110.0, 170.0)]
        orange = [(105.0, 95.0), (130.0, 140.0)]
        orange_big = [(100.0, 90.0), (140.0, 180.0)]
        scale = 10.0
        # The exporter rotates/scales once. Import shifts by boundary centroid;
        # entry/timing cones cannot contaminate that centerline anchor.
        boundaries_m = transform_points_to_export_frame(centerline, blue + yellow, scale)
        origin = np.mean(np.asarray(boundaries_m), axis=0)
        expected_small = np.asarray(transform_points_to_export_frame(centerline, orange, scale)) - origin
        expected_big = np.asarray(transform_points_to_export_frame(centerline, orange_big, scale)) - origin
        for ext, exporter, importer in (
            ("csv", export_track_csv, load_track_csv),
            ("json", export_track_json, load_track_json),
        ):
            with self.subTest(format=ext), tempfile.TemporaryDirectory() as tmp:
                path = str(Path(tmp) / f"track.{ext}")
                exporter(path, centerline, blue, yellow, scale, orange, orange_big)
                if ext == "csv":
                    with open(path, newline="", encoding="utf-8") as handle:
                        rows = list(csv.DictReader(handle))
                else:
                    with open(path, encoding="utf-8") as handle:
                        payload = json.load(handle)
                    self.assertEqual((payload["format"], payload["units"]),
                                     ("fs-trackdraw-cones-v1", "m"))
                    rows = payload["cones"]
                self.assertEqual(
                    [row["tag"] for row in rows].count("orange"), 2
                )
                self.assertEqual(
                    [row["tag"] for row in rows].count("orange_big"), 2
                )
                loaded = importer(path)
                self.assertEqual(len(loaded.left_cones_local_m), 4)
                self.assertEqual(len(loaded.right_cones_local_m), 4)
                self.assertGreaterEqual(len(loaded.control_points_local_m), 8)
                np.testing.assert_allclose(loaded.orange_cones_local_m, expected_small, atol=1e-9)
                np.testing.assert_allclose(loaded.orange_big_cones_local_m, expected_big, atol=1e-9)

    def test_manual_orange_only_csv_and_json_import_without_synthetic_closed_track(self):
        small = [(100.0, 40.0), (120.0, 40.0)]
        big = [(80.0, 60.0), (140.0, 60.0)]
        for ext, exporter, importer in (
            ("csv", export_manual_cones_csv, load_track_csv),
            ("json", export_manual_cones_json, load_track_json),
        ):
            with self.subTest(format=ext), tempfile.TemporaryDirectory() as tmp:
                path = str(Path(tmp) / f"timing.{ext}")
                exporter(path, [], [], 10.0, small, big)
                loaded = importer(path)
                self.assertFalse(loaded.control_points_local_m)
                self.assertEqual((loaded.track_width_m, loaded.cone_spacing_m), (0.0, 0.0))
                self.assertFalse(loaded.left_cones_local_m)
                self.assertFalse(loaded.right_cones_local_m)
                small_local = np.asarray(loaded.orange_cones_local_m)
                big_local = np.asarray(loaded.orange_big_cones_local_m)
                self.assertEqual(small_local.shape, (2, 2))
                self.assertEqual(big_local.shape, (2, 2))
                # Translation is allowed during import, relative geometry stays exact.
                np.testing.assert_allclose(small_local[1] - small_local[0], [2.0, 0.0])
                np.testing.assert_allclose(big_local[1] - big_local[0], [6.0, 0.0])
                np.testing.assert_allclose(big_local[0] - small_local[0], [-2.0, 2.0])

    def test_sparse_boundary_plus_orange_remains_individually_editable(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "sparse.csv")
            export_manual_cones_csv(path, [(10, 10)], [(10, 50)], 10.0,
                                    orange_cones=[(10, 30)], orange_big_cones=[(0, 30)])
            loaded = load_track_csv(path)
        self.assertEqual(len(loaded.left_cones_local_m), 1)
        self.assertEqual(len(loaded.right_cones_local_m), 1)
        self.assertEqual(len(loaded.orange_cones_local_m), 1)
        self.assertEqual(len(loaded.orange_big_cones_local_m), 1)
        self.assertFalse(loaded.control_points_local_m)

    def test_invalid_orange_coordinate_or_scale_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "bad.csv")
            Path(path).write_text("tag,x,y\norange,NaN,1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite"):
                load_track_csv(path)
            with self.assertRaisesRegex(ValueError, "Scale"):
                export_manual_cones_csv(path, [], [], 0, [(10, 10)])
            Path(path).write_text("tag,x,y\norange_big,bad,1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-numeric"):
                load_track_csv(path)

    def test_json_schema_rejects_unknown_units_and_missing_cones(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.json"
            path.write_text(json.dumps({"format": "fs-trackdraw-cones-v1",
                                        "units": "px", "cones": []}))
            with self.assertRaisesRegex(ValueError, "meters"):
                load_track_json(str(path))
            path.write_text('{"units":"m"}')
            with self.assertRaisesRegex(ValueError, "cones array"):
                load_track_json(str(path))


if __name__ == "__main__":
    unittest.main()
