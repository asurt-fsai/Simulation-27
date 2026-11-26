"""Offscreen Qt edit/IO tests; no simulator, ROS graph, or desktop is touched."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from PyQt5.QtCore import QPointF
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox

from trackdraw.exporter import export_track_csv
from trackdraw.project_io import load_project, save_project
from trackdraw.window import TrackDrawWindow


class OrangeEditorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_dir = os.getcwd()
        os.chdir(ROOT)
        cls.app = QApplication.instance() or QApplication([])

    @classmethod
    def tearDownClass(cls):
        os.chdir(cls.original_dir)

    def setUp(self):
        self.window = TrackDrawWindow()
        self.window.cone_button.click()
        self.assertEqual(self.window.edit_target, "cones")

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()

    def place(self, kind, position):
        self.window.add_mode_button.click()
        combo = self.window.cone_color_combo
        index = combo.findData(kind)
        self.assertGreaterEqual(index, 0)
        combo.setCurrentIndex(index)
        self.window._handle_cone_click(position)

    def test_small_and_big_orange_are_independently_placed_moved_and_removed(self):
        w = self.window
        self.place("orange", (150., 160.))
        self.place("orange_big", (440., 170.))
        self.assertEqual(w._editable_cone_array("orange").tolist(), [[150., 160.]])
        self.assertEqual(w._editable_cone_array("orange_big").tolist(), [[440., 170.]])
        self.assertEqual(w.canvas.orange_cones.tolist(), [[150., 160.]])
        self.assertEqual(w.canvas.orange_big_cones.tolist(), [[440., 170.]])
        self.assertIn("orange 1, orange_big 1", w.cone_count_label.text())

        w.move_mode_button.click()
        w._handle_cone_click((440., 170.))
        self.assertEqual((w.dragging_kind, w.dragging_cone_side), ("cone", "orange_big"))
        w.handle_canvas_drag(QPointF(460., 180.))
        w.handle_canvas_release(QPointF(460., 180.))
        self.assertEqual(w._editable_cone_array("orange_big").tolist(), [[460., 180.]])

        w.remove_mode_button.click()
        w._handle_cone_click((150., 160.))
        self.assertEqual(len(w._editable_cone_array("orange")), 0)
        self.assertEqual(len(w._editable_cone_array("orange_big")), 1)

    def test_json_ui_roundtrip_and_project_yaml_preserve_orange_types(self):
        w = self.window
        self.place("orange", (120., 130.))
        self.place("orange_big", (180., 150.))
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "cones.json")
            with patch.object(QFileDialog, "getSaveFileName", return_value=(path, "")), \
                 patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
                w._export_json()
            self.assertTrue(Path(path).is_file())
            with patch.object(QFileDialog, "getOpenFileName", return_value=(path, "")), \
                 patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
                w._load_track_json()
            overlay = w.project_state.imported_track
            self.assertFalse(overlay.control_points_local_m)
            self.assertFalse(overlay.preserve_cones)
            self.assertEqual(w.edit_target, "cones")
            self.assertEqual(len(w._editable_cone_array("orange")), 1)
            self.assertEqual(len(w._editable_cone_array("orange_big")), 1)
            self.assertIn("no centerline reconstructed", w.imported_track_state_label.text())

            w.move_mode_button.click()
            original = w._editable_cone_array("orange_big")[0]
            w._handle_cone_click(tuple(original))
            w.handle_canvas_drag(QPointF(float(original[0]) + 25., float(original[1]) + 13.))
            w.handle_canvas_release(QPointF(0., 0.))
            self.assertEqual(len(overlay.orange_big_cones_local_m), 1)
            project = str(Path(tmp) / "cones.trackdraw.yaml")
            save_project(project, w.project_state)
            reloaded = load_project(project)
            self.assertEqual(reloaded.imported_track.orange_big_cones_local_m,
                             overlay.orange_big_cones_local_m)
            self.assertEqual(len(reloaded.imported_track.orange_cones_local_m), 1)

    def test_locked_imported_blue_yellow_stay_protected_while_orange_can_edit(self):
        w = self.window
        blue = [(110., 120.), (160., 120.), (160., 170.), (110., 170.)]
        yellow = [(210., 120.), (260., 120.), (260., 170.), (210., 170.)]
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "track.csv")
            export_track_csv(
                path, [(100., 100.), (120., 100.)],
                blue, yellow, 10.,
                orange_cones=[(190., 200.)],
                orange_big_cones=[(190., 90.)],
            )
            with patch.object(QFileDialog, "getOpenFileName", return_value=(path, "")), \
                 patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
                w._load_track_csv()
            self.assertTrue(w.project_state.imported_track.preserve_cones)
            self.assertTrue(w.cone_button.isEnabled())
            w.cone_button.click()
            before = w._editable_cone_array("left").copy()
            self.place("left", (200., 300.))
            self.assertEqual(w._editable_cone_array("left").tolist(), before.tolist())
            self.place("orange", (200., 300.))
            self.assertEqual(len(w._editable_cone_array("orange")), 2)
            # Whole-track rotation transforms orange positions, and marker
            # dragging writes the inverse transform into the imported overlay.
            w.project_state.imported_track.rotation_deg = 37.
            w._rebuild_track()
            first = w._editable_cone_array("orange_big")[0].copy()
            w.move_mode_button.click()
            w._handle_cone_click(tuple(first))
            w.handle_canvas_drag(QPointF(float(first[0]) + 12., float(first[1]) - 16.))
            w.handle_canvas_release(QPointF(0., 0.))
            actual = w._editable_cone_array("orange_big")[0]
            self.assertAlmostEqual(actual[0], first[0] + 12., places=6)
            self.assertAlmostEqual(actual[1], first[1] - 16., places=6)


if __name__ == "__main__":
    unittest.main()
