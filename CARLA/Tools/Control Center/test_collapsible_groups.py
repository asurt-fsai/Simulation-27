"""Tk UI contract: disclosure controls never change System Manager switches."""
import importlib.util
import json
from pathlib import Path
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("asurt_control_center_ui", HERE / "control_center.py")
UI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UI)


def module(ident, group, running=True):
    return {
        "id": ident, "label": "Test " + ident, "group": group,
        "topic": "/test/" + ident, "enabled": running,
        "status": "running" if running else "disabled",
        "memory_mib": 16.0 if running else 0.0,
        "pids": [2000] if running else [],
    }


class CollapsibleModuleTests(unittest.TestCase):
    def setUp(self):
        self.storage = tempfile.TemporaryDirectory()
        self.path = Path(self.storage.name) / "view.json"
        self.state = {
            "carla_listening": True,
            "modules": [module("zed2i", "Perception"),
                        module("converter", "Perception"),
                        module("slam_lidar", "SLAM", False)],
        }
        self.file_override = patch.object(UI, "VIEW_STATE", self.path)
        self.snapshot_override = patch.object(UI.module_control, "live_states",
                                              return_value=self.state)
        self.file_override.start()
        self.snapshot_override.start()
        self.root = tk.Tk()
        self.root.withdraw()
        self.addCleanup(self.cleanup_window)
        self.addCleanup(self.snapshot_override.stop)
        self.addCleanup(self.file_override.stop)
        self.addCleanup(self.storage.cleanup)
        self.center = UI.ControlCenter(self.root)

    def cleanup_window(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def test_tools_show_one_rviz_preset_menu_and_keep_simulator_tools(self):
        def descendants(widget):
            for child in widget.winfo_children():
                yield child
                yield from descendants(child)

        choices = [widget for widget in descendants(self.root) if isinstance(widget, tk.Menubutton)]
        self.assertEqual([widget.cget("text") for widget in choices], ["RViz Preset"])
        menu = choices[0].nametowidget(choices[0].cget("menu"))
        views = [menu.entrycget(index, "label") for index in range(menu.index("end") + 1)]
        self.assertEqual(views, ["Perception", "SLAM", "Planning", "Control"])
        with patch.object(self.center, "launch_tool") as launch:
            menu.invoke(2)
            self.center.tool_buttons["Map Switcher"].invoke()
            self.assertEqual([call.args for call in launch.call_args_list], [("Planning",), ("Map Switcher",)])
        self.assertEqual(set(self.center.tool_buttons), {"Quit CARLA", "Cancel Goal", "Map Switcher",
                                                        "ROS Bags", "Track Importer", "Weather Editor", "Trackdraw"})

    def test_group_disclosure_and_independent_switches(self):
        ui = self.center
        self.assertEqual(ui.group_summaries["Perception"].cget("text"), "2/2 running")
        self.assertEqual(ui.group_summaries["SLAM"].cget("text"), "0/1 running")
        self.assertEqual(ui.group_bodies["Perception"].winfo_manager(), "pack")
        self.assertEqual(int(ui.group_headers["Perception"].cget("highlightthickness")), 0)
        self.assertEqual(int(ui.module_headers["zed2i"][0].cget("highlightthickness")), 0)
        with patch.object(ui, "task") as actions:
            ui.group_headers["Perception"].invoke()
            self.assertEqual(ui.group_bodies["Perception"].winfo_manager(), "")
            self.assertIn("▸", ui.group_headers["Perception"].cget("text"))
            self.assertEqual(ui.group_action_buttons["Perception"]["All On"].winfo_manager(),
                             "pack")
            self.assertEqual(ui.group_action_buttons["Perception"]["All Off"].winfo_manager(),
                             "pack")
            self.assertEqual(ui.group_action_buttons["Perception"]["All On"].master.winfo_manager(),
                             "pack")
            # Collapsing one group does not hide other groups.
            self.assertEqual(ui.group_bodies["SLAM"].winfo_manager(), "pack")
            ui.group_headers["Perception"].invoke()
            self.assertEqual(ui.group_bodies["Perception"].winfo_manager(), "pack")
            ui.module_headers["zed2i"][0].invoke()
            self.assertEqual(ui.module_details["zed2i"].winfo_manager(), "")
            # The real module On/Off button stays present and unchanged.
            self.assertEqual(ui.cards["zed2i"][0].cget("text"), "Turn Off")
            actions.assert_not_called()
        self.assertEqual(json.loads(self.path.read_text())["modules"]["zed2i"], False)

    def test_group_on_off_still_actionable_while_collapsed(self):
        ui = self.center
        ui.toggle_section("Perception")
        self.assertEqual(ui.group_bodies["Perception"].winfo_manager(), "")
        with patch.object(ui, "toggle_group") as toggle:
            ui.group_action_buttons["Perception"]["All On"].invoke()
            ui.group_action_buttons["Perception"]["All Off"].invoke()
            self.assertEqual(toggle.call_count, 2)
            self.assertEqual(toggle.call_args_list[0].args, ("Perception", True))
            self.assertEqual(toggle.call_args_list[1].args, ("Perception", False))
        self.assertEqual([m["enabled"] for m in ui.snapshot["modules"]],
                         [True, True, False])

    def test_state_persists_and_registry_rebuild_keeps_disclosure(self):
        ui = self.center
        ui.toggle_section("SLAM")
        ui.toggle_details("converter")
        self.assertFalse(ui.group_expanded["SLAM"])
        self.assertFalse(ui.module_expanded["converter"])
        self.assertEqual(ui.load_view_state()["groups"]["SLAM"], False)
        new_modules = self.state["modules"] + [module("lidar", "Perception")]
        ui.populate(new_modules)
        self.assertEqual(ui.group_bodies["SLAM"].winfo_manager(), "")
        self.assertEqual(ui.module_details["converter"].winfo_manager(), "")
        self.assertEqual(ui.module_details["lidar"].winfo_manager(), "pack")
        # A new ControlCenter loads preferences without calling module switches.
        self.assertFalse(UI.ControlCenter.load_view_state()["groups"]["SLAM"])

    def test_collapse_all_only_changes_visible_bodies(self):
        ui = self.center
        original_states = [m["enabled"] for m in ui.snapshot["modules"]]
        with patch.object(ui, "task") as actions:
            ui.set_all_sections(False)
            self.assertTrue(all(body.winfo_manager() == ""
                                for body in ui.group_bodies.values()))
            ui.set_all_sections(True)
            self.assertTrue(all(body.winfo_manager() == "pack"
                                for body in ui.group_bodies.values()))
            actions.assert_not_called()
        self.assertEqual(original_states,
                         [m["enabled"] for m in ui.snapshot["modules"]])

    def test_invalid_preferences_fallback_without_changing_module_state(self):
        self.path.write_text('{"groups":{"SLAM":"yes"}}')
        self.assertEqual(UI.ControlCenter.load_view_state(), {"groups": {}, "modules": {}})
        self.assertTrue(self.center.snapshot["modules"][0]["enabled"])


if __name__ == "__main__":
    unittest.main()
