"""Regression check for selected local builds and custom modules across releases."""

import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from asurt_update import activate, enable_dynamic_prop_sensor_trace, seed_workspace, stack_with_patch


class WorkspaceUpgradeTest(unittest.TestCase):
    def test_patch_removes_retired_stock_from_the_new_stack_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            base, patch = root / "base", root / "patch"
            package = Path("Simulation/Systems/Control/Modules/Stanley/asurt_stanley_control")
            (base / package).mkdir(parents=True)
            (base / package / "node.py").write_text("old stock")
            patch.mkdir()
            (patch / "retired-modules.json").write_text(json.dumps([package.as_posix()]))
            active_stack = stack_with_patch(base, patch)
            self.assertFalse((active_stack / package).exists())
            self.assertEqual((base / package / "node.py").read_text(), "old stock")

    def test_retirement_removes_stock_module_and_preserves_edits_and_local_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "config"
            modules = Path("Simulation/Systems/Control/Modules")
            old, new = root / "old", root / "new"
            retired = [modules / "Pursuit" / name for name in ("stock", "edited", "selected")]
            installs = [root / "first", root / "second"]
            for installation in installs:
                (installation / modules.parent).mkdir(parents=True)
            for package in retired:
                (old / package).mkdir(parents=True)
                (old / package / "node.py").write_text("stock")
            successor = new / modules / "Legacy" / "pursuit"
            successor.mkdir(parents=True)
            (successor / "node.py").write_text("successor")
            (new / "retired-modules.json").write_text(json.dumps([path.as_posix() for path in retired]))
            with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(config)}):
                seed_workspace(root, old, installs[0])
                workspace = root / "workspace"
                (workspace / retired[1] / "node.py").write_text("local edit")
                selection = config / "asurt/module-sources.json"
                snapshot = json.dumps({"team_control": {"binary": str(workspace / retired[2] / "node.py")}})
                selection.write_text(snapshot)
                seed_workspace(root, new, installs[1])
                self.assertFalse((workspace / retired[0]).exists())
                self.assertEqual((workspace / retired[1] / "node.py").read_text(), "local edit")
                self.assertEqual((workspace / retired[2] / "node.py").read_text(), "stock")
                self.assertEqual(selection.read_text(), snapshot)
                self.assertEqual((installs[1] / successor.relative_to(new) / "node.py").read_text(), "successor")

    def test_packaged_manager_updates_even_with_an_edited_workspace_copy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            entrypoint = Path("Control Center/control_center.py")
            with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(root / "config")}):
                for name in ("first", "second"):
                    stack = root / "stacks" / name
                    path = stack / "tools" / entrypoint
                    path.parent.mkdir(parents=True)
                    path.write_text(name)
                    install = root / "installations" / name
                    install.mkdir(parents=True)
                    seed_workspace(root, stack, install)
                    activate(install, root / "current")
                    if name == "first":
                        (root / "workspace/tools" / entrypoint).write_text("local manager edits")
                current = root / "current"
                self.assertEqual((current / "libexec/asurt-stock-tools" / entrypoint).read_text(), "second")
                self.assertEqual((current / "tools" / entrypoint).read_text(), "local manager edits")

    def test_dynamic_props_keep_other_collision_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ini = root / "CarlaUE4/Config/DefaultEngine.ini"
            ini.parent.mkdir(parents=True)
            original = (b"[/Script/Engine.CollisionProfile]\r\n"
                        b'+EditProfiles=(Name="PhysicsActor",CustomResponses=((Channel="Visibility",Response=ECR_Ignore),'
                        b'(Channel="SensorTrace",Response=ECR_Ignore)))\r\n'
                        b"\r\n[OtherSettings]\r\nQuality=Low\r\n")
            ini.write_bytes(original)
            enable_dynamic_prop_sensor_trace(root)
            enable_dynamic_prop_sensor_trace(root)
            self.assertEqual(ini.read_bytes(), original.replace(b'"SensorTrace",Response=ECR_Ignore',
                                                               b'"SensorTrace",Response=ECR_Block'))

    def test_upgrade_preserves_local_selection_source_and_build(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(root / "config")}):
                original = root / "original"
                update = root / "update"
                relative = Path("tools/local_module.py")
                for stack, source in ((original, "published v1\n"), (update, "published v2\n")):
                    (stack / relative.parent).mkdir(parents=True)
                    (stack / relative).write_text(source)
                installed = root / "first"
                installed.mkdir()
                seed_workspace(root, original, installed)
                activate(installed, root / "current")

                workspace = root / "workspace"
                (workspace / relative).write_text("my edited module\n")
                local_executable = workspace / "build/velodyne/install/node"
                local_executable.parent.mkdir(parents=True)
                local_executable.write_text("my local build\n")
                skidpad = root / "team-skidpad"
                skidpad.mkdir()
                skidpad_source = skidpad / "test_node.py"
                skidpad_source.write_text("my skidpad source\n")
                skidpad_executable = workspace / "build/skidpad/install/node"
                skidpad_executable.parent.mkdir(parents=True)
                skidpad_executable.write_text("my skidpad build\n")
                settings = root / "config/asurt"
                selection = json.dumps({
                    "velodyne": {"binary": str(local_executable)},
                    "skidpad_test": {"binary": str(skidpad_executable), "label": "Skidpad Test",
                                     "group": "Planning", "topic": "/planning/skidpad_test/status"},
                }) + "\n"
                switches = '{"version":1,"enabled":{"velodyne":false,"skidpad_test":true}}\n'
                (settings / "module-sources.json").write_text(selection)
                (settings / "modules.json").write_text(switches)

                installed_update = root / "second"
                installed_update.mkdir()
                seed_workspace(root, update, installed_update)
                activate(installed_update, root / "current")
                self.assertEqual((settings / "module-sources.json").read_text(), selection)
                self.assertEqual((settings / "modules.json").read_text(), switches)
                self.assertEqual(local_executable.read_text(), "my local build\n")
                self.assertEqual(skidpad_executable.read_text(), "my skidpad build\n")
                self.assertEqual(skidpad_source.read_text(), "my skidpad source\n")
                self.assertEqual((root / "current" / relative).read_text(), "my edited module\n")
                self.assertEqual((workspace / ".asurt-updates" / relative).read_text(),
                                 "published v2\n")
