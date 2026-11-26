"""Regression check for selected local builds and custom modules across releases."""

import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from asurt_update import activate, enable_dynamic_prop_sensor_trace, seed_workspace


class WorkspaceUpgradeTest(unittest.TestCase):
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
