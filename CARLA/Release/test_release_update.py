"""Exercise the System Manager update check and installer handoff."""

import hashlib
from io import StringIO
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
import release_update as update
from asurt_update import add_ros_bags_to_edited_manager, record_release, select_release_repository
from types import SimpleNamespace


class ReleaseUpdateTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.state = self.root / "state"
        self.state.mkdir()

    def test_transition_install_records_production_for_future_downloads(self):
        args = SimpleNamespace(github_cli=True, platform="linux", with_wsl_runtime=False,
                               repository="HassanRadwannn/ASURTSIM", branch="old", catalog=None, manifest=None)
        manifest = {"repository": "asurt-fsai/Simulation-27", "tag": "v0.1.26-flat",
                    "runtime": {"linux": {"shipping_sha256": "linux-target"}}}
        select_release_repository(args, manifest)
        self.assertFalse(args.github_cli)
        record_release(self.state, args, manifest)
        installed = json.loads((self.state / update.RECEIPT).read_text())
        self.assertEqual(installed["repository"], "asurt-fsai/Simulation-27")
        self.assertEqual(installed["branch"], "main")
        self.assertIn("asurt-fsai/Simulation-27", update.installer_command(self.root, self.state, installed))

    def test_packaged_update_checker_uses_production_for_a_legacy_receipt(self):
        options = SimpleNamespace(github_cli=True, platform="linux", with_wsl_runtime=False,
                                  repository="hassanradwannn/ASURTSIM", branch="old", catalog=None)
        record_release(self.state, options, {"tag": "v0.1.24-flat", "runtime": {
            "linux": {"shipping_sha256": "linux-target"},
        }})

        urls = {
            "https://github.com/asurt-fsai/Simulation-27/releases/latest/download/asurt-release.json": {
                "format": 1, "tag": "v0.1.26-flat", "runtime": {
                    "linux": {"shipping_sha256": "linux-target"},
                }},
            "https://raw.githubusercontent.com/asurt-fsai/Simulation-27/main/asurt-maps.json": {
                "format": 1, "maps": [{"id": "optional", "label": "Optional",
                                       "base_sha256": {"LinuxNoEditor": "linux-target"}}]},
        }
        with patch("release_update.urlopen", side_effect=lambda url, **_: StringIO(json.dumps(urls[url]))), \
             patch("release_update.subprocess.run") as download, \
             patch("release_update.subprocess.check_output") as command:
            self.assertEqual(update.check_updates(self.state), ("v0.1.26-flat", ["Optional"]))
            download.assert_not_called()
            command.assert_not_called()

    def test_public_installer_records_receipt_and_offline_manifest_stays_offline(self):
        args = SimpleNamespace(github_cli=False, platform="linux", with_wsl_runtime=False,
                               repository="asurt-fsai/Simulation-27", branch="main", catalog=None,
                               manifest="https://github.com/asurt-fsai/Simulation-27/releases/latest/download/asurt-release.json")
        payload = {"tag": "v0.1.26-flat", "runtime": {"linux": {"shipping_sha256": "linux-target"}}}
        record_release(self.state, args, payload)
        self.assertEqual(update.receipt(self.state)["repository"], args.repository)
        args.manifest = str(self.root / "asurt-release.json")
        record_release(self.state, args, payload)
        self.assertIsNone(update.receipt(self.state))

    def test_newer_release_uses_target_compatible_maps(self):
        options = SimpleNamespace(github_cli=True, platform="windows", with_wsl_runtime=True,
                                  repository="owner/repo", branch="main", catalog=None)
        record_release(self.state, options, {"tag": "v0.1.9-flat", "runtime": {
            "windows": {"shipping_sha256": "windows-old"}, "linux": {"shipping_sha256": "linux-old"},
        }})
        manifest = {"format": 1, "tag": "v0.1.10-flat", "runtime": {
            "windows": {"shipping_sha256": "windows-new"}, "linux": {"shipping_sha256": "linux-new"},
        }}
        maps = {"format": 1, "maps": [
            {"id": "new-map", "label": "New map", "base_sha256": {"WindowsNoEditor": "windows-new"}},
            {"id": "linux-map", "label": "Linux map", "base_sha256": {"LinuxNoEditor": "linux-new"}},
            {"id": "old-map", "label": "Old map", "base_sha256": {"WindowsNoEditor": "windows-old"}},
        ]}

        def fetch(command, **_):
            Path(command[-1], "asurt-release.json").write_text(json.dumps(manifest))

        with patch("release_update.subprocess.run", side_effect=fetch), \
             patch("release_update.subprocess.check_output", return_value=json.dumps(maps)):
            self.assertEqual(update.check_updates(self.state), ("v0.1.10-flat", ["New map", "Linux map"]))

    def test_catalog_check_ignores_installed_and_custom_revisions(self):
        registry = self.state / "maps"
        registry.mkdir()
        previous = b"previous optional map"
        latest = b"latest optional map"
        (registry / "complete.zip").write_bytes(latest)
        (registry / "custom.zip").write_bytes(b"private map")
        (registry / "replace.zip").write_bytes(previous)
        maps = [{"id": ident, "label": ident, "base_sha256": {"LinuxNoEditor": "linux"},
                 "sha256": hashlib.sha256(latest).hexdigest(),
                 "replaces_sha256": [hashlib.sha256(previous).hexdigest()]}
                for ident in ("complete", "custom", "replace", "missing")]
        self.assertEqual(update.available_maps(maps, registry, {"LinuxNoEditor": "linux"}),
                         ["replace", "missing"])

    def test_unavailable_latest_release_still_checks_maps(self):
        options = SimpleNamespace(github_cli=True, platform="linux", with_wsl_runtime=False,
                                  repository="owner/repo", branch="main", catalog=None)
        record_release(self.state, options, {"tag": "v0.1.12-flat", "runtime": {
            "linux": {"shipping_sha256": "linux"},
        }})
        data = {"format": 1, "maps": [{"id": "new-map", "label": "New map",
                                      "base_sha256": {"LinuxNoEditor": "linux"}}]}
        with patch("release_update.subprocess.run", side_effect=subprocess.TimeoutExpired("gh", 15)), \
             patch("release_update.subprocess.check_output", return_value=json.dumps(data)):
            self.assertEqual(update.check_updates(self.state), (None, ["New map"]))

    def test_defaults_to_all_maps_on_both_operating_systems(self):
        config = {"platform": "linux", "repository": "owner/repo", "branch": "main"}
        args = update.installer_command(self.root, self.state, config)
        self.assertEqual(args[-2:], ["--maps", "all"])
        self.assertEqual(args[args.index("--root")+1], str(self.root))
        config["platform"] = "windows"
        with patch("release_update.subprocess.check_output", return_value="C:\\ASURT\\Install-ASURT.ps1\n"):
            args = update.installer_command(self.root, self.state, config)
        self.assertEqual(args[-2:], ["-Maps", "all"])
        self.assertIn("-Distribution", args)

    def test_declining_prompts_once_per_manager_launch(self):
        with patch.dict("os.environ", {"ASURT_STATE_ROOT": str(self.state)}), \
             patch("release_update.check_updates", return_value=(None, ["Optional map"])) as check, \
             patch("release_update.prompt", return_value=False) as prompt, \
             patch("release_update.subprocess.Popen") as spawn:
            manager = spawn.return_value.__enter__.return_value
            manager.poll.return_value = None
            manager.wait.return_value = 0
            self.assertEqual(update.run_manager(["manager"]), 0)
            check.assert_called_once()
            prompt.assert_called_once_with(None, ["Optional map"])
            manager.terminate.assert_not_called()
            self.assertEqual(update.run_manager(["manager"]), 0)
            self.assertEqual(prompt.call_count, 2)

    def test_accepting_closes_the_manager_after_terminal_launch(self):
        with patch.dict("os.environ", {"ASURT_STATE_ROOT": str(self.state)}), \
             patch("release_update.check_updates", return_value=("v0.1.13-flat", [])), \
             patch("release_update.prompt", return_value=True), \
             patch("release_update.subprocess.Popen") as spawn:
            manager = spawn.return_value.__enter__.return_value
            manager.poll.return_value = None
            self.assertEqual(update.run_manager(["manager"]), 0)
            manager.terminate.assert_called_once()

    def test_ros_bags_entry_preserves_edited_manager(self):
        manager = self.root / "control_center.py"
        original = (b'# local change\r\nTOOLS = {\r\n    "Map Packs": "map-packs",\r\n'
                    b'    "My importer": "track",\r\n}\r\n')
        stock = b'    "ROS Bags": "ros-bags",'
        manager.write_bytes(original)
        add_ros_bags_to_edited_manager(manager, stock)
        add_ros_bags_to_edited_manager(manager, stock)
        self.assertEqual(manager.read_bytes(), original.replace(b'    "My importer"', stock + b'\r\n    "My importer"'))
