"""Exercise the System Manager update check and installer handoff."""

import hashlib
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
import release_update as update
import asurt_update
from asurt_update import add_ros_bags_to_edited_manager, record_release, select_release_repository
from types import SimpleNamespace


class ReleaseUpdateTest(unittest.TestCase):
    def test_runtime_cleanup_preserves_local_content_and_removes_only_verified_copies(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            (source / "FlatAsphalt.umap").write_bytes(b"shipped map")
            archive = root / "runtime.tar.gz"
            with tarfile.open(archive, "w:gz") as package:
                package.add(source / "FlatAsphalt.umap", arcname="FlatAsphalt.umap")
            checksum = asurt_update.digest(archive)
            cache = root / "cache"
            cache.mkdir()
            cached = cache / f"{checksum}-{archive.name}"
            archive.rename(cached)
            active = root / "runtimes" / ("f" * 64)
            active.mkdir(parents=True)
            old = active.with_name(checksum)
            asurt_update.extract(cached, old)
            custom = old / "LocalTrack.umap"
            custom.touch()
            asurt_update.prune_superseded_runtimes(cache, active, {"file": archive.name})
            self.assertTrue(custom.is_file())
            self.assertTrue(cached.is_file())
            custom.unlink()
            asurt_update.prune_superseded_runtimes(cache, active, {"file": archive.name})
            self.assertFalse(old.exists())
            self.assertFalse(cached.exists())
            self.assertTrue(active.is_dir())

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

    def test_testing_uses_the_highest_published_testing_build(self):
        index = [
            dict(tag_name=tag, prerelease=preview, draft=draft,
                 assets=[{"name": "asurt-release.json"}])
            for tag, preview, draft in (("v0.1.27-flat", False, False), ("testing-2", True, False),
                                        ("testing-10", True, False), ("testing-11", True, True))
        ]
        stable = dict(tag_name="v0.1.27-flat", prerelease=False, draft=False,
                      assets=[{"name": "asurt-release.json"}])
        urls = {
            "https://api.github.com/repos/asurt-fsai/Simulation-27/releases?per_page=100&page=1": [stable] * 100,
            "https://api.github.com/repos/asurt-fsai/Simulation-27/releases?per_page=100&page=2": index,
            "https://github.com/asurt-fsai/Simulation-27/releases/download/testing-10/asurt-release.json": {
                "format": 1, "channel": "testing", "tag": "testing-10",
                "runtime": {"linux": {"shipping_sha256": "linux-target"}}},
            "https://raw.githubusercontent.com/asurt-fsai/Simulation-27/testing/asurt-maps.json": {
                "format": 1, "maps": []},
        }
        (self.state / update.RECEIPT).write_text(json.dumps({
            "repository": "asurt-fsai/Simulation-27", "branch": "main", "tag": "v0.1.27-flat",
            "base_sha256": {"LinuxNoEditor": "linux-target"},
        }))
        update.select_branch(self.state, "testing")
        with patch("release_update.urlopen", side_effect=lambda url, **_: StringIO(json.dumps(urls[url]))):
            self.assertEqual(update.check_updates(self.state), ("testing-10", []))
        bootstrap = (Path(__file__).resolve().parents[2] / "Install-ASURT.sh").read_text()
        lookup = bootstrap.split("testing_tag=$(python3 - \"$repository\" <<'PY'\n", 1)[1].split("\nPY\n", 1)[0]
        output = StringIO()
        with patch("urllib.request.urlopen", side_effect=lambda url, **_: StringIO(json.dumps(urls[url]))), \
             patch.object(sys, "argv", ["installer", "asurt-fsai/Simulation-27"]), redirect_stdout(output):
            exec(compile(lookup, "Install-ASURT.sh", "exec"), {})
        self.assertEqual(output.getvalue().strip(), "testing-10")

    def test_channel_defaults_to_the_version_family_in_older_receipts(self):
        installed = {"repository": "asurt-fsai/Simulation-27", "tag": "testing-2"}
        self.assertEqual(update.selected_branch(self.state, installed), "testing")
        update.select_branch(self.state, "main")
        self.assertEqual(update.selected_branch(self.state, installed), "main")
        self.assertTrue(update.is_update(installed, {"format": 1, "tag": "v0.1.39-flat"}, "main"))

    def test_release_check_retries_a_temporary_connection_failure(self):
        stable = {"format": 1, "tag": "v0.1.39-flat", "channel": "main"}
        temporary = URLError(TimeoutError("GitHub connection timed out"))
        with patch("release_update.urlopen", side_effect=[temporary, StringIO(json.dumps(stable))]) as request, \
             patch("release_update.sleep"):
            self.assertEqual(update.public_manifest("asurt-fsai/Simulation-27", "main"), stable)
            self.assertEqual(request.call_count, 2)

    def test_permanent_missing_release_is_reported_immediately(self):
        missing = HTTPError("https://example.com/release", 404, "Missing", {}, None)
        with patch("release_update.urlopen", side_effect=missing) as request:
            with self.assertRaises(HTTPError):
                update.public_manifest("asurt-fsai/Simulation-27", "main")
            request.assert_called_once()

    def test_switching_can_install_the_same_stable_release_again(self):
        stable = {"format": 1, "tag": "v0.1.27-flat"}
        self.assertTrue(update.is_update({"tag": "testing-14"}, stable, "main"))
        self.assertFalse(update.is_update({"tag": "v0.1.27-flat"}, stable, "main"))
        self.assertFalse(update.is_update({"tag": "testing-14"}, {"format": 1, "tag": "testing-8"}, "testing"))
        self.assertTrue(update.is_update({"tag": "testing-8"}, {"format": 1, "tag": "testing-14"}, "testing"))

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
        with patch("release_update.subprocess.check_output", return_value="C:\\ASURT\\Install-ASURT.ps1\n"), \
             patch("release_update.powershell_exe", return_value="powershell.exe"):
            args = update.installer_command(self.root, self.state, config)
            optional = update.installer_command(self.root, self.state, config, add_wslg=True)
        self.assertEqual(args[-3:], ["-Maps", "all", "-NativeOnly"])
        self.assertEqual(optional[-3:], ["-Maps", "all", "-WithWslg"])
        self.assertIn("-Distribution", optional)

    def test_powershell_can_be_resolved_without_windows_path_entries(self):
        executable = self.root / "powershell.exe"
        executable.touch()
        with patch("windows_interop.shutil.which", return_value=None), \
             patch("windows_interop.subprocess.check_output", return_value=f"{executable}\n"):
            self.assertEqual(update.powershell_exe(), str(executable))

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

    def test_ubuntu_terminal_acknowledgement_uses_clean_desktop_environment(self):
        def opened(command, **options):
            self.assertEqual(command[:5], ["/usr/bin/gnome-terminal", "--wait", "--window", "--title=ASURT Update", "--"])
            self.assertEqual(command[-3:-1], ["fake-installer", "--terminal-token"])
            self.assertNotIn("LD_LIBRARY_PATH", options["env"])
            self.assertEqual(options["env"]["DISPLAY"], ":0")
            self.assertIn(f"ASURT_STATE_ROOT={self.state}", command)
            token = Path(command[-1])
            self.assertEqual(token.parent.parent, self.state / "log/carla/tools")
            token.rename(token.with_name("started"))
            return SimpleNamespace(poll=lambda: 0)

        with patch.dict("os.environ", {"DISPLAY": ":0", "LD_LIBRARY_PATH": "/asurt/ros/lib"}), \
             patch("release_update.shutil.which", side_effect=lambda name: "/usr/bin/" + name), \
             patch("release_update.subprocess.Popen", side_effect=opened) as spawn:
            update.linux_terminal(["fake-installer"], self.state)
        spawn.assert_called_once()

    def test_terminal_exit_zero_without_a_worker_tries_fallback_before_reporting_success(self):
        tokens = []

        def opened(command, **_):
            token = Path(command[-1])
            tokens.append(token)
            if len(tokens) == 2:
                self.assertFalse(tokens[0].exists())
                self.assertEqual(command[:4], ["/usr/bin/xterm", "-T", "ASURT Update", "-e"])
                token.rename(token.with_name("started"))
            return SimpleNamespace(poll=lambda: 0)

        with patch("release_update.shutil.which", side_effect=lambda name: "/usr/bin/" + name), \
             patch("release_update.subprocess.Popen", side_effect=opened), \
             patch("release_update.monotonic", side_effect=iter(range(0, 90, 21))):
            update.linux_terminal(["fake-installer"], self.state)
        self.assertEqual(len(tokens), 2)

    def test_timed_out_interactive_worker_cannot_stop_simulator(self):
        stop = SimpleNamespace(stop_for_update=lambda: self.fail("The cancelled worker stopped CARLA"))
        config = {"ASURT_ROOT": str(self.root), "ASURT_STATE_ROOT": str(self.state)}
        missing = self.root / "expired" / "pending"
        with patch.dict("os.environ", config), patch.dict(sys.modules, {"session": stop}), \
             patch.object(sys, "argv", ["release_update.py", "--terminal-token", str(missing)]), \
             patch.object(sys.stdin, "isatty", return_value=True), \
             patch.object(sys.stdout, "isatty", return_value=True), \
             patch("release_update.subprocess.call") as installer:
            self.assertEqual(update.main(), 1)
        installer.assert_not_called()

    def test_update_reopens_the_current_manager_after_success(self):
        active = self.root / "installations/old"
        env = {"ASURT_ROOT": str(active), "ASURT_STATE_ROOT": str(self.state)}
        for code in (0, 1):
            with self.subTest(code=code), patch.dict("os.environ", env), \
                 patch.dict(sys.modules, {"session": SimpleNamespace(stop_for_update=lambda: None)}), \
                 patch.object(sys, "argv", ["release_update.py", "--reopen-manager", "--channel", "testing"]), \
                 patch("release_update.receipt", return_value={"platform": "linux"}), \
                 patch("release_update.installer_command", return_value=["install"]), \
                 patch("release_update.subprocess.call", return_value=code), \
                 patch("release_update.subprocess.Popen") as reopen:
                self.assertEqual(update.main(), code)
                if code:
                    reopen.assert_not_called()
                else:
                    self.assertEqual(reopen.call_args.args[0],
                                     ["/bin/bash", self.root / "current/bin/asurt", "manager"])
                    self.assertNotIn("ASURT_ROOT", reopen.call_args.kwargs["env"])

    def test_ros_bags_entry_preserves_edited_manager(self):
        manager = self.root / "control_center.py"
        original = (b'# local change\r\nTOOLS = {\r\n    "Map Packs": "map-packs",\r\n'
                    b'    "My importer": "track",\r\n}\r\n')
        stock = b'    "ROS Bags": "ros-bags",'
        manager.write_bytes(original)
        add_ros_bags_to_edited_manager(manager, stock)
        add_ros_bags_to_edited_manager(manager, stock)
        self.assertEqual(manager.read_bytes(), original.replace(b'    "My importer"', stock + b'\r\n    "My importer"'))
