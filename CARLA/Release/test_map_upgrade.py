"""Exercise compatible updates against an installed two-file map fixture."""

import argparse
from contextlib import redirect_stdout
import hashlib
from io import StringIO
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from asurt_update import fetch, update_maps

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
from map_packs import CONTENT, PLATFORMS, catalog, file_digest, install, register


class MapUpgradeTest(unittest.TestCase):
    platform = "LinuxNoEditor"
    area = "ASURTMaps/silverstone_section"
    level = f"{area}/SilverStoneSection.umap"
    road = f"{area}/OpenDrive/SilverStoneSection.xodr"

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.runtime = self.root / "runtime"
        exe = self.runtime / PLATFORMS[self.platform]
        exe.parent.mkdir(parents=True)
        exe.write_bytes(b"test CARLA executable")
        self.base_sha = file_digest(exe)
        (self.runtime / CONTENT).mkdir(parents=True)
        self.registry = self.root / "state/maps"
        self.old = self.make_pack("old", b"old terrain")
        self.new = self.make_pack("new", b"new terrain")
        definition = register(self.old, self.registry)
        install(definition, self.platform, self.runtime)
        self.installed = self.runtime / CONTENT / self.level

    def make_pack(self, directory, map_data):
        path = self.root / directory / "silverstone-section.zip"
        path.parent.mkdir()
        assets = {self.level: map_data, self.road: b"test road"}
        placement = {"x": 74, "y": -847, "z": 1, "yaw": -6}
        manifest = {
            "format": 1, "id": "silverstone-section", "label": "Silverstone Split",
            "map": f"/Game/{self.area}/SilverStoneSection", "spawn": placement,
            "course_origin": placement,
            "platforms": {self.platform: {
                "base_sha256": self.base_sha,
                "files": {name: hashlib.sha256(data).hexdigest() for name, data in assets.items()},
                "shared": {},
            }},
        }
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("manifest.json", json.dumps(manifest))
            for name, data in assets.items():
                archive.writestr(f"{self.platform}/{name}", data)
        return path

    def update(self, mode):
        catalog_path = self.new.parent / "asurt-maps.json"
        entry = {
            "id": "silverstone-section", "label": "Silverstone Split", "file": self.new.name,
            "sha256": file_digest(self.new), "bytes": self.new.stat().st_size, "tag": "maps-v2",
            "base_sha256": {self.platform: self.base_sha},
            "replaces_sha256": [file_digest(self.old)],
        }
        catalog_path.write_text(json.dumps({"format": 1, "maps": [entry]}))
        args = argparse.Namespace(manifest=str(self.new.parent / "asurt-release.json"),
                                  catalog=str(catalog_path), platform="linux", github_cli=False,
                                  tag="stack-v2", maps=mode)
        update_maps(args, self.root, self.runtime)
        return catalog(self.registry)["silverstone-section"]

    def test_default_update_replaces_published_revision(self):
        definition = self.update("ask")
        self.assertEqual(file_digest(definition.archive), file_digest(self.new))
        self.assertEqual(self.installed.read_bytes(), b"old terrain")
        install(definition, self.platform, self.runtime)
        install(definition, self.platform, self.runtime)
        self.assertEqual(self.installed.read_bytes(), b"new terrain")

    def test_skip_maps_leaves_existing_revision(self):
        definition = self.update("none")
        self.assertEqual(file_digest(definition.archive), file_digest(self.old))

    def test_verified_installed_map_does_not_download_again(self):
        self.update("all")
        log = StringIO()
        with patch("asurt_update.fetch") as download, redirect_stdout(log):
            self.update("all")
        download.assert_not_called()
        self.assertIn("already installed and verified", log.getvalue())

    def test_download_progress_and_verified_cache(self):
        asset = {"file": self.new.name, "sha256": file_digest(self.new), "bytes": self.new.stat().st_size}
        args = argparse.Namespace(manifest=str(self.new.parent / "asurt-release.json"), github_cli=False)
        log = StringIO()
        with redirect_stdout(log):
            target = fetch(asset, args, self.root / "cache")
        self.assertIn("100%", log.getvalue())
        self.assertEqual(file_digest(target), asset["sha256"])
        with patch("asurt_update.open_source", side_effect=AssertionError("Unexpected network or disk read")):
            self.assertEqual(fetch(asset, args, self.root / "cache"), target)

    def test_uses_matching_downloads_zip_and_skips_wrong_copies(self):
        work, downloads = self.root / "cwd", self.root / "Downloads"
        work.mkdir()
        downloads.mkdir()
        shutil.copyfile(self.old, work / self.old.name)
        candidate = downloads / self.new.name
        shutil.copyfile(self.new, candidate)
        asset = {"file": self.new.name, "sha256": file_digest(self.new)}
        args = argparse.Namespace(manifest=None, github_cli=True, repository="owner/repo", tag="maps-v2")
        with patch("asurt_update.subprocess.Popen", side_effect=AssertionError("Unexpected download")):
            self.assertEqual(fetch(asset, args, self.root / "cache", search_dirs=(work, downloads)), candidate)

    def test_github_download_reports_in_flight_progress(self):
        archive = self.new.read_bytes()
        asset = {"file": self.new.name, "sha256": file_digest(self.new), "bytes": len(archive)}
        args = argparse.Namespace(manifest=None, github_cli=True, repository="owner/repo", tag="maps-v2")
        log = StringIO()
        with patch("asurt_update.subprocess.Popen") as spawn, redirect_stdout(log):
            download = spawn.return_value.__enter__.return_value
            download.poll.return_value = 0

            def wait(timeout):
                command = spawn.call_args.args[0]
                partial = Path(command[command.index("--dir") + 1]) / self.new.name
                if not partial.exists():
                    partial.write_bytes(archive[:len(archive) // 2])
                    raise subprocess.TimeoutExpired(command, timeout)
                partial.write_bytes(archive)
                return 0

            download.wait.side_effect = wait
            target = fetch(asset, args, self.root / "cache")
        self.assertEqual(target.read_bytes(), archive)
        self.assertRegex(log.getvalue(), r"(49|50)%")
        self.assertIn("100%", log.getvalue())

    def test_install_leaves_locally_edited_cooked_files(self):
        self.installed.write_bytes(b"local edit")
        definition = self.update("ask")
        with self.assertRaisesRegex(ValueError, "differs from a known revision"):
            install(definition, self.platform, self.runtime)
        self.assertEqual(self.installed.read_bytes(), b"local edit")
