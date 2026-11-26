"""Test dedicated Vulkan selection without depending on the build host's GPUs."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
from native_vulkan import native_vulkan_environment


class NativeVulkanTest(unittest.TestCase):
    def setUp(self):
        workspace = tempfile.TemporaryDirectory()
        self.addCleanup(workspace.cleanup)
        self.drivers = Path(workspace.name)
        directories = patch("native_vulkan.DRIVER_DIRS", (self.drivers,))
        directories.start()
        self.addCleanup(directories.stop)

    @patch("native_vulkan.subprocess.run")
    def test_selects_verified_nvidia_driver_for_car_server_only(self, run):
        nvidia = self.drivers / "nvidia_icd.json"
        nvidia.touch()
        run.return_value.stdout = json.dumps([{"type": 2, "name": "GeForce GTX 1050"}])
        environment = {"DISPLAY": ":0"}
        chosen, renderer = native_vulkan_environment(environment)
        self.assertEqual(chosen, {"DISPLAY": ":0", "VK_ICD_FILENAMES": str(nvidia)})
        self.assertEqual(renderer, "GeForce GTX 1050")
        self.assertEqual(environment, {"DISPLAY": ":0"})

    @patch("native_vulkan.subprocess.run")
    def test_respects_explicit_renderer(self, run):
        environment = {"VK_DRIVER_FILES": "/custom/driver.json"}
        self.assertEqual(native_vulkan_environment(environment),
                         (environment, "explicit renderer settings"))
        run.assert_not_called()

    @patch("native_vulkan.subprocess.run")
    def test_tries_another_dedicated_driver_after_a_failed_probe(self, run):
        (self.drivers / "nvidia_icd.json").touch()
        amd = self.drivers / "radeon_icd.x86_64.json"
        amd.touch()
        run.side_effect = [subprocess.CalledProcessError(1, "probe"),
                           SimpleNamespace(stdout='[{"type":2,"name":"Radeon"}]')]
        self.assertEqual(native_vulkan_environment({}),
                         ({"VK_ICD_FILENAMES": str(amd)}, "Radeon"))

    @patch("native_vulkan.subprocess.run")
    def test_falls_back_if_no_driver_isolates_dedicated_devices(self, run):
        (self.drivers / "radeon_icd.x86_64.json").touch()
        run.return_value.stdout = '[{"type":1,"name":"Integrated"},{"type":2,"name":"Discrete"}]'
        self.assertEqual(native_vulkan_environment({}), ({}, "system Vulkan default"))
