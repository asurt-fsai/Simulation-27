from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import build_channel_release
import build_release


class ChannelReleaseTests(unittest.TestCase):
    def test_compiled_feedback_change_requires_the_rebuilt_bridge(self):
        changes = f"M\0{build_release.BRIDGE_FEEDBACK.as_posix()}\0"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            executable = root / "new-bridge"
            executable.write_bytes(b"rebuilt bridge")
            with patch.object(build_channel_release.subprocess, "check_output", return_value=changes):
                with self.assertRaisesRegex(ValueError, "compiled ROS"):
                    build_channel_release.copy_changed_files("previous-release", root / "patch")
                build_channel_release.copy_changed_files("previous-release", root / "patch", executable)
            self.assertEqual((root / "patch" / build_release.BRIDGE_EXECUTABLE).read_bytes(),
                             executable.read_bytes())

    def test_cooked_runtime_pin_requires_same_shipping_executables(self):
        baseline = {"linux": {"file": "linux.tar.gz", "shipping_sha256": "linux"},
                    "windows": {"file": "windows.tar.gz", "shipping_sha256": "windows"}}
        manifest = {"runtime": baseline}
        with tempfile.TemporaryDirectory() as temporary:
            pin = Path(temporary) / "cooked-runtimes.json"
            pinned = {key: item | {"tag": "testing-23"} for key, item in baseline.items()}
            pin.write_text(json.dumps(pinned))
            build_channel_release.pin_compatible_cooked_runtimes(manifest, pin)
            self.assertEqual(manifest["runtime"], pinned)
            pinned["linux"]["shipping_sha256"] = "different"
            pin.write_text(json.dumps(pinned))
            with self.assertRaisesRegex(ValueError, "released simulator build"):
                build_channel_release.pin_compatible_cooked_runtimes({"runtime": baseline}, pin)

    def test_package_python_lidar_updates_for_installed_entry_points(self):
        adapter = ("CARLA/Modules/Perception/Adapters/CARLA/LiDAR/"
                   "asurt_velodyne_sim/asurt_velodyne_sim")
        mcp = "CARLA/Tools/MCP/server.py"
        changes = (f"A\0{adapter}/weather.py\0M\0{adapter}/scan.py\0"
                   f"M\0{mcp}\0M\0{build_release.DRIVING_SOURCE.as_posix()}\0"
                   f"M\0{build_release.BRIDGE_CONFIG.as_posix()}\0")
        workspace = build_release.PERCEPTION_WORKSPACE
        package = "asurt_velodyne_sim"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with patch.object(build_channel_release.subprocess, "check_output", return_value=changes):
                build_channel_release.copy_changed_files("previous-release", root)
            for name in ("weather.py", "scan.py"):
                source = (build_release.REPOSITORY / adapter / name).read_bytes()
                for directory in (
                        build_release.PERCEPTION / adapter.split("Perception/", 1)[1],
                        workspace / "src" / package / package,
                        workspace / "install" / package / "lib/python3.10/site-packages" / package):
                    self.assertEqual((root / directory / name).read_bytes(), source)
            self.assertEqual((root / "libexec/asurt-mcp/server.py").read_bytes(),
                             (build_release.REPOSITORY / mcp).read_bytes())
            self.assertEqual((root / build_release.DRIVING_INSTALL).read_bytes(),
                             (build_release.REPOSITORY / build_release.DRIVING_SOURCE).read_bytes())
            self.assertEqual((root / build_release.BRIDGE / "config" / build_release.BRIDGE_CONFIG.name).read_bytes(),
                             (build_release.REPOSITORY / build_release.BRIDGE_CONFIG).read_bytes())


if __name__ == "__main__":
    unittest.main()
