from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import build_channel_release
import build_release


class ChannelReleaseTests(unittest.TestCase):
    def test_package_python_weather_updates_for_installed_entry_points(self):
        adapter = ("CARLA/Modules/Perception/Adapters/CARLA/LiDAR/"
                   "asurt_velodyne_sim/asurt_velodyne_sim/weather.py")
        mcp = "CARLA/Tools/MCP/server.py"
        changes = f"A\0{adapter}\0M\0{mcp}\0M\0{build_release.DRIVING_SOURCE.as_posix()}\0"
        workspace = build_release.PERCEPTION_WORKSPACE
        package = "asurt_velodyne_sim"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with patch.object(build_channel_release.subprocess, "check_output", return_value=changes):
                build_channel_release.copy_changed_files("previous-release", root)
            source = (build_release.REPOSITORY / adapter).read_bytes()
            for path in (
                    build_release.PERCEPTION / adapter.split("Perception/", 1)[1],
                    workspace / "src" / package / package / "weather.py",
                    workspace / "install" / package / "lib/python3.10/site-packages" / package / "weather.py"):
                self.assertEqual((root / path).read_bytes(), source)
            self.assertEqual((root / "libexec/asurt-mcp/server.py").read_bytes(),
                             (build_release.REPOSITORY / mcp).read_bytes())
            self.assertEqual((root / build_release.DRIVING_INSTALL).read_bytes(),
                             (build_release.REPOSITORY / build_release.DRIVING_SOURCE).read_bytes())


if __name__ == "__main__":
    unittest.main()
