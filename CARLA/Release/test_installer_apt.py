"""Regression coverage for the installer-local APT sources."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


INSTALLER = Path(__file__).resolve().parents[2] / "Install-ASURT.sh"
GAZEBO = "https://packages.osrfoundation.org/gazebo/ubuntu-stable"
ROS = "http://packages.ros.org/ros2/ubuntu"
UBUNTU = "http://archive.ubuntu.com/ubuntu"


@unittest.skipUnless(shutil.which("bash") and shutil.which("awk"), "Bash and awk required")
class AptSourcesTest(unittest.TestCase):
    def test_stage_only_removes_gazebo_from_installer_copy(self):
        installer = INSTALLER.read_text(encoding="utf-8")
        stage_function = installer[installer.index("stage_apt_sources()"):installer.index("\napt_for_asurt()")]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sources = root / "sources.list.d"
            sources.mkdir()
            main = root / "sources.list"
            original_main = f"deb {UBUNTU} jammy main universe\n"
            main.write_text(original_main)
            mixed_list = sources / "third-party.list"
            original_list = (f"deb [arch=amd64 signed-by=/keyrings/gazebo.gpg] {GAZEBO} jammy main\n"
                             f"deb-src {GAZEBO}/ jammy main\n"
                             f"deb [signed-by=/keyrings/ros.gpg] {ROS} jammy main\n")
            mixed_list.write_text(original_list)
            deb822 = sources / "custom.sources"
            original_deb822 = (f"Types: deb\nURIs: {GAZEBO}\nSuites: jammy\nComponents: main\n\n"
                               f"Types: deb\nuRis:\n {GAZEBO}/\n {UBUNTU}\nSuites: jammy\n"
                               "Components: main\nSigned-By: /keyrings/ubuntu.gpg\n")
            deb822.write_text(original_deb822)
            staged = root / "staged"
            subprocess.run(["bash", "-ec", stage_function + '\nstage_apt_sources "$@"',
                            "test-apt", str(root), str(staged)], check=True)
            self.assertEqual(main.read_text(), original_main)
            self.assertEqual(mixed_list.read_text(), original_list)
            self.assertEqual(deb822.read_text(), original_deb822)
            self.assertEqual((staged / "apt-sources.list").read_text(), original_main)
            self.assertEqual((staged / "apt-sources/third-party.list").read_text(),
                             f"deb [signed-by=/keyrings/ros.gpg] {ROS} jammy main\n")
            result = (staged / "apt-sources/custom.sources").read_text()
            self.assertNotIn(GAZEBO, result)
            self.assertEqual(result.count("Types: deb"), 1)
            self.assertIn(f"URIs: {UBUNTU}\n", result)
            self.assertIn("Signed-By: /keyrings/ubuntu.gpg\n", result)
