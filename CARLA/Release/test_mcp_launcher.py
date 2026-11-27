"""Check MCP startup before the launcher sources the target's ROS environment."""

import os
from pathlib import Path
import pwd
import subprocess
import tempfile
import unittest


class McpLauncherTests(unittest.TestCase):
    def launch_environment(self, **overrides):
        source = (Path(__file__).with_name("asurt")).read_text()
        bootstrap, separator, _ = source.partition("source /opt/ros/humble/setup.bash\n")
        self.assertTrue(separator)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / ".asurt-platform").write_text("linux\n")
            result = subprocess.run(
                ["bash", "-c", bootstrap + 'printf "%s\\n" "$HOME" "$ASURT_STATE_ROOT"\n', "asurt", "mcp"],
                env={"PATH": os.defpath, "ASURT_ROOT": str(root), **overrides},
                capture_output=True, text=True, check=True, timeout=8,
            )
        return result.stdout.splitlines()

    def test_uses_account_home_when_the_host_omits_it(self):
        home = Path(pwd.getpwuid(os.getuid()).pw_dir)
        self.assertEqual(self.launch_environment(), [str(home), str(home / ".local/share/asurt/state")])

    def test_uses_account_home_when_the_host_sets_it_empty(self):
        home = Path(pwd.getpwuid(os.getuid()).pw_dir)
        self.assertEqual(self.launch_environment(HOME=""), [str(home), str(home / ".local/share/asurt/state")])

    def test_keeps_an_explicit_home_and_state_root(self):
        self.assertEqual(self.launch_environment(HOME="/custom/home", ASURT_STATE_ROOT="/custom/state"),
                         ["/custom/home", "/custom/state"])
