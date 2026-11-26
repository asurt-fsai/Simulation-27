"""Exercise crash recovery without a running CARLA server."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
from process_ownership import recover_orphaned_processes


class ProcessOwnershipTests(unittest.TestCase):
    def test_recovers_only_children_of_this_installation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logs = root / "asurt/state/log/carla"
            recorded = logs / "20261126-225201-windows-flat-asphalt-3056"
            foreign = root / "other/state/log/carla" / recorded.name
            unrecorded = logs / "20261126-225202-windows-flat-asphalt-3057"
            for run_dir in (recorded, foreign, unrecorded):
                run_dir.mkdir(parents=True)
            (recorded / "carla_interface_config.yaml").touch()
            (foreign / "session.json").touch()
            processes = [subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                                          env=dict(os.environ, ASURT_SESSION_LOG=str(run_dir)))
                         for run_dir in (recorded, foreign, unrecorded)]
            try:
                self.assertEqual(recover_orphaned_processes(logs), 1)
                self.assertIsNotNone(processes[0].poll())
                self.assertIsNone(processes[1].poll())
                self.assertIsNone(processes[2].poll())
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.terminate()
                    process.wait(timeout=5)
