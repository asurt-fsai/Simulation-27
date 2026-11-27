"""Exercise crash recovery without a running CARLA server."""

import fcntl
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
from process_ownership import recover_orphaned_processes, wait_for_session_lock


class ProcessOwnershipTests(unittest.TestCase):
    def test_waits_for_the_lock_after_the_session_receipt_is_gone(self):
        with tempfile.TemporaryDirectory() as temporary:
            logs = Path(temporary)
            with (logs / "session.lock").open("a") as owner:
                fcntl.flock(owner, fcntl.LOCK_EX)
                released = threading.Event()

                def finish_cleanup():
                    released.set()
                    fcntl.flock(owner, fcntl.LOCK_UN)

                cleanup = threading.Timer(.3, finish_cleanup)
                cleanup.start()
                try:
                    wait_for_session_lock(logs, {"state": "original"}, lambda: None, timeout=3)
                    self.assertTrue(released.is_set())
                finally:
                    cleanup.join()

    def test_does_not_wait_on_a_new_session_started_during_shutdown(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError, "A different simulator session"):
                wait_for_session_lock(Path(temporary), {"state": "original"}, lambda: {"state": "new"})

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
