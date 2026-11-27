"""Regression tests for the installer on a machine with an older CUDA runtime."""

from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from asurt_update import digest, prepare_inference, verify_installation


class CudaUpgradeTest(unittest.TestCase):
    def setUp(self):
        work = tempfile.TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.root = Path(work.name)
        self.stack = self.root / 'new-stack'
        self.requirements = self.stack / 'bin/cuda-requirements.txt'
        self.requirements.parent.mkdir(parents=True)
        self.requirements.write_text('onnxruntime-gpu==1.20.2 --hash=sha256:fixture\n')
        previous = self.root / 'previous'
        self.previous_requirements = previous / 'bin/cuda-requirements.txt'
        self.previous_requirements.parent.mkdir(parents=True)
        self.previous_requirements.write_bytes(self.requirements.read_bytes())
        (self.root / 'current').symlink_to(previous)
        self.cuda = self.root / 'local-cuda'
        (self.cuda / 'onnxruntime').mkdir(parents=True)
        self.link = self.root / 'state/inference-cuda'
        self.link.parent.mkdir()
        self.link.symlink_to(self.cuda)
        self.target = self.root / 'inference' / digest(self.requirements)

    @patch('asurt_update.subprocess.run')
    def test_install_verification_requires_the_detected_backend(self, run):
        for runtime, backend in ((self.cuda, 'cuda'), (None, 'cpu')):
            with self.subTest(backend=backend):
                verify_installation(self.stack, self.link.parent, runtime)
                options = run.call_args.kwargs
                self.assertEqual(options['env']['ASURT_EXPECT_INFERENCE_BACKEND'], backend)
                self.assertEqual(options['env']['ASURT_RELEASE_CUDA_ROOT'], str(runtime) if runtime else '')
                self.assertTrue(options['check'])

    @patch('asurt_update.subprocess.run')
    @patch('asurt_update.subprocess.check_output', return_value='cuda\n')
    def test_verifies_and_reuses_selected_local_runtime(self, _detect, run):
        self.assertEqual(prepare_inference(self.root, self.stack), self.cuda)
        self.assertFalse(self.target.exists())
        run.assert_called_once()
        cmd = run.call_args.args[0]
        self.assertEqual(cmd[:3], ['bash', self.root / 'current/bin/asurt', 'exec'])
        self.assertEqual(cmd[-2], '1.20.2')
        self.assertEqual(run.call_args.kwargs['env']['ASURT_RELEASE_CUDA_ROOT'], str(self.cuda))

    @patch('asurt_update.subprocess.run')
    @patch('asurt_update.subprocess.check_output', return_value='cuda\n')
    def test_managed_runtime_takes_priority(self, _detect, run):
        self.target.mkdir(parents=True)
        self.assertEqual(prepare_inference(self.root, self.stack), self.target)
        run.assert_not_called()

    @patch('asurt_update.subprocess.run')
    @patch('asurt_update.subprocess.check_output', return_value='cuda\n')
    def test_changed_pins_fetch_new_wheels(self, _detect, run):
        self.previous_requirements.write_text('different previous requirements\n')
        run.side_effect = subprocess.CalledProcessError(2, 'pip')
        with self.assertRaisesRegex(RuntimeError, 'CUDA wheel download failed'):
            prepare_inference(self.root, self.stack)
        run.assert_called_once()
        cmd = run.call_args.args[0]
        self.assertIn('download', cmd)
        self.assertIn('--require-hashes', cmd)
        self.assertEqual(cmd[cmd.index('--timeout') + 1], '120')
        self.assertEqual(self.link.resolve(), self.cuda)

    @patch('asurt_update.subprocess.run')
    @patch('asurt_update.subprocess.check_output', return_value='cuda\n')
    def test_failed_gpu_probe_uses_pinned_download(self, _detect, run):
        run.side_effect = [subprocess.CalledProcessError(1, 'verify'),
                           subprocess.CalledProcessError(2, 'pip')]
        with self.assertRaisesRegex(RuntimeError, 'CUDA wheel download failed'):
            prepare_inference(self.root, self.stack)
        self.assertIn('download', run.call_args.args[0])
