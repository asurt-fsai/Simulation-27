"""Detect CUDA hardware and verify inference before activating a release."""

import argparse
import ctypes
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def nvidia_present():
    if Path('/dev/dxg').exists():
        smi = shutil.which('nvidia-smi.exe')
        if smi:
            probe = subprocess.run([smi, '-L'], capture_output=True, text=True, timeout=10)
            return 'GPU 0:' in probe.stdout
    for device in Path('/sys/bus/pci/devices').glob('*'):
        if (device / 'vendor').read_text().strip() != '0x10de':
            continue
        if (device / 'class').read_text().startswith('0x03'):
            return True
    return False


def cuda_available():
    if os.environ.get('CUDA_VISIBLE_DEVICES') in {'', '-1'}:
        return False
    wsl_driver = Path('/usr/lib/wsl/lib/libcuda.so.1')
    try:
        driver = ctypes.CDLL(str(wsl_driver) if wsl_driver.is_file() else 'libcuda.so.1')
    except OSError as error:
        if nvidia_present():
            raise RuntimeError('An NVIDIA GPU was detected. Install or update its CUDA-capable host driver and rerun the installer.') from error
        return False
    result = driver.cuInit(0)
    if result == 100:  # CUDA_ERROR_NO_DEVICE
        if not nvidia_present():
            return False
        raise RuntimeError('The NVIDIA GPU is present but unavailable to CUDA. Check the host driver and WSL2 GPU support, then rerun.')
    if result != 0:
        raise RuntimeError(f'The CUDA host driver failed to initialize (error {result}). Update the driver and rerun.')
    count = ctypes.c_int()
    result = driver.cuDeviceGetCount(ctypes.byref(count))
    if result != 0:
        raise RuntimeError(f'CUDA could not enumerate devices (error {result}).')
    return count.value > 0


def verify(root, backend):
    import numpy as np
    import onnxruntime as ort

    provider = {'cpu': 'CPUExecutionProvider', 'cuda': 'CUDAExecutionProvider'}[backend]
    systems = root / 'Simulation/Systems'
    models = {
        'YOLO26 camera': (systems / 'Perception/Models/yolo26n_fsoco_e2e_opset17.onnx', (1, 3, 640, 640), (1, 300, 6)),
        'Deep-learning planner': (systems / 'Legacy/fs-system-26-oldstack/Deeplearning/src/planning_deep_learning/planning_deep_learning/Completed_Models/best_model.onnx', (1, 10, 4), (1, 15, 2)),
    }
    with tempfile.TemporaryDirectory(prefix='asurt-inference-') as tmp:
        for label, (model, input_shape, output_shape) in models.items():
            options = ort.SessionOptions()
            options.intra_op_num_threads = 2
            options.inter_op_num_threads = 1
            if backend == 'cuda':
                options.enable_profiling = True
                options.profile_file_prefix = str(Path(tmp) / 'ort')
            session = ort.InferenceSession(str(model), sess_options=options, providers=[provider])
            if provider not in session.get_providers():
                raise RuntimeError(f'{label}: {provider} did not initialize.')
            inputs = {session.get_inputs()[0].name: np.zeros(input_shape, dtype=np.float32)}
            output, = session.run(None, inputs)
            if output.shape != output_shape or not np.isfinite(output).all():
                raise RuntimeError(f'{label}: unexpected model output.')
            if backend == 'cuda':
                events = json.loads(Path(session.end_profiling()).read_text())
                kernels = sum(e.get('cat') == 'Node' and e.get('args', {}).get('provider') == provider for e in events)
                if not kernels:
                    raise RuntimeError(f'{label}: no operators ran on CUDA.')
                print(f'{label}: CUDA verified ({kernels} GPU operator events).', flush=True)
            else:
                print(f'{label}: CPU inference verified.', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('detect', 'verify'))
    args = parser.parse_args()
    if args.action == 'detect':
        print('cuda' if cuda_available() else 'cpu')
    else:
        verify(Path(os.environ['ASURT_ROOT']), os.environ['ASURT_INFERENCE_BACKEND'])
