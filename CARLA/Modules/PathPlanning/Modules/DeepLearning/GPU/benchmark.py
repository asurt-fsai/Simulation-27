#!/usr/bin/env python3
"""Proof of CUDA operator placement and CPU/GPU parity for legacy ASURT LSTM."""
import json
from pathlib import Path
import resource
import statistics
import subprocess
import sys
import time

import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[6]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'Simulation/Systems/PathPlanning/Modules/'
                       'DeepLearning/asurt_deep_learning_planning'))
from asurt_deep_learning_planning.core import prepare_cones, check_prediction

MODEL = ROOT / ('Simulation/Systems/Legacy/fs-system-26-oldstack/Deeplearning/'
                'src/planning_deep_learning/planning_deep_learning/Completed_Models/'
                'best_model.onnx')


def gpu_memory():
    try:
        p = subprocess.run(['nvidia-smi', '--query-gpu=memory.used',
                            '--format=csv,noheader,nounits'],
                           capture_output=True, text=True, check=True, timeout=4)
        return int(p.stdout.strip().splitlines()[0])
    except Exception:
        return None


def data():
    batches, observations = [], []
    for i in range(12):
        cones = [(x + .02*i, y + .03*(i%3), color, .96)
                 for x in (2., 4., 6., 8., 10.)
                 for color, y in ((0, 2.), (1, -2.))]
        features, selected, reason = prepare_cones(cones)
        if reason != 'ok':
            raise RuntimeError(reason)
        batches.append(np.array(features, dtype=np.float32).reshape((1, 10, 4)))
        observations.append(selected)
    return batches, observations


def session(provider, prefix):
    if provider not in ort.get_available_providers():
        raise RuntimeError('Provider not installed: ' + provider)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.enable_profiling = True
    options.profile_file_prefix = str(prefix)
    model = ort.InferenceSession(str(MODEL), sess_options=options,
                                 providers=[provider])
    if provider not in model.get_providers():
        raise RuntimeError('Requested provider not active: ' + provider)
    return model


def measure(model, batches, count=100):
    key = model.get_inputs()[0].name
    for index in range(10):
        model.run(None, {key: batches[index % len(batches)]})
    ru = resource.getrusage(resource.RUSAGE_SELF)
    cpu_before = ru.ru_utime + ru.ru_stime
    results, times = [], []
    for index in range(count):
        t0 = time.perf_counter()
        out = model.run(None, {key: batches[index % len(batches)]})[0]
        times.append((time.perf_counter()-t0)*1000)
        if index < len(batches):
            results.append(out)
    ru = resource.getrusage(resource.RUSAGE_SELF)
    return results, {
        'runs': count,
        'median_ms': round(statistics.median(times), 4),
        'p95_ms': round(sorted(times)[int(.95*(len(times)-1))], 4),
        'cpu_seconds': round(ru.ru_utime+ru.ru_stime-cpu_before, 4),
        'total_inference_wall_seconds': round(sum(times)/1000, 4),
    }


def placement(path):
    counts = {}
    for event in json.loads(Path(path).read_text()):
        if event.get('cat') != 'Node':
            continue
        provider = event.get('args', {}).get('provider', 'unattributed')
        counts[provider] = counts.get(provider, 0) + 1
    return counts


def main():
    evidence = HERE / 'evidence'
    evidence.mkdir(exist_ok=True)
    batches, observed = data()
    before = gpu_memory()
    cpu = session('CPUExecutionProvider', evidence / 'ort_cpu')
    cpu_out, cpu_time = measure(cpu, batches)
    cpu_nodes = placement(cpu.end_profiling())
    cuda = session('CUDAExecutionProvider', evidence / 'ort_cuda')
    allocated = gpu_memory()
    cuda_out, cuda_time = measure(cuda, batches)
    cuda_nodes = placement(cuda.end_profiling())
    if cuda_nodes.get('CUDAExecutionProvider', 0) == 0:
        raise RuntimeError('CUDA session had no GPU-executed operators')
    largest_difference = max(float(np.max(np.abs(a-b)))
                             for a, b in zip(cpu_out, cuda_out))
    parity = all(np.isfinite(a).all() and np.isfinite(b).all()
                 and np.allclose(a, b, rtol=.01, atol=.01)
                 for a, b in zip(cpu_out, cuda_out))
    decisions = {}
    for prediction, cones in zip(cuda_out, observed):
        reason = check_prediction(prediction[0], cones)[1]
        decisions[reason] = decisions.get(reason, 0) + 1
    report = {
        'gpu': 'GTX 1050', 'compute_capability': '6.1',
        'model_path': str(MODEL), 'onnxruntime': ort.__version__,
        'cuda_session_providers': cuda.get_providers(),
        'cuda_op_placement': cuda_nodes, 'cpu_op_placement': cpu_nodes,
        'cpu': cpu_time, 'cuda': cuda_time,
        'max_abs_prediction_delta': largest_difference,
        'cpu_cuda_parity_rtol_atol_1e_2': bool(parity),
        'gpu_memory_mib_before': before, 'gpu_memory_mib_after_init': allocated,
        'path_rejection_counts': decisions,
    }
    target = evidence / 'gtx1050_onnx_gpu.json'
    target.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps(report, indent=2, sort_keys=True))
    if not parity:
        raise RuntimeError('CPU/GPU output mismatch beyond 0.01 tolerances')


if __name__ == '__main__':
    main()
