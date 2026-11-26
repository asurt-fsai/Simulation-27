#!/usr/bin/env python3
"""Sample live process CPU, resident memory, and NVIDIA GPU load.

Read-only: reports utilization for the existing CARLA/perception processes,
without changing CPU affinity, world settings, or the simulator process tree.
"""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import statistics
import subprocess
import time


def get_processes():
    targets = {
        'CarlaUE4-Linux-Shipping': 'carla',
        '/zed2i_sim_node': 'stereo_camera',
        '/velodyne_sim_node': 'lidar_source',
        '/lidar_perception_node': 'lidar_processor',
        '/conversion_node': 'zed_converter',
    }
    result = {}
    for folder in Path('/proc').iterdir():
        if not folder.name.isdigit():
            continue
        try:
            argv = (folder / 'cmdline').read_bytes().split(b'\x00')[:2]
            fields = (folder / 'stat').read_text().rpartition(')')[2].split()
            cmd = [s.decode(errors='replace') for s in argv]
        except (OSError, UnicodeError):
            continue
        if not cmd:
            continue
        target = None
        for suffix, label in targets.items():
            if any(part.endswith(suffix) for part in cmd):
                target = label
                break
        if target is None:
            continue
        # Fields are numbered from 3 after stripping the process's (comm).
        # utime=14, stime=15, RSS pages=24.
        ticks = int(fields[11]) + int(fields[12])
        rss_mb = int(fields[21]) * os.sysconf('SC_PAGE_SIZE') / (1024 * 1024)
        result[target] = (int(folder.name), ticks, rss_mb)
    return result


def gpu_sample():
    try:
        output = subprocess.check_output([
            'nvidia-smi',
            '--query-gpu=utilization.gpu,memory.used,memory.total',
            '--format=csv,noheader,nounits',
        ], text=True, stderr=subprocess.DEVNULL, timeout=2).strip().splitlines()[0]
        return [float(value.strip()) for value in output.split(',')]
    except (OSError, ValueError, subprocess.SubprocessError, IndexError):
        return None


def summarize(values):
    if not values:
        return None
    return {'mean': round(statistics.mean(values), 2),
            'max': round(max(values), 2), 'samples': len(values)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=int, default=15)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if not 3 <= args.seconds <= 120:
        parser.error('--seconds must be 3..120')
    clk = os.sysconf('SC_CLK_TCK')
    records = defaultdict(lambda: defaultdict(list))
    prev = get_processes()
    start = time.monotonic()
    previous_sample_time = start
    for _ in range(args.seconds):
        time.sleep(1)
        current = get_processes()
        sampled_at = time.monotonic()
        dt = max(0.001, sampled_at - previous_sample_time)
        for label, (pid, ticks, rss_mb) in current.items():
            old = prev.get(label)
            if old and old[0] == pid:
                # CPU percent where 100 means one logical CPU utilized. Use
                # actual elapsed sample time: nvidia-smi costs wall time, so
                # consecutive samples are not exactly one second apart.
                records[label]['cpu_pct'].append(
                    100 * max(0, ticks - old[1]) / (clk * dt))
            records[label]['rss_mb'].append(rss_mb)
        prev = current
        previous_sample_time = sampled_at
        gpu = gpu_sample()
        if gpu is not None:
            for key, val in zip(('util_gpu_pct', 'vram_used_mb', 'vram_total_mb'), gpu):
                records['gpu'][key].append(val)
    report = {
        'time_utc': datetime.now(timezone.utc).isoformat(),
        'elapsed_wall_s': round(time.monotonic() - start, 2),
        'processors': os.cpu_count(),
        'samples': {label: {metric: summarize(values)
                            for metric, values in group.items()}
                    for label, group in records.items()},
    }
    encoded = json.dumps(report, indent=2)
    print(encoded)
    if args.output:
        Path(args.output).write_text(encoded + '\n')


if __name__ == '__main__':
    main()
