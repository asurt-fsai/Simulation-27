#!/usr/bin/env python3
"""Resumable, SHA256-verified parallel download of pinned ORT 1.20.2 GPU wheel.

Uses public PyPI metadata and the official file URL. Each part is verified
against HTTP Content-Range, and the final artifact against PyPI SHA256.
No system-wide package installation occurs.
"""
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.request

HERE = Path(__file__).resolve().parent
WHEELS = HERE / '.wheelhouse'
PARTS = WHEELS / '.parts'
BYTES = 1 << 20
WORKERS = max(1, min(64, int(os.environ.get('ASURT_DL_DOWNLOAD_WORKERS', '48'))))
META = 'https://pypi.org/pypi/onnxruntime-gpu/1.20.2/json'


def wheel_metadata():
    with urllib.request.urlopen(META, timeout=25) as response:
        data = json.load(response)
    wheel = next(
        item for item in data['urls']
        if item['filename'].startswith('onnxruntime_gpu-1.20.2-cp310')
        and 'x86_64' in item['filename']
    )
    if wheel['size'] > 320_000_000 or wheel['size'] < 250_000_000:
        raise RuntimeError('Unexpected wheel size')
    return wheel


def part_download(url, part_id, total):
    begin = part_id * BYTES
    end = min(total, begin + BYTES) - 1
    expected = end-begin+1
    destination = PARTS / f'{part_id:04d}.part'
    if destination.is_file() and destination.stat().st_size == expected:
        return expected
    for attempt in range(4):
        temporary = PARTS / f'{part_id:04d}.tmp'
        try:
            request = urllib.request.Request(
                url, headers={'Range': f'bytes={begin}-{end}',
                              'Accept-Encoding': 'identity'})
            with urllib.request.urlopen(request, timeout=90) as response:
                header = response.headers.get('Content-Range', '')
                if response.status != 206 or not header.startswith(
                        f'bytes {begin}-{end}/'):
                    raise RuntimeError('HTTP ranged download returned ' + str(header))
                payload = response.read(expected + 1)
            if len(payload) != expected:
                raise IOError(f'Incomplete part {part_id}: {len(payload)}')
            temporary.write_bytes(payload)
            os.replace(temporary, destination)
            return len(payload)
        except Exception as error:
            temporary.unlink(missing_ok=True)
            if attempt == 3:
                raise RuntimeError(f'part {part_id} failed: {error}') from error
            time.sleep(1 + attempt)


def main():
    wheel = wheel_metadata()
    WHEELS.mkdir(exist_ok=True)
    PARTS.mkdir(exist_ok=True)
    destination = WHEELS / wheel['filename']
    if destination.exists() and destination.stat().st_size == wheel['size']:
        with destination.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        if digest == wheel['digests']['sha256']:
            print('Already downloaded and verified:', destination, flush=True)
            return
    count = (wheel['size'] + BYTES - 1) // BYTES
    start = time.monotonic()
    with ThreadPoolExecutor(max_workers=WORKERS) as executor:
        tasks = [executor.submit(part_download, wheel['url'], i, wheel['size'])
                 for i in range(count)]
        done_bytes = 0
        for index, future in enumerate(as_completed(tasks)):
            done_bytes += future.result()
            if index == 0 or (index+1) % 24 == 0 or index+1 == count:
                seconds = max(.01, time.monotonic()-start)
                print(f'Finished {index+1}/{count} parts '
                      f'({done_bytes/1e6:.1f} MB, {done_bytes/seconds/1e6:.2f} MB/s)',
                      flush=True)
    hasher = hashlib.sha256()
    temporary = WHEELS / (wheel['filename'] + '.assembling')
    with temporary.open('wb') as out:
        for index in range(count):
            chunk = (PARTS / f'{index:04d}.part').read_bytes()
            out.write(chunk)
            hasher.update(chunk)
    if hasher.hexdigest() != wheel['digests']['sha256']:
        temporary.unlink(missing_ok=True)
        raise RuntimeError('Wheel SHA256 mismatch; refused installation')
    os.replace(temporary, destination)
    print('SHA256 verified:', destination, flush=True)


if __name__ == '__main__':
    main()
