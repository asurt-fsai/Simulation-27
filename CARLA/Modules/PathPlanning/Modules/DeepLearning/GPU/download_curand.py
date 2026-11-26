#!/usr/bin/env python3
"""Download the isolated CUDA 12 cuRAND wheel with official SHA256 validation.

Reuses the bounded ranged-transfer and complete-file hash verification used
for the ONNX Runtime GPU wheel. No packages are installed by this script.
"""
import json
import urllib.request

import download_ort_gpu as transfer

VERSION = "10.3.9.55"
METADATA = f"https://pypi.org/pypi/nvidia-curand-cu12/{VERSION}/json"


def curand_metadata():
    with urllib.request.urlopen(METADATA, timeout=25) as response:
        data = json.load(response)
    matches = [
        item for item in data["urls"]
        if item["filename"].startswith(f"nvidia_curand_cu12-{VERSION}-")
        and item["filename"].endswith("x86_64.whl")
    ]
    if len(matches) != 1 or not 50_000_000 < matches[0]["size"] < 80_000_000:
        raise RuntimeError("Unexpected NVIDIA cuRAND wheel metadata")
    return matches[0]


if __name__ == "__main__":
    transfer.PARTS = transfer.WHEELS / ".parts-curand"
    transfer.wheel_metadata = curand_metadata
    transfer.main()
