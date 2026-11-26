#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
venv="$here/.venv"
site="$venv/lib/python3.10/site-packages"
vendor="$(cd "$here/../../../../Perception/Dependencies/onnxruntime" && pwd)"
if [[ ! -x "$venv/bin/python" || ! -d "$site/onnxruntime" ]]; then
    echo "Missing isolated GPU runtime: $venv" >&2
    exit 2
fi
export LD_LIBRARY_PATH="$site/nvidia/curand/lib:$site/nvidia/cudnn/lib:$site/nvidia/cublas/lib:$site/nvidia/cuda_runtime/lib:${LD_LIBRARY_PATH:-}"
if [[ -d "$here/.cuda_libs" ]]; then
    export LD_LIBRARY_PATH="$here/.cuda_libs:$LD_LIBRARY_PATH"
fi
export PYTHONPATH="$site:$vendor:${PYTHONPATH:-}"
exec "$venv/bin/python" "$here/benchmark.py" "$@"
