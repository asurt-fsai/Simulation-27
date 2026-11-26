# The installer supplies a validated CUDA environment. Local workspaces can
# reuse the team's existing CUDA wheels without modifying other ROS processes.
if [[ -z "${ASURT_INFERENCE_BACKEND:-}" ]]; then
    camera_gpu="${ASURT_ROOT:-$HOME/ASURT}/Simulation/Systems/PathPlanning/Modules/DeepLearning/GPU"
    camera_site="$camera_gpu/.venv/lib/python3.10/site-packages"
    if [[ ${CUDA_VISIBLE_DEVICES-available} != -1 && ${CUDA_VISIBLE_DEVICES-available} != "" &&
          -r "$camera_site/onnxruntime/capi/libonnxruntime_providers_cuda.so" ]] &&
       { [[ -e /dev/dxg ]] || compgen -G '/dev/nvidia[0-9]*' >/dev/null; }; then
        export ASURT_INFERENCE_BACKEND=cuda ASURT_CUDA_SITE="$camera_site"
        export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$camera_gpu/.cuda_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        for camera_library in "$camera_site"/nvidia/*/lib; do
            [[ ! -d "$camera_library" ]] || export LD_LIBRARY_PATH="$camera_library:$LD_LIBRARY_PATH"
        done
    fi
fi
if [[ ${ASURT_INFERENCE_BACKEND:-cpu} == cuda && -n ${ASURT_CUDA_SITE:-} ]]; then
    export PYTHONPATH="$ASURT_CUDA_SITE${PYTHONPATH:+:$PYTHONPATH}"
fi
printf 'ASURT camera inference backend: %s\n' "${ASURT_INFERENCE_BACKEND:-cpu}"
