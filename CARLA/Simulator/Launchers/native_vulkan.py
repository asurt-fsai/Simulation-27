"""Select a dedicated Vulkan renderer for the native Ubuntu CARLA server."""

import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys


DRIVER_DIRS = (Path("/etc/vulkan/icd.d"), Path("/usr/local/share/vulkan/icd.d"),
               Path("/usr/share/vulkan/icd.d"))
DRIVER_NAMES = ("nvidia_icd.json", "nvidia_icd.x86_64.json", "radeon_icd.x86_64.json",
                "amd_icd64.json", "intel_icd.x86_64.json")
RENDERER_OVERRIDES = ("VK_DRIVER_FILES", "VK_ICD_FILENAMES", "VK_LOADER_DRIVERS_SELECT",
                      "VK_LOADER_DRIVERS_DISABLE", "DRI_PRIME", "__NV_PRIME_RENDER_OFFLOAD",
                      "__VK_LAYER_NV_optimus")
DISCRETE_GPU = 2  # VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU


def native_vulkan_environment(environment):
    if any(environment.get(key) for key in RENDERER_OVERRIDES):
        return environment, "explicit renderer settings"
    for name in DRIVER_NAMES:
        for directory in DRIVER_DIRS:
            manifest = directory / name
            if not manifest.is_file():
                continue
            selected = dict(environment, VK_ICD_FILENAMES=str(manifest))
            try:
                probe = subprocess.run([sys.executable, __file__, "--probe"], env=selected,
                                       capture_output=True, text=True, check=True, timeout=6)
                devices = json.loads(probe.stdout)
            except (subprocess.SubprocessError, OSError, ValueError):
                continue
            if devices and all(device["type"] == DISCRETE_GPU for device in devices):
                return selected, ", ".join(device["name"] for device in devices)
    return environment, "system Vulkan default"


def vulkan_devices():
    """Enumerate devices in a child process with the candidate driver selected."""
    u32, pointer = ctypes.c_uint32, ctypes.c_void_p

    class InstanceCreateInfo(ctypes.Structure):
        _fields_ = [("s_type", u32), ("p_next", pointer), ("flags", u32),
                    ("application", pointer), ("layer_count", u32), ("layers", pointer),
                    ("extension_count", u32), ("extensions", pointer)]

    class DeviceProperties(ctypes.Structure):
        _fields_ = [("api_version", u32), ("driver_version", u32), ("vendor", u32),
                    ("device", u32), ("device_type", u32), ("name", ctypes.c_char * 256),
                    ("remaining_properties", ctypes.c_uint64 * 512)]

    vulkan = ctypes.CDLL("libvulkan.so.1")
    instance = pointer()
    create = InstanceCreateInfo(s_type=1)  # VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO
    if vulkan.vkCreateInstance(ctypes.byref(create), None, ctypes.byref(instance)):
        raise RuntimeError("Could not create a Vulkan instance.")
    try:
        count = u32()
        if vulkan.vkEnumeratePhysicalDevices(instance, ctypes.byref(count), None):
            raise RuntimeError("Could not count Vulkan devices.")
        devices = (pointer * count.value)()
        if vulkan.vkEnumeratePhysicalDevices(instance, ctypes.byref(count), devices):
            raise RuntimeError("Could not enumerate Vulkan devices.")
        result = []
        for handle in devices[:count.value]:
            properties = DeviceProperties()
            vulkan.vkGetPhysicalDeviceProperties(pointer(handle), ctypes.byref(properties))
            result.append({"type": properties.device_type, "name": properties.name.decode(errors="replace")})
        return result
    finally:
        vulkan.vkDestroyInstance(instance, None)


if __name__ == "__main__":
    if sys.argv[1:] == ["--probe"]:
        print(json.dumps(vulkan_devices()))
    else:
        _, renderer = native_vulkan_environment(os.environ)
        print(f"ASURT Ubuntu Vulkan renderer: {renderer}")
