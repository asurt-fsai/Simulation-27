"""Resolve Windows PowerShell when WSL omits Windows directories from PATH."""

import os
from pathlib import Path
import shutil
import subprocess


def powershell_exe():
    installed = shutil.which("powershell.exe")
    if installed:
        return installed
    system_root = os.environ.get("SystemRoot") or os.environ.get("WINDIR") or r"C:\Windows"
    windows = system_root.replace("\\", "/").rstrip("/")
    try:
        path = Path(subprocess.check_output(
            ["wslpath", "-u", f"{windows}/System32/WindowsPowerShell/v1.0/powershell.exe"],
            text=True, stderr=subprocess.DEVNULL, timeout=5).strip())
        if path.is_file():
            return str(path)
    except (OSError, subprocess.SubprocessError):
        pass
    raise RuntimeError("Windows PowerShell is unavailable from WSL. Run the Windows installer from PowerShell.")
