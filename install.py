#!/usr/bin/env python3
"""install.py — Deploy pre-built wheel to Raspberry Pi.

Usage:
    RPI_HOST=10.x.x.x python install.py

Env vars:
    RPI_HOST      — Pi IP address (required)
    RPI_USER      — Pi SSH user   (default: pi)
    PROJECT_NAME  — Python import name (default: libstp)
    PYTHON_CMD    — Python command on Pi (default: python3)
"""

import glob
import os
import subprocess
import sys
from pathlib import Path


def ssh(host: str, user: str, command: str, check: bool = True) -> int:
    """Run a command on the Pi via SSH."""
    result = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", f"{user}@{host}", command],
        capture_output=not check,
    )
    if check and result.returncode != 0:
        print(f"ERROR: SSH command failed: {command}")
        sys.exit(1)
    return result.returncode


def scp(source: str, dest: str) -> None:
    """Copy a file to the Pi via SCP."""
    result = subprocess.run(["scp", source, dest])
    if result.returncode != 0:
        print(f"ERROR: SCP failed: {source} -> {dest}")
        sys.exit(1)


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    project_name = os.environ.get("PROJECT_NAME", "libstp")
    python_cmd = os.environ.get("PYTHON_CMD", "python3")
    rpi_user = os.environ.get("RPI_USER", "pi")
    rpi_host = os.environ.get("RPI_HOST", "")

    if not rpi_host:
        print("Error: RPI_HOST is required.")
        print("Usage: RPI_HOST=10.x.x.x python install.py")
        sys.exit(1)

    # Find wheel file relative to this script
    wheels = glob.glob(str(script_dir / "*.whl"))
    if not wheels:
        print(f"Error: No .whl file found in {script_dir}")
        sys.exit(1)

    wheel_file = wheels[0]
    wheel_basename = Path(wheel_file).name
    print(f"▶ Deploying {wheel_basename} to {rpi_user}@{rpi_host}")

    # Preflight: test SSH
    print("• Testing SSH connection...")
    if ssh(rpi_host, rpi_user, "echo 'SSH OK'", check=False) != 0:
        print(f"Error: Cannot connect to {rpi_user}@{rpi_host}")
        print("  Check: SSH enabled, host reachable, keys configured")
        sys.exit(1)

    # Copy wheel
    print("• Copying wheel...")
    scp(wheel_file, f"{rpi_user}@{rpi_host}:/tmp/{wheel_basename}")

    # Install
    print("• Installing package...")
    ssh(
        rpi_host,
        rpi_user,
        f"{python_cmd} -m pip install --user --force-reinstall '/tmp/{wheel_basename}' --break-system-packages",
    )

    # Verify — check stdout for success marker (exit code may be non-zero
    # due to hardware cleanup segfault in atexit handler)
    print("• Verifying import...")
    result = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", f"{rpi_user}@{rpi_host}",
         f"{python_cmd} -c 'import {project_name}; print(\"{project_name} OK\")'"],
        capture_output=True, text=True,
    )
    if f"{project_name} OK" in result.stdout:
        print(f"✓ {project_name} deployed successfully to {rpi_user}@{rpi_host}")
    else:
        print("⚠ Package installed but import test failed")
        print(result.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
