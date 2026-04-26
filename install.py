#!/usr/bin/env python3
"""install.py — Install raccoon locally (stubs) or deploy to Raspberry Pi.

Usage:
    python install.py                         # Install stubs on this machine
    python install.py --stubs                 # Same as above (explicit)
    RPI_HOST=10.x.x.x python install.py      # Deploy full wheel to Pi

Env vars (Pi deploy only):
    RPI_HOST      — Pi IP address (required for Pi deploy)
    RPI_USER      — Pi SSH user   (default: pi)
    PROJECT_NAME  — Python import name (default: raccoon)
    PYTHON_CMD    — Python command on Pi (default: python3)
"""

import glob
import ipaddress
import os
import re
import subprocess
import sys
from pathlib import Path

# Hostname grammar from RFC 1123: each label 1-63 chars, alnum or hyphen,
# no leading/trailing hyphen. Total length capped at 253.
_HOSTNAME_LABEL = r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?"
_HOSTNAME_RE = re.compile(rf"^{_HOSTNAME_LABEL}(?:\.{_HOSTNAME_LABEL})*$")


def _validate_host(host: str) -> str:
    """Reject anything that's not a plain hostname or IP literal.

    Returning the host unchanged on success keeps call sites simple.
    Raises SystemExit on bad input rather than letting shell-y characters
    leak into ssh/scp argv (where they'd be quoted, but we'd rather fail
    loud than silently connect to a typo).
    """
    if not host or len(host) > 253:
        print(f"ERROR: invalid RPI_HOST {host!r}")
        sys.exit(1)
    try:
        ipaddress.ip_address(host)
        return host
    except ValueError:
        pass
    if not _HOSTNAME_RE.match(host):
        print(f"ERROR: RPI_HOST {host!r} is neither a valid IP nor hostname")
        sys.exit(1)
    return host


# accept-new pins the host key on first connect and rejects later changes,
# which is the right tradeoff for a deploy script that talks to one Pi.
_SSH_OPTS = [
    "-o", "ConnectTimeout=5",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "BatchMode=yes",  # password prompts are scripting smells
]


def ssh(host: str, user: str, command: str, check: bool = True) -> int:
    """Run a command on the Pi via SSH."""
    result = subprocess.run(
        ["ssh", *_SSH_OPTS, f"{user}@{host}", command],
        capture_output=not check,
    )
    if check and result.returncode != 0:
        print(f"ERROR: SSH command failed: {command}")
        sys.exit(1)
    return result.returncode


def scp(source: str, dest: str) -> None:
    """Copy a file to the Pi via SCP."""
    result = subprocess.run(["scp", *_SSH_OPTS, source, dest])
    if result.returncode != 0:
        print(f"ERROR: SCP failed: {source} -> {dest}")
        sys.exit(1)


def install_stubs() -> None:
    """Install the platform-independent stubs wheel on this machine."""
    script_dir = Path(__file__).resolve().parent

    # Look for stubs wheel next to this script, then in build-docker-stubs
    stubs = glob.glob(str(script_dir / "raccoon_stubs-*.whl"))
    if not stubs:
        stubs = glob.glob(str(script_dir / "build-docker-stubs" / "raccoon_stubs-*.whl"))
    if not stubs:
        print("Error: No raccoon_stubs-*.whl found.")
        print("  Run the build first, or download from a GitHub release.")
        sys.exit(1)

    wheel_file = stubs[0]
    print(f"▶ Installing {Path(wheel_file).name} locally")

    result = subprocess.run(
        [sys.executable, "-m", "pip", "install", "--force-reinstall",
         "--break-system-packages", wheel_file],
    )
    if result.returncode != 0:
        print("⚠ pip install failed")
        sys.exit(1)

    print("✓ raccoon-stubs installed (type hints & IDE support)")


def deploy_to_pi() -> None:
    """Deploy the full ARM64 wheel to a Raspberry Pi."""
    script_dir = Path(__file__).resolve().parent
    project_name = os.environ.get("PROJECT_NAME", "raccoon")
    python_cmd = os.environ.get("PYTHON_CMD", "python3")
    rpi_user = os.environ.get("RPI_USER", "pi")
    rpi_host = _validate_host(os.environ["RPI_HOST"])

    # Find the platform wheel (not stubs)
    wheels = [
        w for w in glob.glob(str(script_dir / "*.whl"))
        if "stubs" not in Path(w).name
    ]
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
        ["ssh", *_SSH_OPTS, f"{rpi_user}@{rpi_host}",
         f"{python_cmd} -c 'import {project_name}; print(\"{project_name} OK\")'"],
        capture_output=True, text=True,
    )
    if f"{project_name} OK" in result.stdout:
        print(f"✓ {project_name} deployed successfully to {rpi_user}@{rpi_host}")
    else:
        print("⚠ Package installed but import test failed")
        print(result.stderr)
        sys.exit(1)


def main() -> None:
    if "--stubs" in sys.argv or not os.environ.get("RPI_HOST"):
        install_stubs()
    else:
        deploy_to_pi()


if __name__ == "__main__":
    main()
