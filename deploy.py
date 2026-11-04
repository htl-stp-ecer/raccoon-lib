#!/usr/bin/env python3
"""deploy.py — install the local raccoon-lib wheels on a Pi (for development).

Copies the two wheels of the last build.py run to the Pi and installs them there:

  * raccoon-sim      the compiled runtime and simulator (what runs the robot),
  * raccoon-library  the type stubs. Its version is what the Pi reports as the library version.

Releases are installed by `raccoon update` (from PyPI), not by this script. Standard library only.

Usage:
    python3 deploy.py --host 192.168.4.1                    # the wheels from the last build.py run
    python3 deploy.py --host 192.168.4.1 --artifact builds  # a directory (or manifest.json) with both wheels

raccoon-lib is set to "dev" in the Pi's /etc/raccoon-bundle.json: deployed wheels are a custom build.
The wheels are installed without dependencies, so a local raccoon-transport stays as it is.
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path

COMPONENT = "raccoon-lib"
BUNDLE_FILE = "/etc/raccoon-bundle.json"
REMOTE_TMP = "/tmp/raccoon-lib-install"

# Runs on the Pi as root: set this component's version to "dev" in the bundle file and
# leave everything else in it as it is.
MARK_DEV = r"""
import json, os, tempfile
path = "%(path)s"
try:
    with open(path) as handle:
        document = json.load(handle)
    if not isinstance(document, dict):
        document = {}
except (OSError, ValueError):
    document = {}
document.setdefault("bundle", "dev")
components = document.get("components")
if not isinstance(components, dict):
    components = {}
components["%(component)s"] = "dev"
document["components"] = components
fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".raccoon-bundle.")
with os.fdopen(fd, "w") as handle:
    json.dump(document, handle, indent=2)
    handle.write("\n")
os.chmod(tmp, 0o644)
os.replace(tmp, path)
os.makedirs("/run/raccoon", exist_ok=True)
os.chmod("/run/raccoon", 0o1777)
"""


def fail(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    sys.exit(1)


class Pi:
    def __init__(self, host: str, user: str, ssh_params: str) -> None:
        self.target = f"{user}@{host}"
        self.params = shlex.split(ssh_params)

    def run(self, command: str, *, check: bool = True, stdin: str | None = None) -> int:
        result = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=5", *self.params, self.target, command],
            input=stdin,
            text=True,
        )
        if check and result.returncode != 0:
            fail(f"command failed on the Pi: {command}")
        return result.returncode

    def copy(self, local: Path, remote_dir: str) -> None:
        subprocess.run(["scp", *self.params, str(local), f"{self.target}:{remote_dir}/"], check=True)


def find_wheels(artifact: str | None) -> list[Path]:
    """The raccoon-sim and the raccoon-library wheel (from a directory or its manifest.json)."""
    if artifact:
        folder = Path(artifact)
        folder = folder.parent if folder.is_file() else folder
    else:
        folder = Path(__file__).resolve().parent / "builds"
        if not (folder / "manifest.json").is_file():
            fail("no builds/manifest.json. Run build.py first, or pass --artifact <directory with the wheels>.")
    wheels = []
    for prefix in ("raccoon_sim-", "raccoon_library-"):
        found = sorted(folder.glob(f"{prefix}*.whl"))
        if len(found) != 1:
            fail(f"expected one {prefix}*.whl in {folder}, found {len(found)}")
        wheels.append(found[0])
    versions = {w.name.split("-")[1] for w in wheels}
    if len(versions) != 1:
        fail(f"the wheels have different versions: {', '.join(w.name for w in wheels)}")
    return wheels


def main() -> None:
    parser = argparse.ArgumentParser(description="Install the raccoon-lib wheels on a Pi.")
    parser.add_argument("--host", required=True, help="Pi address")
    parser.add_argument("--user", default="pi", help="SSH user (default: pi)")
    parser.add_argument("--artifact", help="directory with both wheels (default: builds/)")
    parser.add_argument("--ssh-params", default="", help='extra ssh/scp options, e.g. "-o StrictHostKeyChecking=no"')
    args = parser.parse_args()

    wheels = find_wheels(args.artifact)
    pi = Pi(args.host, args.user, args.ssh_params)
    print(f"Deploying {', '.join(w.name for w in wheels)} to {pi.target}")

    print("Testing SSH connection...")
    if pi.run("true", check=False) != 0:
        fail(f"cannot connect to {pi.target} (see the ssh message above). "
             f"Host key changed (e.g. after flashing a new image)?  ssh-keygen -R {args.host}")

    print("Uploading wheels...")
    pi.run(f"rm -rf {REMOTE_TMP} && mkdir -p {REMOTE_TMP}")
    for wheel in wheels:
        pi.copy(wheel, REMOTE_TMP)

    print("Installing...")
    # A user-level install would shadow the system-wide one.
    pi.run("python3 -m pip uninstall raccoon-sim raccoon-library -y --break-system-packages 2>/dev/null || true", check=False)
    remote = " ".join(shlex.quote(f"{REMOTE_TMP}/{w.name}") for w in wheels)
    pi.run(f"sudo pip3 install --break-system-packages --force-reinstall --no-deps {remote}")

    # The next run checks itself against the bundle file.
    print(f'Marking {COMPONENT} as "dev" in {BUNDLE_FILE}...')
    pi.run("sudo python3 -", stdin=MARK_DEV % {"path": BUNDLE_FILE, "component": COMPONENT})

    # Check stdout for the marker: the exit code can be non-zero because of a hardware cleanup
    # segfault in an atexit handler.
    print()
    pi.run("pip3 show raccoon-sim raccoon-library | grep -E '^(Name|Version):'; "
           "python3 -c 'import raccoon; print(\"raccoon OK\")' 2>&1 | tail -1", check=False)
    print(f"Deployment to {args.host} completed.")


if __name__ == "__main__":
    main()
