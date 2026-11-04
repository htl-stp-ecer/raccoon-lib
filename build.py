#!/usr/bin/env python3
"""build.py — build the raccoon-lib wheels for the Raspberry Pi in Docker.

Nothing is installed on the host and nothing is written outside ./builds/. The library is cross
compiled on your PC at native speed (aarch64 compiler, nothing is emulated). Only finished programs
run under qemu-user: the C++ tests and the Python that writes the type stubs (no binfmt, no
--privileged).

Usage:
    python3 build.py                  # local build, version 0.0
    python3 build.py --version 157    # release build (CI)
    python3 build.py --test           # also build the C++ tests and run them with qemu-user

Result (both wheels always get the same version, the stubs pin raccoon-sim==<version>):
    builds/raccoon_sim-<version>-cp313-cp313-linux_aarch64.whl   compiled runtime + simulator (Pi)
    builds/raccoon_library-<version>-py3-none-any.whl            type stubs (.pyi), no code
    builds/manifest.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILDS = ROOT / "builds"
IMAGE = "raccoon-lib-build"
VERSION_PATTERN = re.compile(r"^[0-9]+(\.[0-9]+)*$")

TOOLCHAIN = "cmake/toolchains/aarch64-linux-gnu.cmake"
# Root of the aarch64 Python 3.13 and its libraries inside the image (see Dockerfile).
ARM_ROOT = "/opt/arm64-root"
EMULATOR = f"qemu-aarch64;-L;{ARM_ROOT}"
EXT_SUFFIX = ".cpython-313-aarch64-linux-gnu.so"

# Not copied into the working copy. build.py and deploy.py must not be there: `python -m build`
# would import ./build.py instead of the build package.
EXCLUDE = {".git", "builds", "docs", "build.py", "deploy.py", "__pycache__", ".venv", "_skbuild", ".cmake-cache"}


def fail(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    sys.exit(1)


def run(command: list[str], **kwargs) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, check=True, **kwargs)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ── inside the container ─────────────────────────────────────────────────

WORK = Path("/tmp/w")


def prepare_sources(version: str) -> None:
    """A copy of the repository (the mounted one is read-only) with the version patched in."""
    if WORK.exists():
        shutil.rmtree(WORK)
    shutil.copytree(
        "/src", WORK, symlinks=True,
        ignore=lambda directory, names: [n for n in names if n in EXCLUDE and Path(directory) == Path("/src")]
        + [n for n in names if n == "__pycache__"],
    )
    pyproject = WORK / "pyproject.toml"
    text = pyproject.read_text()
    text, count = re.subn(r'^version = "[^"]*"', f'version = "{version}"', text, count=1, flags=re.MULTILINE)
    if count != 1:
        fail("no `version = ...` line in pyproject.toml")
    pyproject.write_text(text)
    run(["python", "tools/generate_step_builders.py"], cwd=WORK, stdout=subprocess.DEVNULL)


def cmake_cross_args(cache: Path) -> list[str]:
    return [
        f"-DCMAKE_TOOLCHAIN_FILE={WORK / TOOLCHAIN}",
        f"-DFETCHCONTENT_BASE_DIR={cache / 'deps'}",
        "-DCMAKE_C_COMPILER_LAUNCHER=ccache", "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
        "-DLIBSTP_RUN_RUFF=OFF", "-DLIBSTP_RUN_MYPY=OFF",
    ]


def build_sim_wheel(cache: Path, out: Path) -> Path:
    env = dict(os.environ, _PYTHON_HOST_PLATFORM="linux-aarch64", SETUPTOOLS_EXT_SUFFIX=EXT_SUFFIX,
               CMAKE_BUILD_PARALLEL_LEVEL=str(os.cpu_count() or 4))
    run([
        "python", "-m", "build", "--wheel", "--no-isolation", "--outdir", str(out),
        "-C", f"build-dir={cache / 'wheel-build'}",
        "-C", "cmake.args=" + ";".join(cmake_cross_args(cache)),
    ], cwd=WORK, env=env)
    wheels = sorted(out.glob("raccoon_sim-*.whl"))
    if len(wheels) != 1:
        fail(f"expected one raccoon_sim wheel, found {len(wheels)}")
    return wheels[0]


def run_tests(cache: Path, version: str) -> None:
    """Build the C++ tests for the Pi and run them with qemu-user."""
    build_dir = cache / "test-build"
    run([
        "cmake", "-S", str(WORK), "-B", str(build_dir), "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release", "-DLIBSTP_BUILD_TESTS=ON",
        "-DSKBUILD_PROJECT_NAME=raccoon", f"-DSKBUILD_PROJECT_VERSION={version}",
        f"-DCMAKE_CROSSCOMPILING_EMULATOR={EMULATOR}",
        *cmake_cross_args(cache),
    ])
    run(["cmake", "--build", str(build_dir), "-j", str(os.cpu_count() or 4)])
    run(["ctest", "--test-dir", str(build_dir), "--output-on-failure"])


STUBS_PYPROJECT = '''\
[build-system]
requires = ["setuptools"]
build-backend = "setuptools.build_meta"

[project]
name = "raccoon-library"
version = "{version}"
description = "Raccoon robotics library — platform-independent Python/.pyi headers (IDE support, type hinting, toolchain codegen). Install the [sim] extra for the compiled runtime + simulator."
requires-python = ">=3.11"

[project.optional-dependencies]
# The compiled runtime + simulator lives in the sibling raccoon-sim distribution. Pinned exact so
# headers and native bindings stay in lockstep.
sim = ["raccoon-sim=={version}"]

[tool.setuptools]
packages = [
{packages}
]

[tool.setuptools.package-data]
"*" = ["*.pyi", "py.typed"]
'''


def qemu_python(site: Path, *args: str, cwd: str = "/tmp") -> None:
    """Run the aarch64 Python of the image under qemu-user, with the packages in `site`."""
    run(["qemu-aarch64", "-L", ARM_ROOT, f"{ARM_ROOT}/usr/bin/python3.13", *args],
        cwd=cwd, env={"PYTHONPATH": str(site), "PYTHONDONTWRITEBYTECODE": "1", "HOME": "/tmp", "PATH": os.environ["PATH"]})


def build_stub_wheel(sim_wheel: Path, version: str, out: Path) -> Path:
    """The raccoon-library wheel: the .pyi files of raccoon-sim, nothing else.

    pybind11-stubgen and mypy's stubgen import the finished module, so they run with the aarch64
    Python under qemu-user, on the wheel that was just built.
    """
    site = Path("/tmp/arm64-site")
    staging = Path("/tmp/stubs-staging")
    py_stubs = Path("/tmp/py-stubs")
    for path in (site, staging, py_stubs):
        shutil.rmtree(path, ignore_errors=True)
    site.mkdir()
    (staging / "raccoon").mkdir(parents=True)

    # The module and what it imports, for the aarch64 Python (the packages are downloaded as
    # aarch64 wheels, nothing is built).
    with zipfile.ZipFile(sim_wheel) as archive:
        archive.extractall(site)
    run([
        "pip", "install", "--quiet", "--target", str(site), "--only-binary=:all:", "--python-version", "3.13",
        "--implementation", "cp", "--platform", "manylinux_2_28_aarch64", "--platform", "manylinux2014_aarch64",
        "--platform", "linux_aarch64", "--no-deps",
        "numpy", "pyyaml", "aiosqlite", "pybind11-stubgen", "mypy", "mypy_extensions", "typing_extensions",
        "pathspec", "librt",
    ])
    run(["pip", "install", "--quiet", "--target", str(site), "--no-deps", str(WORK / "raccoon-transport/python")])

    print("--- pybind11-stubgen ---")
    try:
        qemu_python(site, "-m", "pybind11_stubgen", "raccoon", "-o", str(staging), "--ignore-all-errors")
    except subprocess.CalledProcessError:
        print("pybind11-stubgen reported errors, using what it wrote")
    print("--- mypy stubgen --inspect-mode ---")
    try:
        qemu_python(site, "-m", "mypy.stubgen", "--inspect-mode", "-p", "raccoon",
                    "--search-path", str(site), "-o", str(py_stubs))
    except subprocess.CalledProcessError:
        print("stubgen reported errors, using what it wrote")
    if (py_stubs / "raccoon").is_dir():
        for pyi in (py_stubs / "raccoon").rglob("*.pyi"):
            target = staging / "raccoon" / pyi.relative_to(py_stubs / "raccoon")
            if not target.exists():  # the pybind11-stubgen output wins for the C++ modules
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(pyi, target)

    # Only .pyi: the package has to share the raccoon/ namespace with raccoon-sim without clashing
    # in the RECORD files, so every .py is removed and each directory gets an __init__.pyi.
    for py in (staging / "raccoon").rglob("*.py"):
        py.unlink()
    for directory in [staging / "raccoon", *[d for d in (staging / "raccoon").rglob("*") if d.is_dir()]]:
        (directory / "__init__.pyi").touch(exist_ok=True)
    (staging / "raccoon" / "py.typed").touch()

    count = len(list((staging / "raccoon").rglob("*.pyi")))
    print(f"Total .pyi files: {count}")
    if count == 0:
        fail("no .pyi stubs were generated")

    packages = sorted(
        str(d.relative_to(staging)).replace("/", ".") for d in [staging / "raccoon", *(staging / "raccoon").rglob("*")]
        if d.is_dir()
    )
    (staging / "pyproject.toml").write_text(STUBS_PYPROJECT.format(
        version=version, packages="\n".join(f'    "{p}",' for p in packages)))
    run(["python", "-m", "build", "--wheel", "--no-isolation", "--outdir", str(out)], cwd=staging)
    wheels = sorted(out.glob("raccoon_library-*.whl"))
    if len(wheels) != 1:
        fail(f"expected one raccoon_library wheel, found {len(wheels)}")
    return wheels[0]


def run_in_container(version: str, owner: str, test: bool) -> None:
    cache = Path("/cache")
    out = Path("/tmp/out")
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir()
    prepare_sources(version)
    sim = build_sim_wheel(cache, out)
    if test:
        run_tests(cache, version)
    stubs = build_stub_wheel(sim, version, out)
    uid, gid = (int(x) for x in owner.split(":"))
    for wheel in (sim, stubs):
        target = Path("/out") / wheel.name
        shutil.copy(wheel, target)
        os.chown(target, uid, gid)


# ── on the host ──────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the raccoon-lib wheels for the Pi in Docker.")
    parser.add_argument("--version", default="0.0", help="version of both wheels (default: 0.0 = local build)")
    parser.add_argument("--test", action="store_true", help="also build the C++ tests and run them with qemu-user")
    parser.add_argument("--image", help="use this Docker image instead of building ./Dockerfile")
    parser.add_argument(
        "--cache-dir", default=str(Path.home() / ".cache" / "raccoon-build" / "raccoon-lib"),
        help="ccache, downloaded dependencies and build directories, kept between builds (default: %(default)s)",
    )
    parser.add_argument("--in-container", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--owner", default="0:0", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if not VERSION_PATTERN.match(args.version):
        fail(f"--version must be numbers separated by dots (got {args.version!r})")

    if args.in_container:
        run_in_container(args.version, args.owner, args.test)
        return

    if not shutil.which("docker"):
        fail("docker not found. The build runs in a container so the host stays clean.")
    if not (ROOT / "raccoon-transport" / "cpp").is_dir():
        fail("raccoon-transport is empty. Run `git submodule update --init`.")

    image = args.image
    if not image:
        image = IMAGE
        print(f"▶ Building Docker image {image}")
        run(["docker", "build", "-t", image, "-f", str(ROOT / "Dockerfile"), str(ROOT)])

    cache = Path(args.cache_dir)
    (cache / "ccache").mkdir(parents=True, exist_ok=True)
    user = f"{os.getuid()}:{os.getgid()}"
    BUILDS.mkdir(exist_ok=True)
    for stale in [*BUILDS.glob("raccoon_sim-*.whl"), *BUILDS.glob("raccoon_library-*.whl"), BUILDS / "manifest.json"]:
        stale.unlink(missing_ok=True)

    print(f"▶ Building raccoon-lib {args.version} (cross compiled for aarch64)")
    command = [
        "docker", "run", "--rm", "--user", user, "-e", "HOME=/tmp",
        "-e", "CCACHE_DIR=/cache/ccache", "-e", "CCACHE_BASEDIR=/tmp/w",
        "-v", f"{ROOT}:/src:ro", "-v", f"{BUILDS}:/out", "-v", f"{cache}:/cache",
        image, "python3", "/src/build.py", "--in-container", "--version", args.version, "--owner", user,
    ]
    if args.test:
        command.append("--test")
    run(command)

    wheels = sorted([*BUILDS.glob("raccoon_sim-*.whl"), *BUILDS.glob("raccoon_library-*.whl")])
    if len(wheels) != 2:
        fail(f"expected two wheels in {BUILDS}, found {len(wheels)}")
    manifest = {
        "name": "raccoon-lib",
        "version": args.version,
        "files": [{"name": w.name, "kind": "wheel", "sha256": sha256(w)} for w in wheels],
    }
    (BUILDS / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    for wheel in wheels:
        print(f"✓ {wheel.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
