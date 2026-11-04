#!/usr/bin/env python3
"""build.py — build the raccoon-lib wheels for the Raspberry Pi in Docker.

Nothing is installed on the host and nothing is written outside ./builds/. Two modes, picked by the
CPU of the machine it runs on (or --mode):

  cross   (x86_64 PC)   the library is cross compiled with the aarch64 compiler at native speed,
                        nothing is emulated. Only finished programs run under qemu-user: the C++
                        tests and the Python that writes the type stubs (no binfmt, no --privileged).
  native  (aarch64)     compiled with the normal compiler of an arm64 image, everything runs
                        natively, no qemu. This is what the CI does on an ARM runner.

Usage:
    python3 build.py                  # local build, version 0.0
    python3 build.py --version 157    # release build (CI)
    python3 build.py --test           # also build the C++ tests and run them
    python3 build.py --python-tests   # also run the fast Python tests (tests/python)
    python3 build.py --sim-tests      # also run the simulator Python tests (minutes; use native mode)
    python3 build.py --mode native    # force a mode (native on x86 makes x86_64 wheels, for testing the script)

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
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILDS = ROOT / "builds"
IMAGE = "raccoon-lib-build"  # + "-cross" / "-native"
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


def cmake_args(cache: Path, mode: str) -> list[str]:
    args = [
        f"-DFETCHCONTENT_BASE_DIR={cache / 'deps'}",
        "-DCMAKE_C_COMPILER_LAUNCHER=ccache", "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
        "-DLIBSTP_RUN_RUFF=OFF", "-DLIBSTP_RUN_MYPY=OFF",
        # As the PyPI wheels: on Linux both platform bundles (mock and wombat) are built and switched at
        # runtime; DRIVER_BUNDLE=mock is the static fallback and what builds raccoon.sim.mock.
        "-DDRIVER_BUNDLE=mock",
    ]
    if mode == "cross":
        args.insert(0, f"-DCMAKE_TOOLCHAIN_FILE={WORK / TOOLCHAIN}")
    return args


def build_sim_wheel(cache: Path, out: Path, mode: str, tests: bool = False) -> Path:
    """The raccoon-sim wheel. tests=True builds the flavour for the tests instead: one static mock
    platform (the simulator helpers raccoon.sim.mock are only there then) and the C++ test programs,
    which stay in the build directory for ctest."""
    env = dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL=str(os.cpu_count() or 4))
    if mode == "cross":  # the wheel is tagged and named for the Pi, not for the PC it is built on
        env.update(_PYTHON_HOST_PLATFORM="linux-aarch64", SETUPTOOLS_EXT_SUFFIX=EXT_SUFFIX)
    args = cmake_args(cache, mode)
    defines: list[str] = []
    if tests:
        args += ["-DLIBSTP_BUILD_TESTS=ON", "-DLIBSTP_RUNTIME_PLATFORM=OFF", "-DCMAKE_BUILD_TYPE=Release"]
        if mode == "cross":  # CMake runs the test programs (gtest discovery) through this
            # a define, not in cmake.args: its value is a list and cmake.args is split at ";"
            defines += ["-C", f"cmake.define.CMAKE_CROSSCOMPILING_EMULATOR={EMULATOR}"]
    run([
        "python", "-m", "build", "--wheel", "--no-isolation", "--outdir", str(out),
        "-C", f"build-dir={cache / (f'test-build-{mode}' if tests else f'wheel-build-{mode}')}",
        "-C", "cmake.args=" + ";".join(args), *defines,
    ], cwd=WORK, env=env)
    wheels = sorted(out.glob("raccoon_sim-*.whl"))
    if len(wheels) != 1:
        fail(f"expected one raccoon_sim wheel, found {len(wheels)}")
    return wheels[0]


def run_cpp_tests(cache: Path, mode: str) -> None:
    """Run the C++ tests built together with the test wheel (cross: under qemu-user)."""
    run(["ctest", "--test-dir", str(cache / f"test-build-{mode}"), "--output-on-failure"])


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


def target_python(mode: str, site: Path, *args: str, cwd: str = "/tmp") -> None:
    """Run the Python that can import the finished module, with the packages in `site`:
    cross: the aarch64 Python of the image under qemu-user; native: the image's own Python."""
    interpreter = (["qemu-aarch64", "-L", ARM_ROOT, f"{ARM_ROOT}/usr/bin/python3.13"] if mode == "cross"
                   else ["python"])
    run([*interpreter, *args],
        cwd=cwd, env={"PYTHONPATH": str(site), "PYTHONDONTWRITEBYTECODE": "1", "HOME": "/tmp", "PATH": os.environ["PATH"]})


def prepare_site(site: Path, sim_wheel: Path, mode: str, packages: list[str]) -> None:
    """A folder with the wheel, its Python dependencies and `packages`, for target_python().

    cross: the packages are downloaded as aarch64 wheels (nothing is built); native: the wheels for
    this machine."""
    shutil.rmtree(site, ignore_errors=True)
    site.mkdir(parents=True)
    with zipfile.ZipFile(sim_wheel) as archive:
        archive.extractall(site)
    platform_args = (["--python-version", "3.13", "--implementation", "cp", "--platform", "manylinux_2_28_aarch64",
                      "--platform", "manylinux2014_aarch64", "--platform", "linux_aarch64"] if mode == "cross" else [])
    run(["pip", "install", "--quiet", "--target", str(site), "--only-binary=:all:", *platform_args, "--no-deps",
         "numpy", "pyyaml", "aiosqlite", *packages])
    run(["pip", "install", "--quiet", "--target", str(site), "--no-deps", str(WORK / "raccoon-transport/python")])


# What the CI test job ran: the fast tests (--python-tests), then the simulator tests and the
# auto-tune simulator tests (--sim-tests; they take minutes, and under qemu-user far too long).
PYTEST_COMMON = ["-m", "pytest", "-p", "no:cacheprovider", "-o", "asyncio_default_fixture_loop_scope=function", "-W", "ignore::pytest.PytestAssertRewriteWarning", "-ra"]
PYTEST_FAST = [
    ["tests/python", "--ignore=tests/python/sim", "--ignore=tests/python/test_input_validation.py",
     "--ignore=tests/python/test_resource_conflicts.py", "--ignore=tests/python/test_watchdog.py",
     "--ignore=tests/python/test_stubs.py", "--ignore=tests/python/testing/test_plugin_end_to_end.py"],
]
PYTEST_SIM = [
    ["tests/python/sim", "--ignore=tests/python/sim/test_auto_tune.py", "--reruns", "1", "--reruns-delay", "2"],
    ["tests/python/sim/test_auto_tune.py", "--reruns", "1", "--reruns-delay", "2"],
]


def run_python_tests(test_wheel: Path, mode: str, runs: list[list[str]]) -> None:
    """pytest on the test wheel (cross: with the aarch64 Python under qemu-user)."""
    site = Path("/tmp/test-site")
    prepare_site(site, test_wheel, mode, ["pytest", "pytest-asyncio", "pytest-rerunfailures", "iniconfig",
                                           "pluggy", "packaging", "pygments"])
    failed = []
    for arguments in runs:
        print(f"--- pytest {' '.join(arguments)} ---", flush=True)
        try:
            target_python(mode, site, *PYTEST_COMMON, *arguments, cwd=str(WORK))
        except subprocess.CalledProcessError:
            failed.append(arguments[0])
    if failed:
        fail("python tests failed: " + ", ".join(failed))


def build_stub_wheel(sim_wheel: Path, version: str, out: Path, mode: str) -> Path:
    """The raccoon-library wheel: the .pyi files of raccoon-sim, nothing else.

    pybind11-stubgen and mypy's stubgen import the finished module, so they run on the wheel that
    was just built, with a Python that can load it (see target_python).
    """
    site = Path("/tmp/arm64-site")
    staging = Path("/tmp/stubs-staging")
    py_stubs = Path("/tmp/py-stubs")
    for path in (staging, py_stubs):
        shutil.rmtree(path, ignore_errors=True)
    (staging / "raccoon").mkdir(parents=True)
    prepare_site(site, sim_wheel, mode, ["pybind11-stubgen", "mypy", "mypy_extensions", "typing_extensions",
                                         "pathspec", "librt"])

    print("--- pybind11-stubgen ---")
    try:
        target_python(mode, site, "-m", "pybind11_stubgen", "raccoon", "-o", str(staging), "--ignore-all-errors")
    except subprocess.CalledProcessError:
        print("pybind11-stubgen reported errors, using what it wrote")
    print("--- mypy stubgen --inspect-mode ---")
    try:
        target_python(mode, site, "-m", "mypy.stubgen", "--inspect-mode", "-p", "raccoon",
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


def run_in_container(version: str, owner: str, test: bool, python_tests: bool, sim_tests: bool, mode: str) -> None:
    cache = Path("/cache")
    out = Path("/tmp/out")
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir()
    prepare_sources(version)
    sim = build_sim_wheel(cache, out, mode)
    stubs = build_stub_wheel(sim, version, out, mode)
    uid, gid = (int(x) for x in owner.split(":"))
    for wheel in (sim, stubs):
        target = Path("/out") / wheel.name
        shutil.copy(wheel, target)
        os.chown(target, uid, gid)
    if test or python_tests or sim_tests:
        test_out = Path("/tmp/out-test")
        shutil.rmtree(test_out, ignore_errors=True)
        test_out.mkdir()
        test_wheel = build_sim_wheel(cache, test_out, mode, tests=True)
        if test:
            run_cpp_tests(cache, mode)
        if python_tests:
            run_python_tests(test_wheel, mode, PYTEST_FAST)
        if sim_tests:
            run_python_tests(test_wheel, mode, PYTEST_SIM)


# ── on the host ──────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the raccoon-lib wheels for the Pi in Docker.")
    parser.add_argument("--version", default="0.0", help="version of both wheels (default: 0.0 = local build)")
    parser.add_argument("--test", action="store_true", help="also build the C++ tests and run them")
    parser.add_argument("--python-tests", action="store_true", help="run the fast Python tests (tests/python)")
    parser.add_argument("--sim-tests", action="store_true",
                        help="run the simulator Python tests (minutes; use native mode, qemu-user is far too slow)")
    parser.add_argument("--mode", choices=["auto", "cross", "native"], default="auto",
                        help="cross: aarch64 compiler on x86_64; native: compile on an arm64 machine "
                             "(default: native on aarch64, cross otherwise)")
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

    mode = args.mode if args.mode != "auto" else ("native" if platform.machine() in ("aarch64", "arm64") else "cross")
    if args.in_container:
        run_in_container(args.version, args.owner, args.test, args.python_tests, args.sim_tests, mode)
        return

    if not shutil.which("docker"):
        fail("docker not found. The build runs in a container so the host stays clean.")
    if not (ROOT / "raccoon-transport" / "cpp").is_dir():
        fail("raccoon-transport is empty. Run `git submodule update --init`.")

    image = args.image
    if not image:
        image = f"{IMAGE}-{mode}"
        print(f"▶ Building Docker image {image}")
        run(["docker", "build", "--target", mode, "-t", image, "-f", str(ROOT / "Dockerfile"), str(ROOT)])

    cache = Path(args.cache_dir)
    (cache / "ccache").mkdir(parents=True, exist_ok=True)
    user = f"{os.getuid()}:{os.getgid()}"
    BUILDS.mkdir(exist_ok=True)
    for stale in [*BUILDS.glob("raccoon_sim-*.whl"), *BUILDS.glob("raccoon_library-*.whl"), BUILDS / "manifest.json"]:
        stale.unlink(missing_ok=True)

    print(f"▶ Building raccoon-lib {args.version} ({'cross compiled for aarch64' if mode == 'cross' else 'native'})")
    command = [
        "docker", "run", "--rm", "--user", user, "-e", "HOME=/tmp",
        "-e", "CCACHE_DIR=/cache/ccache", "-e", "CCACHE_BASEDIR=/tmp/w",
        "-v", f"{ROOT}:/src:ro", "-v", f"{BUILDS}:/out", "-v", f"{cache}:/cache",
        image, "python3", "/src/build.py", "--in-container", "--mode", mode, "--version", args.version, "--owner", user,
    ]
    for flag, enabled in (("--test", args.test), ("--python-tests", args.python_tests), ("--sim-tests", args.sim_tests)):
        if enabled:
            command.append(flag)
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
