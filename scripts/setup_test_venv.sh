#!/usr/bin/env bash
# scripts/setup_test_venv.sh — build a local test environment in which the FULL
# Python suite runs (async + mock sim) and mutation testing works.
#
# Why this is needed:
#   - A normal local/dev wheel is built for the runtime-switchable platform
#     (mock+wombat bundles dlopen'd at import). In that mode raccoon.sim has no
#     `.mock` submodule, so the in-house `raccoon_testing` pytest plugin and all
#     sim/async tests refuse to load. We must build with DRIVER_BUNDLE=mock AND
#     LIBSTP_RUNTIME_PLATFORM=OFF (the latter is what CI's test job gets for free
#     because LIBSTP_BUILD_TESTS=ON forces runtime-platform off).
#   - The scikit-build install copies the python files into site-packages, so
#     editing source (or mutating it) would not be seen by tests. We replace
#     those copies with SYMLINKS to the source tree so coverage and mutmut both
#     operate on real source paths, and tests that load source via
#     importlib.spec_from_file_location see mutations too.
#
# Re-run this whenever the C++ changes (rebuilds the extensions). Pure-Python
# edits need no rebuild thanks to the symlinks.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

VENV="${TEST_VENV:-$REPO_ROOT/.venv-test}"
PYVER="${PYVER:-3.12}"

echo "==> creating $VENV (python $PYVER)"
uv venv "$VENV" --python "$PYVER"

echo "==> installing test + mutation deps"
uv pip install --python "$VENV/bin/python" \
  pytest pytest-asyncio "coverage[toml]" pytest-cov numpy "mutmut==2.4.4"

echo "==> building + installing raccoon (DRIVER_BUNDLE=mock, RUNTIME_PLATFORM=OFF)"
uv pip install --python "$VENV/bin/python" . --reinstall-package raccoon-library \
  --config-settings=cmake.define.DRIVER_BUNDLE=mock \
  --config-settings=cmake.define.LIBSTP_RUNTIME_PLATFORM=OFF \
  --config-settings=build-dir=build-mock-test \
  --config-settings=cmake.define.FETCHCONTENT_BASE_DIR="$REPO_ROOT/.cmake-cache"

echo "==> symlinking installed *.py back onto the source tree"
"$VENV/bin/python" - <<'PY'
import os, pathlib
repo = pathlib.Path(".").resolve()
sp = next(repo.glob(".venv-test/lib/python*/site-packages/raccoon"))
roots = [repo / "python"] + sorted(repo.glob("modules/libstp-*/python"))
n = 0
for f in sp.rglob("*.py"):
    rel = f.relative_to(sp.parent)
    src = next((r / rel for r in roots if (r / rel).exists()), None)
    if src and not f.is_symlink():
        f.unlink()
        os.symlink(src, f)
        n += 1
print(f"   symlinked {n} files")
PY

echo "==> verifying the mock sim + a normal import work"
"$VENV/bin/python" -c "import raccoon.testing.sim; from raccoon.step.motion._motion_trim import MotionTrimService; print('TEST VENV OK')"

cat <<EOF

Test venv ready at $VENV
  full suite + coverage :  PYTHON=$VENV/bin/python bash scripts/coverage.sh --python-only
  mutation testing      :  bash scripts/mutation_py.sh <source.py> <test target>
EOF
