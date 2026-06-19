#!/usr/bin/env bash
# Build-free test loop for the path-optimizer worktree.
#
# The venv was copied from the main checkout; its site-packages holds COPIES of
# the python sources (plus the compiled .so files and generated *_dsl.py). This
# script syncs the worktree python sources on top of those copies, then runs
# pytest. No C++ rebuild needed (the .so files are unchanged).
#
# NOTE: never use `rsync --delete` here — it would wipe the generated *_dsl.py
# and .so files that don't exist in the module source trees.
set -euo pipefail
WT="$(cd "$(dirname "$0")" && pwd)"
SP="$WT/.venv/lib/python3.12/site-packages/raccoon"
for m in libstp-step libstp-motion libstp-timing libstp-servo libstp-hal libstp-platforms libstp-mission libstp-debug; do
  src="$WT/modules/$m/python/raccoon/"
  [ -d "$src" ] && rsync -a "$src" "$SP/"
done
exec "$WT/.venv/bin/python" -m pytest "$@" \
  -p no:cacheprovider -p no:raccoon_testing \
  -W ignore::pytest.PytestAssertRewriteWarning
