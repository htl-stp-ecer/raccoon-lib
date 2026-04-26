#!/usr/bin/env bash
# Locate a usable ruff binary and dispatch the requested command.
#
# Discovery order:
#   1. RUFF env var (explicit override; takes any executable path)
#   2. ruff on PATH
#   3. python3 -m ruff (works if `pip install ruff` ran into the env)
#   4. uvx ruff@<pinned version> (zero-install fallback for CI/devs)
#
# The pinned version matches the pre-commit hook in .pre-commit-config.yaml
# so all three gates (CMake build, pre-commit, CI) agree on output.
#
# Usage:
#   bash scripts/run-ruff.sh check ARGS...
#   bash scripts/run-ruff.sh format --check ARGS...
#
# Exit codes pass through from ruff so CMake can fail the build cleanly.
set -euo pipefail

PINNED_VERSION="0.7.3"

if [[ -n "${RUFF:-}" ]] && command -v "${RUFF}" >/dev/null 2>&1; then
    exec "${RUFF}" "$@"
fi

if command -v ruff >/dev/null 2>&1; then
    exec ruff "$@"
fi

if python3 -c "import ruff" >/dev/null 2>&1; then
    exec python3 -m ruff "$@"
fi

if command -v uvx >/dev/null 2>&1; then
    # Quiet the "Downloaded ruff" + "Installed 1 package" preamble so it
    # doesn't pollute Ninja's terse build log on first run.
    exec uvx --quiet "ruff@${PINNED_VERSION}" "$@"
fi

cat >&2 <<EOF
ERROR: ruff not found.

Install one of:
  pip install ruff==${PINNED_VERSION}
  uv tool install ruff
  apt install python3-ruff   # if your distro packages it

Or set RUFF=/path/to/ruff to point at an existing binary.
EOF
exit 1
