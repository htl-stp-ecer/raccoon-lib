#!/usr/bin/env bash
set -euo pipefail

# install.sh — Deploy pre-built wheel to Raspberry Pi
# Usage: RPI_HOST=10.x.x.x bash install.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="${PROJECT_NAME:-libstp}"
PYTHON_CMD="${PYTHON_CMD:-python3}"

RPI_USER="${RPI_USER:-pi}"
if [[ -z "${RPI_HOST:-}" ]]; then
  echo "Error: RPI_HOST is required."
  echo "Usage: RPI_HOST=10.x.x.x bash install.sh"
  exit 1
fi

# Find wheel file relative to this script
WHEEL_FILE=$(find "$SCRIPT_DIR" -maxdepth 1 -name "*.whl" -type f | head -1)
if [[ -z "$WHEEL_FILE" || ! -f "$WHEEL_FILE" ]]; then
  echo "Error: No .whl file found in $SCRIPT_DIR"
  exit 1
fi

WHEEL_BASENAME=$(basename "$WHEEL_FILE")
echo "▶ Deploying $WHEEL_BASENAME to ${RPI_USER}@${RPI_HOST}"

# Preflight: test SSH
echo "• Testing SSH connection..."
if ! ssh -o ConnectTimeout=5 "${RPI_USER}@${RPI_HOST}" "echo 'SSH OK'" >/dev/null 2>&1; then
  echo "Error: Cannot connect to ${RPI_USER}@${RPI_HOST}"
  echo "  Check: SSH enabled, host reachable, keys configured"
  exit 1
fi

# Copy wheel
echo "• Copying wheel..."
scp "$WHEEL_FILE" "${RPI_USER}@${RPI_HOST}:/tmp/${WHEEL_BASENAME}"

# Install
echo "• Installing package..."
ssh "${RPI_USER}@${RPI_HOST}" "${PYTHON_CMD} -m pip install --user --force-reinstall '/tmp/${WHEEL_BASENAME}' --break-system-packages"

# Verify
echo "• Verifying import..."
if ssh "${RPI_USER}@${RPI_HOST}" "${PYTHON_CMD} -c 'import ${PROJECT_NAME}; print(\"${PROJECT_NAME} OK\")'"; then
  echo "✓ ${PROJECT_NAME} deployed successfully to ${RPI_USER}@${RPI_HOST}"
else
  echo "⚠ Package installed but import test failed"
  exit 1
fi
