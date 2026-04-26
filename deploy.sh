#!/usr/bin/env bash
set -euo pipefail

# deploy.sh — Build ARM64 wheel then deploy to Pi
# Usage: RPI_HOST=10.x.x.x bash deploy.sh

BUILD_DIR="${BUILD_DIR:-build-docker}"

# Preflight: confirm the Pi is reachable before sinking minutes into the
# Docker cross-build. Ping is best-effort (some networks block ICMP); fall
# back to a TCP probe on port 22 so a firewalled-but-online Pi still passes.
if [[ -z "${RPI_HOST:-}" ]]; then
  echo "Error: RPI_HOST not set"
  exit 1
fi

echo "▶ Pinging $RPI_HOST..."
if ping -c 1 -W 2 "$RPI_HOST" >/dev/null 2>&1; then
  echo "  ✓ ICMP reply"
elif command -v nc >/dev/null 2>&1 && nc -z -w 2 "$RPI_HOST" 22 >/dev/null 2>&1; then
  echo "  ✓ SSH port open (ICMP blocked)"
else
  echo "Error: $RPI_HOST is unreachable (no ICMP, no SSH)."
  echo "  Check: device powered on, on the same network, RPI_HOST correct."
  exit 1
fi

# Build
echo "▶ Building wheel..."
bash ./build.sh

# Find the built wheel
WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f -exec ls -t {} + | head -1)
if [[ ! -f "$WHEEL_FILE" ]]; then
  echo "Error: No wheel found in $BUILD_DIR after build"
  exit 1
fi

# Stage wheel + install.py in a temp dir (build dir is root-owned from Docker)
STAGE_DIR=$(mktemp -d)
trap 'rm -rf "$STAGE_DIR"' EXIT
cp "$WHEEL_FILE" "$STAGE_DIR/"
cp install.py "$STAGE_DIR/install.py"

# Deploy
echo "▶ Deploying to Pi..."
python3 "$STAGE_DIR/install.py"
