#!/usr/bin/env bash
set -euo pipefail

# deploy.sh — Build ARM64 wheel then deploy to Pi
# Usage: RPI_HOST=10.x.x.x bash deploy.sh

BUILD_DIR="${BUILD_DIR:-build-docker}"

# Build
echo "▶ Building wheel..."
bash ./build.sh

# Find the built wheel
WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f | head -1)
if [[ ! -f "$WHEEL_FILE" ]]; then
  echo "Error: No wheel found in $BUILD_DIR after build"
  exit 1
fi

# Stage wheel + install.sh in a temp dir (build dir is root-owned from Docker)
STAGE_DIR=$(mktemp -d)
trap 'rm -rf "$STAGE_DIR"' EXIT
cp "$WHEEL_FILE" "$STAGE_DIR/"
cp install.sh "$STAGE_DIR/install.sh"

# Deploy
echo "▶ Deploying to Pi..."
bash "$STAGE_DIR/install.sh"
