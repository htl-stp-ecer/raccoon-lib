#!/usr/bin/env bash
set -euo pipefail

# deploy.sh — Build ARM64 wheel then deploy to Pi
# Usage: RPI_HOST=10.x.x.x bash deploy.sh

BUILD_DIR="${BUILD_DIR:-build-docker}"

# Build
echo "▶ Building wheel..."
bash ./build.sh

# Copy wheel + install.sh into build dir for self-contained deployment
WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f | head -1)
if [[ ! -f "$WHEEL_FILE" ]]; then
  echo "Error: No wheel found in $BUILD_DIR after build"
  exit 1
fi

cp install.sh "$BUILD_DIR/install.sh"

# Deploy
echo "▶ Deploying to Pi..."
bash "$BUILD_DIR/install.sh"
