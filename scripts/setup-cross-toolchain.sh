#!/usr/bin/env bash
# One-time host setup for aarch64 cross-compilation.
# Run once; idempotent (safe to re-run).
# Requires sudo. Tested on Ubuntu 24.10+ / 25.x (has python3.13 in repos).
set -euo pipefail

CODENAME=$(lsb_release -cs 2>/dev/null || grep VERSION_CODENAME /etc/os-release | cut -d= -f2)

echo "▶ Setting up aarch64 cross-compilation toolchain (Ubuntu $CODENAME)..."

# ---- Cross-compiler ----
echo "• Installing aarch64-linux-gnu cross-compiler..."
sudo apt-get install -y --no-install-recommends \
  gcc-aarch64-linux-gnu \
  g++-aarch64-linux-gnu \
  binutils-aarch64-linux-gnu

# ---- ARM64 multiarch APT source ----
# Ubuntu ARM64 packages are served by ports.ubuntu.com (not archive.ubuntu.com).
# We add a dedicated deb822 source for arm64, and pin the existing main source to
# amd64 only to avoid 404 errors on the AT mirror.
echo "• Configuring apt for ARM64 packages (ports.ubuntu.com)..."

# Restrict main ubuntu.sources to amd64 if not done already
if ! grep -q "^Architectures: amd64" /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null; then
  sudo sed -i '/^Types: deb/a Architectures: amd64' /etc/apt/sources.list.d/ubuntu.sources
fi

PORTS_FILE=/etc/apt/sources.list.d/ubuntu-ports-arm64.sources
if [[ ! -f "$PORTS_FILE" ]]; then
  sudo tee "$PORTS_FILE" > /dev/null <<EOF
# ARM64 packages for cross-compilation — ports.ubuntu.com serves non-x86 arches
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: ${CODENAME} ${CODENAME}-updates ${CODENAME}-security ${CODENAME}-backports
Components: main restricted universe multiverse
Architectures: arm64
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
  echo "  Created $PORTS_FILE"
fi

echo "• Registering arm64 as a foreign architecture..."
sudo dpkg --add-architecture arm64
sudo apt-get update -qq

# ---- ARM64 Python headers (via manual deb extraction) ----
# We only need the *headers*, not the ARM64 Python runtime.
# Installing python3.13-minimal:arm64 would fail because its postinst script
# tries to execute the ARM64 binary (requires binfmt/QEMU).
# So we download libpython3.13-dev:arm64 and extract only the headers.
# ---- ARM64 GLib2 (needed by LCM which raccoon-transport builds from source) ----
echo "• Installing ARM64 GLib2 dev libraries (required by LCM)..."
sudo apt-get install -y --no-install-recommends libglib2.0-dev:arm64

echo "• Extracting ARM64 Python 3.13 dev headers from libpython3.13-dev:arm64..."
PYCONFIG=/usr/include/aarch64-linux-gnu/python3.13/pyconfig.h
if [[ -f "$PYCONFIG" ]]; then
  echo "  Already present: $PYCONFIG"
else
  TMPDIR=$(mktemp -d)
  trap 'rm -rf "$TMPDIR"' EXIT

  (cd "$TMPDIR" && apt-get download libpython3.13-dev:arm64)
  dpkg-deb --extract "$TMPDIR"/libpython3.13-dev*.deb "$TMPDIR/extracted"
  sudo cp -r "$TMPDIR/extracted/usr/include/aarch64-linux-gnu" /usr/include/
  echo "  Installed: $PYCONFIG"
fi

# ---- Optional: ccache ----
if ! command -v ccache >/dev/null 2>&1; then
  echo "• Installing ccache (speeds up incremental cross builds)..."
  sudo apt-get install -y --no-install-recommends ccache
fi

echo ""
echo "✓ Cross-toolchain setup complete."
echo ""
echo "Verification:"
aarch64-linux-gnu-g++ --version | head -1
ls "$PYCONFIG" && echo "ARM64 pyconfig.h: OK"
echo ""
echo "Run: bash build-cross.sh"
