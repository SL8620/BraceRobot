#!/usr/bin/env bash
set -euo pipefail

# install_soem.sh
# Installs the locally built SOEM into /usr/local so that CMake find_package(soem CONFIG)
# can locate it. Run this from the repo root or pass the build dir as first arg.

BUILD_DIR="${1:-src/SOEM/build}"

if [ ! -d "$BUILD_DIR" ]; then
  echo "SOEM build directory not found: $BUILD_DIR" >&2
  exit 2
fi

echo "Installing SOEM from: $BUILD_DIR -> /usr/local"
sudo cmake --install "$BUILD_DIR"
echo "Running ldconfig (may require sudo)"
sudo ldconfig
echo "Done. SOEM should now be visible to find_package(soem CONFIG)."
