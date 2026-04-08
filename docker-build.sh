#!/usr/bin/env bash
set -euo pipefail

# One-click Docker build wrapper
# Usage: ./docker-build.sh [IMAGE] [OUTLOG]
# Default IMAGE: antnic/bbb-crosscompile:debian12
# Default OUTLOG: docker-make.log

IMG=${1:-antnic/bbb-crosscompile:debian12}
OUTLOG=${2:-docker-make.log}

ROOT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$ROOT_DIR"

echo "Ensuring build directory exists: $ROOT_DIR/build"
mkdir -p build

echo "Running docker build (image=${IMG})"
tools/docker-build.sh "$IMG" "$OUTLOG"

echo "Fixing ownership on build/"
chown -R "$(id -u):$(id -g)" build || true

echo "Build complete. Log: $OUTLOG"
