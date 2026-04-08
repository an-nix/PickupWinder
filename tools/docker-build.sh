#!/usr/bin/env bash
set -euo pipefail

# Usage: tools/docker-build.sh [IMAGE] [OUTLOG] [-- make-args]
# Default IMAGE: antnic/bbb-crosscompile:debian12
# Default OUTLOG: docker-make.log

IMG=${1:-antnic/bbb-crosscompile:debian12}
OUTLOG=${2:-docker-make.log}

# Shift past positional args if provided
if [[ $# -ge 2 ]]; then
  shift 2
fi

MAKE_ARGS="$*"

echo "Running build in image: ${IMG}"
echo "Output log: ${OUTLOG}"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$PWD":/workspace -w /workspace \
  "${IMG}" \
  bash -lc "make -j\"$(nproc)\" ${MAKE_ARGS}" | tee "${OUTLOG}"
