#!/usr/bin/env bash
set -euo pipefail

# concat_py_src.sh
# Concatenate Python host sources into a single generated dump file.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_DIR="$ROOT_DIR/doc/generated"
DEST_FILE="$DEST_DIR/rpi_python_src_full_dump.txt"

SEARCH_DIRS=(
  "$ROOT_DIR/src/rpi"
  "$ROOT_DIR/src/wendy"
)

mkdir -p "$DEST_DIR"
rm -f "$DEST_FILE"

while IFS= read -r f; do
  [[ -f "$f" ]] || continue
  printf -- '--- FILE: %s ---\n' "$f" >> "$DEST_FILE"
  cat "$f" >> "$DEST_FILE"
  printf -- '\n--- END FILE ---\n\n' >> "$DEST_FILE"
done < <(
  find "${SEARCH_DIRS[@]}" \
    -type f \
    -name '*.py' \
    ! -path '*/__pycache__/*' \
    2>/dev/null | sort
)

echo "Wrote $DEST_FILE with concatenated Python host sources."
