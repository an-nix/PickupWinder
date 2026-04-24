#!/usr/bin/env bash
set -euo pipefail

# concat_src.sh
# Concatenate selected ESP32 source files into a single generated dump file.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_DIR="$ROOT_DIR/doc/generated"
DEST_FILE="$DEST_DIR/esp32_src_full_dump.txt"

ESP32_DIR="$ROOT_DIR/src/esp32"
ESP32_SRC_DIR="$ESP32_DIR/src"

FILES=(
  "$ESP32_DIR/CMakeLists.txt"
  "$ESP32_SRC_DIR/CMakeLists.txt"
  "$ESP32_SRC_DIR/main.cpp"
)

while IFS= read -r f; do
  FILES+=("$f")
done < <(
  find "$ESP32_SRC_DIR/comm" "$ESP32_SRC_DIR/motion" \
    -type f \( -name '*.h' -o -name '*.cpp' \) 2>/dev/null | sort
)

mkdir -p "$DEST_DIR"
rm -f "$DEST_FILE"

for f in "${FILES[@]}"; do
  if [[ ! -f "$f" ]]; then
    echo "Warning: file not found: $f" >&2
    continue
  fi
  printf -- '--- FILE: %s ---\n' "$f" >> "$DEST_FILE"
  cat "$f" >> "$DEST_FILE"
  printf -- '\n--- END FILE ---\n\n' >> "$DEST_FILE"
done

echo "Wrote $DEST_FILE with the full raw concatenated source files."
