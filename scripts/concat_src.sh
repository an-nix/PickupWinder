#!/usr/bin/env bash
set -euo pipefail

# concat_src.sh
# Concatenate selected ESP32 source files into a single generated dump file.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST_DIR="$ROOT_DIR/doc/generated"
DEST_FILE="$DEST_DIR/esp32_src_full_dump.txt"

FILES=(
  "$ROOT_DIR/src/esp32/src/step_types.h"
  "$ROOT_DIR/src/esp32/src/stepper_queue.h"
  "$ROOT_DIR/src/esp32/src/stepper_queue.cpp"
  "$ROOT_DIR/src/esp32/src/stepper_driver.h"
  "$ROOT_DIR/src/esp32/src/stepper_driver.cpp"
  "$ROOT_DIR/src/esp32/src/motion_planner.h"
  "$ROOT_DIR/src/esp32/src/motion_planner.cpp"
  "$ROOT_DIR/src/esp32/src/messages.h"
  "$ROOT_DIR/src/esp32/src/main.cpp"
  "$ROOT_DIR/src/esp32/src/comm_interface.h"
  "$ROOT_DIR/src/esp32/src/comm_interface.cpp"
  "$ROOT_DIR/src/esp32/CMakeLists.txt"
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
