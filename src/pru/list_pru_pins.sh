#!/usr/bin/env bash
# list_pru_pins.sh — enumerate P8/P9 header pins and query supported modes
# Usage: sudo ./list_pru_pins.sh
set -euo pipefail

CONFIG_PIN=$(command -v config-pin || true)

# build P8_1..P8_46 and P9_1..P9_31
PINS=()
for i in $(seq 1 46); do
  PINS+=("P8_$i")
done
for i in $(seq 1 31); do
  PINS+=("P9_$i")
done

if [ -n "$CONFIG_PIN" ]; then
  echo "Using config-pin at: $CONFIG_PIN"
  echo
  for pin in "${PINS[@]}"; do
    # Skip pins that are not present in the kernel (config-pin will return non-zero)
    echo "=== $pin ==="
    if $CONFIG_PIN -l "$pin" >/dev/null 2>&1; then
      echo "  supported modes:"
      $CONFIG_PIN -l "$pin" | sed 's/^/    /'
    else
      echo "  supported modes: (unknown or not supported by config-pin)"
    fi
    if $CONFIG_PIN -q "$pin" >/dev/null 2>&1; then
      CUR=$($CONFIG_PIN -q "$pin")
      echo "  current mode:  $CUR"
    fi
    echo
  done
  exit 0
fi

# fallback: no config-pin available
cat <<EOF
config-pin not found in PATH. This script prefers using 'config-pin' to list
pin capabilities and current mode.

Install the bb.org utilities (e.g. on Debian/Ubuntu: 'sudo apt install -y bb-cape-overlays' or the package that provides 'config-pin'),
or run the following manual checks on the BeagleBone:

  1) Check pinctrl debug info (may require debugfs):
     sudo cat /sys/kernel/debug/pinctrl/*/pins | grep -E "P8_|P9_" -n || true

  2) Use 'devmem' or read device-tree overlays under /sys/devices/platform/ocp/

Because the kernel layout varies, install 'config-pin' for a reliable listing.
EOF
