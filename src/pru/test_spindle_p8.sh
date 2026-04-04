#!/usr/bin/env bash
# test_spindle_p8.sh — Quick pin/PRU test for dual-stepper firmware on P8 header.
#
# Run on the BeagleBone Black AFTER copying am335x-pru1-fw to /lib/firmware/.
#
# What this script does:
#   1. Set P8_41..P8_46 to 'pruout' via config-pin  (no reboot needed)
#   2. Stop + reload PRU1 with the dual-stepper firmware
#   3. Print PRU1 remoteproc state
#
# Usage:
#   sudo ./test_spindle_p8.sh
#
# To restore default (gpio) after testing:
#   sudo ./test_spindle_p8.sh --restore
#
# Pin map (for reference):
#   P8_46  SPINDLE_STEP  R30[1]
#   P8_44  SPINDLE_DIR   R30[3]
#   P8_42  SPINDLE_EN    R30[5]  (active-low)
#   P8_45  LATERAL_STEP  R30[0]
#   P8_43  LATERAL_DIR   R30[2]
#   P8_41  LATERAL_EN    R30[4]  (active-low)
#
# Dependencies: config-pin (bb-cape-overlays / bone-scripts package)
#   sudo apt install bb-cape-overlays

set -euo pipefail

# ── Firmware name (must exist in /lib/firmware/) ─────────────────────────────
FW_PRU1="am335x-pru1-fw"

# ── remoteproc sysfs node for PRU1 ───────────────────────────────────────────
# On Debian 10+ kernels: remoteproc1 = PRU0, remoteproc2 = PRU1
# Adjust if your kernel numbers them differently.
RPROC1="/sys/class/remoteproc/remoteproc2"

RESTORE=0
[[ "${1:-}" == "--restore" ]] && RESTORE=1

# ── Helper ───────────────────────────────────────────────────────────────────
info()  { echo -e "[\e[32m OK \e[0m] $*"; }
warn()  { echo -e "[\e[33mWARN\e[0m] $*"; }
error() { echo -e "[\e[31mFAIL\e[0m] $*" >&2; exit 1; }

require_root() {
    [[ $EUID -eq 0 ]] || error "This script must be run as root (sudo)."
}
require_root

if [[ $RESTORE -eq 1 ]]; then
    echo "Restoring P8_41..P8_46 to default mode..."
    for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46; do
        config-pin "$pin" default && info "$pin → default" || warn "$pin restore failed"
    done
    exit 0
fi

# ── 1. Set pins to pruout ─────────────────────────────────────────────────────
echo ""
echo "=== Step 1: Configure P8_41..P8_46 as pruout ==="
declare -A PIN_ROLE=(
    [P8_46]="SPINDLE_STEP (R30[1])"
    [P8_44]="SPINDLE_DIR  (R30[3])"
    [P8_42]="SPINDLE_EN   (R30[5])"
    [P8_45]="LATERAL_STEP (R30[0])"
    [P8_43]="LATERAL_DIR  (R30[2])"
    [P8_41]="LATERAL_EN   (R30[4])"
)
for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46; do
    if config-pin "$pin" pruout 2>/dev/null; then
        info "$pin → pruout  [${PIN_ROLE[$pin]}]"
    else
        error "$pin: failed to set pruout. Is config-pin installed? (apt install bb-cape-overlays)"
    fi
done

# Verify
echo ""
echo "=== Pin state after config-pin ==="
for pin in P8_41 P8_42 P8_43 P8_44 P8_45 P8_46; do
    state=$(config-pin -q "$pin" 2>/dev/null || echo "unknown")
    echo "  $pin : $state"
done

# ── 2. Check firmware file ────────────────────────────────────────────────────
echo ""
echo "=== Step 2: Check firmware ==="
FW_PATH="/lib/firmware/${FW_PRU1}"
if [[ ! -f "$FW_PATH" ]]; then
    error "Firmware not found: $FW_PATH\n  Build and copy with:\n    make && sudo cp build/${FW_PRU1} /lib/firmware/"
fi
info "Found: $FW_PATH  ($(wc -c < "$FW_PATH") bytes)"

# ── 3. Reload PRU1 ───────────────────────────────────────────────────────────
echo ""
echo "=== Step 3: Reload PRU1 ==="
if [[ ! -d "$RPROC1" ]]; then
    warn "remoteproc node $RPROC1 not found. Trying remoteproc1..."
    RPROC1="/sys/class/remoteproc/remoteproc1"
    [[ -d "$RPROC1" ]] || error "No remoteproc node found. Is the PRU subsystem enabled in the DT?"
fi

STATE=$(cat "${RPROC1}/state" 2>/dev/null || echo "unknown")
echo "  Current state: $STATE"

if [[ "$STATE" == "running" ]]; then
    echo "  Stopping PRU1..."
    echo stop > "${RPROC1}/state"
    sleep 0.5
fi

echo "  Setting firmware: $FW_PRU1"
echo "$FW_PRU1" > "${RPROC1}/firmware"

echo "  Starting PRU1..."
echo start > "${RPROC1}/state"
sleep 0.5

STATE=$(cat "${RPROC1}/state" 2>/dev/null || echo "unknown")
if [[ "$STATE" == "running" ]]; then
    info "PRU1 is running with firmware: $FW_PRU1"
else
    error "PRU1 failed to start. State: $STATE\n  Check kernel log: dmesg | tail -20"
fi

# ── 4. Summary ───────────────────────────────────────────────────────────────
echo ""
echo "==================================================================="
echo " READY — dual-stepper firmware is running on PRU1"
echo "==================================================================="
echo ""
echo " Stepper drivers wired to P8 header:"
echo "   P8_46  SPINDLE STEP  (R30[1])   ─┐"
echo "   P8_44  SPINDLE DIR   (R30[3])    ├─ Spindle A4988/DRV8825"
echo "   P8_42  SPINDLE EN    (R30[5])   ─┘  (active-low EN)"
echo ""
echo "   P8_45  LATERAL STEP  (R30[0])   ─┐"
echo "   P8_43  LATERAL DIR   (R30[2])    ├─ Lateral A4988/DRV8825"
echo "   P8_41  LATERAL EN    (R30[4])   ─┘  (active-low EN)"
echo ""
echo " Send moves via the Linux IPC layer (IpcChannel / StepperPRU)."
echo " To stop the PRU:   echo stop > ${RPROC1}/state"
echo " To restore pins:   sudo $0 --restore"
echo ""
