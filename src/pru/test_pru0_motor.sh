#!/usr/bin/env bash
# test_pru0_motor.sh — Quick pin/PRU test for PRU0 motor firmware (P9/P8 map).
#
# Run on the BeagleBone Black AFTER copying am335x-pru0-fw to /lib/firmware/.
#
# What this script does:
#   1. Optionally set motor pins to 'pruout' and endstops to 'pruin' via config-pin
#      (skipped when DTBO is already loaded at boot or if --skip-pinmux is used)
#   2. Stop + reload PRU0 with the motor firmware
#   3. Print PRU0 remoteproc state
#
# Usage:
#   sudo ./test_pru0_motor.sh
#   sudo ./test_pru0_motor.sh --skip-pinmux
#
# To restore default (gpio) after testing:
#   sudo ./test_pru0_motor.sh --restore
#
# Pin map (for reference):
#   P9_29  STEP_A     R30[1]
#   P9_27  DIR_A      R30[5]
#   P9_25  EN_A       R30[7]  (active-low)
#   P9_42  STEP_B     R30[4]
#   P9_31  DIR_B      R30[0]
#   P9_28  EN_B       R30[3]  (active-low)
#   P8_15  ENDSTOP_1  R31[15] (pruin)
#   P8_16  ENDSTOP_2  R31[14] (pruin)
#
# Dependencies:
#   - config-pin is optional (needed only when changing pinmux at runtime)
#   - if DTBO is loaded at boot, script can run without config-pin

set -euo pipefail

# ── Firmware name (must exist in /lib/firmware/) ─────────────────────────────
FW_PRU0="am335x-pru0-fw"

# ── remoteproc sysfs node for PRU0 ───────────────────────────────────────────
# On Debian 10+ kernels: remoteproc1 = PRU0, remoteproc2 = PRU1
# Adjust if your kernel numbers them differently.
RPROC0="/sys/class/remoteproc/remoteproc1"

RESTORE=0
SKIP_PINMUX=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --restore)
            RESTORE=1
            shift
            ;;
        --skip-pinmux)
            SKIP_PINMUX=1
            shift
            ;;
        *)
            error "Unknown option: $1 (use --restore or --skip-pinmux)"
            ;;
    esac
done

# ── Helper ───────────────────────────────────────────────────────────────────
info()  { echo -e "[\e[32m OK \e[0m] $*"; }
warn()  { echo -e "[\e[33mWARN\e[0m] $*"; }
error() { echo -e "[\e[31mFAIL\e[0m] $*" >&2; exit 1; }

require_root() {
    [[ $EUID -eq 0 ]] || error "This script must be run as root (sudo)."
}
require_root

if [[ $RESTORE -eq 1 ]]; then
    echo "Restoring P9/P8 mapping pins to default mode..."
    if ! command -v config-pin >/dev/null 2>&1; then
        warn "config-pin not found; cannot restore pinmux. Skipping."
        exit 0
    fi
    for pin in P9_25 P9_27 P9_28 P9_29 P9_31 P9_42 P8_15 P8_16; do
        config-pin "$pin" default && info "$pin → default" || warn "$pin restore failed"
    done
    exit 0
fi

# ── 1. Set pins to pruout (optional) ──────────────────────────────────────────
echo ""
echo "=== Step 1: Configure canonical PRU0 pinmux (optional) ==="
declare -A PIN_ROLE=(
    [P9_29]="STEP_A (R30[1])"
    [P9_27]="DIR_A  (R30[5])"
    [P9_25]="EN_A   (R30[7])"
    [P9_42]="STEP_B (R30[4])"
    [P9_31]="DIR_B  (R30[0])"
    [P9_28]="EN_B   (R30[3])"
    [P8_15]="ENDSTOP_1 (R31[15])"
    [P8_16]="ENDSTOP_2 (R31[14])"
)

if [[ $SKIP_PINMUX -eq 1 ]]; then
    warn "Skipping runtime pinmux as requested (--skip-pinmux)."
    warn "Assuming DTBO already configures canonical pins at boot."
elif command -v config-pin >/dev/null 2>&1; then
    for pin in P9_25 P9_27 P9_28 P9_29 P9_31 P9_42; do
        if config-pin "$pin" pruout 2>/dev/null; then
            info "$pin → pruout  [${PIN_ROLE[$pin]}]"
        else
            error "$pin: failed to set pruout via config-pin."
        fi
    done
    for pin in P8_15 P8_16; do
        if config-pin "$pin" pruin 2>/dev/null; then
            info "$pin → pruin   [${PIN_ROLE[$pin]}]"
        else
            error "$pin: failed to set pruin via config-pin."
        fi
    done

    # Verify
    echo ""
    echo "=== Pin state after config-pin ==="
    for pin in P9_25 P9_27 P9_28 P9_29 P9_31 P9_42 P8_15 P8_16; do
        state=$(config-pin -q "$pin" 2>/dev/null || echo "unknown")
        echo "  $pin : $state"
    done
else
    warn "config-pin not found; skipping runtime pinmux."
    warn "If DTBO is not loaded at boot, PRU outputs may not be routed to P8 pins."
fi

# ── 2. Check firmware file ────────────────────────────────────────────────────
echo ""
echo "=== Step 2: Check firmware ==="
FW_PATH="/lib/firmware/${FW_PRU0}"
if [[ ! -f "$FW_PATH" ]]; then
    error "Firmware not found: $FW_PATH\n  Build and copy with:\n    make && sudo cp build/${FW_PRU0} /lib/firmware/"
fi
info "Found: $FW_PATH  ($(wc -c < "$FW_PATH") bytes)"

# ── 3. Reload PRU1 ───────────────────────────────────────────────────────────
echo ""
echo "=== Step 3: Reload PRU0 ==="
if [[ ! -d "$RPROC0" ]]; then
    warn "remoteproc node $RPROC0 not found. Trying remoteproc2..."
    RPROC0="/sys/class/remoteproc/remoteproc2"
    [[ -d "$RPROC0" ]] || error "No remoteproc node found. Is the PRU subsystem enabled in the DT?"
fi

STATE=$(cat "${RPROC0}/state" 2>/dev/null || echo "unknown")
echo "  Current state: $STATE"

if [[ "$STATE" == "running" ]]; then
    echo "  Stopping PRU0..."
    echo stop > "${RPROC0}/state"
    sleep 0.5
fi

echo "  Setting firmware: $FW_PRU0"
echo "$FW_PRU0" > "${RPROC0}/firmware"

echo "  Starting PRU0..."
echo start > "${RPROC0}/state"
sleep 0.5

STATE=$(cat "${RPROC0}/state" 2>/dev/null || echo "unknown")
if [[ "$STATE" == "running" ]]; then
    info "PRU0 is running with firmware: $FW_PRU0"
else
    error "PRU0 failed to start. State: $STATE\n  Check kernel log: dmesg | tail -20"
fi

# ── 4. Summary ───────────────────────────────────────────────────────────────
echo ""
echo "==================================================================="
echo " READY — motor firmware is running on PRU0"
echo "==================================================================="
echo ""
echo " Stepper drivers wired to canonical P9/P8 mapping:"
echo "   P9_29  STEP_A  (R30[1])   ─┐"
echo "   P9_27  DIR_A   (R30[5])    ├─ Motor A A4988/DRV8825"
echo "   P9_25  EN_A    (R30[7])   ─┘  (active-low EN)"
echo ""
echo "   P9_42  STEP_B  (R30[4])   ─┐"
echo "   P9_31  DIR_B   (R30[0])    ├─ Motor B A4988/DRV8825"
echo "   P9_28  EN_B    (R30[3])   ─┘  (active-low EN)"
echo ""
echo "   P8_15  ENDSTOP_1 (R31[15])"
echo "   P8_16  ENDSTOP_2 (R31[14])"
echo ""
echo " Send moves via the Linux IPC layer (IpcChannel / StepperPRU)."
echo " To stop the PRU:   echo stop > ${RPROC0}/state"
echo " To restore pins:   sudo $0 --restore"
echo ""
