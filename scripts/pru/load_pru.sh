#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
STAGE_DIR="$SCRIPT_DIR"
INSTALL_MODE=0
START_NOW=0
START_ONLY=0
STOP_ONLY=0
ENABLE_SERVICE=0
ENABLE_BOOT_OVERLAY=0
SKIP_PINMUX=0
RPROC0=""
RPROC1=""
WAIT_FOR_RPROC_SECS=20
FW0="am335x-pru0-fw"
FW1="am335x-pru1-fw"
DTBO="pickup-winder-p8-steppers.dtbo"
SERVICE_NAME="pickupwinder-pru.service"
INSTALL_BIN_DIR="/usr/local/bin"
INSTALL_FW_DIR="/lib/firmware"
TEST_HELPER_NAME="test_pru0_motor.sh"
SPIN_TEST_NAME="pru_spin_test.py"

info()  { echo "[INFO] $*"; }
warn()  { echo "[WARN] $*"; }
error() { echo "[FAIL] $*" >&2; exit 1; }

discover_remoteprocs() {
    local deadline=$((SECONDS + WAIT_FOR_RPROC_SECS))
    local node name
    local pru_nodes=()

    while (( SECONDS <= deadline )); do
        RPROC0=""
        RPROC1=""
        pru_nodes=()

        for node in /sys/class/remoteproc/remoteproc*; do
            [[ -d "$node" ]] || continue
            name=$(cat "$node/name" 2>/dev/null || true)

            case "$name" in
                *pru0*|*4a334000.pru*)
                    RPROC0="$node"
                    ;;
                *pru1*|*4a338000.pru*)
                    RPROC1="$node"
                    ;;
            esac

            case "$name" in
                *pru*|*4a334000.pru*|*4a338000.pru*)
                    pru_nodes+=("$node")
                    ;;
            esac
        done

        if [[ -z "$RPROC0" || -z "$RPROC1" ]]; then
            if [[ ${#pru_nodes[@]} -ge 2 ]]; then
                IFS=$'\n' pru_nodes=($(printf '%s\n' "${pru_nodes[@]}" | sort -V))
                unset IFS
                [[ -n "$RPROC0" ]] || RPROC0="${pru_nodes[0]}"
                [[ -n "$RPROC1" ]] || RPROC1="${pru_nodes[1]}"
            fi
        fi

        if [[ -n "$RPROC0" && -n "$RPROC1" ]]; then
            info "Detected PRU remoteprocs: PRU0=$RPROC0 PRU1=$RPROC1"
            return 0
        fi

        sleep 1
    done

    error "PRU remoteproc nodes not found after ${WAIT_FOR_RPROC_SECS}s"
}

usage() {
    cat <<'EOF'
Usage:
  sudo ./load_pru.sh --from-dir DIR --install --enable-service --enable-boot-overlay --start-now
  sudo ./load_pru.sh --start-only
  sudo ./load_pru.sh --stop-only

Options:
  --from-dir DIR             Directory containing am335x-pru*.fw, .dtbo and helper scripts
  --install                  Install firmware, overlay and helper scripts on target
  --start-now                Start PRU firmware immediately after install
  --start-only               Only configure pins and (re)start PRUs
  --stop-only                Stop PRUs only
  --enable-service           Install and enable systemd service
  --enable-boot-overlay      Add DT overlay entry to /boot/uEnv.txt when possible
  --skip-pinmux              Skip runtime config-pin setup during start
  --help                     Show this message
EOF
}

require_root() {
    [[ ${EUID:-$(id -u)} -eq 0 ]] || error "This script must run as root."
}

require_file() {
    local path="$1"
    [[ -f "$path" ]] || error "Required file not found: $path"
}

copy_if_present() {
    local src="$1"
    local dst="$2"
    if [[ -f "$src" ]]; then
        install -m 0755 "$src" "$dst"
    else
        return 1
    fi
}

set_runtime_pinmux() {
    if [[ $SKIP_PINMUX -eq 1 ]]; then
        warn "Skipping runtime pinmux as requested."
        return 0
    fi
    if ! command -v config-pin >/dev/null 2>&1; then
        warn "config-pin not available; assuming overlay or boot pinmux already configured."
        return 0
    fi

    info "Applying runtime pinmux for canonical PRU0 motor map"
    local out_pins=(P9_25 P9_29 P9_31 P9_41 P9_28 P9_30)
    local in_pins=(P8_15 P8_16)
    local pin

    for pin in "${out_pins[@]}"; do
        config-pin "$pin" pruout >/dev/null
    done
    for pin in "${in_pins[@]}"; do
        config-pin "$pin" pruin >/dev/null
    done
}

stop_prus() {
    discover_remoteprocs
    local node
    for node in "$RPROC0" "$RPROC1"; do
        if [[ -d "$node" ]]; then
            local state
            state=$(cat "$node/state" 2>/dev/null || true)
            if [[ "$state" == "running" ]]; then
                info "Stopping $(basename "$node")"
                echo stop > "$node/state"
            fi
        fi
    done
}

start_prus() {
    discover_remoteprocs

    set_runtime_pinmux
    stop_prus

    info "Assigning firmware to PRU0 and PRU1"
    echo "$FW0" > "$RPROC0/firmware"
    echo "$FW1" > "$RPROC1/firmware"

    info "Starting PRU0"
    echo start > "$RPROC0/state"
    info "Starting PRU1"
    echo start > "$RPROC1/state"

    sleep 1
    info "PRU0 state: $(cat "$RPROC0/state")"
    info "PRU1 state: $(cat "$RPROC1/state")"
}

ensure_uenv_overlay() {
    local uenv="/boot/uEnv.txt"
    [[ -f "$uenv" ]] || { warn "$uenv not found; skipping boot overlay registration."; return 0; }

    if grep -Fq "$DTBO" "$uenv"; then
        info "Boot overlay already configured in $uenv"
        return 0
    fi

    if ! grep -Eq '^enable_uboot_overlays=1$' "$uenv"; then
        echo "enable_uboot_overlays=1" >> "$uenv"
    fi

    local slot
    for slot in 4 5 6 7; do
        if ! grep -Eq "^uboot_overlay_addr${slot}=" "$uenv"; then
            echo "uboot_overlay_addr${slot}=/lib/firmware/${DTBO}" >> "$uenv"
            info "Registered boot overlay in $uenv via uboot_overlay_addr${slot}"
            return 0
        fi
    done

    warn "No free uboot_overlay_addr slot available in $uenv; overlay copied but not auto-enabled."
}

install_payload() {
    require_file "$STAGE_DIR/$FW0"
    require_file "$STAGE_DIR/$FW1"
    require_file "$STAGE_DIR/$DTBO"
    require_file "$STAGE_DIR/$TEST_HELPER_NAME"
    require_file "$STAGE_DIR/load_pru.sh"
    require_file "$STAGE_DIR/$SPIN_TEST_NAME"

    info "Installing firmware and overlay"
    install -m 0644 "$STAGE_DIR/$FW0" "$INSTALL_FW_DIR/$FW0"
    install -m 0644 "$STAGE_DIR/$FW1" "$INSTALL_FW_DIR/$FW1"
    install -m 0644 "$STAGE_DIR/$DTBO" "$INSTALL_FW_DIR/$DTBO"

    info "Installing helper scripts"
    install -m 0755 "$STAGE_DIR/$TEST_HELPER_NAME" "$INSTALL_BIN_DIR/$TEST_HELPER_NAME"
    install -m 0755 "$STAGE_DIR/load_pru.sh" "$INSTALL_BIN_DIR/load_pru.sh"
    install -m 0755 "$STAGE_DIR/$SPIN_TEST_NAME" "$INSTALL_BIN_DIR/$SPIN_TEST_NAME"

    if [[ -f "$STAGE_DIR/$SERVICE_NAME" ]]; then
        install -m 0644 "$STAGE_DIR/$SERVICE_NAME" "/etc/systemd/system/$SERVICE_NAME"
    fi

    if [[ $ENABLE_BOOT_OVERLAY -eq 1 ]]; then
        ensure_uenv_overlay
    fi

    if [[ $ENABLE_SERVICE -eq 1 ]]; then
        [[ -f "/etc/systemd/system/$SERVICE_NAME" ]] || error "Service file not installed: /etc/systemd/system/$SERVICE_NAME"
        systemctl daemon-reload
        systemctl enable "$SERVICE_NAME"
        info "Enabled systemd service: $SERVICE_NAME"
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --from-dir)
            [[ $# -ge 2 ]] || error "--from-dir requires a directory argument"
            STAGE_DIR="$2"
            shift 2
            ;;
        --install)
            INSTALL_MODE=1
            shift
            ;;
        --start-now)
            START_NOW=1
            shift
            ;;
        --start-only)
            START_ONLY=1
            shift
            ;;
        --stop-only)
            STOP_ONLY=1
            shift
            ;;
        --enable-service)
            ENABLE_SERVICE=1
            shift
            ;;
        --enable-boot-overlay)
            ENABLE_BOOT_OVERLAY=1
            shift
            ;;
        --skip-pinmux)
            SKIP_PINMUX=1
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            error "Unknown option: $1"
            ;;
    esac
done

require_root

if [[ $STOP_ONLY -eq 1 ]]; then
    stop_prus
    exit 0
fi

if [[ $START_ONLY -eq 1 ]]; then
    start_prus
    exit 0
fi

if [[ $INSTALL_MODE -eq 1 ]]; then
    install_payload
fi

if [[ $START_NOW -eq 1 ]]; then
    start_prus
fi

if [[ $INSTALL_MODE -eq 0 && $START_NOW -eq 0 ]]; then
    usage
fi
