#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STAGE_DIR="$SCRIPT_DIR"
INSTALL_MODE=0
ENABLE_SERVICE=0
ENABLE_BOOT_OVERLAY=0
START_NOW=0

FW0="am335x-pru0-fw"
FW1="am335x-pru1-fw"
DTBO="pickup-winder-p8-steppers.dtbo"
PRU_SERVICE="pickupwinder-pru.service"
DAEMON_SERVICE="pickupwinder-daemon.service"
DAEMON_BIN="pickup_daemon"
INSTALL_BIN_DIR="/usr/local/bin"
INSTALL_FW_DIR="/lib/firmware"
INSTALL_SERVICE_DIR="/etc/systemd/system"

info()  { echo "[INFO] $*"; }
warn()  { echo "[WARN] $*"; }
error() { echo "[FAIL] $*" >&2; exit 1; }

require_root() {
    [[ ${EUID:-$(id -u)} -eq 0 ]] || error "This script must run as root."
}

require_file() {
    local path="$1"
    [[ -f "$path" ]] || error "Required file not found: $path"
}

install_file() {
    local src="$1" dst="$2" mode="$3"
    install -m "$mode" "$src" "$dst"
}

ensure_uenv_overlay() {
    local uenv="/boot/uEnv.txt"
    [[ -f "$uenv" ]] || { warn "$uenv not found; skipping boot overlay registration."; return 0; }

    if grep -Fq "$DTBO" "$uenv"; then
        info "Boot overlay already registered in $uenv"
        return 0
    fi

    if ! grep -Eq '^enable_uboot_overlays=1$' "$uenv"; then
        echo "enable_uboot_overlays=1" >> "$uenv"
    fi

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
    require_file "$STAGE_DIR/$PRU_SERVICE"
    require_file "$STAGE_DIR/$DAEMON_SERVICE"
    require_file "$STAGE_DIR/$DAEMON_BIN"
    require_file "$STAGE_DIR/load_pru.sh"

    info "Installing PRU firmware"
    install_file "$STAGE_DIR/$FW0" "$INSTALL_FW_DIR/$FW0" 0644
    install_file "$STAGE_DIR/$FW1" "$INSTALL_FW_DIR/$FW1" 0644
    install_file "$STAGE_DIR/$DTBO" "$INSTALL_FW_DIR/$DTBO" 0644

    info "Installing helper script and services"
    install_file "$STAGE_DIR/load_pru.sh" "$INSTALL_BIN_DIR/load_pru.sh" 0755
    install_file "$STAGE_DIR/$PRU_SERVICE" "$INSTALL_SERVICE_DIR/$PRU_SERVICE" 0644
    install_file "$STAGE_DIR/$DAEMON_SERVICE" "$INSTALL_SERVICE_DIR/$DAEMON_SERVICE" 0644
    install_file "$STAGE_DIR/$DAEMON_BIN" "$INSTALL_BIN_DIR/$DAEMON_BIN" 0755

    if [[ $ENABLE_SERVICE -eq 1 ]]; then
        systemctl daemon-reload
        systemctl enable "$PRU_SERVICE"
        systemctl enable "$DAEMON_SERVICE"
        info "Enabled systemd services: $PRU_SERVICE, $DAEMON_SERVICE"
    fi

    if [[ $ENABLE_BOOT_OVERLAY -eq 1 ]]; then
        ensure_uenv_overlay
    fi
}

usage() {
    cat <<'USAGE'
Usage:
  sudo ./install.sh --from-dir DIR --install [--enable-service] [--enable-boot-overlay] [--start-now]

Options:
  --from-dir DIR             Directory containing firmware, services, helper scripts, and daemon binary
  --install                  Install runtime files on target
  --enable-service           Enable PRU and daemon systemd services
  --enable-boot-overlay      Register the PRU DTBO in /boot/uEnv.txt
  --start-now                Start both systemd services after install
  --help                     Show this message
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --from-dir)
            [[ $# -ge 2 ]] || error "--from-dir requires a directory"
            STAGE_DIR="$2"
            shift 2
            ;;
        --install)
            INSTALL_MODE=1
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
        --start-now)
            START_NOW=1
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

if [[ $INSTALL_MODE -eq 1 ]]; then
    install_payload
fi

if [[ $ENABLE_SERVICE -eq 1 ]]; then
    systemctl daemon-reload
fi

if [[ $START_NOW -eq 1 ]]; then
    if [[ $ENABLE_SERVICE -eq 1 ]]; then
        systemctl start "pickupwinder-pru.service"
        systemctl start "pickupwinder-daemon.service"
    else
        warn "--start-now has no effect without --enable-service"
    fi
fi

if [[ $INSTALL_MODE -eq 0 && $START_NOW -eq 0 ]]; then
    usage
fi
