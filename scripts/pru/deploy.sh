#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
TARGET_USER="debian"
TARGET_HOST=""
TARGET_STAGE_DIR="/tmp/pickupwinder-pru-deploy"
SSH_KEY=""
SSH_OPTS=( -o BatchMode=yes -o StrictHostKeyChecking=accept-new )
DO_BUILD=1
ENABLE_SERVICE=1
ENABLE_BOOT_OVERLAY=1
START_NOW=1

info()  { echo "[INFO] $*"; }
error() { echo "[FAIL] $*" >&2; exit 1; }

usage() {
    cat <<'USAGE'
Usage:
  ./deploy.sh --host HOST [options]

Options:
  --host HOST               Target BeagleBone host or IP
  --user USER               SSH user (default: debian)
  --ssh-key PATH            SSH private key
  --stage-dir DIR           Remote staging directory (default: /tmp/pickupwinder-pru-deploy)
  --no-build                Skip local build of PRU firmware, DTBO and daemon
  --no-service              Do not enable systemd services on target
  --no-boot-overlay         Do not register the PRU DTBO in /boot/uEnv.txt
  --no-start                Do not start services after install
  --help                   Show this message
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)
            [[ $# -ge 2 ]] || error "--host requires a value"
            TARGET_HOST="$2"
            shift 2
            ;;
        --user)
            [[ $# -ge 2 ]] || error "--user requires a value"
            TARGET_USER="$2"
            shift 2
            ;;
        --ssh-key)
            [[ $# -ge 2 ]] || error "--ssh-key requires a value"
            SSH_KEY="$2"
            shift 2
            ;;
        --stage-dir)
            [[ $# -ge 2 ]] || error "--stage-dir requires a value"
            TARGET_STAGE_DIR="$2"
            shift 2
            ;;
        --no-build)
            DO_BUILD=0
            shift
            ;;
        --no-service)
            ENABLE_SERVICE=0
            shift
            ;;
        --no-boot-overlay)
            ENABLE_BOOT_OVERLAY=0
            shift
            ;;
        --no-start)
            START_NOW=0
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

[[ -n "$TARGET_HOST" ]] || error "Target host is required. Use --host HOST"

if [[ -n "$SSH_KEY" ]]; then
    SSH_OPTS+=( -i "$SSH_KEY" )
fi

SSH_CMD=( ssh "${SSH_OPTS[@]}" "$TARGET_USER@$TARGET_HOST" )
SCP_CMD=( scp "${SSH_OPTS[@]}" )

run_ssh() {
    "${SSH_CMD[@]}" "$@"
}

run_scp() {
    "${SCP_CMD[@]}" "$@"
}

if [[ $DO_BUILD -eq 1 ]]; then
    info "Building PRU firmware, DTBO and daemon"
    make -C "$PROJ_ROOT" dtbo pru daemon
fi

local_files=(
    "$PROJ_ROOT/build/pru/am335x-pru0-fw"
    "$PROJ_ROOT/build/pru/am335x-pru1-fw"
    "$PROJ_ROOT/build/dtbo/pickup-winder-p8-steppers.dtbo"
    "$PROJ_ROOT/build/daemon/pickup_daemon"
    "$SCRIPT_DIR/load_pru.sh"
    "$SCRIPT_DIR/install.sh"
    "$SCRIPT_DIR/pickupwinder-pru.service"
    "$SCRIPT_DIR/pickupwinder-daemon.service"
)

for f in "${local_files[@]}"; do
    [[ -f "$f" ]] || error "Missing local artifact: $f"
done

info "Preparing remote staging directory: $TARGET_STAGE_DIR"
run_ssh "rm -rf '$TARGET_STAGE_DIR' && mkdir -p '$TARGET_STAGE_DIR'"

info "Copying artifacts to target"
run_scp "${local_files[@]}" "$TARGET_USER@$TARGET_HOST:$TARGET_STAGE_DIR/"

remote_cmd=( sudo "$TARGET_STAGE_DIR/install.sh" --from-dir "$TARGET_STAGE_DIR" --install )
if [[ $ENABLE_SERVICE -eq 1 ]]; then
    remote_cmd+=( --enable-service )
fi
if [[ $ENABLE_BOOT_OVERLAY -eq 1 ]]; then
    remote_cmd+=( --enable-boot-overlay )
fi
if [[ $START_NOW -eq 1 ]]; then
    remote_cmd+=( --start-now )
fi

info "Installing on target"
run_ssh "${remote_cmd[*]}"

info "Deployment complete"
