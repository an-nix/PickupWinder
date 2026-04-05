#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PRU_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
TARGET_USER="debian"
TARGET_HOST=""
TARGET_STAGE_DIR="/tmp/pickupwinder-pru-deploy"
SSH_KEY=""
SSH_OPTS=()
DO_BUILD=1
ENABLE_SERVICE=1
ENABLE_BOOT_OVERLAY=1
START_NOW=1
RUN_SPIN_TEST=0
SPIN_RPM=20
SPIN_SECONDS=6
SPIN_FORWARD=1

info()  { echo "[INFO] $*"; }
error() { echo "[FAIL] $*" >&2; exit 1; }

usage() {
    cat <<'EOF'
Usage:
  ./deploy.sh --host 192.168.x.y [options]

Options:
  --host HOST                Target BeagleBone host or IP
  --user USER                SSH user (default: debian)
  --ssh-key PATH             SSH private key
  --stage-dir DIR            Remote staging directory (default: /tmp/pickupwinder-pru-deploy)
  --no-build                 Skip local make / make dtbo
  --no-service               Do not install/enable systemd startup service
  --no-boot-overlay          Do not register DT overlay in /boot/uEnv.txt
  --no-start                 Do not start PRUs after install
  --spin-test                Run a spindle spin test after deployment
  --spin-rpm N               Spin test RPM (default: 20)
  --spin-seconds N           Spin test duration in seconds (default: 6)
  --spin-forward 0|1         Spin direction (default: 1)
  --help                     Show this message
EOF
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
            [[ $# -ge 2 ]] || error "--ssh-key requires a path"
            SSH_KEY="$2"
            shift 2
            ;;
        --stage-dir)
            [[ $# -ge 2 ]] || error "--stage-dir requires a path"
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
        --spin-test)
            RUN_SPIN_TEST=1
            shift
            ;;
        --spin-rpm)
            [[ $# -ge 2 ]] || error "--spin-rpm requires a value"
            SPIN_RPM="$2"
            shift 2
            ;;
        --spin-seconds)
            [[ $# -ge 2 ]] || error "--spin-seconds requires a value"
            SPIN_SECONDS="$2"
            shift 2
            ;;
        --spin-forward)
            [[ $# -ge 2 ]] || error "--spin-forward requires a value"
            SPIN_FORWARD="$2"
            shift 2
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

SSH_OPTS+=( -o BatchMode=yes -o StrictHostKeyChecking=accept-new )

run_ssh() {
    ssh "${SSH_OPTS[@]}" "$TARGET_USER@$TARGET_HOST" "$@"
}

run_scp() {
    scp "${SSH_OPTS[@]}" "$@"
}

remote_install_cmd() {
    local cmd=(
        sudo
        "$TARGET_STAGE_DIR/load_pru.sh"
        --from-dir "$TARGET_STAGE_DIR"
        --install
    )

    if [[ $ENABLE_SERVICE -eq 1 ]]; then
        cmd+=( --enable-service )
    fi
    if [[ $ENABLE_BOOT_OVERLAY -eq 1 ]]; then
        cmd+=( --enable-boot-overlay )
    fi
    if [[ $START_NOW -eq 1 ]]; then
        cmd+=( --start-now )
    fi

    printf '%q ' "${cmd[@]}"
}

if [[ $DO_BUILD -eq 1 ]]; then
    info "Building PRU firmware and DT overlay"
    make -C "$PRU_DIR" all
    make -C "$PRU_DIR" dtbo
fi

local_files=(
    "$PRU_DIR/build/am335x-pru0-fw"
    "$PRU_DIR/build/am335x-pru1-fw"
    "$PRU_DIR/build/dtbo/pickup-winder-p8-steppers.dtbo"
    "$PRU_DIR/test_pru0_motor.sh"
    "$SCRIPT_DIR/load_pru.sh"
    "$SCRIPT_DIR/pickupwinder-pru.service"
    "$SCRIPT_DIR/pru_spin_test.py"
)

for f in "${local_files[@]}"; do
    [[ -f "$f" ]] || error "Missing local artifact: $f"
done

info "Preparing remote staging directory: $TARGET_STAGE_DIR"
run_ssh "rm -rf '$TARGET_STAGE_DIR' && mkdir -p '$TARGET_STAGE_DIR'"

info "Copying artifacts to target"
run_scp \
    "$PRU_DIR/build/am335x-pru0-fw" \
    "$PRU_DIR/build/am335x-pru1-fw" \
    "$PRU_DIR/build/dtbo/pickup-winder-p8-steppers.dtbo" \
    "$PRU_DIR/test_pru0_motor.sh" \
    "$SCRIPT_DIR/load_pru.sh" \
    "$SCRIPT_DIR/pickupwinder-pru.service" \
    "$SCRIPT_DIR/pru_spin_test.py" \
    "$TARGET_USER@$TARGET_HOST:$TARGET_STAGE_DIR/"

if ! run_ssh "sudo -n true" >/dev/null 2>&1; then
    error "Remote sudo is interactive-only. Payload is staged in $TARGET_STAGE_DIR on $TARGET_HOST. Run this on the target: $(remote_install_cmd)"
fi

info "Installing on target"
run_ssh "$(remote_install_cmd)"

if [[ $RUN_SPIN_TEST -eq 1 ]]; then
    info "Running spindle spin test on target"
    run_ssh "sudo /usr/local/bin/pru_spin_test.py --rpm '$SPIN_RPM' --seconds '$SPIN_SECONDS' --forward '$SPIN_FORWARD'"
fi

info "Deployment complete"
