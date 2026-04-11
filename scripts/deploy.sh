#!/usr/bin/env bash
# =============================================================================
# scripts/deploy.sh — Full PickupWinder deploy: build → transfer → install
#
# What it does:
#   1. Cross-compile daemon (ARM) via Docker image
#   2. Build PRU firmware (pru-unknown-elf-gcc, also via Docker if needed)
#   3. SCP all artifacts to the BBB
#   4. Install firmware to /lib/firmware/
#   5. Install daemon binary to /usr/local/bin/
#   6. Install systemd service and enable it
#   7. Reload PRU firmware via remoteproc
#   8. Restart pickup-winder service
#
# Usage:
#   ./scripts/deploy.sh [OPTIONS]
#
# Options:
#   -h HOST    BBB hostname or IP  (default: $BBB_IP or 192.168.74.171)
#   -u USER    SSH user            (default: $BBB_USER or beagle)
#   -p         Deploy PRU firmware only (skip daemon)
#   -d         Deploy daemon only (skip PRU firmware)
#   -s         Deploy service file only (no rebuild)
#   --no-build Skip Docker build step (use existing build/ artifacts)
#   --help     Show this help
#
# Examples:
#   ./scripts/deploy.sh                          # full deploy
#   ./scripts/deploy.sh -h 192.168.7.2 -u debian
#   ./scripts/deploy.sh -d                       # daemon only
#   ./scripts/deploy.sh --no-build               # skip rebuild, just deploy
# =============================================================================
set -euo pipefail

# ── Colours ──────────────────────────────────────────────────────────────────
if [[ -t 1 ]]; then
    R='\033[0;31m'; G='\033[0;32m'; C='\033[0;36m'; B='\033[1m'; X='\033[0m'
else
    R=''; G=''; C=''; B=''; X=''
fi

info()  { echo -e "${C}[deploy]${X} $*"; }
ok()    { echo -e "${G}[  OK  ]${X} $*"; }
err()   { echo -e "${R}[ERROR ]${X} $*" >&2; exit 1; }
warn()  { echo -e "${R}[ WARN ]${X} $*" >&2; }

# ── Locate project root ───────────────────────────────────────────────────────
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD="${ROOT}/build"

# ── Defaults ─────────────────────────────────────────────────────────────────
BBB_HOST="${BBB_IP:-192.168.74.171}"
BBB_USER="${BBB_USER:-beagle}"
DO_PRU=true
DO_DAEMON=true
DO_SERVICE=true
DO_BUILD=true

# ── Argument parsing ─────────────────────────────────────────────────────────
usage() {
    sed -n '2,24p' "${BASH_SOURCE[0]}"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h) shift; BBB_HOST="$1"; shift ;;
        -u) shift; BBB_USER="$1"; shift ;;
        -p) DO_DAEMON=false; DO_SERVICE=false; shift ;;
        -d) DO_PRU=false; shift ;;
        -s) DO_BUILD=false; DO_PRU=false; DO_DAEMON=false; shift ;;
        --no-build) DO_BUILD=false; shift ;;
        --help) usage ;;
        *) err "Unknown option: $1. Use --help for usage." ;;
    esac
done

SSH="ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=10"
SCP="scp -o BatchMode=yes -o StrictHostKeyChecking=no"

# ── Pre-flight checks ─────────────────────────────────────────────────────────
echo -e "${B}${C}═══════════════════════════════════════════${X}"
echo -e "${B}${C}  PickupWinder — Full Deploy              ${X}"
echo -e "${B}${C}═══════════════════════════════════════════${X}"
info "Target : ${BBB_USER}@${BBB_HOST}"
info "Deploy : PRU=${DO_PRU} Daemon=${DO_DAEMON} Service=${DO_SERVICE} Build=${DO_BUILD}"
echo

# Test SSH connectivity
info "Testing SSH connectivity to ${BBB_HOST}..."
$SSH "${BBB_USER}@${BBB_HOST}" 'echo ok' > /dev/null \
    || err "Cannot connect to ${BBB_USER}@${BBB_HOST} via SSH"
ok "SSH connection OK"

# ── Step 1: Build ─────────────────────────────────────────────────────────────
if $DO_BUILD; then
    BUILD_TARGETS=()
    $DO_PRU    && BUILD_TARGETS+=(pru)
    $DO_DAEMON && BUILD_TARGETS+=(daemon)

    if [[ ${#BUILD_TARGETS[@]} -gt 0 ]]; then
        info "Building: ${BUILD_TARGETS[*]} via Docker cross-compiler..."
        cd "$ROOT"
        ./docker-build.sh -- "${BUILD_TARGETS[@]}" \
            || err "Docker build failed. Check docker-make.log"
        ok "Build complete → ${BUILD}/."
    fi
fi

# ── Verify artifacts ──────────────────────────────────────────────────────────
if $DO_PRU; then
    [[ -f "${BUILD}/pru/am335x-pru0-fw" ]] \
        || err "Missing ${BUILD}/pru/am335x-pru0-fw — run build first"
    [[ -f "${BUILD}/pru/am335x-pru1-fw" ]] \
        || err "Missing ${BUILD}/pru/am335x-pru1-fw — run build first"
fi
if $DO_DAEMON; then
    [[ -f "${BUILD}/daemon/pickup_daemon" ]] \
        || err "Missing ${BUILD}/daemon/pickup_daemon — run build first"
    ARCH=$(file "${BUILD}/daemon/pickup_daemon" | grep -o 'ARM\|x86')
    [[ "$ARCH" == "ARM" ]] \
        || warn "pickup_daemon is not an ARM binary (got: $ARCH) — wrong cross-compiler?"
fi

# ── Step 2: Transfer files ────────────────────────────────────────────────────
info "Transferring files to ${BBB_HOST}..."
TRANSFER_FILES=()

if $DO_PRU; then
    TRANSFER_FILES+=("${BUILD}/pru/am335x-pru0-fw" "${BUILD}/pru/am335x-pru1-fw")
fi
if $DO_DAEMON; then
    TRANSFER_FILES+=("${BUILD}/daemon/pickup_daemon")
fi
if $DO_SERVICE; then
    TRANSFER_FILES+=("${ROOT}/src/linux/daemon/pickup-winder.service")
fi

$SCP "${TRANSFER_FILES[@]}" "${BBB_USER}@${BBB_HOST}:/tmp/" \
    || err "SCP transfer failed"
ok "Transfer complete"

# ── Step 3: Install on BBB ───────────────────────────────────────────────────
info "Installing on ${BBB_HOST}..."

REMOTE_SCRIPT=""

# Install PRU firmware
if $DO_PRU; then
    REMOTE_SCRIPT+='
    echo "[remote] Installing PRU firmware..."
    sudo cp /tmp/am335x-pru0-fw /tmp/am335x-pru1-fw /lib/firmware/
'
fi

# Install daemon binary
if $DO_DAEMON; then
    REMOTE_SCRIPT+='
    echo "[remote] Stopping service before binary install..."
    sudo systemctl stop pickup-winder 2>/dev/null || true
    sudo pkill -9 pickup_daemon 2>/dev/null || true
    sleep 0.5
    echo "[remote] Installing daemon binary..."
    sudo cp /tmp/pickup_daemon /usr/local/bin/pickup_daemon
    sudo chmod 755 /usr/local/bin/pickup_daemon
'
fi

# Install systemd service
if $DO_SERVICE; then
    REMOTE_SCRIPT+='
    echo "[remote] Installing systemd service..."
    sudo cp /tmp/pickup-winder.service /etc/systemd/system/pickup-winder.service
    sudo systemctl daemon-reload
    sudo systemctl enable pickup-winder
'
fi

# Reload PRU firmware and restart service
if $DO_PRU || $DO_DAEMON; then
    REMOTE_SCRIPT+='
    echo "[remote] Restarting pickup-winder service..."
    sudo systemctl restart pickup-winder
    sleep 1.5
    echo "[remote] Service status:"
    sudo systemctl is-active pickup-winder && echo "  → ACTIVE" || echo "  → FAILED"
'
fi

$SSH "${BBB_USER}@${BBB_HOST}" "bash -s" <<< "$REMOTE_SCRIPT" \
    || err "Remote install script failed"

ok "Installation complete"

# ── Step 4: Smoke test ────────────────────────────────────────────────────────
if $DO_DAEMON; then
    info "Running smoke test (enable + telem)..."
    RESULT=$($SSH "${BBB_USER}@${BBB_HOST}" 'python3 - << '"'"'EOF'"'"'
import socket, json, time, sys

try:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect("/run/pickup-winder.sock")
    s.settimeout(2.0)
    msg = json.dumps({"cmd": "enable", "value": 1}) + "\n"
    s.sendall(msg.encode())
    # Parse only the first complete JSON line (skip telem events)
    buf = ""
    deadline = time.monotonic() + 3.0
    resp = None
    while time.monotonic() < deadline:
        try:
            buf += s.recv(4096).decode(errors="replace")
        except socket.timeout:
            pass
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
                if "ok" in obj:
                    resp = obj
                    break
            except Exception:
                pass
        if resp:
            break
    s.close()
    if resp and resp.get("ok"):
        print("OK")
    else:
        print("FAIL: " + str(resp))
except Exception as e:
    print("FAIL: " + str(e))
EOF' 2>/dev/null || echo "FAIL: SSH error")

    if [[ "$RESULT" == "OK" ]]; then
        ok "Smoke test passed — daemon responding"
    else
        warn "Smoke test: ${RESULT}"
        warn "Check with: ssh ${BBB_USER}@${BBB_HOST} 'sudo journalctl -u pickup-winder -n 30'"
    fi
fi

echo
echo -e "${B}${G}═══════════════════════════════════════════${X}"
echo -e "${B}${G}  Deploy complete!                         ${X}"
echo -e "${B}${G}═══════════════════════════════════════════${X}"
echo -e "  Service : sudo systemctl status pickup-winder"
echo -e "  Logs    : sudo journalctl -fu pickup-winder"
echo -e "  Test    : python3 scripts/test_motor.py --host ${BBB_HOST}"
