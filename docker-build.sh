#!/usr/bin/env bash
# =============================================================================
# docker-build.sh — Compile PickupWinder inside the cross-compile Docker image.
#
# Output artifacts are written to <project-root>/build/ and are owned by the
# current user (enforced via --user flag on docker run).
#
# Usage:
#   ./docker-build.sh [OPTIONS] [-- MAKE_TARGET...]
#
# Options:
#   -i, --image IMAGE   Docker image  (default: $DOCKER_IMG or antnic/bbb-crosscompile:debian12)
#   -l, --log   FILE    Append output to FILE (default: docker-make.log)
#   -j, --jobs  N       Parallel make jobs (default: nproc)
#   -h, --help          Show this help
#
# Examples:
#   ./docker-build.sh                        # build everything (dtbo + pru + daemon)
#   ./docker-build.sh -- pru                 # PRU firmware only
#   ./docker-build.sh -- clean               # clean build artefacts
#   ./docker-build.sh -i myimage:tag -- pru daemon
# =============================================================================
set -uo pipefail

# ── Colours (disabled when not a terminal) ───────────────────────────────────
if [[ -t 1 ]]; then
    RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'
    BOLD='\033[1m';   RESET='\033[0m'
else
    RED=''; GREEN=''; CYAN=''; BOLD=''; RESET=''
fi

# ── Resolve project root (directory containing this script) ──────────────────
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# ── Defaults ─────────────────────────────────────────────────────────────────
IMAGE="${DOCKER_IMG:-antnic/bbb-crosscompile:debian12}"
LOG_FILE="$ROOT_DIR/docker-make.log"
JOBS="$(nproc)"
MAKE_TARGETS=()

# ── Argument parsing ─────────────────────────────────────────────────────────
usage() {
    sed -n '2,21p' "${BASH_SOURCE[0]}"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--image) shift; [[ $# -gt 0 ]] || { echo "Missing value for --image" >&2; exit 1; }
                    IMAGE="$1"; shift ;;
        -l|--log)   shift; [[ $# -gt 0 ]] || { echo "Missing value for --log" >&2; exit 1; }
                    LOG_FILE="$1"; shift ;;
        -j|--jobs)  shift; [[ $# -gt 0 ]] || { echo "Missing value for --jobs" >&2; exit 1; }
                    JOBS="$1"; shift ;;
        -h|--help)  usage ;;
        --)         shift; MAKE_TARGETS=("$@"); break ;;
        -*)         echo "Unknown option: $1" >&2; usage ;;
        *)          MAKE_TARGETS+=("$1"); shift ;;
    esac
done

# ── Banner ────────────────────────────────────────────────────────────────────
echo -e "${BOLD}${CYAN}=== PickupWinder — Docker build ===${RESET}"
echo -e "  Image  : ${IMAGE}"
echo -e "  Jobs   : ${JOBS}"
echo -e "  Log    : ${LOG_FILE}"
[[ ${#MAKE_TARGETS[@]} -gt 0 ]] \
    && echo -e "  Target : ${MAKE_TARGETS[*]}" \
    || echo -e "  Target : all"
echo

# ── Pre-flight ────────────────────────────────────────────────────────────────
mkdir -p "$ROOT_DIR/build"
: > "$LOG_FILE"          # truncate log at start of each run

if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker daemon is not running.${RESET}" >&2
    exit 1
fi

# ── Build ─────────────────────────────────────────────────────────────────────
START=$(date +%s)

# --user    : artefacts in build/ are owned by the calling user, not root.
# -v ... :z : ":z" relabels the mount for SELinux hosts (no-op on non-SELinux).
set +e
docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "${ROOT_DIR}:/workspace:z" \
    -w /workspace \
    "$IMAGE" \
    make -j"${JOBS}" "${MAKE_TARGETS[@]}" \
    2>&1 | tee -a "$LOG_FILE"
EXIT_CODE=${PIPESTATUS[0]}
set -e

# ── Result ────────────────────────────────────────────────────────────────────
ELAPSED=$(( $(date +%s) - START ))

if [[ $EXIT_CODE -eq 0 ]]; then
    echo -e "\n${BOLD}${GREEN}Build succeeded in ${ELAPSED}s.${RESET}"
    echo -e "  Artefacts → ${ROOT_DIR}/build/"
else
    echo -e "\n${BOLD}${RED}Build FAILED (exit ${EXIT_CODE}) after ${ELAPSED}s.${RESET}" >&2
    echo -e "  Log → ${LOG_FILE}" >&2
    exit $EXIT_CODE
fi

