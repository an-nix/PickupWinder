#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}" )" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/config"
TEMP_CONFIG_PRU=""
TEMP_CONFIG_ARM=""
BUILDER_UID=""
BUILDER_GID=""

usage() {
    cat <<EOF2
Usage: $0 [--pru-config PATH] [--arm-config PATH] [--uid UID] [--gid GID] [--tag NAME]

Options:
  --pru-config PATH   Use a custom crosstool-ng .config file for PRU toolchain build.
  --arm-config PATH   Use a custom crosstool-ng .config file for ARM cross-toolchain build.
  --uid UID           Set the UID of the builder user created in the Docker image.
  --gid GID           Set the GID of the builder group created in the Docker image.
  --tag NAME          Docker image tag (default: pickupwinder-builder).
  --help              Show this message.
EOF2
}

TAG="antnic/bbb-cross-compile:latest"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pru-config)
            shift
            [[ $# -gt 0 ]] || { echo "Missing argument for --pru-config" >&2; exit 1; }
            if [[ ! -f "$1" ]]; then
                echo "PRU config not found: $1" >&2
                exit 1
            fi
            mkdir -p "$CONFIG_DIR/pru"
            TEMP_CONFIG_PRU="$CONFIG_DIR/pru/.config"
            cp "$1" "$TEMP_CONFIG_PRU"
            shift
            ;;
        --arm-config)
            shift
            [[ $# -gt 0 ]] || { echo "Missing argument for --arm-config" >&2; exit 1; }
            if [[ ! -f "$1" ]]; then
                echo "ARM config not found: $1" >&2
                exit 1
            fi
            mkdir -p "$CONFIG_DIR/arm"
            TEMP_CONFIG_ARM="$CONFIG_DIR/arm/.config"
            cp "$1" "$TEMP_CONFIG_ARM"
            shift
            ;;
        --tag)
            shift
            [[ $# -gt 0 ]] || { echo "Missing argument for --tag" >&2; exit 1; }
            TAG="$1"
            shift
            ;;
        --uid)
            shift
            [[ $# -gt 0 ]] || { echo "Missing argument for --uid" >&2; exit 1; }
            BUILDER_UID="$1"
            shift
            ;;
        --gid)
            shift
            [[ $# -gt 0 ]] || { echo "Missing argument for --gid" >&2; exit 1; }
            BUILDER_GID="$1"
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
    esac
done

cleanup() {
    if [[ -n "$TEMP_CONFIG_PRU" && -f "$TEMP_CONFIG_PRU" ]]; then
        rm -f "$TEMP_CONFIG_PRU"
    fi
    if [[ -n "$TEMP_CONFIG_ARM" && -f "$TEMP_CONFIG_ARM" ]]; then
        rm -f "$TEMP_CONFIG_ARM"
    fi
}
trap cleanup EXIT

cd "$SCRIPT_DIR/.."
CACHE_DIR="$SCRIPT_DIR/.docker-cache"
mkdir -p "$CACHE_DIR"

BUILD_ARGS=()
if [[ -n "$BUILDER_UID" ]]; then
    BUILD_ARGS+=(--build-arg "BUILDER_UID=$BUILDER_UID")
fi
if [[ -n "$BUILDER_GID" ]]; then
    BUILD_ARGS+=(--build-arg "BUILDER_GID=$BUILDER_GID")
fi

if command -v docker buildx >/dev/null 2>&1; then
    BUILDX_DRIVER=$(docker buildx inspect 2>/dev/null | awk '/driver:/ {print $2; exit}') || true
    if [[ "$BUILDX_DRIVER" == "docker" ]]; then
        echo "Using docker driver with standard build cache"
        DOCKER_BUILDKIT=1 docker build --progress=plain \
            "${BUILD_ARGS[@]}" \
            -t "$TAG" \
            -f docker/Dockerfile .
    else
        echo "Using buildx driver '$BUILDX_DRIVER' with cache export"
        DOCKER_BUILDKIT=1 docker buildx build \
            --progress=plain \
            --no-cache \
            --load \
            "${BUILD_ARGS[@]}" \
            -t "$TAG" \
            -f docker/Dockerfile .
    fi
else
    echo "Buildx not available; using standard docker build"
    DOCKER_BUILDKIT=1 docker build \
        "${BUILD_ARGS[@]}" \
        -t "$TAG" \
        -f docker/Dockerfile .
fi
