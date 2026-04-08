# PickupWinder Docker Build Image

This directory contains a multi-stage Dockerfile for building PickupWinder on a Debian 12 base.
The image compiles the PRU toolchain and copies the resulting toolchain into the final build image.

## What it includes

- Debian 12 base image
- native build tools (`gcc`, `g++`, `make`, `python3`, `dtc`, etc.)
- DT overlay compilation tools (`dtc`) in the final image
- ARM cross-toolchain built via `crosstool-ng` when a custom ARM config is provided
- `crosstool-ng` built from source for PRU toolchain compilation
- `ct-ng` installed in the final image so you can re-run or update configs after build

## Build the image

From the repository root:

```bash
docker build -t pickupwinder-builder -f docker/Dockerfile .
```

## Usage

Mount the repository into the container and run the build:

```bash
docker run --rm -it -v "$(pwd):/workspace" -w /workspace pickupwinder-builder bash
```

Inside the container you can run:

```bash
make dtbo
make pru
make daemon
```

If you need to build the PRU cross-toolchain with crosstool-ng, run:

```bash
make -C src/pru toolchain
```

## Custom toolchain config

You can provide custom `crosstool-ng` `.config` files for both toolchains:

- `--pru-config /path/to/pru.config`
- `--arm-config /path/to/arm.config`

Example:

```bash
./docker/build.sh --pru-config /path/to/pru.config --arm-config /path/to/arm.config
```

This copies the custom configs into `docker/config/pru/.config` and `docker/config/arm/.config` for the build stage, then removes them after image creation.

If no `--arm-config` is provided, the ARM cross-toolchain stage is skipped and only the PRU toolchain is built.

Build caching is stored in `docker/.docker-cache` so repeated builds can reuse previous layers.

## Notes

- The container uses `/root/x-tools` as the default toolchain path, matching the repo's PRU Makefile defaults.
- The daemon build searches for an ARM cross-compiler in `PATH` and `~/x-tools/*/bin`.
- Docker containers inherit the kernel from the host, so the Debian 12 image runs on the host kernel available at runtime.
