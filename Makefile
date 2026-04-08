# Makefile — PickupWinder top-level build orchestrator.
#
# Targets:
#   make                   — build everything (DTBOs + PRU firmware + daemon)
#   make dtbo              — compile device-tree overlays only
#   make pru               — build PRU firmware only
#   make daemon            — build the C daemon only
#   make clean             — remove all build artefacts
#   make install           — install PRU firmware + daemon on BBB (run on target)
#   make deploy            — cross-deploy via SSH (requires BBB_IP)
#
#   make docker-build          — compile inside Docker image (all targets)
#   make docker-build T=pru    — compile only PRU firmware inside Docker
#   make docker-build T=clean  — clean artefacts inside Docker
#   make docker-image          — (re)build the cross-compile Docker image locally
#   make docker-image-push     — build + push image to registry
#
# Build outputs → build/ at the project root:
#   build/dtbo/    — compiled .dtbo files
#   build/pru/     — PRU firmware binaries (am335x-pru{0,1}-fw)
#   build/daemon/  — pickup_daemon binary

.PHONY: all dtbo pru daemon clean install deploy \
        docker-build docker-image docker-image-push

all: dtbo pru daemon

# ── Device-tree overlays ──────────────────────────────────────────────────
DTSDIR := src/dts
DTBODIR := build/dtbo

dtbo:
	@mkdir -p $(DTBODIR)
	@echo "=== Building DTBOs ==="
	@for f in $(DTSDIR)/*.dts; do \
		[ -f "$$f" ] || continue; \
		base=$$(basename "$$f" .dts); \
		up=$$(echo "$$base" | tr '[:lower:]' '[:upper:]'); \
		out=$(DTBODIR)/$$up-00A0.dtbo; \
		echo "  $$f -> $$out"; \
		dtc -O dtb -b 0 -@ "$$f" -o "$$out" || exit 1; \
	done

# ── PRU firmware ──────────────────────────────────────────────────────────
pru:
	@echo "=== Building PRU firmware ==="
	$(MAKE) -C src/pru OUT_DIR=$(CURDIR)/build/pru

# ── C daemon ─────────────────────────────────────────────────────────────
daemon:
	@echo "=== Building daemon ==="
	$(MAKE) -C src/linux/daemon OUT_DIR=$(CURDIR)/build/daemon

# ── Clean ─────────────────────────────────────────────────────────────────
clean:
	rm -rf build/dtbo build/pru build/daemon
	@echo "Cleaned build/"

# ── Docker: compile project inside the cross-compile image ──────────────────
# Usage:
#   make docker-build            → build all targets
#   make docker-build T=pru      → build only PRU firmware
#   make docker-build T="clean"  → clean artefacts
DOCKER_IMG    ?= antnic/bbb-crosscompile:debian12
DOCKER_LOG    ?= docker-make.log
DOCKER_CONTEXT := tools/bbb-crosscompile
DOCKER_FILE    := $(DOCKER_CONTEXT)/Dockerfile

docker-build:
	@./docker-build.sh \
		--image "$(DOCKER_IMG)" \
		--log   "$(DOCKER_LOG)" \
		$(if $(T),-- $(T))

# ── Docker: build the cross-compile image itself ─────────────────────────────
# --progress=plain : stream all BuildKit output (including ct-ng) to terminal.
# tee              : save the full build log to build/docker-image-build.log.
docker-image:
	@echo "=== Building Docker image: $(DOCKER_IMG) ==="
	@mkdir -p build
	DOCKER_BUILDKIT=1 BUILDKIT_STEP_LOG_MAX_SIZE=-1 docker build --progress=plain \
		--build-arg BUILDER_UID=$(shell id -u) \
		--build-arg BUILDER_GID=$(shell id -g) \
		-t "$(DOCKER_IMG)" \
		-f "$(DOCKER_FILE)" \
		"$(DOCKER_CONTEXT)" \
		2>&1 | tee build/docker-image-build.log

docker-image-push: docker-image
	@echo "=== Pushing Docker image: $(DOCKER_IMG) ==="
	docker push "$(DOCKER_IMG)"

# ── Install (on BBB target) ──────────────────────────────────────────────
install: all
	@echo "=== Installing on target ==="
	sudo cp build/pru/am335x-pru0-fw /lib/firmware/
	sudo cp build/pru/am335x-pru1-fw /lib/firmware/
	sudo cp build/daemon/pickup_daemon /usr/local/bin/
	@echo "Reloading PRUs..."
	sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true'
	sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true'
	sleep 1
	sudo sh -c 'echo am335x-pru0-fw > /sys/class/remoteproc/remoteproc1/firmware'
	sudo sh -c 'echo am335x-pru1-fw > /sys/class/remoteproc/remoteproc2/firmware'
	sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
	sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state'
	@echo "Done."

# ── Deploy via SSH ────────────────────────────────────────────────────────
BBB_USER ?= debian
BBB_IP   ?=

deploy: all
ifeq ($(strip $(BBB_IP)),)
	$(error BBB_IP must be set. Example: make deploy BBB_IP=192.168.7.2)
endif
	@echo "=== Deploying to $(BBB_IP) ==="
	scp build/pru/am335x-pru0-fw build/pru/am335x-pru1-fw \
	    build/daemon/pickup_daemon \
	    $(BBB_USER)@$(BBB_IP):/tmp/
	ssh $(BBB_USER)@$(BBB_IP) "\
	    sudo cp /tmp/am335x-pru0-fw /tmp/am335x-pru1-fw /lib/firmware/ && \
	    sudo cp /tmp/pickup_daemon /usr/local/bin/ && \
	    sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true' && \
	    sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true' && \
	    sleep 1 && \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state' && \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state' && \
	    echo 'PRUs reloaded, daemon installed.'"
