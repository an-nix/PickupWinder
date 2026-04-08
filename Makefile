# Makefile — PickupWinder top-level build orchestrator.
#
# Targets:
#   make          — build everything (DTBOs + PRU firmware + daemon)
#   make dtbo     — compile device-tree overlays only
#   make pru      — build PRU firmware only
#   make daemon   — build the C daemon only
#   make clean    — remove all build artifacts
#   make install  — install PRU firmware + daemon on BBB (run on target)
#   make deploy   — cross-deploy via SSH (requires BBB_IP)
#
# Build outputs go into build/ at the project root:
#   build/dtbo/    — compiled .dtbo files
#   build/pru/     — PRU firmware binaries (am335x-pru{0,1}-fw)
#   build/daemon/  — pickup_daemon binary

.PHONY: all dtbo pru daemon clean install deploy docker-build

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

# Run the Docker-based build (one-click, produces build/ artifacts)
DOCKER_IMG ?= antnic/bbb-crosscompile:debian12
DOCKER_LOG ?= docker-make.log

docker-build:
	@echo "=== Building inside Docker image ==="
	@./docker-build.sh "$(DOCKER_IMG)" "$(DOCKER_LOG)" $(TARGET)

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
