# Makefile — PRU firmware build (crosstool-ng pru-unknown-elf toolchain)
#
# Prerequisites on host (Arch Linux):
#   yay -S crosstool-ng
#   ct-ng pru-unknown-elf && ct-ng build
#
# If installed via package manager, pru-unknown-elf-gcc is expected in PATH.
# Override with PRU_CC or CROSS_COMPILE if needed:
#   make PRU_CC=/custom/path/to/pru-unknown-elf-gcc
#   make CROSS_COMPILE=/custom/path/to/pru-unknown-elf-
#
# Prerequisites headers:
#   git clone https://git.ti.com/git/pru-software-support-package/pru-software-support-package.git
#   export PRU_SWPKG=~/pru-software-support-package
#
# Usage:
#   make                             → build PRU0 motor fw + PRU1 orchestration fw (default)
#   make dtbo                        → compile DTS overlays to .dtbo
#   make clean                       → remove build artefacts
#   make install                     → copy .fw to /lib/firmware/ and reload PRUs (on BBB)
#   make deploy BBB_IP=192.168.7.2   → cross-deploy via SSH + reload PRUs (from host)
#   make toolchain                   → build crosstool-ng toolchain from scratch
#   make toolchain-info              → display current toolchain config
#
# DTS overlay prerequisites (for dtbo target):
#   sudo apt install device-tree-compiler

# ── Crosstool-ng ────────────────────────────────────────────────────────────
CT_NG     ?= ct-ng
CT_TARGET ?= pru-unknown-elf

# ── Toolchain ───────────────────────────────────────────────────────────────
# Toolchain installed via package manager (e.g. yay -S crosstool-ng).
# pru-unknown-elf-gcc is expected in PATH.
# Override with PRU_CC or CROSS_COMPILE if needed.
CROSS_COMPILE ?= pru-unknown-elf-
PRU_CC        ?=

# Optional explicit toolchain directory (user-provided). Default to $HOME/x-tools
# so per-user toolchains like /home/nicolas/x-tools/pru-elf are detected.
TOOLCHAIN_DIR ?= $(HOME)/x-tools

# If PRU_CC not provided, try to detect a suitable gcc binary in
# $(TOOLCHAIN_DIR)/bin matching '*pru*gcc'. Otherwise fall back to
# CROSS_COMPILE handling (unchanged behaviour).
ifeq ($(strip $(PRU_CC)),)
PRU_CANDIDATE := $(shell ls $(TOOLCHAIN_DIR)/bin/*pru*gcc $(TOOLCHAIN_DIR)/*/bin/*pru*gcc 2>/dev/null | head -n1)
ifneq ($(strip $(PRU_CANDIDATE)),)
PRU_CC := $(PRU_CANDIDATE)
else
ifeq ($(strip $(CROSS_COMPILE)),)
PRU_CC := $(CROSS_COMPILE)
else
PRU_CC := $(CROSS_COMPILE)gcc
endif
endif
endif

# ── PRU Software Support Package (TI headers) ───────────────────────────────
PRU_SWPKG ?=

PRU_INCLUDES = -Iinclude
ifneq ($(strip $(PRU_SWPKG)),)
PRU_INCLUDES += -I$(PRU_SWPKG)/include \
                -I$(PRU_SWPKG)/include/am335x
endif

# ── Compiler flags ──────────────────────────────────────────────────────────
PRU_CFLAGS = -O2 -Wall -Wextra \
			 $(PRU_INCLUDES)   \
			 -DPRU_FIRMWARE \
			 -DPRU_SRAM_PHYS_BASE=0x00010000u

# ── Outputs ─────────────────────────────────────────────────────────────────
OUT_DIR   = build
FW_NAME_0 = am335x-pru0-fw
FW_NAME_1 = am335x-pru1-fw

# ── Remote deploy (cross-compilation workflow) ──────────────────────────────
BBB_USER ?= debian
BBB_IP   ?=
SSH_KEY  ?=
SSH_OPTS ?=
REMOTE_FW_DIR ?= /lib/firmware

# SCP/SSH command selection (evaluated once at parse time, before any target).
# Moving these here avoids the GNU Make "recipe commences before first target"
# error that occurs when ifeq blocks with variable assignments appear inside a
# recipe body.
ifeq ($(strip $(SSH_KEY)),)
SCP_CMD = scp $(SSH_OPTS)
SSH_CMD = ssh $(SSH_OPTS)
else
SCP_CMD = scp -i $(SSH_KEY) $(SSH_OPTS)
SSH_CMD = ssh -i $(SSH_KEY) $(SSH_OPTS)
endif

# ── Targets ─────────────────────────────────────────────────────────────────
.PHONY: all clean install deploy toolchain toolchain-info dtbo

all: $(OUT_DIR)/$(FW_NAME_0) $(OUT_DIR)/$(FW_NAME_1)

# ── Toolchain info ───────────────────────────────────────────────────────────
toolchain-info:
	@echo "CT_NG         = $(CT_NG)"
	@echo "CT_TARGET     = $(CT_TARGET)"
	@echo "PRU_CC        = $(PRU_CC)"
	@echo "CROSS_COMPILE = $(CROSS_COMPILE)"
	@echo "PRU_SWPKG     = $(PRU_SWPKG)"
	@echo "BBB_IP        = $(BBB_IP)"
	@which $(PRU_CC) || echo "WARNING: $(PRU_CC) not found in PATH"

# ── Build crosstool-ng toolchain ─────────────────────────────────────────────
toolchain:
	$(CT_NG) $(CT_TARGET)
	$(CT_NG) build
	@echo "Toolchain built. Add to PATH:"
	@echo "  export PATH=\$$(dirname \$$(which $(PRU_CC))):\$$PATH"

# ── Compile ──────────────────────────────────────────────────────────────────
$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(OUT_DIR)/$(FW_NAME_0): pru0_motor_control/main.c include/pru_ipc.h include/pru_stepper.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[PRU0 motor-control] built → $@"

$(OUT_DIR)/$(FW_NAME_1): pru1_orchestration/main.c include/pru_ipc.h include/pru_stepper.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[PRU1 orchestration] built → $@"

# ── Device-tree overlays ─────────────────────────────────────────────────────
DTC       ?= dtc
DTS_DIR    = dts
DTBO_DIR   = $(OUT_DIR)/dtbo

DTS_SOURCES := $(wildcard $(DTS_DIR)/*.dts)
DTBO_TARGETS := $(patsubst $(DTS_DIR)/%.dts,$(DTBO_DIR)/%.dtbo,$(DTS_SOURCES))

dtbo: $(DTBO_TARGETS)

$(DTBO_DIR):
	mkdir -p $(DTBO_DIR)

$(DTBO_DIR)/%.dtbo: $(DTS_DIR)/%.dts | $(DTBO_DIR)
	$(DTC) -O dtb -b 0 -@ $< -o $@
	@echo "[DTS overlay] built → $@"

# ── Install (sur le BBB lui-même) ────────────────────────────────────────────
install: all
	sudo cp $(OUT_DIR)/$(FW_NAME_0) /lib/firmware/
	sudo cp $(OUT_DIR)/$(FW_NAME_1) /lib/firmware/
	@echo "Stopping PRUs..."
	sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true'
	sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true'
	sleep 1
	@echo "Starting PRUs..."
	sudo sh -c 'echo $(FW_NAME_0) > /sys/class/remoteproc/remoteproc1/firmware'
	sudo sh -c 'echo $(FW_NAME_1) > /sys/class/remoteproc/remoteproc2/firmware'
	sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
	sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state'
	@echo "PRUs loaded."

# ── Deploy (depuis le PC via SSH) ─────────────────────────────────────────────
deploy: all
ifeq ($(strip $(BBB_IP)),)
	$(error BBB_IP must be set. Example: make deploy BBB_IP=192.168.7.2)
endif

	@echo "Copying firmwares to $(BBB_IP):$(REMOTE_FW_DIR)"
	$(SCP_CMD) $(OUT_DIR)/$(FW_NAME_0) $(BBB_USER)@$(BBB_IP):$(REMOTE_FW_DIR)/
	$(SCP_CMD) $(OUT_DIR)/$(FW_NAME_1) $(BBB_USER)@$(BBB_IP):$(REMOTE_FW_DIR)/

	@echo "Reloading PRUs on $(BBB_IP) (backing up existing firmwares)"
	$(SSH_CMD) $(BBB_USER)@$(BBB_IP) " \
	    set -e; \
	    TS=$$(date +%s); \
	    echo 'Stopping PRUs...'; \
	    echo stop | sudo tee /sys/class/remoteproc/remoteproc1/state 2>/dev/null || true; \
	    echo stop | sudo tee /sys/class/remoteproc/remoteproc2/state 2>/dev/null || true; \
	    sleep 1; \
	    echo 'Backing up existing firmwares (if any)...'; \
	    if [ -f $(REMOTE_FW_DIR)/$(FW_NAME_0) ]; then sudo cp $(REMOTE_FW_DIR)/$(FW_NAME_0) $(REMOTE_FW_DIR)/$(FW_NAME_0).bak.$$TS; fi; \
	    if [ -f $(REMOTE_FW_DIR)/$(FW_NAME_1) ]; then sudo cp $(REMOTE_FW_DIR)/$(FW_NAME_1) $(REMOTE_FW_DIR)/$(FW_NAME_1).bak.$$TS; fi; \
	    echo 'Installing new firmwares...'; \
	    echo $(FW_NAME_0) | sudo tee /sys/class/remoteproc/remoteproc1/firmware; \
	    echo $(FW_NAME_1) | sudo tee /sys/class/remoteproc/remoteproc2/firmware; \
	    echo start | sudo tee /sys/class/remoteproc/remoteproc1/state; \
	    echo start | sudo tee /sys/class/remoteproc/remoteproc2/state; \
	    echo 'PRUs loaded.'"

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -rf $(OUT_DIR)