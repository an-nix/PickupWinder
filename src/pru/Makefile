# Makefile — PRU firmware build (crosstool-ng pru-unknown-elf toolchain)
#
# Usage:
#   make                          — build PRU0 + PRU1 firmware
#   make clean                    — remove build artifacts
#   make install                  — copy firmware and reload PRUs (on BBB)
#   make deploy BBB_IP=<ip>       — cross-deploy via SSH
#   make toolchain                — build crosstool-ng toolchain
#   make toolchain-info           — display toolchain config
#
# Called from root Makefile with:
#   make -C src/pru OUT_DIR=$(CURDIR)/build/pru

# ── Toolchain ───────────────────────────────────────────────────────────────
CROSS_COMPILE ?= pru-unknown-elf-
PRU_CC        ?=
TOOLCHAIN_DIR ?= $(HOME)/x-tools
CT_NG         ?= ct-ng
CT_TARGET     ?= pru-unknown-elf

# Auto-detect toolchain
ifeq ($(strip $(PRU_CC)),)
PRU_CANDIDATE := $(shell ls $(TOOLCHAIN_DIR)/bin/*pru*gcc $(TOOLCHAIN_DIR)/*/bin/*pru*gcc 2>/dev/null | head -n1)
ifneq ($(strip $(PRU_CANDIDATE)),)
PRU_CC := $(PRU_CANDIDATE)
else
PRU_CC := $(CROSS_COMPILE)gcc
endif
endif

# ── PRU Software Support Package (optional TI headers) ───────────────────
PRU_SWPKG ?=
PRU_INCLUDES = -Iinclude
ifneq ($(strip $(PRU_SWPKG)),)
PRU_INCLUDES += -I$(PRU_SWPKG)/include -I$(PRU_SWPKG)/include/am335x
endif

# ── Compiler flags ──────────────────────────────────────────────────────────
PRU_CFLAGS = -O2 -Wall -Wextra \
             -Wno-volatile-register-var \
             $(PRU_INCLUDES) \
             -DPRU_FIRMWARE \
             -DPRU_SRAM_PHYS_BASE=0x00010000u

# ── Outputs ─────────────────────────────────────────────────────────────────
OUT_DIR   ?= build
FW_NAME_0        = am335x-pru0-fw
FW_NAME_1        = am335x-pru1-fw
FW_NAME_TEST_SP   = am335x-pru0-fw-test-spindle
FW_NAME_TEST_GPIO = am335x-pru0-fw-test-gpio
FW_NAME_TEST_STATIC = am335x-pru0-fw-test-static

# ── Remote deploy ───────────────────────────────────────────────────────────
BBB_USER ?= debian
BBB_IP   ?=

# ── Targets ─────────────────────────────────────────────────────────────────
.PHONY: all clean install deploy toolchain toolchain-info test-spindle test-gpio test-static

all: $(OUT_DIR)/$(FW_NAME_0) $(OUT_DIR)/$(FW_NAME_1)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(OUT_DIR)/$(FW_NAME_0): orchestrator/main.c include/pru_ipc.h include/pru_regs.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[PRU orchestrator -> PRU0] built -> $@"

$(OUT_DIR)/$(FW_NAME_1): motor_control/main.c include/pru_ipc.h include/pru_stepper.h include/pru_regs.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[PRU motor -> PRU1] built -> $@"

# ── Standalone spindle test (no IPC, no daemon) ──────────────────────────────
# Drives spindle at 25 RPM directly from PRU0 to validate hardware wiring.
# Usage: make test-spindle   → builds am335x-pru0-fw-test-spindle
test-spindle: $(OUT_DIR)/$(FW_NAME_TEST_SP)

$(OUT_DIR)/$(FW_NAME_TEST_SP): test_spindle/main.c include/pru_stepper.h include/pru_regs.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[TEST] built -> $@"

# ── GPIO blink test (all stepper pins, 1 s toggle) ──────────────────────────
# Lets you probe each PRU output with a 330Ω + LED to verify pad driving.
# Usage: make test-gpio
test-gpio: $(OUT_DIR)/$(FW_NAME_TEST_GPIO)

$(OUT_DIR)/$(FW_NAME_TEST_GPIO): test_gpio/main.c include/pru_stepper.h include/pru_regs.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[TEST] built -> $@"

# ── Static HIGH test (no timer — diagnose R30 vs IEP) ────────────────────────
# All stepper pins go HIGH immediately. No blink, no IEP.
test-static: $(OUT_DIR)/$(FW_NAME_TEST_STATIC)

$(OUT_DIR)/$(FW_NAME_TEST_STATIC): test_static/main.c include/pru_regs.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[TEST] built -> $@"

# ── Toolchain ────────────────────────────────────────────────────────────────
toolchain:
	$(CT_NG) $(CT_TARGET)
	$(CT_NG) build

toolchain-info:
	@echo "PRU_CC = $(PRU_CC)"
	@which $(PRU_CC) 2>/dev/null || echo "WARNING: $(PRU_CC) not found"

# ── Install (on BBB) ────────────────────────────────────────────────────────
install: all
	sudo cp $(OUT_DIR)/$(FW_NAME_0) /lib/firmware/
	sudo cp $(OUT_DIR)/$(FW_NAME_1) /lib/firmware/
	sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true'
	sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true'
	sleep 1
	sudo sh -c 'echo $(FW_NAME_0) > /sys/class/remoteproc/remoteproc1/firmware'
	sudo sh -c 'echo $(FW_NAME_1) > /sys/class/remoteproc/remoteproc2/firmware'
	sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state'
	sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state'
	@echo "PRUs loaded."

# ── Deploy via SSH ───────────────────────────────────────────────────────────
deploy: all
ifeq ($(strip $(BBB_IP)),)
	$(error BBB_IP must be set)
endif
	scp $(OUT_DIR)/$(FW_NAME_0) $(OUT_DIR)/$(FW_NAME_1) $(BBB_USER)@$(BBB_IP):/tmp/
	ssh $(BBB_USER)@$(BBB_IP) "\
	    sudo cp /tmp/$(FW_NAME_0) /tmp/$(FW_NAME_1) /lib/firmware/ && \
	    sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true' && \
	    sudo sh -c 'echo stop > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true' && \
	    sleep 1 && \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state' && \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state' && \
	    echo 'PRUs reloaded.'"

# ── Clean ────────────────────────────────────────────────────────────────────
clean:
	rm -rf $(OUT_DIR)
