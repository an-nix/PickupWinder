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
PRU_CFLAGS = -O1 -Wall -Wextra \
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

# ── BBB-native build (clpru) ────────────────────────────────────────────────
# NOTE: pru-elf-gcc inline-asm IEP bug is FIXED in pru_stepper.h (volatile
# pointer approach).  Both compilers now produce correct 32-bit IEP reads.
# clpru uses CT_IEP.TMR_CNT via the constant table; pru-elf-gcc uses a
# volatile pointer dereference to 0x0002E00C, both verified by disassembly.
#
# Usage:
#   make bbb-build BBB_IP=192.168.74.171   — rsync + compile on BBB
#   make bbb-deploy BBB_IP=192.168.74.171  — bbb-build + install + reload PRUs

BBB_USER ?= beagle
BBB_IP   ?= 192.168.74.171
BBB_BUILD_DIR ?= /home/beagle/pru-build

# TI paths on the BBB (ti-pru-cgt-v2.3 + ti-pru-software-v6.3)
BBB_SWPKG  = /usr/lib/ti/pru-software-support-package-v6.3
BBB_CGTI   = /usr/share/ti/cgt-pru/include
BBB_CGTLIB = /usr/share/ti/cgt-pru/lib
BBB_LNK    = $(BBB_BUILD_DIR)/AM335x_PRU.cmd

BBB_CLPRU_FLAGS = \
    --include_path=$(BBB_CGTI) \
    --include_path=$(BBB_SWPKG)/include/am335x \
    --include_path=$(BBB_SWPKG)/include \
    --include_path=$(BBB_BUILD_DIR)/include \
    --hardware_mac=on -O2 \
    -DPRU_SRAM_PHYS_BASE=0x00010000u \
    -DPRU_FIRMWARE

# Remote deploy ──────────────────────────────────────────────────────────────

# ── Targets ─────────────────────────────────────────────────────────────────
.PHONY: all clean install deploy toolchain toolchain-info test-spindle test-gpio test-static bbb-build bbb-deploy

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

# ── BBB-native build (clpru) ─────────────────────────────────────────────────
# Rsyncs the entire src/pru/ tree to the BBB, compiles all firmware binaries
# using clpru, then copies the resulting ELF files back into $(OUT_DIR).

bbb-build:
ifeq ($(strip $(BBB_IP)),)
	$(error BBB_IP must be set, e.g.: make bbb-build BBB_IP=192.168.74.171)
endif
	@echo "[BBB] Syncing sources to $(BBB_USER)@$(BBB_IP):$(BBB_BUILD_DIR)/"
	ssh $(BBB_USER)@$(BBB_IP) "mkdir -p $(BBB_BUILD_DIR)"
	rsync -a --delete \
	    --exclude='build/' \
	    . $(BBB_USER)@$(BBB_IP):$(BBB_BUILD_DIR)/
	@echo "[BBB] Compiling firmware with clpru..."
	ssh $(BBB_USER)@$(BBB_IP) " \
	    set -e ; \
	    CLPRU=/usr/bin/clpru ; \
	    CGTLIB=$(BBB_CGTLIB) ; \
	    FLAGS='$(BBB_CLPRU_FLAGS)' ; \
	    LNK=$(BBB_LNK) ; \
	    cd $(BBB_BUILD_DIR) ; \
	    \$$CLPRU \$$FLAGS -fe mc_obj.out   motor_control/main.c  -z -i\$$CGTLIB -o $(FW_NAME_1) \$$LNK \$$CGTLIB/rtspruv1_le.lib && \
	    \$$CLPRU \$$FLAGS -fe orch_obj.out orchestrator/main.c   -z -i\$$CGTLIB -o $(FW_NAME_0) \$$LNK \$$CGTLIB/rtspruv1_le.lib && \
	    \$$CLPRU \$$FLAGS -fe tsp_obj.out  test_spindle/main.c   -z -i\$$CGTLIB -o $(FW_NAME_TEST_SP) \$$LNK \$$CGTLIB/rtspruv1_le.lib && \
	    echo '[BBB] Compilation OK'"
	@echo "[BBB] Fetching binaries..."
	mkdir -p $(OUT_DIR)
	scp $(BBB_USER)@$(BBB_IP):$(BBB_BUILD_DIR)/$(FW_NAME_0)       $(OUT_DIR)/
	scp $(BBB_USER)@$(BBB_IP):$(BBB_BUILD_DIR)/$(FW_NAME_1)       $(OUT_DIR)/
	scp $(BBB_USER)@$(BBB_IP):$(BBB_BUILD_DIR)/$(FW_NAME_TEST_SP) $(OUT_DIR)/
	@echo "[BBB] Binaries fetched to $(OUT_DIR)/"

bbb-deploy: bbb-build
	@echo "[BBB] Installing firmware..."
	ssh $(BBB_USER)@$(BBB_IP) " \
	    set -e ; \
	    sudo cp $(BBB_BUILD_DIR)/$(FW_NAME_0) /lib/firmware/ ; \
	    sudo cp $(BBB_BUILD_DIR)/$(FW_NAME_1) /lib/firmware/ ; \
	    sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc1/state 2>/dev/null; true' ; \
	    sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true' ; \
	    sleep 1 ; \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc1/state' ; \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state' ; \
	    echo '[BBB] PRUs reloaded.' ; \
	    cat /sys/class/remoteproc/remoteproc1/state ; \
	    cat /sys/class/remoteproc/remoteproc2/state"

# ── BBB deploy test-spindle ──────────────────────────────────────────────────
bbb-deploy-test-spindle: bbb-build
	ssh $(BBB_USER)@$(BBB_IP) " \
	    set -e ; \
	    sudo cp $(BBB_BUILD_DIR)/$(FW_NAME_TEST_SP) /lib/firmware/$(FW_NAME_1) ; \
	    sudo sh -c 'echo stop  > /sys/class/remoteproc/remoteproc2/state 2>/dev/null; true' ; \
	    sleep 1 ; \
	    sudo sh -c 'echo start > /sys/class/remoteproc/remoteproc2/state' ; \
	    cat /sys/class/remoteproc/remoteproc2/state"

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -rf $(OUT_DIR)
