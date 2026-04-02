# Makefile — PRU firmware build (pru-gcc toolchain)
#
# Prerequisites on BeagleBone Black:
#   sudo apt install gcc-pru  (or: pru-gcc from TI PRU toolchain)
#
# Usage:
#   make          → build pru0_spindle.fw + pru1_lateral.fw
#   make clean    → remove build artefacts
#   make install  → copy .fw to /lib/firmware/ and reload PRUs
#
# Firmware files are named am335x-pru0-fw and am335x-pru1-fw to match the
# default remoteproc firmware names expected by the kernel driver.

PRU_CC     ?= pru-gcc
PRU_CFLAGS  = -O2 -Wall -Wextra -mmcu=am335x-pru \
              -I../include \
              -DPRU_FIRMWARE

OUT_DIR     = build
FW_NAME_0   = am335x-pru0-fw
FW_NAME_1   = am335x-pru1-fw

.PHONY: all clean install

all: $(OUT_DIR)/$(FW_NAME_0) $(OUT_DIR)/$(FW_NAME_1)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(OUT_DIR)/$(FW_NAME_0): pru0_spindle/main.c ../include/pru_ipc.h ../include/pru_ramp.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[PRU0 spindle] built → $@"

$(OUT_DIR)/$(FW_NAME_1): pru1_lateral/main.c ../include/pru_ipc.h ../include/pru_ramp.h | $(OUT_DIR)
	$(PRU_CC) $(PRU_CFLAGS) -o $@ $<
	@echo "[PRU1 lateral] built → $@"

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

clean:
	rm -rf $(OUT_DIR)
