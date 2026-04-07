# Makefile for the PickupWinder hardware daemon.
# This file is included by the top-level Makefile via make -C src/linux/daemon.

OUT_DIR ?= $(CURDIR)/../../build/daemon
DAEMON_SRC := pickup_daemon.c
DAEMON_BIN := $(OUT_DIR)/pickup_daemon

HOST_ARCH := $(shell uname -m)
TOOLCHAIN_DIR ?= $(HOME)/x-tools

DAEMON_CC ?= $(shell command -v arm-linux-gnueabihf-gcc 2>/dev/null || true)
ifeq ($(HOST_ARCH),x86_64)
ifneq ($(strip $(DAEMON_CC)),)
else
  DAEMON_CC := $(shell ls $(TOOLCHAIN_DIR)/*/bin/arm-linux-gnueabihf-gcc 2>/dev/null | head -n1)
  ifeq ($(strip $(DAEMON_CC)),)
    DAEMON_CC := $(shell ls $(TOOLCHAIN_DIR)/*/bin/*arm*-linux-gnueabihf-gcc 2>/dev/null | head -n1)
  endif
  ifeq ($(strip $(DAEMON_CC)),)
    DAEMON_CC := $(shell ls $(TOOLCHAIN_DIR)/*/bin/*arm*gnueabihf-gcc 2>/dev/null | head -n1)
  endif
  ifeq ($(strip $(DAEMON_CC)),)
    $(error No ARM cross-compiler found in PATH or $(TOOLCHAIN_DIR). Install arm-linux-gnueabihf toolchain or set DAEMON_CC.)
  endif
endif
else
  DAEMON_CC ?= gcc
endif

DAEMON_CFLAGS ?= -O2 -Wall -Wextra -I ../../pru/include

.PHONY: all clean

all: $(DAEMON_BIN)

$(DAEMON_BIN): $(DAEMON_SRC)
	@mkdir -p $(OUT_DIR)
	@echo "=== Building daemon ==="
	$(DAEMON_CC) $(DAEMON_CFLAGS) $(DAEMON_SRC) -o $(DAEMON_BIN)
	@echo "  -> $(DAEMON_BIN)"

clean:
	rm -rf $(OUT_DIR)
	@echo "Cleaned daemon build/"
