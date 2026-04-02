Python prototype for the PickupWinder Linux userland.

This folder contains a small prototype daemon and helpers to:
- Map PRU shared RAM via /dev/mem (`ipc.py`)
- Read the BeagleBone ADC for the potentiometer (`pot.py`)
- Read a rotary encoder via sysfs GPIO polling (`encoder.py`)
- Small daemon that ties them together (`daemon.py`)

Usage (on target as root):

    python3 port/beaglebone/linux/python/daemon.py

Notes:
- This is a prototype for fast iteration. For production you may move
  code to a system service, add proper error handling, and prefer the
  `gpiod` bindings or a PRU-based encoder for robustness against EMI.
