pickup-winder DTS overlays
==========================

This folder contains DT overlays grouped by logical function. u-boot on the
target typically allows up to 4 overlay slots at boot; group overlays so you
can enable the set you need (the repo provides 4 canonical overlays).

Overlays in this directory:
- 00-gpio.dts    : All GPIO pinmux (steppers PRU0 + endstops PRU1 + footswitch P9_23).
                   Attaches pru_endstop_pins + host_gpio_input_pins → &gpio1
                   Attaches pru_step_pins → &gpio3
                   **Remplace l'ancien 00-pru-steppers-endstops.dts.**
- 02-eqep.dts    : eQEP overlays (eQEP0 on P9_42/P9_27, eQEP2 on P8_12/P8_11).

Build instructions (on dev host):

```bash
cd src/dts
# compile a single overlay
dtc -O dtb -b 0 -@ 00-gpio.dts -o 00-gpio-00A0.dtbo
# or compile all overlays at once (from project root)
make
```

Deploy to BeagleBone (example):

```bash
sudo cp 00-steppers-00A0.dtbo /lib/firmware/
# edit /boot/uEnv.txt to add e.g. uboot_overlay_addr4=/lib/firmware/00-steppers-00A0.dtbo
# reboot
```

Notes:
- Keep the number of overlays enabled at boot ≤ 4 (u-boot/uboot_overlay slots).
- If you need more complex pinmux combinations, combine pinctrl blocks into one
  overlay to stay within the 4-overlay limit.
