pickup-winder DTS overlays
==========================

This folder contains DT overlays grouped by logical function. u-boot on the
target typically allows up to 4 overlay slots at boot; group overlays so you
can enable the set you need (the repo provides 4 canonical overlays).

Overlays in this directory:
- 00-pru-steppers-endstops.dts : Combined PRU overlay (steppers on PRU0 + endstops on PRU1)
- 02-host_IO.dts               : Host I/O overlay (encoders eqep0+eqep1 + UART1 pins + P9_23 stop/start button input)

Build instructions (on dev host):

```bash
cd src/dts
# compile a single overlay
dtc -O dtb -b 0 -@ 00-pru-steppers-endstops.dts -o 00-pru-steppers-endstops-00A0.dtbo
# or compile selected overlays (keep to ≤4 overlays for u-boot)
for f in 00-pru-steppers-endstops.dts 02-host_IO.dts; do
  dtc -O dtb -b 0 -@ "$f" -o "${f%.dts}-00A0.dtbo"
done
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
