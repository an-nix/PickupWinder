#!/bin/bash
# Check environment and help debug eQEP overlay/pinmux on BeagleBone
set -euo pipefail

DTBO=BB-EQEP-EXAMPLE-00A0.dtbo
FWDIR=/lib/firmware

echo "1) Kernel and dmesg (recent lines for eqep/counter/epwmss)"
uname -a
echo
echo "dmesg | egrep -i 'eqep|epwmss|ti-eqep|counter' (last 80 lines)"
dmesg | egrep -i 'eqep|epwmss|ti-eqep|counter' | tail -n 80 || true

echo
echo "2) Check /boot/uEnv.txt for loaded overlays (video overlays may reserve pins)"
if [ -r /boot/uEnv.txt ]; then
  echo "--- /boot/uEnv.txt ---"
  grep -E 'uboot_overlay_addr|uboot_overlay' /boot/uEnv.txt || true
  echo "----------------------"
else
  echo "/boot/uEnv.txt not readable"
fi

echo
echo "3) Check that DTBO is present locally"
if [ -f "$DTBO" ]; then
  echo "Local DTBO present: $DTBO"
else
  echo "No local DTBO found in $(pwd). Run: make dtbo" >&2
fi

echo
echo "4) List overlays already loaded via configfs (/sys/kernel/config/device-tree/overlays)"
if [ -d /sys/kernel/config/device-tree/overlays ]; then
  ls -l /sys/kernel/config/device-tree/overlays || true
else
  echo "configfs device-tree overlays not present on this system (CONFIG_OF_CONFIGFS missing)"
fi

echo
echo "5) Check counter sysfs"
ls -la /sys/bus/counter/devices || true
find /sys/bus/counter/devices -maxdepth 3 -type f -name 'count' -print 2>/dev/null || true

echo
echo "6) Query config-pin for P8_33 / P8_35 (if config-pin available)"
if command -v config-pin >/dev/null 2>&1; then
  echo "config-pin status P8_33:"; config-pin -q P8_33 || true
  echo "config-pin status P8_35:"; config-pin -q P8_35 || true
else
  echo "config-pin not installed. Install package: bb-cape-overlays or util-linux? (depends on image)"
fi

echo
echo "7) Show debug pinctrl state if available"
if [ -d /sys/kernel/debug/pinctrl ]; then
  echo "(listing pinctrl debug)"; find /sys/kernel/debug/pinctrl -maxdepth 2 -type f -print -exec sed -n '1,120p' {} \; 2>/dev/null || true
else
  echo "/sys/kernel/debug/pinctrl not available"
fi

echo
echo "8) Quick-test: enable pull-ups temporarily via config-pin (requires config-pin)"
echo "Run these on target if you want to force pulls temporarily for testing:"
cat <<'EOF'
sudo config-pin P8_33 gpio_pu
sudo config-pin P8_35 gpio_pu
# Verify:
config-pin -q P8_33
config-pin -q P8_35
EOF

echo
echo "9) If overlay not applied, install dtbo and reboot (U-Boot overlay method):"
cat <<'EOF'
sudo cp "${DTBO}" ${FWDIR}/
# Add to /boot/uEnv.txt (example using uboot_overlay_addr4):
echo 'Add or edit in /boot/uEnv.txt: uboot_overlay_addr4=/lib/firmware/BB-EQEP-EXAMPLE-00A0.dtbo'
echo 'Then reboot: sudo reboot'
EOF

echo
echo "10) After reboot, re-check:
  dmesg | egrep -i 'eqep|epwmss|ti-eqep|counter'
  ls -la /sys/bus/counter/devices
  python3 read_eqep.py --once"

echo
echo "Script complete. If pull-ups remain inactive after overlay application, use 'config-pin ... gpio_pu' as workaround, then investigate if a conflicting overlay (e.g. video) is reserving pins." 

exit 0
