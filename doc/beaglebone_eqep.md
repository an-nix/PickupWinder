# BeagleBone eQEP (Kernel 6.6+)

Purpose: Concise checklist for using the modern Linux `counter` framework
with the AM335x eQEP peripheral and avoiding common "busy" or zero-count
failures.

- Hardware & overlay:
  - eQEP1 is part of EPWMSS1 (physical base `0x48302000`).
  - eQEP1 device address (counter child): `0x48302180` (appears as `48302180.counter`).
  - Pins to configure for eQEP1: **P8.33** and **P8.35** must be set to the QEP
    function (MODE2 on some images) — use the pad offsets and muxmode required by your image.
  - Always use 3-cell pin entries in overlays for kernel 6.x: `<offset config_flags mux_mode>`
    (example: `0x0D4 0x30 0x02`).

- Driver & sysfs location:
  - Modern driver: `ti-eqep-cnt` (counter framework). Do not rely on legacy
    `/sys/devices/.../eqep*` paths.
  - The device shows up on the counter bus: `/sys/bus/counter/devices/counterX/`.
  - The driver node binds to the device at the physical address: `48302180.counter` — verify with
    `readlink -f /sys/bus/counter/devices/counter0/of_node`.

- Runtime configuration (critical):
  - `count0/ceiling`: set to `4294967295` (32-bit max). If `ceiling` is left at `0`,
    the counter will remain at zero.
    - `echo 4294967295 | sudo tee /sys/bus/counter/devices/counter0/count0/ceiling`
  - `count0/function`: set to `quadrature x4` for full quadrature counting.
    - `echo 'quadrature x4' | sudo tee /sys/bus/counter/devices/counter0/count0/function`
  - `count0/enable`: set to `1` to start counting.
    - `echo 1 | sudo tee /sys/bus/counter/devices/counter0/count0/enable`

- Diagnostics & sanity checks:
  - Confirm pinmux: `show-pins | grep -E 'P8\.(33|35|31|32)'` (or inspect DT overlay produced values / devmem2 pad registers).
  - Confirm counter device: `ls /sys/bus/counter/devices/` should list `counter0` (or similar).
  - Confirm driver binding: `ls -l /sys/bus/platform/devices/48302180.counter/driver` (should point to `ti-eqep-cnt`).
  - Confirm runtime config:
    - `cat /sys/bus/counter/devices/counter0/count0/ceiling`
    - `cat /sys/bus/counter/devices/counter0/count0/function`
    - `cat /sys/bus/counter/devices/counter0/count0/enable`
  - Check `signal0_action` / `signal1_action` — for quadrature both signals should be active
    (e.g. `both edges`). If one signal is `none` the counter will not increment.

- Common failure modes:
  - `ceiling=0` → counter stays at zero even with signals present.
  - `function` not set to `quadrature x4` → wrong counting mode.
  - `enable=0` → counter disabled.
  - Pinmux still in old 2-cell format → only first pin applied; second pin remains GPIO.
  - `pinctrl-0` attached to `&eqep1` may cause only one pin to be claimed on some kernels; prefer `bone-pinmux-helper` under `&ocp` to apply pinctrl.

Add this document as a companion to `.github/copilot-instructions.md` or paste the section into that file when updating the central instructions.
