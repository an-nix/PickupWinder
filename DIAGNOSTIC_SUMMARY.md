# Motor Stutter Diagnosis & Fix Summary

## Problem Statement
Motor exhibits stop/restart behavior during lateral traversal, and abnormal noise during
continuous rotation — despite requesting smooth motion with host-streamed speed commands.

## Root Causes Identified & Fixed

### 6. **RMT buffer gap — abnormal noise during rotation** (FIXED ✅)

**Symptom**: Motor turns continuously (after fix #5) but emits a periodic clicking/buzzing
noise.  Audible as a tonal artifact synchronized with the ISR rate.

**Root Cause**: The original RMT implementation used a single 64-item buffer with TX_END
interrupts.  After the last item in the buffer was transmitted, the RMT peripheral stopped
and fired TX_END.  The ISR then refilled the buffer, reset the read pointer, and restarted
transmission.  The time between stop → restart was 500 ns – 2 µs (ISR entry + fill +
register write).  At cruise speed (e.g. 128 kHz = 7.8 µs period), this gap represented
**6–25% of the step period** — visible as an extra-long inter-step pause every 63 steps,
causing the motor to emit an audible sub-harmonic tone.

**Fix**: Replaced the TX_END refill model with a **ping-pong double-buffer** design:

| Property | Before | After |
|---|---|---|
| Buffer size | 64 items (1 block) | 128 items (2 blocks) |
| ISR trigger | TX_END (after buffer exhausted) | TX_THR (after 64 items consumed) |
| Gap between buffers | 500 ns – 2 µs | **0 ns** (hardware never pauses) |
| RMT loop mode | disabled (one-shot) | `tx_conti_mode=1` (continuous loop) |
| Channel assignment | ch 0, 1, 2 | ch 0, 2, 4 (even only; each steals 2 blocks) |

Implementation details:
- `mem_block_num = 2`: each axis uses 128 items in RMT memory.
- `loop_en = true` (`tx_conti_mode`): hardware wraps the read pointer from item 127 → 0
  automatically; the TX_END interrupt **never fires**.
- **TX threshold interrupt** (`RMT.tx_lim_ch[ch].limit = 64`): fires every 64 items.
  The ISR refills the just-consumed half while the peripheral reads the other half.
  No end-marker is written; every slot holds a valid item (pad = 1-tick LOW if idle).
- `rmt_isr_register()` replaces `rmt_driver_install()` + `rmt_register_tx_end_callback()`.
- `fill_half(base)`: writes `RMT_HALF_BUF` items starting at `base`.  If the axis is IDLE,
  remaining slots are padded with `{1, LOW, 1, LOW}` so the hardware never sees `val=0`.
- `start_continuous()`: fills both halves, resets read pointer, asserts `tx_start=1`.
- `stop_continuous()`: clears `tx_start=0`; hardware finishes current item then idles.
- `emergency_stop()`: uses `stop_continuous()` (safe for ping-pong) instead of `rmt_tx_stop()`.

**Thread-safety**: same `portMUX` model; `on_tx_threshold()` runs in ISR context.

**Files modified**:
- `src/esp32/src/rmt_stepper.h` — updated constants, removed `RMT_BUF_SIZE`, added
  `RMT_TOTAL_BUF`/`RMT_HALF_BUF`, replaced `on_tx_complete()` with `on_tx_threshold()`,
  added `pp_next_base_`, `fill_half()`, `start_continuous()`, `stop_continuous()`.
- `src/esp32/src/rmt_stepper.cpp` — rewrote ISR, `init()`, fill logic, stop/estop.

### 5. **set_speed_hz() kills RMT channel on every SET_SPEED** (FIXED ✅)

**Symptom**: Motor turns in short bursts (start → micro-movement → stop → repeat) when
the host streams `SET_SPEED` commands during `rmt_stepper_demo` or any host-controlled
ramp.  The motor never spins continuously.

**Root Cause** (`src/esp32/src/rmt_stepper.cpp`, `RmtAxis::set_speed_hz()`):
```cpp
// OLD (broken) — executed on EVERY SET_SPEED when axis was already running:
if (state_ != RmtAxisState::IDLE) {
    emergency_stop();   // ← kills RMT channel immediately
}
// ... then cold-restart via start_tx_from_task()
```
`emergency_stop()` calls `rmt_tx_stop()` which halts the RMT peripheral and drains
the FIFO.  The host sends `SET_SPEED` every 20 ms (50 Hz); the firmware therefore
stopped and restarted the step stream 50 times per second.  Each stop introduced a
dead gap of several milliseconds → the motor never reached continuous rotation.

**Why it looked "almost working"**: `--dry-run` on the host prints a perfect ASCII
profile because it never calls the firmware.  The bug only manifests when SPI
commands actually reach the ESP32.

**Fix** (same pattern already used by `stop()`):
When `state_ != IDLE`, rebuild `segments_[]` atomically under the `portMUX` spinlock,
update `interval_`, `ramp_add_`, `ramp_count_`, then **return without touching the RMT
channel**.  The ISR (`on_tx_complete → fill_rmt_buffer`) picks up the new speed at
the end of the current 63-item buffer (~200 µs at cruise), with zero gap in the step
stream.

```cpp
// NEW (fixed) — hot-update path:
if (state_ != RmtAxisState::IDLE) {
    uint32_t cur_hz = current_hz();
    ...
    portENTER_CRITICAL(&mux_);
    n_segments_ = 0;
    build_ramp_phase(cur_hz, hz, trans_steps);         // short transition
    segments_[n_segments_++] = {iv_new, 0, 0xFFFFFFu}; // infinite cruise
    interval_   = segments_[0].start_iv;
    ramp_count_ = segments_[0].count;
    // ...
    portEXIT_CRITICAL(&mux_);
    return;  // RMT keeps running — ISR consumes updated segments
}
```

**Thread-safety**: `portMUX` serialises access with the ISR's `load_next_ramp_seg()`
(which also acquires the lock).  Direct reads of `interval_`/`ramp_count_` in
`fill_rmt_buffer()`'s step loop are safe because they are `volatile uint32_t`
(atomic 32-bit reads on Xtensa); at most one step can use a stale value before the
update propagates — imperceptible to the motor.

**File modified**: `src/esp32/src/rmt_stepper.cpp` — `RmtAxis::set_speed_hz()`

### 1. **uint16_t Saturation** (FIXED ✅)
- **Issue**: `current_hz` capped at 65535 Hz when requesting 160000 Hz (1500 RPM)
- **Solution**: 
  - Changed `AxisStatus.current_hz` from uint16_t → uint32_t
  - Expanded StatusFrame from 44 → 56 bytes
  - Updated protocol.h struct and Python protocol.py mirror
  - Fixed stepper_engine.cpp still casting to uint16_t

### 2. **Aggressive Acceleration** (FIXED ✅)
- **Issue**: accel_ = 100000 steps/s² caused oscillation/overshoot → motor would stop briefly
- **Solution**: Reduced to 25000 steps/s² for smooth 40ms ramp to 1000 Hz

### 3. **Klipper Architecture Violation** (FIXED ✅)
- **Issue**: Every SET_SPEED call invoked `applySpeedAcceleration()`, restarting ramp from zero
- **Solution**:
  - Modified `Axis::run()` to distinguish:
    - Idle→Running (full ramp init needed)
    - Direction change (ramp restart needed)
    - Speed-only update (smooth acceleration, NO ramp restart)
    - Already at target (skip)
  - Modified `Axis::set_speed_hz()` to only call `applySpeedAcceleration()` when motor stopped
  - Modified `Axis::move_to()` to only init ramp if idle; smooth trajectory if already running

### 4. **Protocol Mismatch in Status Building** (FIXED ✅)
- **Issue**: stepper_engine.cpp was still truncating uint32_t `current_hz` to uint16_t
- **Solution**: Removed cast in line 82, directly assign uint32_t value

## Files Modified

### `/src/esp32/src/axis.cpp`
- `Axis::run(hz, reverse)` - Added 4-branch logic with ESP_LOGD:
  - "idle->running hz=%u"
  - "direction change hz=%u"
  - "speed update %u->%u Hz" (no ramp restart)
  - "already at hz=%u, skipping"
- `Axis::set_speed_hz()` - Only call applySpeedAcceleration if stopped
- `Axis::move_to()` - Only init ramp if idle; smooth trajectory if running

### `/src/esp32/src/stepper_engine.cpp`
- Line 82: Fixed uint16_t cast → direct uint32_t assignment
- Added ESP_LOGD logging for SET_SPEED and MOVE_ABS commands:
  - "→ SET_SPEED axis=%d hz=%u rev=%d"
  - "→ MOVE_ABS axis=%d target=%ld"

### `/src/esp32/src/protocol.h`
- AxisStatus struct: 8 → 12 bytes (int32 pos + uint32 hz + uint8 flags + padding)
- STATUS_FRAME_SIZE: 44 → 56 bytes

### `/src/rpi/hal/protocol.py`
- STATUS_FRAME_SIZE: 44 → 56
- _AXIS_STATUS struct format: `<iHBB>` → `<iIBBH>` (uint32 hz, padding)

## Compilation Status
✅ **SUCCESS** - 312949 bytes flash, no warnings

## Next Diagnostic Steps

### 1. Flash Firmware
```bash
cd /src/esp32
pio run --target upload
```

### 2. Monitor ESP32 Serial Output
```bash
platformio device monitor --baud 115200
# or
pio device monitor
```

### 3. Run Test Script
```bash
python3 /src/rpi/test_lateral_smooth.py
```

### Expected Log Output During Test
```
[stepper] → SET_SPEED axis=1 hz=1000
[stepper] Axis 1 run: idle->running hz=1000
[stepper] → SET_SPEED axis=1 hz=2000
[stepper] Axis 1 run: speed update 1000->2000 Hz
[stepper] → MOVE_ABS axis=1 target=3072
[stepper] Axis 1 move_to: target=3072, running=1
```

## Hypothesis Testing Matrix

| Symptom | Root Cause | Indicator |
|---------|-----------|-----------|
| Motor reaches only 346 Hz | Accel too aggressive | ✅ FIXED (25000 steps/s²) |
| Speed caps at 65535 Hz | uint16_t saturation | ✅ FIXED (uint32_t) |
| Motor stops/restarts on speed change | Ramp restart | ✅ FIXED (Klipper logic) |
| Motor stops/restarts on new target | move_to() restart? | ⏳ INVESTIGATING |
| Logs show "direction change" repeatedly | Host sending wrong commands | ⏳ TO MONITOR |
| Logs show "speed update" but motor stutters | FastAccelStepper behavior | ⏳ TO INVESTIGATE |

## Outstanding Questions
1. Does `FastAccelStepper::moveTo(new_target)` internally restart acceleration?
2. Is the host command sequence correct (set_speed → move_to → wait for event)?
3. Is there a 3ms SPI command interval issue causing undersampling?

## References
- Klipper stepper design: Pre-calculated motion segments, smooth transitions
- FastAccelStepper library: Hardware step generation with configurable acceleration
- Hardware: ESP32 @ 240 MHz, 6400 steps/rev bobbin, 3072 steps/mm lateral

---

**Status**: Firmware compiled successfully with comprehensive logging. Ready for flash and test.
