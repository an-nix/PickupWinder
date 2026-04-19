# Copilot Instructions — PickupWinder (RPi + ESP32)

> Platform: Raspberry Pi (Python application) + ESP32 (real-time stepper controller).
> ESP32 firmware uses **ESP-IDF framework** (not Arduino). Entry point is `app_main()`.
> Old BeagleBone Black sources are in `resources/esp32/` (reference only — do not modify).

---

## 1. Project Identity & Goals

- **Purpose**: Automated/assisted guitar pickup coil winding — precise lateral
  traversal, real-time speed control, wire tension control, recipe persistence.
- **Hardware**: ESP32 (dual-core FreeRTOS, ESP-IDF), three A4988/DRV8825 stepper drivers
  (bobbin + lateral + tensioner), two HX711 load cells, 2-contact home sensor,
  potentiometer on GPIO 36, quadrature encoder on GPIO 0/15, SPI link to Raspberry Pi.
- **Core constraint**: Firmware drives physical motors under wire tension.
  Correctness and determinism always outweigh elegance.

---

## 2. Architecture Overview

Two-processor architecture: Raspberry Pi (Python) ↔ ESP32 (C++/FreeRTOS/ESP-IDF) over SPI.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Python application  (asyncio, Raspberry Pi)                         │
│  demo_spi.py · streamer.py · ramp.py                                │
│                      │ spidev SPI0, 4 MHz                            │
│            Fixed 512-byte SPI frames (CRC16 + sequence)             │
├──────────────────────────────────────────────────────────────────────┤
│  ESP32  (240 MHz, FreeRTOS/ESP-IDF, dual-core)                       │
│                                                                      │
│  Core 0 (priority  5): sensor_task                                  │
│    HX711 non-blocking poll every 1 ms (~80 Hz actual)               │
│    ADC1 potentiometer every 20 ms (~50 Hz)                          │
│    PCNT quadrature encoder every 1 ms                               │
│    Writes SensorState g_sensor under portMUX spinlock               │
│                                                                      │
│  Core 0 (priority 10): spi_task                                     │
│    Receives frame → validates CRC16 → dispatches payload            │
│    Sends StatusPayload (queue/ring/underrun/result fields)          │
│                                                                      │
│  Core 1 (priority 24): stepper_task                                 │
│    Executes motion queue blocks                                     │
│    Expands SEGMENT_BLOCK into concrete step timings                 │
│    Feeds RMT simple_encoder via software ring                       │
└──────────────────────────────────────────────────────────────────────┘
```

Key files:
- `src/esp32/src/main.cpp` — pin config, `app_main()`
- `src/esp32/src/messages.h` — fixed SPI frame protocol + payloads
- `src/esp32/src/comm_interface.h/.cpp` — SPI slave task + message dispatch
- `src/esp32/src/step_types.h` — shared step/segment constants and types
- `src/esp32/src/stepper_queue.h/.cpp` — motion queue + segment expansion
- `src/esp32/src/stepper_driver.h/.cpp` — RMT streaming driver + ring buffer
- `src/esp32/src/endstop.h/.cpp` — 2-contact endstop ISR + homing
- `src/esp32/src/hx711.h/.cpp` — HX711 bitbang driver (no PID — Pi handles PID)
- `src/esp32/src/encoder.h/.cpp` — PCNT quadrature decoder (GPIO 0/15)
- `src/esp32/src/pot.h/.cpp` — ADC1 potentiometer driver (GPIO 36)
- `src/esp32/src/sensor_task.h/.cpp` — Core 0 sensor acquisition task (pri 5)
- `src/rpi/messages.py` — Python mirror of `messages.h`
- `src/rpi/ramp.py` — segment planner / generators
- `src/rpi/spi_transport.py` — SPI frame transport
- `src/rpi/streamer.py` — sequence-aware block streamer

---

## 3. SPI Protocol

Full-duplex, SPI Mode 0, 4 MHz. Each transfer exchanges one fixed-size frame.

- Frame size: **512 bytes**
- Header size: **12 bytes** (`SpiMessageHeader`)
- Integrity: **CRC16-CCITT**
- Endianness: little-endian

Primary motion message:

- `MULTI_AXIS_SEGMENT_BLOCK` (`0x13`)
  - payload: `MultiAxisSegmentBlockPayload`
  - contains synchronised multi-axis time segments (`step_counts`, `duration_us`, `direction_mask`)
  - expanded by ESP32 multiAxisExecutorTask via drain loop before RMT start

Legacy/debug motion messages:

- `SEGMENT_BLOCK` (`0x11`) — single-axis arithmetic segments
- `STEP_BLOCK` (`0x10`) — explicit per-step intervals (debug only)

Status response:

- message type `STATUS` (`0x80`)
- payload `StatusPayload` with:
  - queue free slots
  - ring free slots
  - underrun counts
  - last RX sequence/type/result
  - enabled/running masks

## 5. Hard Rules

### 5.1 ESP32 Firmware

- **Framework is ESP-IDF**, not Arduino. Use `app_main()`, not `setup()/loop()`.
  Never use Arduino HAL functions (`digitalWrite`, `digitalRead`, `millis`, `pinMode`, etc.).
  Use ESP-IDF equivalents: `gpio_set_level()`, `gpio_get_level()`, `esp_timer_get_time()`, `gpio_config()`.
- **NEVER** call `vTaskDelay()` or blocking I/O from a timer ISR.
- All timer ISRs must be `IRAM_ATTR` and declared as free functions with `void(*)(void*)` signature.
- `IRAM_ATTR` placement: on the function signature line in the `.cpp` file, not on the declaration in the header.
- The `SensorState g_sensor` is protected by a `portMUX_TYPE` spinlock.
  Always use `portENTER_CRITICAL / portEXIT_CRITICAL` for cross-core access.
- `volatile` required on all shared-state fields in `Axis` that are written by the ISR
  and read by the stepper task or SPI task.
- `AxisPins.endstop_no` / `endstop_nc` — set to -1 when not used.
  Never assume a pin is valid without checking `>= 0`.
- HX711 reads are **non-blocking**: check `gpio_get_level(DOUT) == 0` first.
  Never spin-wait for DOUT in sensor_task tight loop.
- GPIO 12 must not be driven HIGH at boot.
- For GPIO >= 32, use `GPIO.out1_w1ts.val` / `GPIO.out1_w1tc.val` for fast bit-bang.
  `GPIO.out_w1ts` only affects GPIO 0–31.

#### Motion execution invariants (NEVER violate)

- `encode_steps()` and `on_trans_done_isr()` must always be `IRAM_ATTR`.
- `s_rx_frame`, `s_tx_frame_a`, `s_tx_frame_b` must always have `DMA_ATTR`.
- Ring buffer is lock-free SPSC: **only Core 1 writes `ring_write_`**, **only the ISR writes `ring_read_`**. Never add locks around these.
- `executeConstantRateBlock()` must **NEVER** call `maybeStartDriver()`.
  The drain loop in `multiAxisExecutorTask` owns the start decision.
- `kickStart()` inside the drain loop is called **ONCE per batch**, after all available blocks are written to the ring.
- `STEP_STREAM_START_FILL` must always be `>= 2 * PART_SIZE`.
  The `static_assert` in `stepper_queue.cpp` enforces this at compile time.

### 5.2 Python Application

- Python **never** touches GPIO directly.
- SPI access must go through `src/rpi/spi_transport.py`.
- Motion planning belongs on host (`ramp.py` / kinematics modules).
- Host should stream **MULTI_AXIS_SEGMENT_BLOCK** messages for production use.
- `MAX_INFLIGHT_SEGMENTS = 24` in `streamer.py` — do not raise above 24.
- For `src/rpi` host code, prefer absolute bare imports such as `from motion import ...`, `from transport import ...`, `from domain import ...`; do not use `rpi.*` or conditional relative imports in this folder.

### 5.3 Protocol

- CRC16-CCITT over each frame header+payload.
- Fixed frame size = 512 bytes.
- `messages.h` and `messages.py` must stay bit-identical (sizes, ordering, packing).
- Sequence/result semantics are pipelined by one SPI transaction; keep host
  confirmation logic sequence-aware.

### Stepper axes

| Axis            | STEP | DIR | EN  | Notes                  |
|-----------------|------|-----|-----|------------------------|
| 0 — Bobbin      | 26   | 27  | 14  | No endstop             |
| 1 — Lateral     | 32   | 33  | 25  | 2-contact home sensor  |
| 2 — Tensioner   | 16   | 17  | 4   | No dedicated endstop   |

EN pins: active LOW (driver ON when GPIO = LOW).

### Lateral home sensor (2-contact)

Both pins configured as input with pull-up (`gpio_config()`, `GPIO_PULLUP_ONLY`):

| Contact | GPIO | Away       | At home    | Fault      |
|---------|------|------------|------------|------------|
| NO      | 21   | HIGH (open)| LOW (closed)| LOW       |
| NC      | 22   | LOW (closed)| HIGH (open)| LOW       |

Valid home: NO=LOW AND NC=HIGH.
Fault (disconnected): NO=LOW AND NC=LOW.

> ⚠️ Old `Config.h` had `HOME_PIN_NO = 23`. GPIO 23 is now SPI MOSI.
> The NO contact was moved to **GPIO 21**.

### SPI (VSPI / SPI3)

| Signal | GPIO |
|--------|------|
| MOSI   | 23   |
| MISO   | 19   |
| SCLK   | 18   |
| CS     | 5    |

### HX711 load cells

| Sensor     | SCK    | DOUT   | Notes                                    |
|------------|--------|--------|------------------------------------------|
| Tension[0] | GPIO 13| GPIO 34| Read by ESP32 and forwarded to RPi       |
| Aux[1]     | GPIO 12| GPIO 39| Read by ESP32 and forwarded to RPi       |

GPIO 12: strapping pin — must be LOW at boot. HX711 SCK idle = LOW ✓.
GPIO 34, 39: input-only (no OUTPUT capability). DOUT only ever needs to be read.

### Potentiometer & Encoder

| Function       | GPIO | Notes                                                     |
|----------------|------|-----------------------------------------------------------|
| Potentiometer  | 36   | ADC1_CH0 (VP), input-only, 12-bit, 32-sample MA ~50 Hz   |
| Encoder A      | 0    | PCNT_UNIT_0 — strapping pin, keep safe level at boot |
| Encoder B      | 15   | PCNT_UNIT_0 — strapping pin, keep safe level at boot |

> Old `Config.h` `POT_PIN = 34` is occupied by HX711[0] DOUT. Pot moved to GPIO 36.
> Old encoder pins (GPIO 18/19) are now SPI SCLK/MISO.
> GPIO0/15 are used for the manual encoder. These are strapping pins on
> many ESP32 modules — avoid driving the encoder during reset/flash or add
> pull resistors to guarantee a safe boot level.

> Note: UART for TMC2209 is handled by the Raspberry Pi, not the ESP32.

### Other

| Signal | GPIO | Notes                          |
|--------|------|--------------------------------|
| —      | —    | No dedicated E-STOP pin used   |

Any pin change **must** update `src/esp32/src/main.cpp` + `doc/architecture.md` + this file.

---

## 5. Hard Rules

### 5.1 ESP32 Firmware

- **Framework is ESP-IDF**, not Arduino. Use `app_main()`, not `setup()/loop()`.
  Never use Arduino HAL functions (`digitalWrite`, `digitalRead`, `millis`, `pinMode`, etc.).
  Use ESP-IDF equivalents: `gpio_set_level()`, `gpio_get_level()`, `esp_timer_get_time()`, `gpio_config()`.
- **NEVER** call `vTaskDelay()` or blocking I/O from a timer ISR.
- All timer ISRs must be `IRAM_ATTR` and declared as free functions with `void(*)(void*)` signature.
- `IRAM_ATTR` placement: on the function signature line in the `.cpp` file, not on the declaration in the header.
- The `SensorState g_sensor` is protected by a `portMUX_TYPE` spinlock.
  Always use `portENTER_CRITICAL / portEXIT_CRITICAL` for cross-core access.
- `volatile` required on all shared-state fields in `Axis` that are written by the ISR
  and read by the stepper task or SPI task.
- `AxisPins.endstop_no` / `endstop_nc` — set to -1 when not used.
  Never assume a pin is valid without checking `>= 0`.
- HX711 reads are **non-blocking**: check `gpio_get_level(DOUT) == 0` first.
  Never spin-wait for DOUT in sensor_task tight loop.
- GPIO 12 must not be driven HIGH at boot.
- For GPIO >= 32, use `GPIO.out1_w1ts.val` / `GPIO.out1_w1tc.val` for fast bit-bang.
  `GPIO.out_w1ts` only affects GPIO 0–31.

### 5.2 Python Application

- Python **never** touches GPIO directly.
- SPI access must go through `src/rpi/spi_transport.py`.
- Motion planning belongs on host (`ramp.py` / kinematics modules).
- Host should stream **segment blocks**, not explicit per-step blocks, for
  production speed ranges.

### 5.3 Protocol

- CRC16-CCITT over each frame header+payload.
- Fixed frame size = 512 bytes.
- `messages.h` and `messages.py` must stay bit-identical (sizes, ordering, packing).
- Sequence/result semantics are pipelined by one SPI transaction; keep host
  confirmation logic sequence-aware.

---

## 6. Acceleration Model (Klipper-style)

```
for each segment:
  ticks = start_ticks
  for i in range(step_count):
    emit_step(ticks)
    ticks += add_ticks
```

Segments are computed on the host and sent as `SEGMENT_BLOCK` payloads.
ESP32 expands to concrete steps and streams them through RMT.

---

## 7. HX711 & Sensor Architecture

- **sensor_task** (Core 0, priority 5) owns all sensor acquisition.
  It calls `hx711_tick()`, `pot_read()`, and `encoder_get_and_clear_delta()`.
- HX711 tick fires every 1 ms but only reads when `gpio_get_level(DOUT) == 0` (data ready, ~80 Hz).
- Potentiometer: `pot_read()` every 20 ms, 32-sample moving average, ~50 Hz.
- Encoder: PCNT hardware (4X quadrature), `encoder_get_and_clear_delta()` every 1 ms.
- **No PID on the ESP32.** Tension PID runs on the Raspberry Pi.
  The Pi sends back a setpoint via `SET_TENSION`; the ESP32 stores it in `g_sensor.tension_setpoint`.
- Output unit: 0.1 g (decigrams). Range: ±3276.7 g in int16_t.
- `hx711_tare()` is blocking — call only during machine idle via `sensor_request_tare()`.

---

## 8. Common Pitfalls

| Pitfall | Why it matters |
|---------|----------------|
| Using Arduino API (digitalWrite, millis, etc.) | Framework is ESP-IDF — Arduino symbols are undefined |
| `IRAM_ATTR` on declaration not definition | ISR can end up in flash, causing runtime faults/jitter |
| Missing `volatile` on ISR/task shared fields | Compiler may cache stale values |
| Blocking in HX711 tick | `gpio_get_level(DOUT)` before reading; never spin-wait in the sensor loop |
| GPIO 12 HIGH at boot | Causes ESP32 flash voltage issue on some modules |
| GPIO 34-39 used as output | Input-only — any `gpio_set_level()` silently ignored |
| GPIO >= 32 with `out_w1ts` | Must use `out1_w1ts.val` for GPIO 32–39 in fast ISR code |
| Protocol packing mismatch | `messages.h` / `messages.py` size mismatch breaks decode/CRC |
| Host uses `STEP_BLOCK` for high speed | SPI bandwidth bottleneck causes underruns |
| Sequence handling ignored on host | false ACK/NACK interpretation and silent drops |
| portMUX not used on shared sensor/status data | tasks run on different cores concurrently |
| HX711 scale = 0 | Division by zero in `raw_to_dg()` — always validate before calibrating |
| Start stream with too little ring fill | high-rate runs underrun before host can refill |
| `executeConstantRateBlock()` calls `maybeStartDriver()` | Starts RMT too early (after 1 segment = 2-5 steps); causes per-segment underruns at low speed |
| `kickStart()` inside per-segment loop | Starts RMT before ring is pre-filled; use the post-drain batch kickStart instead |
| `STEP_STREAM_START_FILL < 2 * PART_SIZE` | Breaks the static_assert in stepper_queue.cpp |
| `MAX_INFLIGHT_SEGMENTS > 24` on host | Risk of firmware queue overflow and deferred notification backlog |

## 8b. Debugging with Status Logs

Key `StatusPayload` fields to watch during a motion run:

| Field | Healthy range | Problem if... |
|---|---|---|
| `ring_free[0]` | < 3900 during cruise | = 4096 → motor stopped / ring empty |
| `underrun[0]` | 0 during `running=1` | any non-zero = real ring starvation |
| `running` | `0x01` after first kickStart | stays `0x00` → kickStart not firing |
| `inflight` (host) | 20–24 during steady motion | < 10 → host not sending fast enough |
| `buf` (host ms) | 80–100 ms | = 0 → all segments executed, move done |

`underrun` during `running=0` at move **start** is benign — the RMT fires a
pause symbol during the first encoder callback before the ring is seeded.

---

## 9. Quick Reference: Adding a New Feature

1. **New message type**: add enum in `messages.h` and `messages.py`.
2. **New payload**: add packed struct/dataclass mirror; update static asserts.
3. **Dispatch**: handle in `comm_interface.cpp::handleFrame()`.
4. **Execution path**: enqueue through `StepperQueue` as `motion_block_t` when motion-related.
5. **Host API**: add helper in `spi_transport.py` and streamer/planner support if needed.

---

## 10. File Organization

```
.github/                    CI + copilot instructions
src/esp32/                  ESP32 PlatformIO project (C++17, ESP-IDF)
  src/
    messages.h              512-byte frame protocol + payloads
    comm_interface.h/.cpp   SPI slave transport + dispatch
    step_types.h            Step/segment constants and core structs
    stepper_queue.h/.cpp    Motion queue + segment expansion
    stepper_driver.h/.cpp   RMT output + software ring
    endstop.h/.cpp          2-contact ISR + homing
    hx711.h/.cpp            HX711 bitbang driver (no PID)
    encoder.h/.cpp          PCNT quadrature decoder (GPIO 0/15)
    pot.h/.cpp              ADC1 potentiometer driver (GPIO 36)
    sensor_task.h/.cpp      Core 0 sensor acquisition task (pri 5)
    main.cpp                Pin config, app_main()
  platformio.ini
src/rpi/                    Raspberry Pi Python application
  messages.py               Python mirror of `messages.h`
  ramp.py                   Segment planners/generators
  spi_transport.py          Frame transport + sequence helpers
  streamer.py               Streaming/backpressure logic
  demo_spi.py               Segment-streaming demo
doc/                        Architecture docs
  architecture.md           Full design reference
resources/                  Reference code (DO NOT MODIFY)
  esp32/                    Old standalone ESP32 project
  FastAccelStepper/         Behavioral reference for queue/RMT model
```

---

## 11. Hardware Constants

| Parameter           | Value              | Notes                              |
|---------------------|--------------------|------------------------------------|
| Spindle steps/rev   | 6400               | 200 full × 32 µstep                |
| Lateral steps/mm    | 3072               | 96 full × 32 µstep, M6 1 mm pitch  |
| Speed range (Hz)    | 100 – 160 000      | ~0.9 – 1500 RPM at 6400 steps/rev  |
| RMT resolution      | 2 MHz              | 1 tick = 0.5 µs                    |
| HX711 sample rate   | ~80 Hz             | At VCC ≥ 4.8 V (RATE pin = HIGH)   |
| HX711 output unit   | 0.1 g (decigram)   | int16_t, range ±3276.7 g           |
| Potentiometer range | 0 – 4095           | 12-bit ADC1_CH0, 32-sample MA      |
| Encoder interface   | PCNT_UNIT_0, 4X    | GPIO 0 (A) / GPIO 15 (B)           |

---

## 12. Winding Kinematics (Electronic Gearing)

All winding geometry computation runs on the Raspberry Pi host using an Electronic Gearing architecture.
The ESP32 receives only multi-axis segments (`MULTI_AXIS_SEGMENT_BLOCK`) natively synced by the generator and has no knowledge
of wire geometry or synchronization.

### The Electronic Gearing Model

The generation is split into four decoupled components in `src/rpi/motion/ramp.py`:

- **`SpindleKinematics`** (The Master): Calculates the theoretical absolute angular position (in turns) of the Bobbin over time, integrating acceleration, cruise RPM, and deceleration.
- **`WindingPattern`** (The Slave function): A pure mathematical function converting a Spindle position (turns) into a Traverse position (mm). For a standard coil, this is a triangular wave constrained between 0 and `bobbin_width_mm` with a slope of `turns_per_mm`.
- **`ScatterEngine`**: Introduces a spatial, non-harmonic offset to the Traverse position to avoid exact wire stacking. Uses an edge-damping factor to automatically kill the offset at the spool flanges (0 and `bobbin_width_mm`) to prevent wire spillage.
- **`SynchronizedSegmentGenerator`**: The main iterator spanning the time domain by `segment_duration_s` steps. Computes target positions for both axes, applies a global `round(target_steps - current_steps)` to eliminate cumulative floating-point errors, and chunks the delta steps into `MultiAxisSegment` payloads for the ESP32.

### Manual Mode / Single Axis

For UI jog buttons, homing moves, or individual axis tests, the system transparently utilizes the older `MultiAxisSegmentGenerator` fed by `AxisMotionConfig` arrays.
Both `SynchronizedSegmentGenerator` (winding) and `MultiAxisSegmentGenerator` (jogging) output standard `MultiAxisSegment` objects. The host `MultiAxisRampStreamer` does not need to know which one is driving it.

