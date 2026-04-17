# PickupWinder — RPi + ESP32 Architecture

## Overview

Two-processor architecture: a Raspberry Pi (Python application + HAL) and an ESP32 (real-time stepper controller).
The ESP32 firmware uses the **ESP-IDF framework** (not Arduino). Entry point is `app_main()`.

Current pulse-output path:

`segment planner (RPi)` → `SPI MULTI_AXIS_SEGMENT_BLOCK` → `multiAxisExecutorTask drain loop`
→ `executeConstantRateBlock() → ring buffer` → `RMT simple_encoder` → STEP GPIO

The queue/RMT hand-off uses the Klipper look-ahead drain pattern:
the executor fills the ring from all queued blocks before starting the RMT,
providing maximum buffer depth at motion start.

Protocol definitions: `src/esp32/src/messages.h` / `src/rpi/messages.py`

## Full Data Flow

```
Host (Python — Raspberry Pi)
  │  SPI frames: MULTI_AXIS_SEGMENT_BLOCK (type 0x13)
  │  Frame rate: ~500 Hz (1 frame per 2 ms poll cycle)
  │  Segments per frame: up to MULTI_AXIS_BLOCK_SIZE (60)
  │  Inflight cap: MAX_INFLIGHT_SEGMENTS = 24 (~96 ms at 4 ms/segment)
  │  Buffer target: TARGET_BUFFER_TIME_S = 100 ms
  ▼
CommInterface — Core 0, priority 10
  │  spiTask: double-buffered TX (s_tx_frame_a / s_tx_frame_b ping-pong)
  │  Build order: transmit prev frame → parse RX → handleFrame → build next
  │  Status pipeline: TX reflects state AFTER previous RX was handled
  │  s_multi_axis_queue: depth 64 × multi_axis_block_t
  ▼
multiAxisExecutorTask — Core 1, priority 24
  │  DRAIN LOOP: dequeue first block (blocking, 1 ms timeout), then
  │    non-blocking dequeue of all subsequent available blocks into ring.
  │    No kickStart inside the loop — ring fills without RMT running.
  │  After drain: kickStart() once per active axis (starts RMT).
  │  Deferred notifications: notifySegmentExecuted() fires at wall-clock
  │    finish time of each segment (not at write time).
  │  Timeout path: kickStart() to recover from RMT underrun while waiting.
  ▼
StepperQueue::executeConstantRateBlock()
  │  Computes uniform interval: (duration_us × 80) / step_count ticks
  │  Clamps to [RMT_STEP_MIN_TICKS=16, RMT_STEP_MAX_TICKS=0xFFFF]
  │  Writes to ring via pushExpandedBlock() — does NOT start RMT
  ▼
StepperDriver ring buffer
  │  STEP_RING_SIZE = 4096 entries (lock-free SPSC)
  │  Producer: Core 1 (multiAxisExecutorTask) writes ring_write_
  │  Consumer: ISR (encode_steps) reads ring_read_
  │  Auto-start threshold: STEP_STREAM_START_FILL = 32 steps
  ▼
encode_steps ISR — IRAM_ATTR, RMT clock 80 MHz (12.5 ns/tick)
  │  PART_SIZE = 16 symbols per callback
  │  Balanced pulse: HIGH = ticks/2, LOW = ticks − HIGH
  │  Both halves clamped to ≥ RMT_STEP_PULSE_TICKS (8) = 100 ns
  │  On empty ring: emit pause chunk, set rmt_stopped_=true
  │  Notifies producer_task_ AND executor_task_ on each callback
  ▼
RMT hardware → STEP GPIO → motor driver (A4988/DRV8825) → stepper motor
```

## Key Timing Parameters

| Parameter | Value | Notes |
|---|---|---|
| RMT clock | 80 MHz | 12.5 ns/tick |
| RMT_STEP_MIN_TICKS | 16 | 200 ns min interval |
| Max step rate | 5 000 000 steps/sec | 80e6 / 16 |
| Max RPM (32µstep) | 781 RPM | 5e6 / (200 × 32) |
| Segment duration | 4000 µs | host configurable |
| PART_SIZE | 16 | ISR callback size |
| STEP_RING_SIZE | 4096 | ~820 ms at 5 kHz |
| STEP_STREAM_START_FILL | 32 | = 2 × PART_SIZE |
| MULTI_AXIS_BLOCK_SIZE | 60 | segments per frame |
| MULTI_AXIS_QUEUE_DEPTH | 64 | ~960 ms look-ahead |
| MAX_INFLIGHT_SEGMENTS | 24 | ~96 ms at 4ms/seg |

```
|---------------|------|----------|-------|-----------------------------------------|
| sensor_task   | 0    | 5        | 4 KB  | HX711, ADC pot, PCNT encoder            |
| spi_task      | 0    | 10       | 4 KB  | SPI frame RX/TX, CRC16 validation, dispatch |
| stepper_task  | 1    | 24       | 8 KB  | Motion-block execution, segment expansion, RMT feed |

### sdkconfig Overrides (`src/esp32/sdkconfig.defaults`)

| Key                                    | Value | Reason                                      |
|----------------------------------------|-------|---------------------------------------------|
| `CONFIG_ESP_CONSOLE_UART_NONE`         | y     | Frees UART console (encoder uses GPIO 0/15) |
| `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240`  | y     | Full speed for ISR timing                   |
| `CONFIG_FREERTOS_HZ`                   | 1000  | 1 ms FreeRTOS tick resolution               |

---

## SPI Protocol

Full-duplex, SPI Mode 0, 4 MHz. Every transfer exchanges one fixed-size
512-byte frame.

Frame format:

- 12-byte header: `magic`, `version`, `msg_type`, `sequence`,
  `payload_length`, `flags`, `crc16`
- payload: message-specific packed struct
- CRC16-CCITT over header+payload bytes

Primary motion command:

- `SEGMENT_BLOCK` (`src/esp32/src/messages.h::SegmentBlockPayload`)
  - contains up to `SEGMENT_BLOCK_SIZE` arithmetic motion segments
  - each segment encodes: `step_count`, `start_ticks`, `add_ticks`, `dir`
  - ESP32 expands segments locally into concrete step intervals

Legacy/debug command:

- `STEP_BLOCK` (explicit per-step intervals)

Status response (`StatusPayload`) includes:

- uptime
- queue free slots per axis
- ring free slots per axis
- underrun counters per axis
- last RX sequence/type/result
- enabled and running masks

---

## Pin Assignments

### Stepper Axes

| Axis            | STEP | DIR | EN  | Notes                              |
|-----------------|------|-----|-----|------------------------------------|
| 0 — Bobbin      | 26   | 27  | 14  | Highest frequency, no endstop      |
| 1 — Lateral     | 32   | 33  | 25  | 2-contact home sensor (see below)  |
| 2 — Tensioner   | 16   | 17  | 4   | No dedicated endstop                |

All EN pins are active LOW (driver enabled when GPIO = LOW).

### Lateral Home Sensor (2-contact)

The lateral home sensor has both a NO (normally-open) and NC (normally-closed) contact, both read with `INPUT_PULLUP`:

| State           | NO (GPIO 21) | NC (GPIO 22) | Interpretation          |
|-----------------|-------------|-------------|-------------------------|
| Away from home  | HIGH (open)  | LOW (closed) | Normal travel position  |
| **At home**     | **LOW**      | **HIGH**     | Valid home detected     |
| Fault           | LOW          | LOW          | Wiring fault / short    |

> ⚠️ The original `Config.h` used GPIO 23 for `HOME_PIN_NO`. GPIO 23 is now SPI MOSI — home NO has been moved to **GPIO 21**.

### SPI (VSPI — bus SPI3)

| Signal | GPIO |
|--------|------|
| MOSI   | 23   |
| MISO   | 19   |
| SCLK   | 18   |
| CS     | 5    |

### HX711 Load Cells

| Sensor      | SCK   | DOUT  | Notes                                     |
|-------------|-------|-------|-------------------------------------------|
| Tension [0] | GPIO 13 | GPIO 34 | Read by ESP32 and forwarded to Pi       |
| Auxiliary [1] | GPIO 12 | GPIO 39 | Read by ESP32 and forwarded to Pi       |

GPIO 12 must be LOW at boot — HX711 SCK idle state is LOW, so this is safe.
GPIO 34 and 39 are input-only (5V tolerant), ideal for DOUT.

### Potentiometer & Encoder

| Signal          | GPIO | Notes                                                     |
|-----------------|------|-----------------------------------------------------------|
| Potentiometer   | 36   | ADC1_CH0 (VP), input-only, 12-bit, 32-sample MA ~50 Hz   |
| Encoder A       | 0    | PCNT_UNIT_0 — strap pin (boot strap; ensure safe state at boot) |
| Encoder B       | 15   | PCNT_UNIT_0 — strap pin (boot strap; ensure safe state at boot) |

> The old `Config.h` `POT_PIN = 34` is now occupied by HX711[0] DOUT.
> The potentiometer has moved to GPIO 36 (VP, ADC1_CH0, input-only).
> The old encoder pins (GPIO 18/19) are now SPI SCLK/MISO.
> GPIO0/15 are used for the manual encoder. These are strapping pins on
> many ESP32 modules — avoid driving the encoder during reset/flash or add
> pull resistors to guarantee a safe boot level.

### Other

| Function | GPIO | Notes                                    |
|----------|------|------------------------------------------|
| —        | —    | No dedicated E-STOP pin is used in this build |

---

## Sensor Architecture

All sensor acquisition is owned by **sensor_task** (Core 0, priority 5).
It writes to the shared `SensorState g_sensor` struct under a `portMUX_TYPE` spinlock.
The communication task reads the shared state under the same spinlock when
building `StatusPayload`.

```
sensor_task (Core 0):
  every 1  ms: hx711_tick()    — non-blocking DOUT check, ~80 Hz actual reads
  every 20 ms: pot_read()      — 32-sample moving average, ~50 Hz
  every 1  ms: encoder_get_and_clear_delta()  — PCNT hardware counter

g_sensor (shared, spinlock protected):
  tension_dg[2]      — latest HX711 readings in 0.1 g
  tension_setpoint   — optional host-provided setpoint value
  pot_raw            — 0–4095 filtered ADC value
  encoder_manual     — signed int16 PCNT delta since last read
```

**Tare request**: `sensor_request_tare(uint8_t sensor_idx)` sets a volatile flag.
`sensor_task` processes it on the next tick. Safe to call from stepper_task (Core 1).

---

## HX711 Tension PID

HX711 sensors are read at ~80 Hz (DOUT pulses LOW when conversion is ready).
The read is **non-blocking**: sensor_task polls `gpio_get_level(DOUT)` each ms
and reads 24 bits in ~50 µs only when data is ready. **No busy-wait.**

The Raspberry Pi performs the PID loop using the streamed HX711 readings.
The ESP32 forwards sensor values to the Pi in `StatusPayload`.

A simple PI loop on the Pi looks like:

```
error      = setpoint_dg - reading_dg[0]
integral  += ki * error
output     = kp * error + integral

if abs(output) < 50:
    stop_tensioner()
else:
    set_tensioner_speed(abs(output), direction=sign(output))
```

Current readings are returned in `StatusPayload` on each SPI cycle.

---

## Homing Sequence (Lateral Axis)

Lateral homing uses the 2-contact sensor logic (NO=LOW and NC=HIGH = valid home).
The current transport layer is motion-centric (`SEGMENT_BLOCK`/`STEP_BLOCK`).
Any host-side homing strategy must preserve this contact validation and avoid
single-contact shortcuts.

---

## RMT Queue / Streaming Model

The current ESP32 step-output layer is split into two buffers:

1. `StepperQueue` stores coarse `step_block_t` packets received from the host.
2. `StepperDriver` expands those packets into a software ring of
  `ring_entry_t`, consumed directly by the RMT `simple_encoder` callback.

At the RMT boundary, the behavior is deliberately matched to
`resources/FastAccelStepper` on ESP32 IDF5:

- one `rmt_transmit()` per continuous run,
- `simple_encoder` refill in `PART_SIZE` chunks,
- `trans_queue_depth = 1`,
- explicit LOW-level pause chunk before DIR toggles when needed,
- one LOW-level pause chunk plus stop on queue starvation,
- no task-side busy-spin while waiting for ring space.

The executor task blocks on a task notification from the encoder ISR whenever
the software ring is full. This keeps CPU 1 watchdog-safe while preserving
deterministic RMT timing.

### Multi-axis completion feedback (`last_executed_sequence`)

For `SEGMENT_BLOCK` streaming, the ESP32 multi-axis executor publishes
`last_executed_sequence` through **deferred notifications**:

- each segment is scheduled on a motor timeline (`duration_us` cumulative),
- notification is emitted when that segment is expected to have physically
  finished,
- host (`streamer.py`) retires in-flight segments from this feedback.

To avoid false early completion during real stalls (ring empty / stream
stopped), the firmware pauses this deferred timeline while stalled and
compensates all pending deadlines by the stall duration before resuming.
This keeps host-side buffered-time estimation aligned with actual motor output.

## Acceleration Model (Klipper-style)

Host-side planner emits arithmetic segments:

```
for each segment:
  ticks = start_ticks
  for step in range(step_count):
    emit_step(ticks)
    ticks += add_ticks
```

The host sends those segments via `SEGMENT_BLOCK`; ESP32 expands them to
`step_block_t` and streams through RMT.

This decouples link bandwidth from step frequency and avoids the old
`one transport entry per step` bottleneck.

---

## Winding Kinematics  (host side — `src/rpi/machine/winding_kinematics.py`)

All winding geometry computation runs on the Raspberry Pi host, not on the ESP32.
The ESP32 receives only compressed motion segments and has no knowledge of
wire geometry.

### Frequency Ratio

The bobbin and lateral axes may have **different** motor specifications.  For every
bobbin revolution the lateral axis must advance exactly one pitch:

$$R = \frac{\text{pitch\_mm}}{\text{p\_lead\_mm}} \times \frac{\text{lateral\_ppr}}{\text{bobbin\_ppr}}$$

$$\text{hz\_lateral} = \text{hz\_bobbin} \times R \quad \text{clamped to } [1, \text{HZ\_MAX}]$$

When both axes share the same ppr (lateral_ppr = bobbin_ppr) the fraction cancels
and the formula reduces to the simpler `R = pitch / p_lead`.

**Numerical example** (equal motors: 200 full × 32 µstep = 6400 ppr each;
wire d=0.3 mm, leadscrew p_lead=2 mm/rev):

| Parameter        | Value          |
|------------------|----------------|
| R                | 0.15           |
| hz\_bobbin       | 160 000 Hz     |
| hz\_lateral      | 24 000 Hz      |
| turns per layer  | 100            |
| layer duration   | 4 s            |
| lateral travel   | 30 mm ✓        |

### WindingGeometry fields

```python
@dataclass
class WindingGeometry:
    wire_diameter_mm:      float
    bobbin_width_mm:       float
    traverse_pitch_mm:     float   # leadscrew mm/rev
    mandrel_diameter_mm:   float   # D0 — runtime, never hardcoded
    bobbin_steps_per_rev:  int = 200
    bobbin_microsteps:     int = 32
    lateral_steps_per_rev: int = 200   # may differ from bobbin motor
    lateral_microsteps:    int = 32
```

### Winding Modes

| Mode            | Pitch per turn     | Radial advance per layer     |
|-----------------|--------------------|------------------------------|
| FIXED_PITCH     | d_wire             | d_wire                       |
| ORTHOCYCLIC     | d_wire             | d_wire × sin(60°) = 0.866 d  |
| CUSTOM_PITCH    | user-supplied (mm) | d_wire (or custom)           |

### Layer Diameter

$$D(n) = D_0 + d_\text{wire} \times (1 + 2n \times \text{PACK})$$

where PACK = 1.0 (fixed/custom) or 0.86603 (orthocyclic).

### Velocity Segment

`src/rpi/ramp.py::SegmentBlockGenerator` creates segment blocks where each
segment is an arithmetic run (`step_count`, `start_ticks`, `add_ticks`, dir).
`src/rpi/streamer.py` sends those blocks via `SEGMENT_BLOCK`.

| Field          | Description                                  |
|----------------|----------------------------------------------|
| step_count     | Number of steps in segment                    |
| start_ticks    | First step interval in RMT ticks              |
| add_ticks      | Delta applied after each emitted step         |
| direction      | Segment direction                              |

---

## File Structure

```
src/esp32/                    ← ESP32 firmware (PlatformIO, ESP-IDF framework)
  src/
    messages.h                ← Fixed SPI frame protocol (CRC16, payloads)
    comm_interface.*          ← SPI slave task + message dispatch
    step_types.h              ← Step/segment core types and constants
    stepper_queue.*           ← Motion queue + segment expansion executor
    stepper_driver.*          ← RMT streaming + software ring
    main.cpp                  ← Pin config, task startup
  platformio.ini

src/rpi/                       ← RPi Python application
  messages.py                 ← Python mirror of `messages.h`
  ramp.py                     ← Segment planner / generators
  spi_transport.py            ← SPI frame transport helper
  streamer.py                 ← Segment block streaming and backpressure
  demo_spi.py                 ← Segment-streaming demo CLI

doc/                          ← Architecture documentation
resources/                    ← External reference material (do not modify)
  esp32/                      ← Old standalone ESP32 project (reference only)
  FastAccelStepper/           ← behavioral reference for queue/RMT model
```
