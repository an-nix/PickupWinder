# Stepper queue / RMT implementation

## Active stack

- `src/esp32/src/messages.h`
- `src/esp32/src/comm_interface.h`
- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/step_types.h`
- `src/esp32/src/stepper_queue.h`
- `src/esp32/src/stepper_queue.cpp`
- `src/esp32/src/stepper_driver.h`
- `src/esp32/src/stepper_driver.cpp`
- `src/rpi/transport/messages.py`
- `src/rpi/motion/ramp.py`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/transport/streamer.py`

## Design (current)

- Host sends **compressed motion** through `MULTI_AXIS_SEGMENT_BLOCK`.
- `CommInterface` validates CRC16 and enqueues `multi_axis_block_t`.
- `multiAxisExecutorTask` drain loop expands segments into concrete `step_block_t` chunks.
- Expanded steps are pushed into `StepperDriver` software ring.
- RMT `simple_encoder` callback (`encode_steps`) is the only layer emitting STEP symbols.

## Coast Mode (anti-underrun architecture)

The RMT driver uses a **coast mode** to eliminate the 200–500 µs restart penalty
that previously caused underruns at low RPM:

- **Ring not empty**: `encode_steps` emits PART_SIZE step symbols as usual.
- **Ring temporarily empty**: instead of stopping the RMT, emits PART_SIZE
  **pause symbols** (LOW level, ~25 µs each). The motor holds position while
  the executor refills the ring. `ring_underrun_count_` is incremented for
  diagnostics but `rmt_stopped_` stays `false`.
- **Ring empty for too long**: after `COAST_IDLE_LIMIT` consecutive empty
  callbacks (~1.25 s), the driver auto-stops to save power.
- **Mid-chunk exhaustion**: if the ring drains partway through a PART_SIZE
  chunk, remaining symbols are filled with pause (no early stop).

This means `rmt_transmit()` is called once per continuous run, and the RMT
never stops/restarts for transient ring starvation.

## Runtime rules

- one `rmt_transmit()` per continuous run (coast mode keeps it alive),
- `simple_encoder` refill in `PART_SIZE = 8` chunks,
- `trans_queue_depth = 1`,
- balanced 50/50 HIGH/LOW symbol for normal steps,
- fixed LOW pause before DIR toggle when required,
- coast-mode LOW pause on transient ring starvation (no stop),
- auto-stop after COAST_IDLE_LIMIT (~1.25 s) of sustained empty ring,
- no task-side busy-spin while waiting for ring space.

## Key constants

| Constant | Value | Notes |
|---|---|---|
| PART_SIZE | 8 | Symbols per ISR callback (was 32) |
| RMT_MEM_SYMBOLS | 64 | RMT DMA buffer depth |
| STEP_STREAM_START_FILL | 16 | = 2 × PART_SIZE, initial ring fill before RMT start |
| STEP_STREAM_RESTART_FILL | 4 | Ring fill to resume after explicit stop |
| STEP_RING_SIZE | 4096 | Lock-free SPSC ring capacity |
| COAST_IDLE_LIMIT | 6250 | ~1.25 s of empty callbacks before auto-stop |

## Execution lifecycle

1. Host planner generates multi-axis time segments (`step_counts[]`, `duration_us`, `direction_mask`).
2. Host segment generator adapts segment duration (2–50 ms) to guarantee ≥32 steps/segment.
3. Host streamer sends `MULTI_AXIS_SEGMENT_BLOCK` messages over SPI.
4. ESP32 Core 0 SPI task validates and enqueues multi-axis blocks.
5. ESP32 Core 1 executor drain loop expands segments and fills driver ring.
6. After drain: `kickStart()` once per active axis (starts RMT if not already running).
7. RMT ISR emits step symbols; on empty ring, coasts with pause symbols.
8. After COAST_IDLE_LIMIT of sustained empty ring, ISR auto-stops.

## Notes

- Legacy explicit `STEP_BLOCK` is still accepted for debug.
- Production path uses `MULTI_AXIS_SEGMENT_BLOCK`.
- `pushExpandedBlock()` does NOT call `maybeStartDriver()` — only `kickStart()` in the drain loop starts RMT.
- `resources/FastAccelStepper` remains reference-only; library is not linked.
