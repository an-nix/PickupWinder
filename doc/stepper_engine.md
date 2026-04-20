# Stepper queue / RMT implementation

## Active stack

- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/motion_planner.cpp`
- `src/esp32/src/step_types.h`
- `src/esp32/src/stepper_queue.cpp`
- `src/esp32/src/stepper_driver.cpp`
- `src/rpi/transport/messages.py`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/motion/segment_generator.py`
- `src/rpi/motion/synchronized_segment_generator.py`

## Current design

- The host sends compressed synchronized motion through `MULTI_AXIS_SEGMENT_BLOCK`.
- `CommInterface` validates CRC, dedupes exact retries, and forwards decoded blocks to the planner.
- `MotionPlanner` converts blocks into individual scheduled segments and drops stale or out-of-order `motion_sequence` values.
- The executor expands those segments into constant-rate step bursts.
- Expanded steps are written into the `StepperDriver` software ring.
- The RMT `simple_encoder` callback is the only layer that emits STEP symbols.

## Coast mode

The RMT driver uses coast mode to avoid the restart penalty that previously caused underruns:

- Ring not empty: `encode_steps()` emits up to `PART_SIZE` step symbols.
- Ring temporarily empty: the ISR emits pause symbols instead of stopping the RMT.
- Ring empty for too long: after `COAST_IDLE_LIMIT`, the driver auto-stops.
- Mid-chunk exhaustion: the remainder of the chunk is filled with pauses, not a stop.

This keeps `rmt_transmit()` to one call per continuous run and removes transient restart jitter from the motion path.

## Runtime rules

- `PART_SIZE = 8` symbols per encoder callback.
- `trans_queue_depth = 1`.
- Normal steps use a balanced HIGH/LOW pulse split.
- Direction changes insert a fixed LOW pause before toggling DIR.
- `pushExpandedBlock()` must not start the driver.
- `kickStart()` is owned by the post-drain batch decision, not by per-segment code.
- Host-side send confirmation only counts after `wait_for_request_result()` returns `OK`.

## Key constants

| Constant | Value | Notes |
|---|---|---|
| `PART_SIZE` | 8 | Symbols per ISR callback |
| `RMT_MEM_SYMBOLS` | 64 | RMT DMA buffer depth |
| `STEP_STREAM_START_FILL` | 128 | Initial buffered steps before first start |
| `STEP_STREAM_RESTART_FILL` | 16 | Restart threshold after a true stop |
| `STEP_RING_SIZE` | 4096 | Lock-free SPSC ring capacity |
| `COAST_IDLE_LIMIT` | 6250 | Empty callbacks before auto-stop |
| `SEGMENT_QUEUE_DEPTH` | 128 | Planner to executor look-ahead queue |

## Execution lifecycle

1. The host generates multi-axis segments with adaptive duration.
2. The streamer sends one or more segments per SPI frame and confirms each request via the pipelined ACK path.
3. The ESP32 SPI task validates and enqueues decoded blocks.
4. The planner expands blocks into scheduled segments.
5. The executor drains available work into the ring.
6. After the drain batch, `kickStart()` starts the RMT if needed.
7. The ISR emits step symbols and coasts through transient starvation.
8. Deferred completion updates `last_executed_sequence` for host-side retirement.

## Notes

- `STEP_BLOCK` and `SEGMENT_BLOCK` still exist for debug and legacy tooling.
- The production path is `MULTI_AXIS_SEGMENT_BLOCK` end-to-end.
- `resources/FastAccelStepper` remains reference-only and is not linked into the firmware.
