# PickupWinder Architecture

PickupWinder is a two-processor system:

- Raspberry Pi: high-level planning, orchestration, RPC, and SPI master transport.
- ESP32: deterministic real-time executor, sensor concentrator, and SPI slave.

The split is deliberate. Geometry, winding strategy, retry policy, and session logic live on the host. Hard real-time pulse generation and safety-critical queue execution live on the ESP32.

## Active code map

### Host

- `src/rpi/winding_main.py`: process entry point.
- `src/rpi/app/runtime.py`: runtime composition and lifecycle.
- `src/rpi/core/engine.py`: runtime orchestration for moves and winding programs.
- `src/rpi/core/lateral.py`: traverse-axis homing, home-state, and soft-limit rules.
- `src/rpi/core/status.py`: explicit snapshots for `winder.*` and `winding.status`.
- `src/rpi/motion/move_queue.py`: move serialization, flush coordination, sequence seeding.
- `src/rpi/motion/segment_generator.py`: host-side segment generation utilities.
- `src/rpi/motion/multi_axis_segment_generator.py`: general multi-axis move generator.
- `src/rpi/motion/synchronized_segment_generator.py`: winding-specific synchronized bobbin/traverse generator.
- `src/rpi/motion/spindle_kinematics.py`: bobbin kinematics over time.
- `src/rpi/motion/winding_pattern.py`: traverse position from bobbin turns.
- `src/rpi/motion/scatter_engine.py`: non-harmonic scatter offset with edge damping.
- `src/rpi/transport/messages.py`: Python protocol mirror and wrap-aware sequence helpers.
- `src/rpi/transport/spi_transport.py`: SPI frame transport and request confirmation.
- `src/rpi/transport/streamer.py`: buffered segment streaming and in-flight retirement.
- `src/rpi/jsonrpc/rpc_server.py`: RPC server bootstrap.
- `src/rpi/jsonrpc/winding_handler.py`: JSON-RPC surface for the winding engine.
- `src/rpi/winding/program.py`: high-level winding program definitions.

### Firmware

- `src/esp32/src/main.cpp`: pin map and `app_main()`.
- `src/esp32/src/messages.h`: packed protocol types.
- `src/esp32/src/comm_interface.cpp`: SPI task, CRC validation, retry dedupe, request dispatch.
- `src/esp32/src/motion_planner.cpp`: monotonic motion filtering and executor queue management.
- `src/esp32/src/stepper_queue.cpp`: segment expansion into step entries.
- `src/esp32/src/stepper_driver.cpp`: RMT streaming driver and coast mode.
- `src/esp32/src/sensor_task.cpp`: HX711, potentiometer, and encoder acquisition.
- `src/esp32/src/endstop.cpp`: 2-contact lateral home sensor handling.

## Runtime flow

1. A JSON-RPC request reaches the host process.
2. `motion.engine` builds a move or winding program.
3. A generator produces `MultiAxisSegment` objects with monotonic `motion_sequence` values.
4. `transport.streamer` batches those segments into `MULTI_AXIS_SEGMENT_BLOCK` requests.
5. `spi_transport` sends fixed 512-byte SPI frames and waits for confirmed results.
6. `CommInterface` validates the frame, dedupes exact retries, and dispatches the payload.
7. `MotionPlanner` rejects stale sequences, queues valid work, and honors flush floors.
8. The executor expands segments into step timings, fills the RMT ring, and kicks the driver once per drain batch.
9. The RMT ISR emits step pulses or pause symbols until motion completes.
10. `StatusPayload.last_executed_sequence` feeds completion state back to the host streamer, while `last_planned_sequence` and `segments_dropped` expose planner backlog and stale/drop diagnostics.

## Protocol model

The SPI link is full-duplex and fixed size.

- Frame size: `512` bytes
- Header size: `12` bytes
- CRC: `CRC16-CCITT-FALSE`
- Endianness: little-endian
- Electrical mode: SPI mode 1 on both the Raspberry Pi host and the ESP32 slave
- ESP32 uses IO_MUX-native SPI pins with an active `ready` handshake GPIO on GPIO17, as recommended by ESP-IDF for reliable slave timing
- ESP32 also exposes a reserved Raspberry Pi sideband output `shutdown_req` on GPIO16 for a future coordinated host shutdown path

The production motion message is `MULTI_AXIS_SEGMENT_BLOCK`.

### Pipelined ACK rule

Status is pipelined by one transfer. The frame returned during request `N` reflects the processing status of request `N-1`.

The host must therefore confirm completion with `wait_for_request_result()` rather than trusting the immediate full-duplex reply as the ACK for the current request.

## Sequence domains

There are three separate 16-bit sequence spaces.

- Transport sequence: `SpiMessageHeader.sequence`
- Block sequence: `MultiAxisSegmentBlockHeader.block_seq`
- Motion sequence: `multi_axis_segment_t.motion_sequence`

All ordering checks use signed wrap-aware comparisons in both Python and firmware. This is overflow-safe because the real queue and inflight depths stay far below half the 16-bit space.

Further details are in `doc/sequencing.md`.

## Host architecture

The host owns all high-level motion semantics.

- planning,
- winding geometry,
- scatter behavior,
- RPC session control,
- flush/retry policy,
- transport sequencing.
- lateral homing state and host-side soft-limit enforcement.

The host runtime is now split by responsibility rather than by startup order:

- `winding_main.py` only handles process startup and signals.
- `app/runtime.py` wires transport, shared state, engine, and RPC.
- `core/engine.py` owns program orchestration and command entry points.
- `core/lateral.py` owns traverse-specific rules.
- `core/status.py` builds explicit status/config payloads instead of relying on generic object introspection.
- `winding/adaptive.py` defines the adaptive winding session model, chunk planner, and tracked synchronized winding move.
- `winding/service.py` owns the live winding session thread: homing, chunk planning, controlled pause/resume, window retargeting, and progress tracking.

The winding path follows an electronic gearing model:

- `SpindleKinematics` computes bobbin turns over time.
- `WindingPattern` maps turns to traverse position.
- `ScatterEngine` perturbs traverse position without spilling at the flanges.
- `SynchronizedSegmentGenerator` samples the time domain and emits synchronized multi-axis segments.

Manual moves and jogs use `build_jog_move()` (in `motion/move_builders.py`) which constructs a single-axis `RampMove`. All move types produce the same `MultiAxisSegment` objects consumed by the streamer.

Segment producers implement the `SegmentProducer` structural protocol (`motion/segment_producer.py`): any object with `__iter__(self) -> Iterator[MultiAxisSegment]` is accepted by the streamer. `MultiAxisSegmentGenerator` is the general implementation; `RampMove` uses the trapezoidal profile generator.

Each `MultiAxisSegment` carries a `direction_mask: int` bitmask (one bit per axis) replacing the former `directions: list[int]` per-axis list.

The adaptive winding path is host-driven and chunked on purpose:

- spindle turns remain the primary progress unit,
- traverse window low/high bounds can be updated while the session is paused or while the next chunk is being planned,
- target RPM can be changed live and a target RPM of zero is treated as a controlled pause request,
- the host tracks turns completed, turns remaining, guide position, and active window in shared state,
- near a traverse edge the planner brakes the spindle to zero before reversing the guide so lateral inversion time is explicit rather than implicit.

### Lateral axis state model

- The lateral axis home position is volatile and is treated as lost after a restart.
- `HomingMove` is isolated in `motion/move.py` and executed phase-by-phase in `motion/move_queue.py`. Homing logic must not leak into `MultiAxisRampStreamer` or transport layers.
- The host refuses lateral free-motion commands until homing completes.
- Soft travel limits are enforced on the host before a lateral move is enqueued, so queue serialization and SPI block delivery remain unchanged.
- After homing, the host streamer keeps the lateral enable pin asserted across later moves; if firmware status shows the enable bit dropped, the host invalidates the stored home state.

## Firmware architecture

The ESP32 is split by responsibility, not by UI concepts.

### Core 0

- SPI slave task
- sensor task
- endstop event publication

### Core 1

- planner execution path
- ring fill and kick-start decisions
- RMT pulse streaming

The firmware does not compute winding geometry. It only validates, queues, expands, and executes what the host planned.

## Motion execution invariants

These invariants are intentional and should not be weakened:

- `encode_steps()` and `on_trans_done_isr()` stay in IRAM.
- SPI DMA frame buffers stay `DMA_ATTR`.
- The ring buffer is SPSC lock-free.
- `pushExpandedBlock()` does not start the driver.
- `executeConstantRateBlock()` does not start the driver.
- `kickStart()` runs once after a drain batch, not per segment.
- Host-side streamer maintains a deeper planner queue at high speed: 32-segment lookahead and up to 200ms of buffered motion.
- Coast mode emits pause symbols on transient starvation instead of stopping the RMT.
- Coast pauses are timed to the last step interval so the ISR does not flood the executor at high speed.

## Sensor model

The ESP32 owns raw sensor acquisition.

- HX711 load cells are read non-blockingly.
- The potentiometer is sampled through ADC1.
- The manual encoder is decoded with PCNT.
- The two-contact home sensor validates both NO and NC states.

The Raspberry Pi consumes normalized state and owns higher-level policy such as tension control and workflow decisions.

## Hardware map summary

| Function | GPIO |
|---|---:|
| Bobbin STEP / DIR / EN | 26 / 27 / 14 |
| Lateral STEP / DIR / EN | 32 / 33 / 25 |
| Lateral home NO / NC | 22 / 21 |
| SPI MOSI / MISO / SCLK / CS | 23 / 19 / 18 / 5 |
| SPI READY | 17 |
| Raspberry Pi SHUTDOWN_REQ | 16 |
| HX711 #0 SCK / DOUT | 13 / 34 |
| HX711 #1 SCK / DOUT | 12 / 39 |
| Potentiometer | 36 |
| Encoder A / B | 0 / 15 |

## Repository boundaries

- `src/` contains the active code.
- `doc/` contains maintained project documentation.
- `doc/generated/` is reserved for generated plots and exported segment JSON.
- `resources/` and `migration/` are reference trees and are not the active implementation.

## Related documents

- `doc/spi_protocol.md`
- `doc/stepper_engine.md`
- `doc/sequencing.md`
- `.github/copilot-instructions.md`
