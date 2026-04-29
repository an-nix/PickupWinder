# Copilot Instructions — PickupWinder

Project: Raspberry Pi host application plus ESP32 real-time stepper controller for guitar pickup winding.

## Scope

- Active code lives under `src/`.
- Maintained documentation lives under `doc/`.
- Generated plots and exported JSON belong under `doc/generated/`.
- `resources/` and `migration/` are reference-only unless a task explicitly targets them.

## Architecture rules

- The ESP32 firmware uses ESP-IDF, not Arduino.
- Firmware entry point is `app_main()`.
- Python owns planning, winding geometry, retry policy, and session logic.
- ESP32 owns SPI slave handling, sensor acquisition, queueing, and pulse execution.
- Production motion uses `MULTI_AXIS_SEGMENT_BLOCK`.

## Active file map

### Host

- `src/rpi/winding_main.py`: host entry point.
- `src/rpi/motion/engine.py`: orchestration.
- `src/rpi/motion/move_queue.py`: move serialization and flush coordination.
- `src/rpi/transport/messages.py`: protocol mirror and sequence helpers.
- `src/rpi/transport/spi_transport.py`: SPI transport and ACK confirmation.
- `src/rpi/transport/streamer.py`: buffered motion streaming.
- `src/rpi/jsonrpc/`: JSON-RPC server and handlers.

### Firmware

- `src/esp32/src/main.cpp`: pin config and startup.
- `src/esp32/src/messages.h`: packed protocol types.
- `src/esp32/src/comm_interface.cpp`: SPI dispatch and request dedupe.
- `src/esp32/src/motion_planner.cpp`: monotonic planning and flush floor.
- `src/esp32/src/stepper_queue.cpp`: segment expansion.
- `src/esp32/src/stepper_driver.cpp`: RMT streaming.
- `src/esp32/src/sensor_task.cpp`: sensors.

Do not document or extend `src/rpi/core/app.py` as the active runtime path. It is a deprecated compatibility shim.

## SPI and sequencing rules

- SPI frame size is fixed at `512` bytes.
- CRC is `CRC16-CCITT-FALSE`.
- `messages.h` and `src/rpi/transport/messages.py` must stay binary-compatible.
- SPI status is pipelined by one transfer.
- Host code must confirm requests through `wait_for_request_result()`.
- Exact request retries are deduped in firmware by transport sequence, type, length, and CRC.
- Accepted multi-axis blocks are deduped by `block_seq`.
- Motion ordering is enforced by `motion_sequence`.
- All sequence comparisons must stay wrap-aware. Never use raw integer ordering on these 16-bit fields.

## Motion invariants

- `encode_steps()` and `on_trans_done_isr()` must remain `IRAM_ATTR`.
- SPI DMA frame buffers must remain `DMA_ATTR`.
- The software ring is single-producer/single-consumer and lock-free.
- `pushExpandedBlock()` must not start the RMT driver.
- `executeConstantRateBlock()` must not start the RMT driver.
- `kickStart()` is called once after a drain batch.
- Coast mode must emit pause symbols on transient starvation rather than stopping immediately.
- `STEP_STREAM_START_FILL` is `128`.
- `PART_SIZE` is `8`.

## ESP32 coding rules

- Use ESP-IDF APIs, not Arduino APIs.
- Do not block in timer ISRs.
- Place `IRAM_ATTR` on the definition line in `.cpp` files.
- Use `portENTER_CRITICAL` and `portEXIT_CRITICAL` for shared cross-core sensor state.
- Treat `AxisPins.endstop_no` and `endstop_nc` as optional and validate `>= 0`.
- HX711 reads must be non-blocking.
- Do not drive GPIO 12 high at boot.
- For GPIO `>= 32`, use the `GPIO.out1_*` registers in fast paths.

## Host coding rules

- Python never touches GPIO directly.
- SPI goes through `src/rpi/transport/spi_transport.py`.
- Production streaming goes through `MultiAxisRampStreamer`.
- Segment duration must stay adaptive rather than fixed.
- Avoid redundant `get_status()` calls inside the main streaming loop when a fresh send response already contains usable status.
- Keep all new generated artifacts out of the repository root.

## Documentation rules

- Keep the authoritative docs in sync with code changes:
  - `README.md`
  - `doc/architecture.md`
  - `doc/spi_protocol.md`
  - `doc/stepper_engine.md`
  - `doc/sequencing.md` when sequencing logic changes
- If hardware pins change, update `src/esp32/src/main.cpp`, `doc/architecture.md`, and this file.
- Current SPI sideband pins: `READY=GPIO17`, `RPI_SHUTDOWN_REQ=GPIO16`.
- If protocol structs change, update both firmware and Python mirrors in the same change.

## Cleanup rules

- Prefer deleting obsolete first-party analysis files after their useful content has been merged into maintained docs.
- Do not delete vendored third-party trees such as `resources/idf/` without an explicit request.
- Treat generated plots, exported JSON, and transient diagnostics as disposable outputs.
