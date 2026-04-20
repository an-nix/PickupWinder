# PickupWinder — RPi + ESP32 Real-time Winding Controller

Automated and assisted guitar pickup winding with a Raspberry Pi host and an ESP32 real-time motion controller.

- Host: Python on Raspberry Pi
- MCU: ESP32 with ESP-IDF and FreeRTOS
- Link: full-duplex SPI, fixed 512-byte frames
- Production motion path: `MULTI_AXIS_SEGMENT_BLOCK`

## Active code layout

The active host entry point is `src/rpi/winding_main.py`. The deprecated `WinderApp` stub in `src/rpi/core/app.py` is not part of the runtime path.

- `src/rpi/winding_main.py`: boots SPI transport, `WindingEngine`, and the JSON-RPC server.
- `src/rpi/motion/engine.py`: orchestration layer for moves and RPC-triggered actions.
- `src/rpi/motion/move_queue.py`: serializes moves and aligns motion sequences with firmware state.
- `src/rpi/transport/messages.py`: Python protocol mirror and 16-bit sequence helpers.
- `src/rpi/transport/spi_transport.py`: SPI framing, polling, and pipelined ACK confirmation.
- `src/rpi/transport/streamer.py`: sequence-aware multi-axis streaming and backpressure logic.
- `src/rpi/motion/`: ramp, winding, scatter, and synchronized segment generators.
- `src/esp32/src/main.cpp`: pin configuration and firmware startup.
- `src/esp32/src/comm_interface.cpp`: SPI slave task, request dedupe, block dispatch, and status publishing.
- `src/esp32/src/motion_planner.cpp`: planner queue, monotonic motion filtering, and flush handling.
- `src/esp32/src/stepper_queue.cpp`: expansion into ring entries and RMT start policy.
- `src/esp32/src/stepper_driver.cpp`: RMT streaming, coast mode, and underrun behavior.

## Runtime model

1. The host generates synchronized motion segments.
2. The host sends `MULTI_AXIS_SEGMENT_BLOCK` requests over SPI.
3. The ESP32 validates CRC, dedupes exact request retries, dedupes accepted block retries by `block_seq`, and queues work for the planner.
4. The planner drops stale or out-of-order `motion_sequence` values and feeds the executor queue.
5. The executor expands segments into step timings, fills the RMT ring, then starts motion once the ring is prefed.
6. The host confirms each request through `wait_for_request_result()` because the SPI status frame is pipelined by one transfer.

## Sequencing and retries

The project now tracks three separate 16-bit sequences:

- `SpiMessageHeader.sequence`: transport request/ACK correlation.
- `MultiAxisSegmentBlockHeader.block_seq`: accepted block dedupe.
- `multi_axis_segment_t.motion_sequence`: monotonic execution ordering.

All ordering comparisons use signed 16-bit wrap-aware helpers. This is overflow-safe as long as compared values never drift by `>= 32768`, which is well above the actual pipeline depth in this project.

Authoritative details are in `doc/sequencing.md`.

## Build and run

ESP32 firmware:

```bash
cd src/esp32
pio run -t upload
```

Host application:

```bash
cd src/rpi
python3 winding_main.py
```

Default JSON-RPC socket:

```bash
/tmp/winding.sock
```

## ESP32 pin map

| Signal | GPIO |
|---|---:|
| Bobbin STEP / DIR / EN | 26 / 27 / 14 |
| Lateral STEP / DIR / EN | 32 / 33 / 25 |
| Tensioner STEP / DIR / EN | 16 / 17 / 4 |
| Lateral home NO / NC | 21 / 22 |
| SPI MOSI / MISO / SCLK / CS | 23 / 19 / 18 / 5 |
| HX711 #0 SCK / DOUT | 13 / 34 |
| HX711 #1 SCK / DOUT | 12 / 39 |
| Potentiometer | 36 |
| Encoder A / B | 0 / 15 |

## Documentation

- `doc/architecture.md`: current architecture and runtime flow.
- `doc/spi_protocol.md`: wire protocol, ACK semantics, and payload rules.
- `doc/stepper_engine.md`: RMT/ring/executor behavior.
- `doc/sequencing.md`: retry, dedupe, flush, and wrap-around review.
- `doc/async_streaming_refactoring.md`: historical refactoring note kept as background material.
- `.github/copilot-instructions.md`: repository-specific coding rules.

## Repository hygiene

- Generated plots and segment JSON outputs belong under `doc/generated/`.
- `resources/` and `migration/` are reference trees, not active runtime code.
- ESP32 firmware is ESP-IDF (`app_main()`), not Arduino.
