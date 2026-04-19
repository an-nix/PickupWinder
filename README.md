# PickupWinder — RPi + ESP32 Real-time Winding Controller

Automated/assisted guitar pickup coil winding.

- **Host**: Raspberry Pi (Python)
- **MCU**: ESP32 (ESP-IDF / FreeRTOS)
- **Link**: SPI full-duplex, fixed 512-byte frames
- **Motion transport**: compressed multi-axis segment blocks (`MULTI_AXIS_SEGMENT_BLOCK`)

## Architecture overview

The host is the motion planner and stream controller. The ESP32 is a deterministic executor.

- `src/rpi/main.py`: application entry point launching the JSON-RPC server.
- `src/rpi/core/app.py`: `WinderApp`, host state, and background streaming session manager.
- `src/rpi/transport/spi_transport.py`: SPI frame transport wrapper for request/response exchanges.
- `src/rpi/transport/streamer.py`: deterministic streamer that sends motion segments and maintains a modest look-ahead buffer.
- `src/rpi/transport/messages.py`: frame and payload packing/unpacking.
- `src/rpi/motion/ramp.py`: motion segment generation for axis ramps.

## Runtime flow

1. The host computes motion segments from `RampConfig`.
2. The host sends `MULTI_AXIS_SEGMENT_BLOCK` frames over SPI.
3. The ESP32 receives segments, enqueues them, expands them to step timing, and streams pulses through RMT.
4. The host polls status and keeps the MCU queue/ring filled without overflowing it.

## Protocol summary

Key points:

- Frame size: **512 bytes**
- Header size: 12 bytes
- CRC: **CRC16-CCITT** over header + payload
- Primary production message: `MULTI_AXIS_SEGMENT_BLOCK`
- Status includes queue/ring free slots, `last_executed_sequence`, `last_rx_sequence`, and execution results

## Streaming semantics

The host streamer maintains a small in-flight queue of sent segments and tracks buffered motion time in seconds.

- Target look-ahead: ~100 ms
- Minimum look-ahead: ~60 ms
- Maximum in-flight segments: 24
- The host stops sending when the MCU reports a full queue/ring or when the in-flight window is reached.

## JSON-RPC client model

There is no session concept in the host application. Multiple JSON-RPC clients share the same `WinderApp` instance and command the same motion pipeline.

Only one motion operation can run at a time; concurrent motion requests are serialized or rejected to protect the motors and the shared SPI/ESP32 state.

The JSON-RPC API exposes the shared operation state and supports graceful stop:

- `winder.operation.status` — query the current shared operation.
- `winder.operation.stop` — request a graceful stop of the active motion.
- `winder.session.status` / `winder.session.wait` are compatibility aliases for the shared operation state.

## Build / Flash / Run

ESP32 firmware:

```bash
cd src/esp32
pio run -t upload
```

Host app:

```bash
cd src/rpi
python3 main.py
```

Default JSON-RPC socket:

```bash
/tmp/pickup_winder_rpc.sock
```

## Pin assignments (ESP32)

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

- Architecture: `doc/architecture.md`
- Stepper runtime details: `doc/stepper_engine.md`
- Agent instructions: `.github/copilot-instructions.md`

## Notes

- ESP32 firmware is **ESP-IDF** (`app_main()`), not Arduino.
- `resources/` is reference-only.
