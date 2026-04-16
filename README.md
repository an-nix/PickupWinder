# PickupWinder — RPi + ESP32 Real-time Winding Controller

Automated/assisted guitar pickup coil winding.

- **Host**: Raspberry Pi (Python)
- **MCU**: ESP32 (ESP-IDF / FreeRTOS)
- **Link**: SPI full-duplex, fixed 512-byte frames
- **Motion transport**: compressed arithmetic segments (`SEGMENT_BLOCK`)

## Current runtime model

The active motion path is:

1. Host computes trajectory.
2. Host sends `SEGMENT_BLOCK` messages (compact segments).
3. ESP32 expands segments into `step_block_t` chunks locally.
4. RMT `simple_encoder` streams step pulses.

Legacy `STEP_BLOCK` (explicit per-step entries) is still supported for debug,
but the production path is segment mode.

## Protocol summary (current)

Protocol definitions:

- `src/esp32/src/messages.h`
- `src/rpi/messages.py`

Key points:

- Frame size: **512 bytes**
- Header: 12 bytes (`magic`, `version`, `type`, `sequence`, `length`, `flags`, `crc16`)
- CRC: **CRC16-CCITT**
- Motion messages:
  - `SEGMENT_BLOCK` (primary)
  - `STEP_BLOCK` (legacy/debug)
- Status includes queue/ring fill and underrun counters per axis

## Build / Flash / Run

ESP32:

```bash
cd src/esp32
pio run -t upload
```

RPi demo:

```bash
cd /home/pi/winder
python3 demo_spi.py --queue-prefill-blocks 12 --queue-low-watermark-blocks 4
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
