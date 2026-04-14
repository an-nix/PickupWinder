# PickupWinder — RPi + ESP32 Real-time Winding Controller

Automated/assisted guitar pickup coil winding.
Real-time stepper control on the **ESP32** (dual-core, FreeRTOS, **ESP-IDF** framework),
commanded by a **Raspberry Pi** Python application over SPI.

## Architecture

```
Raspberry Pi (Python / asyncio)
  CoilWinder · TensionController · WebUI
       │ SPI 4 MHz (8-byte CmdFrame → / ← 44-byte StatusFrame)
ESP32 (ESP-IDF, FreeRTOS, 240 MHz)
  Core 0 (pri  5): sensor_task  — HX711 poll, ADC pot, PCNT encoder, tension setpoint
  Core 0 (pri 10): spi_task     — receives commands, sends StatusFrame
  Core 1 (pri 24): stepper_task — 3 hardware timers, endstop poll, axis dispatch
  HX711 load cells              — non-blocking ~80 Hz, values forwarded to Pi
  Potentiometer (ADC1)          — speed control knob on GPIO 36
  Quadrature encoder (PCNT)     — manual axis control on GPIO 0/15 (strap pins)
```

Full architecture details: [doc/architecture.md](doc/architecture.md)

## Hardware

- **ESP32** DevKit (38-pin) — stepper controller, sensor acquisition, SPI link to Pi
- **Raspberry Pi** (3B+ or 4) — application host, WebSocket UI, PID and TMC2209 UART
- **Steppers**: 3× A4988/DRV8825 (Bobbin + Lateral + Tensioner), 32 µstep
- **Home sensor**: 2-contact reed/optical (NO + NC), lateral axis
- **Load cells**: 2× HX711 — read by ESP32, forwarded to Pi for PID
- **Potentiometer**: 10 kΩ on GPIO 36 (ADC1_CH0) — speed control
- **Encoder**: quadrature on GPIO 0/15 (PCNT) — manual axis control
- **TMC2209 UART**: handled by the Raspberry Pi, not the ESP32

## Pin Assignments (ESP32)

| Signal             | GPIO | Notes                                            |
|--------------------|------|--------------------------------------------------|
| Bobbin STEP        | 26   |                                                  |
| Bobbin DIR         | 27   |                                                  |
| Bobbin EN          | 14   | Active LOW                                       |
| Lateral STEP       | 32   |                                                  |
| Lateral DIR        | 33   |                                                  |
| Lateral EN         | 25   | Active LOW                                       |
| Lateral HOME NO    | 21   | Normally-open contact (pull-up, LOW at home)     |
| Lateral HOME NC    | 22   | Normally-closed contact (pull-up, HIGH at home)  |
| Tensioner STEP     | 16   |                                                  |
| Tensioner DIR      | 17   |                                                  |
| Tensioner EN       | 4    | Active LOW                                       |
| Tensioner endstop  | —    | No dedicated endstop                             |
| SPI MOSI           | 23   | From RPi GPIO 10                                 |
| SPI MISO           | 19   | To RPi GPIO 9                                    |
| SPI SCLK           | 18   | From RPi GPIO 11                                 |
| SPI CS             | 5    | From RPi GPIO 8                                  |
| HX711 tension SCK  | 13   | Load cell #0 (forwarded to Pi)                   |
| HX711 tension DOUT | 34   | Input-only                                       |
| HX711 aux SCK      | 12   | Load cell #1 (forwarded to Pi)                   |
| HX711 aux DOUT     | 39   | Input-only (VN)                                  |
| Potentiometer      | 36   | ADC1_CH0 (VP), input-only, 12-bit, 32-sample MA  |
| Encoder manual A   | 0    | PCNT — strap pin (ensure safe state at boot)      |
| Encoder manual B   | 15   | PCNT — strap pin (ensure safe state at boot)      |

> ⚠️ GPIO 12 must be LOW at boot — HX711 SCK is LOW at idle ✓.
> GPIO 23 was `HOME_PIN_NO` in the old `resources/esp32/Config.h`.
> It is now SPI MOSI — the home sensor NO contact has moved to GPIO 21.
> The old POT_PIN (GPIO 34) from `Config.h` is now HX711[0] DOUT.
> The potentiometer has moved to GPIO 36 (VP, ADC1_CH0, input-only).
> GPIO0/15 are used for the manual encoder. These are strapping pins on
> many ESP32 modules — avoid driving the encoder during reset/flash or add
> pull resistors to guarantee a safe boot level.

> Note: UART for TMC2209 is handled by the Raspberry Pi, not the ESP32.

## Build & Flash

### ESP32 (PlatformIO — ESP-IDF framework)

```bash
cd src/esp32/
pio run -t upload              # build + flash over USB
pio device monitor -b 115200  # serial monitor (if console is re-enabled)
```

> The firmware uses `framework = espidf`. Entry point is `app_main()`, not `setup()/loop()`.
> `sdkconfig.defaults` overrides are applied automatically by PlatformIO on first build.

### RPi Python

```bash
cd src/rpi/
pip install -r requirements.txt
python3 -m pytest tests/ -v              # run unit tests (no hardware)
python3 main.py --preset strat --dry-run # dry-run with mock ESP32
python3 main.py --preset strat           # real hardware
```

## SPI Protocol Summary

- **Command frame**: 8 bytes (opcode + axis + uint32 data + flags + CRC-8/MAXIM)
- **Status frame**: 44 bytes (3× axis state + HX711 readings + pot_raw + encoder_manual)
- **Speed unit**: Hz (step frequency). 6400 steps/rev → 6400 Hz = 60 RPM.

## Tension Control

HX711 readings are acquired by the ESP32 sensor_task and forwarded to the Raspberry Pi via SPI.
The Pi performs the PID loop at ~80 Hz and sends back a tension setpoint to the ESP32.

```python
await controller.set_tension(setpoint_dg=500)  # 50 g
status = await controller.get_status()
print(f"Tension: {status.tension_raw[0] / 10:.1f} g")
print(f"Pot: {status.pot_raw}")          # 0–4095
print(f"Encoder: {status.encoder_manual}")  # signed int16 delta
```

## Directory Structure

```
src/esp32/          ESP32 PlatformIO firmware (C++17, ESP-IDF + FreeRTOS)
  src/
    encoder.h/.cpp    PCNT quadrature decoder (GPIO 0/15)
    pot.h/.cpp        ADC1 potentiometer driver (GPIO 36)
    sensor_task.h/.cpp  Core 0 sensor acquisition task
    hx711.h/.cpp      HX711 bitbang driver (no PID)
    axis.h/.cpp       Per-axis step ISR, trapezoidal ramp
    stepper_engine.h/.cpp  3-axis engine, Core 1 task
    spi_slave.h/.cpp  SPI slave DMA driver, Core 0 task
    endstop.h/.cpp    2-contact endstop + homing
    protocol.h        CmdFrame / StatusFrame / CRC-8
    main.cpp          Pin config, app_main()
  sdkconfig.defaults  SDK config overrides (console=none, 240 MHz, 1 kHz tick)
src/rpi/        RPi Python application (asyncio)
  hal/          SPI transport, protocol, axis config
  machine/      CoilWinder, TensionController, homing
  tests/        32 unit tests (pytest, no hardware needed)
doc/            Architecture documentation
resources/      Reference code (do not modify)
  esp32/        Old standalone ESP32 project (original pinout reference)
  klipper/      Klipper stepper acceleration reference
  fastaccelstepper/  FastAccelStepper reference
```

## Motor Constants

| Parameter          | Value                                     |
|--------------------|-------------------------------------------|
| Spindle steps/rev  | 6400 (200 full × 32 µstep)               |
| Lateral steps/mm   | 3072 (96 full × 32 µstep, M6 1 mm pitch) |
| Speed range        | 100 – 160 000 Hz (~0.9 – 1500 RPM)       |
| ESP32 timer clock  | 40 MHz (APB/2, 25 ns resolution)          |
