# PickupWinder — RPi + ESP32 Architecture

## Overview

Two-processor architecture: a Raspberry Pi (Python application + HAL) and an ESP32 (real-time stepper controller).
The ESP32 firmware uses the **ESP-IDF framework** (not Arduino). Entry point is `app_main()`.

```
┌──────────────────────────────────────────────────────────────┐
│  Raspberry Pi — Python application (asyncio)                 │
│  main.py  ·  CoilWinder  ·  TensionController  ·  WebUI     │
│                    │ spidev (SPI0)                            │
│           src/rpi/hal/esp32_controller.py                        │
│              8-byte CmdFrame → / ← 44-byte StatusFrame       │
├──────────────────────────────────────────────────────────────┤
│  ESP32 — dual-core, 240 MHz, FreeRTOS (ESP-IDF)              │
│                                                              │
│  Core 0 (priority  5): sensor_task                           │
│    HX711 non-blocking poll every 1 ms                        │
│    ADC1 potentiometer every 20 ms (~50 Hz)                   │
│    PCNT quadrature encoder every 1 ms                        │
│    Writes SensorState g_sensor under spinlock                │
│                                                              │
│  Core 0 (priority 10): spi_task                              │
│    Receives CmdFrame, pushes to CmdQueue, sends StatusFrame  │
│                                                              │
│  Core 1 (priority 24): stepper_task                          │
│    Dispatches commands → Axis objects                        │
│    Polls endstops every 1 ms                                 │
│    Reads g_sensor under spinlock → fills StatusFrame         │
│    Hardware timers → STEP pulse ISRs (jitter < 1 µs)         │
│                                                              │
│    Axis 0 (Bobbin):    Timer Group 0 / Timer 0              │
│    Axis 1 (Lateral):   Timer Group 0 / Timer 1              │
│    Axis 2 (Tensioner): Timer Group 1 / Timer 0              │
└──────────────────────────────────────────────────────────────┘
```

### Task Summary

| Task          | Core | Priority | Stack | Responsibility                          |
|---------------|------|----------|-------|-----------------------------------------|
| sensor_task   | 0    | 5        | 4 KB  | HX711, ADC pot, PCNT encoder            |
| spi_task      | 0    | 10       | 4 KB  | SPI slave DMA, CmdFrame RX / StatusFrame TX |
| stepper_task  | 1    | 24       | 8 KB  | Axis dispatch, endstop poll, ISR timer scheduling |

### sdkconfig Overrides (`src/esp32/sdkconfig.defaults`)

| Key                                    | Value | Reason                                      |
|----------------------------------------|-------|---------------------------------------------|
| `CONFIG_ESP_CONSOLE_UART_NONE`         | y     | Frees GPIO 1/3 for the quadrature encoder   |
| `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240`  | y     | Full speed for ISR timing                   |
| `CONFIG_FREERTOS_HZ`                   | 1000  | 1 ms FreeRTOS tick resolution               |

---

## SPI Protocol

Full-duplex, SPI Mode 0, 4 MHz clock.  Every RPi→ESP32 transfer sends one `CmdFrame` (8 bytes) and simultaneously receives one `StatusFrame` (44 bytes).

### CmdFrame (8 bytes, RPi → ESP32)

| Byte | Field  | Description                          |
|------|--------|--------------------------------------|
| 0    | cmd    | `CmdOpcode` (see below)              |
| 1    | axis   | `AxisId` (0=Bobbin, 1=Lateral, 2=Tensioner, 0xFF=ALL) |
| 2–5  | data   | uint32_t payload, little-endian      |
| 6    | flags  | `CmdFlags` bitfield                  |
| 7    | crc8   | CRC-8/MAXIM over bytes 0–6           |

### StatusFrame (44 bytes, ESP32 → RPi)

| Bytes | Field             | Description                              |
|-------|-------------------|------------------------------------------|
| 0     | global_flags      | `StatusFlags` bitfield                   |
| 1     | event_type        | `EventType`                              |
| 2     | event_axis        | Axis that generated the event            |
| 3     | endstop_mask      | Bit per axis (bit 0 = axis 0, etc.)      |
| 4–7   | uptime_ms         | Milliseconds since boot (uint32_t)       |
| 8–15  | axis[0]           | `AxisStatus` — Bobbin                    |
| 16–23 | axis[1]           | `AxisStatus` — Lateral                   |
| 24–31 | axis[2]           | `AxisStatus` — Tensioner                 |
| 32–33 | tension_raw[0]    | HX711 #0 (tension) in 0.1 g (int16_t)   |
| 34–35 | tension_raw[1]    | HX711 #1 (auxiliary) in 0.1 g (int16_t) |
| 36–37 | tension_setpoint  | Active PID setpoint in 0.1 g (int16_t)  |
| 38–39 | pot_raw           | ADC1 potentiometer value 0–4095 (int16_t)|
| 40–41 | encoder_manual    | PCNT quadrature delta (int16_t, signed)  |
| 42–43 | reserved          | 0x00 0x00                                |

### AxisStatus (8 bytes)

| Bytes | Field      | Description                              |
|-------|------------|------------------------------------------|
| 0–3   | position   | Step counter (int32_t, signed)           |
| 4–5   | current_hz | Step frequency (uint16_t, 0 = stopped)   |
| 6     | flags      | `StatusFlags` per-axis bitfield          |
| 7     | _pad       | 0                                        |

### Command Opcodes

| Opcode      | Value | Data field               | Description                        |
|-------------|-------|--------------------------|------------------------------------|
| NOP         | 0x00  | —                        | No operation                       |
| SET_SPEED   | 0x01  | uint32 Hz                | Set constant speed for axis        |
| MOVE_ABS    | 0x02  | int32 steps              | Absolute move                      |
| MOVE_REL    | 0x03  | int32 steps              | Relative move                      |
| STOP        | 0x04  | —                        | Controlled deceleration            |
| ESTOP       | 0x05  | —                        | Immediate all-axis halt            |
| ENABLE      | 0x06  | uint32 (1=en, 0=dis)     | Enable/disable stepper drivers     |
| HOME        | 0x07  | —                        | Start homing sequence              |
| SET_ACCEL   | 0x08  | uint32 steps/s²          | Set axis acceleration              |
| GET_STATUS  | 0x09  | —                        | Request status (same as NOP)       |
| SET_MODE    | 0x0A  | uint32 (0=free, 1=wind)  | Set winding sync mode              |
| RESET_POS   | 0x0B  | —                        | Reset position counter to 0        |
| SET_LIMITS  | 0x0C  | int32 limit value        | Set software position limit        |
| ACK_EVENT   | 0x0D  | —                        | Acknowledge pending event          |
| SET_TENSION | 0x0E  | uint16 setpoint (0.1 g)  | Set tension PID target (0=disable) |
| TARE_HX711  | 0x0F  | axis=sensor index (0/1)  | Zero a load cell                   |

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
The stepper_task (Core 1) reads from `g_sensor` under the same spinlock when building the `StatusFrame`.

```
sensor_task (Core 0):
  every 1  ms: hx711_tick()    — non-blocking DOUT check, ~80 Hz actual reads
  every 20 ms: pot_read()      — 32-sample moving average, ~50 Hz
  every 1  ms: encoder_get_and_clear_delta()  — PCNT hardware counter

g_sensor (shared, spinlock protected):
  tension_dg[2]      — latest HX711 readings in 0.1 g
  tension_setpoint   — setpoint forwarded from Pi via SET_TENSION
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
The ESP32 forwards `tension_raw[0]` and `tension_raw[1]` to the Pi, and the Pi
sends back a setpoint with `SET_TENSION`.

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

The RPi sets the setpoint with `SET_TENSION` (0 = disable PID). Current reading
is returned in `StatusFrame.tension_raw[0]` every SPI cycle.

---

## Homing Sequence (Lateral Axis)

1. RPi sends `HOME` command (axis = LATERAL).
2. ESP32 sets lateral axis direction = reverse, speed = 2000 Hz.
3. Stepper task polls `update_endstop()` every 1 ms.
4. When NO=LOW AND NC=HIGH (valid home): axis stops, event `HOME_COMPLETE` raised.
5. When NO=LOW AND NC=LOW (fault): axis stops, event `FAULT` raised.
6. RPi receives `HOME_COMPLETE` event and resets lateral position to 0.

---

## Acceleration Model (Klipper-style)

Both axes use a multi-segment trapezoidal ramp:

```
for each segment i in [0, N_SEG=16):
    interval  = start_iv[i]      ← force-loaded at segment boundary
    for step in range(count[i]):
        step()
        interval += add[i]
```

Segments are pre-computed by `Axis::build_simple_ramp()` on the ESP32
using floating-point arithmetic (240 MHz, FPU available on ESP32).

Speed range: 100 Hz (~ 0.9 RPM) to 160 000 Hz (~ 1500 RPM) at 6400 steps/rev.

---

## File Structure

```
src/esp32/                    ← ESP32 firmware (PlatformIO, ESP-IDF framework)
  sdkconfig.defaults          ← SDK overrides (console=none, 240 MHz, 1 kHz tick)
  src/
    protocol.h                ← CmdFrame / StatusFrame definitions + CRC-8
    command_queue.h           ← Lock-free SPSC ring buffer (CmdFrame)
    axis.h / axis.cpp         ← Per-axis state, step ISR, trapezoidal ramp
    stepper_engine.h/.cpp     ← 3-axis engine, FreeRTOS Core 1 task
    spi_slave.h/.cpp          ← SPI slave DMA driver, Core 0 task (pri 10)
    endstop.h/.cpp            ← 2-contact endstop ISR + homing
    hx711.h/.cpp              ← HX711 bitbang driver (no PID — Pi handles PID)
    encoder.h/.cpp            ← PCNT quadrature decoder (GPIO 0/15)
    pot.h/.cpp                ← ADC1 potentiometer driver (GPIO 36)
    sensor_task.h/.cpp        ← Core 0 sensor acquisition task (pri 5)
    main.cpp                  ← Pin config, app_main()
  platformio.ini

src/rpi/                       ← RPi Python application
  hal/
    protocol.py               ← Python mirror of protocol.h
    spi_transport.py          ← Thread-safe spidev wrapper
    axis.py                   ← Axis config, unit conversions
    esp32_controller.py       ← Async ESP32 command/event interface
  machine/
    coil_winder.py            ← WindingState machine, CoilWinder
    tensioner.py              ← Tension control (uses SET_TENSION)
    homing.py                 ← Homing sequence helpers
  config/
    machine_config.yaml       ← Hardware constants + bobbin presets
  tests/                      ← pytest (32 tests, host-runnable, no hardware)
  main.py                     ← asyncio CLI entry point

doc/                          ← Architecture documentation
resources/                    ← Reference material (do not modify)
  esp32/                      ← Old standalone ESP32 project (reference pinout)
  klipper/                    ← Klipper stepper.c reference
  fastaccelstepper/           ← FastAccelStepper reference
```
