# Copilot Instructions — PickupWinder (RPi + ESP32)

> Platform: Raspberry Pi (Python application) + ESP32 (real-time stepper controller).
> ESP32 firmware uses **ESP-IDF framework** (not Arduino). Entry point is `app_main()`.
> Old BeagleBone Black sources are in `resources/esp32/` (reference only — do not modify).

---

## 1. Project Identity & Goals

- **Purpose**: Automated/assisted guitar pickup coil winding — precise lateral
  traversal, real-time speed control, wire tension control, recipe persistence.
- **Hardware**: ESP32 (dual-core FreeRTOS, ESP-IDF), three A4988/DRV8825 stepper drivers
  (bobbin + lateral + tensioner), two HX711 load cells, 2-contact home sensor,
  potentiometer on GPIO 36, quadrature encoder on GPIO 1/3, SPI link to Raspberry Pi.
- **Core constraint**: Firmware drives physical motors under wire tension.
  Correctness and determinism always outweigh elegance.

---

## 2. Architecture Overview

Two-processor architecture: Raspberry Pi (Python) ↔ ESP32 (C++/FreeRTOS/ESP-IDF) over SPI.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Python application  (asyncio, Raspberry Pi)                         │
│  main.py · CoilWinder · TensionController · WebUI                   │
│                      │ spidev SPI0, 4 MHz                            │
│              rpi/hal/esp32_controller.py                             │
│           CmdFrame (8 B) → / ← StatusFrame (44 B)                   │
├──────────────────────────────────────────────────────────────────────┤
│  ESP32  (240 MHz, FreeRTOS/ESP-IDF, dual-core)                       │
│                                                                      │
│  Core 0 (priority  5): sensor_task                                  │
│    HX711 non-blocking poll every 1 ms (~80 Hz actual)               │
│    ADC1 potentiometer every 20 ms (~50 Hz)                          │
│    PCNT quadrature encoder every 1 ms                               │
│    Writes SensorState g_sensor under portMUX spinlock               │
│                                                                      │
│  Core 0 (priority 10): spi_task                                     │
│    Receives CmdFrame → CmdQueue → Core 1                            │
│    Sends StatusFrame (reads g_sensor under spinlock)                │
│                                                                      │
│  Core 1 (priority 24): stepper_task                                 │
│    Dispatches CmdQueue → Axis commands                              │
│    Polls endstops every 1 ms (non-blocking)                         │
│    Hardware timers → STEP ISRs (jitter < 1 µs)                      │
│      Timer 0/0 → Axis 0 (Bobbin)                                    │
│      Timer 0/1 → Axis 1 (Lateral)                                   │
│      Timer 1/0 → Axis 2 (Tensioner)                                 │
└──────────────────────────────────────────────────────────────────────┘
```

Key files:
- `src/esp32/src/main.cpp` — pin config, `app_main()`
- `src/esp32/sdkconfig.defaults` — SDK overrides (console=none, 240 MHz, 1 kHz tick)
- `src/esp32/src/protocol.h` — CmdFrame / StatusFrame / CRC-8/MAXIM
- `src/esp32/src/axis.h/.cpp` — per-axis step ISR, trapezoidal ramp
- `src/esp32/src/stepper_engine.h/.cpp` — 3-axis engine, command dispatch
- `src/esp32/src/spi_slave.h/.cpp` — SPI slave driver, Core 0 task (pri 10)
- `src/esp32/src/endstop.h/.cpp` — 2-contact endstop ISR + homing
- `src/esp32/src/hx711.h/.cpp` — HX711 bitbang driver (no PID — Pi handles PID)
- `src/esp32/src/encoder.h/.cpp` — PCNT quadrature decoder (GPIO 0/15)
- `src/esp32/src/pot.h/.cpp` — ADC1 potentiometer driver (GPIO 36)
- `src/esp32/src/sensor_task.h/.cpp` — Core 0 sensor acquisition task (pri 5)
- `src/rpi/hal/protocol.py` — Python mirror of protocol.h
- `src/rpi/hal/esp32_controller.py` — async command/event interface
- `src/rpi/machine/coil_winder.py` — WindingState machine

---

## 3. SPI Protocol

Full-duplex, SPI Mode 0, 4 MHz. Each transfer sends **CmdFrame (8 bytes)** and
receives **StatusFrame (44 bytes)** simultaneously.

### CmdFrame layout

```
Byte  Field   Description
0     cmd     CmdOpcode (uint8)
1     axis    AxisId: 0=Bobbin, 1=Lateral, 2=Tensioner, 0xFF=ALL
2..5  data    uint32_t little-endian payload
6     flags   CmdFlags bitfield
7     crc8    CRC-8/MAXIM over bytes 0..6
```

### StatusFrame layout (44 bytes)

```
Byte   Field              Description
0      global_flags       StatusFlags bitfield
1      event_type         EventType
2      event_axis         Axis that raised the event
3      endstop_mask       Bit per axis (bit 0 = axis 0)
4..7   uptime_ms          uint32_t LE
8..15  axis[0]            AxisStatus — Bobbin  (position i32, hz u16, flags u8, pad)
16..23 axis[1]            AxisStatus — Lateral
24..31 axis[2]            AxisStatus — Tensioner
32..33 tension_raw[0]     HX711 #0 in 0.1 g (int16_t LE)
34..35 tension_raw[1]     HX711 #1 in 0.1 g (int16_t LE)
36..37 tension_setpoint   Active PID setpoint in 0.1 g (int16_t LE)
38..39 pot_raw            ADC1 potentiometer 0–4095 (int16_t LE)
40..41 encoder_manual     PCNT quadrature delta, signed (int16_t LE)
42..43 reserved           0x00 0x00
```

### Command opcodes

| Opcode      | Value | Data              |
|-------------|-------|-------------------|
| NOP         | 0x00  | —                 |
| SET_SPEED   | 0x01  | Hz (uint32)       |
| MOVE_ABS    | 0x02  | steps (int32)     |
| MOVE_REL    | 0x03  | steps (int32)     |
| STOP        | 0x04  | —                 |
| ESTOP       | 0x05  | —                 |
| ENABLE      | 0x06  | 1=on, 0=off       |
| HOME        | 0x07  | —                 |
| SET_ACCEL   | 0x08  | steps/s² (uint32) |
| GET_STATUS  | 0x09  | —                 |
| SET_MODE    | 0x0A  | 0=free, 1=winding |
| RESET_POS   | 0x0B  | —                 |
| SET_LIMITS  | 0x0C  | limit (int32)     |
| ACK_EVENT   | 0x0D  | —                 |
| SET_TENSION | 0x0E  | 0.1g setpoint     |
| TARE_HX711  | 0x0F  | axis=sensor index |

---

## 4. Pin Assignments (ESP32 — MUST NOT change implicitly)

### Stepper axes

| Axis            | STEP | DIR | EN  | Notes                  |
|-----------------|------|-----|-----|------------------------|
| 0 — Bobbin      | 26   | 27  | 14  | No endstop             |
| 1 — Lateral     | 32   | 33  | 25  | 2-contact home sensor  |
| 2 — Tensioner   | 16   | 17  | 4   | No dedicated endstop   |

EN pins: active LOW (driver ON when GPIO = LOW).

### Lateral home sensor (2-contact)

Both pins configured as input with pull-up (`gpio_config()`, `GPIO_PULLUP_ONLY`):

| Contact | GPIO | Away       | At home    | Fault      |
|---------|------|------------|------------|------------|
| NO      | 21   | HIGH (open)| LOW (closed)| LOW       |
| NC      | 22   | LOW (closed)| HIGH (open)| LOW       |

Valid home: NO=LOW AND NC=HIGH.
Fault (disconnected): NO=LOW AND NC=LOW.

> ⚠️ Old `Config.h` had `HOME_PIN_NO = 23`. GPIO 23 is now SPI MOSI.
> The NO contact was moved to **GPIO 21**.

### SPI (VSPI / SPI3)

| Signal | GPIO |
|--------|------|
| MOSI   | 23   |
| MISO   | 19   |
| SCLK   | 18   |
| CS     | 5    |

### HX711 load cells

| Sensor     | SCK    | DOUT   | Notes                                    |
|------------|--------|--------|------------------------------------------|
| Tension[0] | GPIO 13| GPIO 34| Read by ESP32 and forwarded to RPi       |
| Aux[1]     | GPIO 12| GPIO 39| Read by ESP32 and forwarded to RPi       |

GPIO 12: strapping pin — must be LOW at boot. HX711 SCK idle = LOW ✓.
GPIO 34, 39: input-only (no OUTPUT capability). DOUT only ever needs to be read.

### Potentiometer & Encoder

| Function       | GPIO | Notes                                                     |
|----------------|------|-----------------------------------------------------------|
| Potentiometer  | 36   | ADC1_CH0 (VP), input-only, 12-bit, 32-sample MA ~50 Hz   |
| Encoder A      | 1    | PCNT_UNIT_0 — UART0 TX, freed by CONFIG_ESP_CONSOLE_UART_NONE |
| Encoder B      | 3    | PCNT_UNIT_0 — UART0 RX, freed by CONFIG_ESP_CONSOLE_UART_NONE |

> Old `Config.h` `POT_PIN = 34` is occupied by HX711[0] DOUT. Pot moved to GPIO 36.
> Old encoder pins (GPIO 18/19) are now SPI SCLK/MISO.
> GPIO0/15 are used for the manual encoder. These are strapping pins on
> many ESP32 modules — avoid driving the encoder during reset/flash or add
> pull resistors to guarantee a safe boot level.

> Note: UART for TMC2209 is handled by the Raspberry Pi, not the ESP32.

### Other

| Signal | GPIO | Notes                          |
|--------|------|--------------------------------|
| —      | —    | No dedicated E-STOP pin used   |

Any pin change **must** update `src/esp32/src/main.cpp` + `doc/architecture.md` + this file.

---

## 5. Hard Rules

### 5.1 ESP32 Firmware

- **Framework is ESP-IDF**, not Arduino. Use `app_main()`, not `setup()/loop()`.
  Never use Arduino HAL functions (`digitalWrite`, `digitalRead`, `millis`, `pinMode`, etc.).
  Use ESP-IDF equivalents: `gpio_set_level()`, `gpio_get_level()`, `esp_timer_get_time()`, `gpio_config()`.
- **NEVER** call `vTaskDelay()` or blocking I/O from a timer ISR.
- All timer ISRs must be `IRAM_ATTR` and declared as free functions with `void(*)(void*)` signature.
- `IRAM_ATTR` placement: on the function signature line in the `.cpp` file, not on the declaration in the header.
- The `SensorState g_sensor` is protected by a `portMUX_TYPE` spinlock.
  The `StatusFrame` in `g_engine.status_` is also protected by a spinlock.
  Always use `portENTER_CRITICAL / portEXIT_CRITICAL` for access across tasks.
- `volatile` required on all shared-state fields in `Axis` that are written by the ISR
  and read by the stepper task or SPI task.
- `AxisPins.endstop_no` / `endstop_nc` — set to -1 when not used.
  Never assume a pin is valid without checking `>= 0`.
- HX711 reads are **non-blocking**: check `gpio_get_level(DOUT) == 0` first.
  Never spin-wait for DOUT in sensor_task tight loop.
- GPIO 12 must not be driven HIGH at boot.
- For GPIO >= 32, use `GPIO.out1_w1ts.val` / `GPIO.out1_w1tc.val` for fast bit-bang.
  `GPIO.out_w1ts` only affects GPIO 0–31.

### 5.2 Python Application

- Python **never** touches SPI or GPIO directly.
  All hardware access goes through `EspController` → SPI → ESP32.
- No hardware constants (GPIO numbers, step counts, intervals) in Python
  application code (`machine/`). Hardware constants live in `hal/axis.py`
  or `config/machine_config.yaml`.
- `EspController` is the only Python class that knows about `CmdOpcode` / `StatusFrame`.
  `CoilWinder` only sees `set_speed()`, `move_to()`, `home()`, etc.

### 5.3 Protocol

- CRC-8/MAXIM over bytes 0..6 of each CmdFrame. Drop silently on mismatch.
- **Status frame size = 44 bytes.** Python and C must stay in sync.
- All multi-byte fields: little-endian on both sides (LE native on Xtensa and ARM).

---

## 6. Acceleration Model (Klipper-style)

```
for each segment i in [0, N_SEG=16):
    interval = start_iv[i]      ← force-loaded
    for s in range(count[i]):
        step()
        interval += add[i]
```

Segments are pre-computed by `Axis::build_simple_ramp()` on the ESP32.
A single `{start_iv, add, count}` segment cannot cover a wide speed range —
linear `add` means constant ΔHz per step but ΔHz/Hz is non-constant. Use N_SEG=16+.

---

## 7. HX711 & Sensor Architecture

- **sensor_task** (Core 0, priority 5) owns all sensor acquisition.
  It calls `hx711_tick()`, `pot_read()`, and `encoder_get_and_clear_delta()`.
- HX711 tick fires every 1 ms but only reads when `gpio_get_level(DOUT) == 0` (data ready, ~80 Hz).
- Potentiometer: `pot_read()` every 20 ms, 32-sample moving average, ~50 Hz.
- Encoder: PCNT hardware (4X quadrature), `encoder_get_and_clear_delta()` every 1 ms.
- **No PID on the ESP32.** Tension PID runs on the Raspberry Pi.
  The Pi sends back a setpoint via `SET_TENSION`; the ESP32 stores it in `g_sensor.tension_setpoint`.
- Output unit: 0.1 g (decigrams). Range: ±3276.7 g in int16_t.
- `hx711_tare()` is blocking — call only during machine idle via `sensor_request_tare()`.

---

## 8. Common Pitfalls

| Pitfall | Why it matters |
|---------|----------------|
| Using Arduino API (digitalWrite, millis, etc.) | Framework is ESP-IDF — Arduino symbols are undefined |
| `IRAM_ATTR` on declaration not definition | ISR placed in flash → cache miss → hard fault under load |
| Timer ISR not a free function | `timer_isr_register()` expects `void(*)(void*)`, rejects member pointers |
| Missing `volatile` on ISR-written fields | Compiler caches stale value in register |
| Blocking in HX711 tick | `gpio_get_level(DOUT)` before reading; never spin-wait in the sensor loop |
| GPIO 12 HIGH at boot | Causes ESP32 flash voltage issue on some modules |
| GPIO 34-39 used as output | Input-only — any `gpio_set_level()` silently ignored |
| GPIO >= 32 with `out_w1ts` | Must use `out1_w1ts.val` for GPIO 32–39 in fast ISR code |
| EndStop NC check skipped | Single-pin mode masks sensor faults; NO=LOW AND NC=LOW = wiring break |
| STATUS_FRAME_SIZE mismatch | Python/C must both be **44 bytes**; assert in both languages |
| portMUX not used on g_sensor or StatusFrame | sensor_task and stepper_task run on different cores simultaneously |
| HX711 scale = 0 | Division by zero in `raw_to_dg()` — always validate before calibrating |
| Encoder on GPIO 1/3 without sdkconfig | UART0 drives those pins by default; `CONFIG_ESP_CONSOLE_UART_NONE=y` required |

---

## 9. Quick Reference: Adding a New Feature

1. **New command opcode**: add to `CmdOpcode` enum in `protocol.h` and `protocol.py`,
   handle in `stepper_engine.cpp::dispatch_command()`,
   add method to `EspController`, add test in `rpi/tests/`.
2. **New status field**: extend `StatusFrame` in `protocol.h` (must stay packed,
   update `STATUS_FRAME_SIZE` and `static_assert`), decode in `protocol.py`,
   update `MockSpiTransport._build_status()`, update tests.
3. **New axis feature**: add to `Axis` class, update `StatusFlags` if needed,
   expose via `StepperEngine`, wire into `EspController`.
4. **New sensor**: add acquisition to `sensor_task.cpp`, store in `SensorState g_sensor`,
   expose in `StatusFrame` extension bytes, decode in `protocol.py`.

---

## 10. File Organization

```
.github/                    CI + copilot instructions
esp32/                      ESP32 PlatformIO project (C++17, ESP-IDF)
  sdkconfig.defaults        SDK overrides (console=none, 240 MHz, 1 kHz tick)
  src/
    protocol.h              CmdFrame, StatusFrame (44 B), CRC-8/MAXIM
    command_queue.h         SPSC ring buffer (16 slots)
    axis.h / axis.cpp       Per-axis state, step ISR, Klipper-style ramp
    stepper_engine.h/.cpp   3-axis engine, FreeRTOS Core 1 task (pri 24)
    spi_slave.h/.cpp        SPI slave DMA driver, Core 0 task (pri 10)
    endstop.h/.cpp          2-contact ISR + homing
    hx711.h/.cpp            HX711 bitbang driver (no PID)
    encoder.h/.cpp          PCNT quadrature decoder (GPIO 1/3)
    pot.h/.cpp              ADC1 potentiometer driver (GPIO 36)
    sensor_task.h/.cpp      Core 0 sensor acquisition task (pri 5)
    main.cpp                Pin config, app_main()
  platformio.ini
rpi/                        Raspberry Pi Python application
  hal/
    protocol.py             Python mirror of protocol.h (STATUS_FRAME_SIZE=44)
    spi_transport.py        spidev wrapper, thread-safe
    axis.py                 AxisConfig, unit conversions
    esp32_controller.py     Async ESP32Controller
  machine/
    coil_winder.py          WindingState FSM, CoilWinder
    tensioner.py            TensionController (SET_TENSION command)
    homing.py               home_axis(), home_all()
  config/
    machine_config.yaml     Hardware constants + bobbin presets
  tests/                    pytest (32 tests, no hardware required)
  main.py                   asyncio CLI entry point
doc/                        Architecture docs
  architecture.md           Full design reference
resources/                  Reference code (DO NOT MODIFY)
  esp32/                    Old standalone ESP32 project (original pinout)
  klipper/                  Klipper stepper.c reference
  fastaccelstepper/         FastAccelStepper reference
```

---

## 11. Hardware Constants

| Parameter           | Value              | Notes                              |
|---------------------|--------------------|------------------------------------|
| Spindle steps/rev   | 6400               | 200 full × 32 µstep                |
| Lateral steps/mm    | 3072               | 96 full × 32 µstep, M6 1 mm pitch  |
| Speed range (Hz)    | 100 – 160 000      | ~0.9 – 1500 RPM at 6400 steps/rev  |
| ESP32 timer clock   | 40 MHz             | APB 80 MHz / prescaler 2           |
| Timer resolution    | 25 ns              | 1 tick = 25 ns                     |
| HX711 sample rate   | ~80 Hz             | At VCC ≥ 4.8 V (RATE pin = HIGH)   |
| HX711 output unit   | 0.1 g (decigram)   | int16_t, range ±3276.7 g           |
| Potentiometer range | 0 – 4095           | 12-bit ADC1_CH0, 32-sample MA      |
| Encoder interface   | PCNT_UNIT_0, 4X    | GPIO 1 (A) / GPIO 3 (B)            |

---

## 1. Project Identity & Goals

- **Purpose**: Automated/assisted guitar pickup coil winding — precise lateral
  traversal, real-time speed control, wire tension control, recipe persistence.
- **Hardware**: ESP32 (dual-core FreeRTOS), three A4988/DRV8825 stepper drivers
  (bobbin + lateral + tensioner), two HX711 load cells, 2-contact home sensor,
  SPI link to Raspberry Pi.
- **Core constraint**: Firmware drives physical motors under wire tension.
  Correctness and determinism always outweigh elegance.

---

## 2. Architecture Overview

Two-processor architecture: Raspberry Pi (Python) ↔ ESP32 (C++/FreeRTOS) over SPI.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Python application  (asyncio, Raspberry Pi)                         │
│  main.py · CoilWinder · TensionController · WebUI                   │
│                      │ spidev SPI0, 4 MHz                            │
│              rpi/hal/esp32_controller.py                             │
│           CmdFrame (8 B) → / ← StatusFrame (40 B)                   │
├──────────────────────────────────────────────────────────────────────┤
│  ESP32  (240 MHz, FreeRTOS, dual-core)                               │
│                                                                      │
│  Core 0 (priority 10): spi_slave task                               │
│    Receives CmdFrame → CmdQueue → Core 1                            │
│    Sends StatusFrame built from g_engine.get_status()               │
│                                                                      │
│  Core 1 (priority 24): stepper_task                                 │
│    Dispatches CmdQueue → Axis commands                              │
│    Polls endstops + HX711 every 1 ms (non-blocking)                 │
│    Samples encoders + HX711 and forwards values to Pi               │
│    Hardware timers → STEP ISRs (jitter < 1 µs)                      │
│      Timer 0/0 → Axis 0 (Bobbin)                                    │
│      Timer 0/1 → Axis 1 (Lateral)                                   │
│      Timer 1/0 → Axis 2 (Tensioner)                                 │
└──────────────────────────────────────────────────────────────────────┘
```

Key files:
- `esp32/src/main.cpp` — pin config, FreeRTOS startup
- `esp32/src/protocol.h` — CmdFrame / StatusFrame / CRC-8/MAXIM
- `esp32/src/axis.h/.cpp` — per-axis step ISR, trapezoidal ramp
- `esp32/src/stepper_engine.h/.cpp` — 3-axis engine, command dispatch
- `esp32/src/spi_slave.h/.cpp` — SPI slave driver, Core 0 task
- `esp32/src/endstop.h/.cpp` — 2-contact endstop ISR + homing
- `esp32/src/hx711.h/.cpp` — HX711 driver + tension PI controller
- `rpi/hal/protocol.py` — Python mirror of protocol.h
- `rpi/hal/esp32_controller.py` — async command/event interface
- `rpi/machine/coil_winder.py` — WindingState machine

---

## 3. SPI Protocol

Full-duplex, SPI Mode 0, 4 MHz. Each transfer sends **CmdFrame (8 bytes)** and
receives **StatusFrame (40 bytes)** simultaneously.

### CmdFrame layout

```
Byte  Field   Description
0     cmd     CmdOpcode (uint8)
1     axis    AxisId: 0=Bobbin, 1=Lateral, 2=Tensioner, 0xFF=ALL
2..5  data    uint32_t little-endian payload
6     flags   CmdFlags bitfield
7     crc8    CRC-8/MAXIM over bytes 0..6
```

### StatusFrame layout (40 bytes)

```
Byte   Field              Description
0      global_flags       StatusFlags bitfield
1      event_type         EventType
2      event_axis         Axis that raised the event
3      endstop_mask       Bit per axis (bit 0 = axis 0)
4..7   uptime_ms          uint32_t LE
8..15  axis[0]            AxisStatus — Bobbin  (position i32, hz u16, flags u8, pad)
16..23 axis[1]            AxisStatus — Lateral
24..31 axis[2]            AxisStatus — Tensioner
32..33 tension_raw[0]     HX711 #0 in 0.1 g (int16_t LE)
34..35 tension_raw[1]     HX711 #1 in 0.1 g (int16_t LE)
36..37 tension_setpoint   Active PID setpoint in 0.1 g (int16_t LE)
38..39 reserved           0x00 0x00
```

### Command opcodes

| Opcode      | Value | Data              |
|-------------|-------|-------------------|
| NOP         | 0x00  | —                 |
| SET_SPEED   | 0x01  | Hz (uint32)       |
| MOVE_ABS    | 0x02  | steps (int32)     |
| MOVE_REL    | 0x03  | steps (int32)     |
| STOP        | 0x04  | —                 |
| ESTOP       | 0x05  | —                 |
| ENABLE      | 0x06  | 1=on, 0=off       |
| HOME        | 0x07  | —                 |
| SET_ACCEL   | 0x08  | steps/s² (uint32) |
| GET_STATUS  | 0x09  | —                 |
| SET_MODE    | 0x0A  | 0=free, 1=winding |
| RESET_POS   | 0x0B  | —                 |
| SET_LIMITS  | 0x0C  | limit (int32)     |
| ACK_EVENT   | 0x0D  | —                 |
| SET_TENSION | 0x0E  | 0.1g setpoint     |
| TARE_HX711  | 0x0F  | axis=sensor index |

---

## 4. Pin Assignments (ESP32 — MUST NOT change implicitly)

### Stepper axes

| Axis            | STEP | DIR | EN  | Notes                  |
|-----------------|------|-----|-----|------------------------|
| 0 — Bobbin      | 26   | 27  | 14  | No endstop             |
| 1 — Lateral     | 32   | 33  | 25  | 2-contact home sensor  |
| 2 — Tensioner   | 16   | 17  | 4   | Safety endstop GPIO 35 |

EN pins: active LOW (driver ON when GPIO = LOW).

### Lateral home sensor (2-contact)

Both pins `INPUT_PULLUP`. The sensor wiring matches `resources/esp32/Config.h`:

| Contact | GPIO | Away       | At home    | Fault      |
|---------|------|------------|------------|------------|
| NO      | 21   | HIGH (open)| LOW (closed)| LOW       |
| NC      | 22   | LOW (closed)| HIGH (open)| LOW       |

Valid home: NO=LOW AND NC=HIGH.
Fault (disconnected): NO=LOW AND NC=LOW.

> ⚠️ Old `Config.h` had `HOME_PIN_NO = 23`. GPIO 23 is now SPI MOSI.
> The NO contact was moved to **GPIO 21**.

### SPI (VSPI / SPI3)

| Signal | GPIO |
|--------|------|
| MOSI   | 23   |
| MISO   | 19   |
| SCLK   | 18   |
| CS     | 5    |

### HX711 load cells

| Sensor     | SCK    | DOUT   | Notes                                    |
|------------|--------|--------|------------------------------------------|
| Tension[0] | GPIO 13| GPIO 34| Read by ESP32 and forwarded to RPi       |
| Aux[1]     | GPIO 12| GPIO 39| Read by ESP32 and forwarded to RPi       |

GPIO 12: strapping pin — must be LOW at boot. HX711 SCK idle = LOW ✓.
GPIO 34, 39: input-only (no OUTPUT capability). DOUT only ever needs to be read.

### Encoders

| Function     | GPIO | Notes                                   |
|--------------|------|-----------------------------------------|
| Manual axis A| 36   | Input-only encoder input                 |
| Manual axis B| 37   | Input-only encoder input                 |
| UI encoder A | 1    | UART0 TX pin — use only if USB serial is unused |
| UI encoder B | 3    | UART0 RX pin — use only if USB serial is unused |

> Note: UART for TMC2209 is handled by the Raspberry Pi, not the ESP32.

### Other

| Signal | GPIO | Notes                          |
|--------|------|--------------------------------|
| —      | —    | No dedicated E-STOP pin used   |

Any pin change **must** update `esp32/src/main.cpp` + `doc/architecture.md` + this file.

---

## 5. Hard Rules

### 5.1 ESP32 Firmware

- **NEVER** call `vTaskDelay()` or blocking I/O from a timer ISR.
- All timer ISRs must be `IRAM_ATTR` and declared as free functions with `void(*)(void*)` signature.
- `IRAM_ATTR` placement: on the function signature line in the `.cpp` file, not on the declaration in the header.
- The `StatusFrame` in `g_engine.status_` is protected by a `portMUX_TYPE` spinlock.
  Always use `portENTER_CRITICAL / portEXIT_CRITICAL` for access across tasks.
- `volatile` required on all shared-state fields in `Axis` that are written by the ISR
  and read by the stepper task or SPI task.
- `AxisPins.endstop_no` / `endstop_nc` — set to -1 when not used.
  Never assume a pin is valid without checking `>= 0`.
- HX711 reads are **non-blocking**: check `digitalRead(DOUT) == LOW` first.
  Never spin-wait for DOUT in a timer ISR or the stepper task tight loop.
- GPIO 12 must not be driven HIGH at boot.

### 5.2 Python Application

- Python **never** touches SPI or GPIO directly.
  All hardware access goes through `EspController` → SPI → ESP32.
- No hardware constants (GPIO numbers, step counts, intervals) in Python
  application code (`machine/`). Hardware constants live in `hal/axis.py`
  or `config/machine_config.yaml`.
- `EspController` is the only Python class that knows about `CmdOpcode` / `StatusFrame`.
  `CoilWinder` only sees `set_speed()`, `move_to()`, `home()`, etc.

### 5.3 Protocol

- CRC-8/MAXIM over bytes 0..6 of each CmdFrame. Drop silently on mismatch.
- Status frame size = 40 bytes. Python and C must stay in sync.
- All multi-byte fields: little-endian on both sides (LE native on Xtensa and ARM).

---

## 6. Acceleration Model (Klipper-style)

```
for each segment i in [0, N_SEG=16):
    interval = start_iv[i]      ← force-loaded
    for s in range(count[i]):
        step()
        interval += add[i]
```

Segments are pre-computed by `Axis::build_simple_ramp()` on the ESP32.
A single `{start_iv, add, count}` segment cannot cover a wide speed range —
linear `add` means constant ΔHz per step but ΔHz/Hz is non-constant. Use N_SEG=16+.

---

## 7. HX711 Tension PID

- Runs in Core 1 (stepper task), called from `hx711_tick()` every 1 ms.
- Only fires when new data is ready (DOUT=LOW). Data rate ≈ 80 Hz.
- Sensor [0] → PI controller → tensioner axis speed.
- Sensor [1] → value stored in `g_hx711.reading_dg[1]`, forwarded in StatusFrame.
- Output unit: 0.1 g (decigrams). Range: ±3276.7 g in int16_t.
- `hx711_tare()` is blocking — call only during machine idle.

---

## 8. Common Pitfalls

| Pitfall | Why it matters |
|---------|----------------|
| `IRAM_ATTR` on declaration not definition | ISR placed in flash → cache miss → hard fault under load |
| Timer ISR not a free function | `timer_isr_register()` expects `void(*)(void*)`, rejects member pointers |
| Missing `volatile` on ISR-written fields | Compiler caches stale value in register |
| Blocking in HX711 tick | `digitalRead(DOUT)` before reading; never spin-wait in the stepper loop |
| GPIO 12 HIGH at boot | Causes ESP32 flash voltage issue on some modules |
| GPIO 34-39 used as output | Input-only — any `digitalWrite()` silently ignored |
| EndStop NC check skipped | Single-pin mode masks sensor faults; NO=LOW AND NC=LOW = wiring break |
| STATUS_FRAME_SIZE mismatch | Python/C must both be 40 bytes; assert in both languages |
| portMUX not used on StatusFrame | SPI task and stepper task run on different cores simultaneously |
| HX711 scale = 0 | Division by zero in `raw_to_dg()` — always validate before calibrating |

---

## 9. Quick Reference: Adding a New Feature

1. **New command opcode**: add to `CmdOpcode` enum in `protocol.h` and `protocol.py`,
   handle in `stepper_engine.cpp::dispatch_command()`,
   add method to `EspController`, add test in `rpi/tests/`.
2. **New status field**: extend `StatusFrame` in `protocol.h` (must stay packed,
   update `STATUS_FRAME_SIZE`, update `static_assert`), decode in `protocol.py`,
   update `MockSpiTransport._build_status()`, update tests.
3. **New axis feature**: add to `Axis` class, update `StatusFlags` if needed,
   expose via `StepperEngine`, wire into `EspController`.
4. **New sensor**: if it drives a motor, integrate in `stepper_engine.cpp::run()`.
   If data-only, store in `StatusFrame` extension bytes.

---

## 10. File Organization

```
.github/                    CI + copilot instructions
src/esp32/                  ESP32 PlatformIO project (C++17, ESP-IDF)
  sdkconfig.defaults        SDK overrides (console=none, 240 MHz, 1 kHz tick)
  src/
    protocol.h              CmdFrame, StatusFrame, CRC-8/MAXIM
    command_queue.h         SPSC ring buffer (16 slots)
    axis.h / axis.cpp       Per-axis state, step ISR, Klipper-style ramp
    stepper_engine.h/.cpp   3-axis engine, FreeRTOS Core 1 task
    spi_slave.h/.cpp        SPI slave DMA driver, Core 0 task
    endstop.h/.cpp          2-contact ISR + homing
    hx711.h/.cpp            HX711 bitbang driver (no PID)
    encoder.h/.cpp          PCNT quadrature decoder (GPIO 0/15)
    pot.h/.cpp              ADC1 potentiometer driver (GPIO 36)
    sensor_task.h/.cpp      Core 0 sensor acquisition task (pri 5)
    main.cpp                Pin config, app_main()
  platformio.ini
src/rpi/                    Raspberry Pi Python application
  hal/
    protocol.py             Python mirror of protocol.h
    spi_transport.py        spidev wrapper, thread-safe
    axis.py                 AxisConfig, unit conversions
    esp32_controller.py     Async ESP32Controller
  machine/
    coil_winder.py          WindingState FSM, CoilWinder
    tensioner.py            TensionController (SET_TENSION command)
    homing.py               home_axis(), home_all()
  config/
    machine_config.yaml     Hardware constants + bobbin presets
  tests/                    pytest (32 tests, no hardware required)
  main.py                   asyncio CLI entry point
doc/                        Architecture docs
  architecture.md           Full design reference
resources/                  Reference code (DO NOT MODIFY)
  esp32/                    Old standalone ESP32 project (original pinout)
  klipper/                  Klipper stepper.c reference
  fastaccelstepper/         FastAccelStepper reference
```

---

## 11. Hardware Constants

| Parameter           | Value              | Notes                              |
|---------------------|--------------------|------------------------------------|
| Spindle steps/rev   | 6400               | 200 full × 32 µstep                |
| Lateral steps/mm    | 3072               | 96 full × 32 µstep, M6 1 mm pitch  |
| Speed range (Hz)    | 100 – 160 000      | ~0.9 – 1500 RPM at 6400 steps/rev  |
| ESP32 timer clock   | 40 MHz             | APB 80 MHz / prescaler 2           |
| Timer resolution    | 25 ns              | 1 tick = 25 ns                     |
| HX711 sample rate   | ~80 Hz             | At VCC ≥ 4.8 V (RATE pin = HIGH)   |
| HX711 output unit   | 0.1 g (decigram)   | int16_t, range ±3276.7 g           |
