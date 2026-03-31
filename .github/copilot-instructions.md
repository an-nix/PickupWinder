# Copilot Instructions — ESP32 Embedded C++ (Arduino + PlatformIO + FreeRTOS)

You are assisting on an **embedded C++ project targeting the ESP32** using the **Arduino framework**, managed with **PlatformIO**, with **FreeRTOS** for multitasking.
Always prioritize **determinism, low memory footprint, and hardware safety** over convenience or abstraction.

---

## Project Context

- **Target**: ESP32 (Xtensa LX6/LX7)
- **Framework**: Arduino (via PlatformIO)
- **RTOS**: FreeRTOS (included with Arduino-ESP32 core)
- **Build system**: PlatformIO (`platformio.ini`)
- **Language**: C++17
- **Constraints**: Limited RAM (~520 KB SRAM), no OS-level memory safety, real-time requirements

### Project-specific rules

- Application: micro-guitar pickup winder. Lateral wire carriage movement plus spindle motor.
- Core goal: maintain constant turns-per-mm during pass movement and reversals.
- Algorithm: dynamic spindle speed compensation based on current carriage velocity (`current_lat_speed_mm_s / nominal_lat_speed_mm_s`).
- Control architecture: responsibility chain `WebInterface` / `CommandController` / `SessionController` / `WinderApp`.
- Telemetry expectation: JSON via WebSocket (`/ws`); includes command `auto_calib` and calibration feedback `calib*`.
- Winding geometry authority: `WindingGeometry` (symmetric margins), `turnsPerPassCalc`, `turnsPerPassOffset`, and `effectiveWidth()`.
- Traverse pattern engine: `WindingPatternPlanner` (straight/scatter/human). Supports floating `turnsPerPass` values.

---

## 1. Project Structure

```
project/
├── .github/
│   └── copilot-instructions.md
├── include/                  # Public headers (.hpp)
│   ├── config/
│   │   ├── PinConfig.hpp     # All pin definitions
│   │   └── AppConfig.hpp     # App-level constants
│   ├── drivers/              # Hardware abstraction headers
│   ├── sensors/
│   ├── communication/
│   └── tasks/                # FreeRTOS task declarations
├── src/                      # Implementation files (.cpp)
│   ├── main.cpp              # setup() + loop() — keep minimal
│   ├── drivers/
│   ├── sensors/
│   ├── communication/
│   └── tasks/                # FreeRTOS task definitions
├── lib/                      # Private project libraries
├── test/                     # Unit tests (Unity via PlatformIO)
├── docs/
└── platformio.ini
```

- **One class per file.** Filename matches class name exactly (`TemperatureSensor.hpp` / `TemperatureSensor.cpp`).
- `setup()` initializes peripherals and creates FreeRTOS tasks — nothing more.
- `loop()` should be **empty** or contain only a `vTaskDelay`. All logic lives in tasks.
- Group by **hardware domain** (sensors, drivers, comms), never by type (no generic `utils/` dumping ground).
- All source code, comments, identifiers, and documentation must be written in **English**.
- All functions and classes must be documented with clear comments (purpose/inputs/outputs/behavior).  All non-trivial logic must have explanatory comments or design notes.

---

## 2. platformio.ini

Always use this baseline configuration:

```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags =
    -std=gnu++17
    -DCORE_DEBUG_LEVEL=3       ; 0=None 1=Error 2=Warn 3=Info 4=Debug 5=Verbose
lib_deps =
    ; list dependencies here
```

- Never hardcode board variants (e.g., `esp32s3`) without updating `board` in `platformio.ini`.
- `CORE_DEBUG_LEVEL` controls `log_e/w/i/d` verbosity — set to `0` for production builds.

---

## 3. Naming Conventions

| Element              | Convention         | Example                        |
|----------------------|--------------------|--------------------------------|
| Classes / Structs    | `PascalCase`       | `TemperatureSensor`            |
| Functions / Methods  | `camelCase`        | `readTemperature()`            |
| Variables            | `camelCase`        | `lastReadingMs`                |
| Constants            | `UPPER_SNAKE_CASE` | `MAX_RETRY_COUNT`              |
| Member variables     | `_` prefix         | `_sensorPin`                   |
| Global variables     | `g_` prefix        | `g_dataQueue`                  |
| Files                | `PascalCase`       | `UartDriver.cpp`               |
| FreeRTOS task funcs  | `task` prefix      | `taskSensorRead`               |
| FreeRTOS task names  | `snake_case`       | `"sensor_read"` (string arg)   |

- **No abbreviations** unless universally known in embedded context (`i2c`, `uart`, `gpio`, `isr`, `pwm`).
- **Member variable prefix:** This repository uses a single leading underscore for private member variables (e.g., `_stepper`, `_homeOffsetMm`). Prefer `_` for non-public data members to match existing code and minimize churn from large refactors.

- **Unit suffixes:** Use CamelCase unit suffixes in identifiers (e.g., `Mm` for millimetres). Examples: `windingStartMm()`, `totalWidthMm`, `getCurrentPositionMm()`. Avoid snake-case suffixes like `_mm` and keep unit notation consistent across APIs and members.

- **Booleans:** Public boolean getters should use `is`/`has`/`can` (e.g., `isHomed()`). Private boolean flags may keep the `_` prefix and can be named without `is` if clearly documented (e.g., `_freerun`, `_burstActive`). Prefer `is`/`has` on public APIs for clarity.

---

## 4. C++ Usage Rules

### ✅ Prefer

- **Stack allocation** over heap when size is known at compile time.
- `constexpr` and `const` everywhere possible — never `#define` for values.
- `std::array<T, N>` over raw C arrays.
- `std::optional<T>` for nullable return values instead of sentinel values (`-1`, `nullptr`).
- Scoped enums: `enum class PinState : uint8_t { Low, High };`
- RAII for resource management (mutexes, handles).
- `static_assert` to validate hardware assumptions at compile time.

### ❌ Avoid

- **`new` / `delete`** — heap fragmentation is fatal on embedded. Use static or stack allocation.
- **`String` (Arduino)** — it fragments heap aggressively. Use `const char*`, `char[]`, or `std::string_view`.
- **`std::string`** — same reason. Use fixed-size char buffers.
- **`std::vector` / `std::map`** — dynamic allocation; use `std::array` or fixed-size structures.
- **Exceptions** — disabled by default in Arduino-ESP32 (`-fno-exceptions`).
- **RTTI** — disabled by default (`-fno-rtti`).
- **Virtual functions in hot paths** — overhead in ISRs and tight loops.
- **`delay()`** — blocks the calling task; use `vTaskDelay(pdMS_TO_TICKS(ms))` instead.
- **`millis()` for task timing** — use `vTaskDelayUntil()` for precise periodic tasks.

---

## 5. setup() and loop()

```cpp
// ✅ Correct pattern
void setup() {
    Serial.begin(115200);
    initHardware();     // configure pins, peripherals
    createTasks();      // launch all FreeRTOS tasks
}

void loop() {
    vTaskDelay(portMAX_DELAY); // surrender CPU — all logic is in tasks
}
```

- **Never put application logic in `loop()`.**
- All blocking waits use `vTaskDelay` or `vTaskDelayUntil`, never `delay()`.

---

## 6. FreeRTOS Tasks

### Task declaration (`include/tasks/TaskSensorRead.hpp`)

```cpp
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Tasks {
    void taskSensorRead(void* pvParameters);

    constexpr uint32_t    SENSOR_TASK_STACK = 4096;
    constexpr UBaseType_t SENSOR_TASK_PRIO  = 5;
}
```

### Task definition (`src/tasks/TaskSensorRead.cpp`)

```cpp
#include "tasks/TaskSensorRead.hpp"

void Tasks::taskSensorRead(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    constexpr TickType_t xPeriod = pdMS_TO_TICKS(100); // 10 Hz

    for (;;) {
        // Do work here
        vTaskDelayUntil(&xLastWakeTime, xPeriod); // precise periodic execution
    }
}
```

### Creating tasks (`src/main.cpp`)

```cpp
xTaskCreatePinnedToCore(
    Tasks::taskSensorRead,    // function
    "sensor_read",            // name (visible in debugger)
    Tasks::SENSOR_TASK_STACK, // stack size in bytes
    nullptr,                  // parameters
    Tasks::SENSOR_TASK_PRIO,  // priority (higher = more urgent)
    nullptr,                  // task handle (store it to suspend/delete later)
    1                         // core: 0 = protocol core, 1 = application core
);
```

- **Always use `xTaskCreatePinnedToCore`** — never `xTaskCreate` on ESP32.
- Core 0 runs the WiFi/BT stack — prefer **Core 1** for application tasks.
- Always define stack size and priority as `constexpr` — never inline magic numbers.
- Tasks must **never return** — they loop forever or call `vTaskDelete(nullptr)` at the end.

---

## 7. Inter-Task Communication

- Use **queues** for passing data between tasks — never raw shared globals without protection.
- Use **mutexes** (`SemaphoreHandle_t`) to protect shared resources accessed by multiple tasks.
- Use **semaphores** to signal events between tasks or from ISRs.

```cpp
// Shared header
extern QueueHandle_t g_sensorDataQueue;

// Producer task
SensorData data = { .temperature = 23.5f, .timestamp = millis() };
xQueueSend(g_sensorDataQueue, &data, pdMS_TO_TICKS(10));

// Consumer task
SensorData received;
if (xQueueReceive(g_sensorDataQueue, &received, portMAX_DELAY) == pdTRUE) {
    // process data
}
```

- Create queues in `setup()` **before** creating tasks.
- Queue item types must be **plain data structs** (POD) — no pointers to stack-allocated objects.

---

## 8. ISR (Interrupt Service Routines)

- Mark ISR functions with `IRAM_ATTR` to place them in fast IRAM:

```cpp
void IRAM_ATTR onButtonPress(void* arg);
```

- Keep ISRs **minimal**: post to a queue or give a semaphore, handle all logic in a task.
- Use the `FromISR` variants of FreeRTOS API:

```cpp
void IRAM_ATTR onButtonPress(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_buttonSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

- **Never** use `Serial.print`, `delay`, `malloc`, or any blocking call inside an ISR.

---

## 9. Hardware & Peripheral Access

- **Never access hardware directly** outside of a dedicated driver class.
- Each peripheral gets its own driver class in `src/drivers/`.
- Pin numbers are always **named constants** in `include/config/PinConfig.hpp`:

```cpp
// ✅ Good — PinConfig.hpp
constexpr uint8_t PIN_LED_STATUS  = 2;
constexpr uint8_t PIN_BUTTON_BOOT = 0;
constexpr uint8_t PIN_I2C_SDA     = 21;
constexpr uint8_t PIN_I2C_SCL     = 22;

// ❌ Bad — magic numbers inline
digitalWrite(2, HIGH);
```

- Always check return values from peripheral operations (`Wire`, `SPI`, `Serial`).

---

## 10. Logging

Use the Arduino-ESP32 structured logging macros — not `Serial.print` directly:

```cpp
log_e("Sensor init failed: %d", errorCode);       // Error
log_w("Retry %d of %d", attempt, MAX_RETRIES);    // Warning
log_i("Sensor ready, temp: %.2f C", temperature); // Info
log_d("Raw ADC: %d", adcRaw);                     // Debug (stripped at CORE_DEBUG_LEVEL < 4)
```

- Verbosity is controlled by `CORE_DEBUG_LEVEL` in `platformio.ini` — no code changes needed.
- **Do not use `Serial.print` for application logging** — use `log_x` macros exclusively.
- At startup, always log free heap:

```cpp
log_i("Boot complete. Free heap: %u bytes", ESP.getFreeHeap());
```

---

## 11. Memory Management

- Prefer **static and stack allocation** — avoid `new`/`malloc` in application code.
- Declare large buffers as `static` class members or file-scope variables — never on the task stack.
- Use `xTaskCreateStaticPinnedToCore` in production for fully deterministic task memory.
- Log `ESP.getFreeHeap()` at startup and after task creation to track consumption.
- Never assume allocation succeeds — always null-check.

---

## 12. Configuration & Constants

- All pin definitions → `include/config/PinConfig.hpp`
- All app-level constants (timeouts, thresholds, buffer sizes) → `include/config/AppConfig.hpp`
- Use `constexpr` for all compile-time constants — never `#define` for values:

```cpp
// ✅ Good
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

// ❌ Bad
#define WIFI_CONNECT_TIMEOUT_MS 10000
```

- Feature flags are defined via `build_flags` in `platformio.ini`, with a comment in the header:

```cpp
#ifdef FEATURE_OTA_ENABLED
// OTA update support — enable with -DFEATURE_OTA_ENABLED in platformio.ini build_flags
#endif
```

---

## Quick Reference Checklist

Before suggesting or generating code, verify:

- [ ] No `String` (Arduino), `std::string`, `std::vector`, or dynamic allocation
- [ ] No `delay()` — use `vTaskDelay(pdMS_TO_TICKS(ms))`
- [ ] `loop()` is empty — all logic lives in FreeRTOS tasks
- [ ] Tasks created with `xTaskCreatePinnedToCore`, never `xTaskCreate`
- [ ] Task stack and priority defined as `constexpr`, not inline literals
- [ ] Application tasks pinned to **Core 1**
- [ ] Inter-task data flows through queues or semaphores, not shared globals
- [ ] ISR functions marked `IRAM_ATTR` and use `FromISR` FreeRTOS variants
- [ ] Pin numbers are named constants in `PinConfig.hpp`, never magic numbers
- [ ] Logging uses `log_e/w/i/d` macros, not `Serial.print`
- [ ] One class per file, filename matches class name exactly