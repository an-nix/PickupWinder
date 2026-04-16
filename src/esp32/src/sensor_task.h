/* sensor_task.h — Sensor acquisition FreeRTOS task (Core 0).
 *
 * Responsibilities:
 *   HX711 load cells   — non-blocking poll at ~80 Hz, 2 sensors
 *   Potentiometer ADC  — filtered reading at ~50 Hz (GPIO 36)
 *   Manual encoder     — PCNT hardware count at 1 kHz (GPIO 1 / 3)
 *
 * Note: GPIO1/3 are the UART0 TX/RX pins. They may be used for the manual
 * encoder, but driving the encoder during flashing or when the serial
 * bootloader is active can interfere with programming. Keep encoder idle
 * during flashing if you keep the console enabled.
 *
 * All readings are published to g_sensor, protected by a portMUX spinlock
 * so that stepper_task (Core 1) can safely read them when building the
 * StatusFrame.
 *
 * The Pi receives tension, pot, and encoder values via the 44-byte StatusFrame.
 * The Pi is responsible for PID control and for sending setpoint commands back.
 */

#pragma once

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

// ── Shared sensor state ──────────────────────────────────────────────────────

struct SensorState {
    volatile int16_t tension_dg[2];    // HX711 readings in 0.1 g
    volatile int16_t tension_setpoint; // Setpoint echoed from SET_TENSION command
    volatile int16_t pot_raw;          // Potentiometer ADC filtered value (0–4095)
    volatile int16_t encoder_manual;   // Manual encoder PCNT count (int16, wrapping)
    portMUX_TYPE     mux;              // Spinlock: Core 0 write / Core 1 read
};

extern SensorState g_sensor;

// ── Lifecycle ────────────────────────────────────────────────────────────────

/// Initialize sensor hardware (HX711 + pot + encoder).
/// Must be called from app_main() before sensor_task_start().
void sensor_task_init();

/// Start sensor acquisition FreeRTOS task on Core 0 (priority 5).
void sensor_task_start();

// ── Command callbacks (called from stepper_task on SET_TENSION / TARE_HX711) ─

/// Set tension setpoint echoed in StatusFrame.  Thread-safe.
void sensor_set_tension_setpoint(int16_t dg);

/// Request a tare operation on a sensor (0 or 1).
/// Processed asynchronously by the sensor_task on the next tick.
/// Thread-safe: sensor_id is written atomically (uint8_t).
void sensor_request_tare(uint8_t sensor_id);
