/**
 * @file demo_local.h
 * @brief Local step-sequence generator — simulates the Linux host.
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  THIS FILE SIMULATES THE LINUX HOST.                                    ║
 * ║  In production, REMOVE or DISABLE this file and replace it with the     ║
 * ║  real communication task that receives pre-computed step_block_t arrays  ║
 * ║  from the Raspberry Pi over UART / SPI.                                 ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * The demo generates a full trapezoidal velocity profile for two independent
 * motors (motor_a and motor_b) without using any on-ESP32 kinematics:
 *
 *   motor_a: 0 → 1000 RPM (accel) → hold 3 s (cruise) → 0 (decel)
 *   motor_b: 0 →  600 RPM (accel) → hold 3 s (cruise) → 0 (decel)
 *
 * The profile is discretised into step_cmd_t records (one per physical step)
 * and packed into STEP_BLOCK_SIZE-sized step_block_t that are enqueued into
 * the respective StepperQueue.  Block generation runs on Core 0 at a moderate
 * priority, safely decoupled from the Core 1 executor tasks.
 */

#pragma once

#include <esp_err.h>
#include "stepper_queue.h"

/**
 * @brief Launch the demo task.
 *
 * Spawns a FreeRTOS task that generates and enqueues trapezoidal velocity
 * profiles for both motors, then logs instantaneous frequencies every 100 ms.
 *
 * @param queue_a  StepperQueue for motor A (initialised).
 * @param queue_b  StepperQueue for motor B (initialised).
 */
esp_err_t demo_local_start(StepperQueue* queue_a, StepperQueue* queue_b);
