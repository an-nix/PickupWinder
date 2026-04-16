#pragma once

/*
 * hardware_config.h — Compile-time hardware pin configuration
 *
 * Centralized definitions for GPIO pin assignments and other hardware-specific
 * constants. Included by multiple modules to ensure consistent configuration.
 *
 * This file should contain only compile-time constants and simple structs.
 * Avoid including complex logic or dependencies here to keep it lightweight.
 */

 /* Pins */
#define STEP_GPIO      GPIO_NUM_18
#define DIR_GPIO       GPIO_NUM_19
#define ENABLE_GPIO    GPIO_NUM_21  // nouvelle pin ENABLE (adapter selon ton câblage)