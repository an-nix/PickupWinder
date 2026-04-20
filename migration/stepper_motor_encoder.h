/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stepper motor curve encoder configuration
 */
typedef struct {
    uint32_t resolution;    // Encoder resolution, in Hz
    uint32_t sample_points; // Sample points used for deceleration phase. Note: |end_freq_hz - start_freq_hz| >= sample_points
    uint32_t start_freq_hz; // Start frequency on the curve, in Hz
    uint32_t end_freq_hz;   // End frequency on the curve, in Hz
} stepper_motor_curve_encoder_config_t;

/**
 * @brief Stepper motor uniform encoder configuration
 */
typedef struct {
    uint32_t resolution; // Encoder resolution, in Hz
} stepper_motor_uniform_encoder_config_t;

/**
 * @brief Stepper motor uniform batch encoder configuration
 * Generates N symbols of the same frequency without relying on loop_count
 */
typedef struct {
    uint32_t resolution;      // Encoder resolution, in Hz
    uint32_t symbol_count;    // Number of symbols to generate
    uint32_t target_freq_hz;  // Target frequency for all symbols
} stepper_motor_uniform_batch_encoder_config_t;

esp_err_t rmt_new_stepper_motor_curve_encoder(const stepper_motor_curve_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

esp_err_t rmt_new_stepper_motor_uniform_encoder(const stepper_motor_uniform_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

esp_err_t rmt_new_stepper_motor_uniform_batch_encoder(const stepper_motor_uniform_batch_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
