/**
 * @file step_types.h
 * @brief Shared data types for the Klipper-style stepper architecture.
 *
 * The host (Linux / Raspberry Pi) pre-computes all trajectories and sends
 * blocks of pre-timed step commands.  The ESP32 is a pure executor: it has
 * no knowledge of acceleration, kinematics, or wire geometry.
 *
 * ── RMT streaming architecture (FastAccelStepper-style) ────────────────────
 *   A simple_encoder callback streams step pulses from a lock-free ring buffer.
 *   The RMT hardware calls the encoder on-demand (ISR context), filling
 *   PART_SIZE symbols at a time in ping-pong fashion.  This eliminates the
 *   inter-block gaps that caused step loss with the old copy_encoder approach.
 *
 * ── RMT resolution analysis ────────────────────────────────────────────────
 *   Resolution : 2 MHz  (1 tick = 0.5 µs)
 *   Step shape : one RMT symbol per step, split 50/50 HIGH/LOW
 *                (minimum high/low still comfortably above DRV8825 limits)
 *
 *   160 kHz max : interval = 6.25 µs  →  12 ticks  (actual 166.7 kHz, <4% err)
 *   100 Hz  min : interval = 10 000 µs → 20 000 ticks  ≤  32 767 max ✓
 *    61 Hz  abs : interval = 16 384 µs → 32 767 ticks  (RMT 15-bit limit)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// RMT timing constants
// ---------------------------------------------------------------------------

/** RMT TX channel resolution: 2 MHz  (1 tick = 0.5 µs) */
#define RMT_STEP_RESOLUTION_HZ  2000000UL

/** Minimum half-period guard in RMT ticks (2 µs — meets DRV8825 1.9 µs min) */
#define RMT_STEP_PULSE_TICKS    4U

/**
 * Minimum total interval in ticks (6 µs → ~166 kHz).
 * Ensures duration1 = interval_ticks - PULSE_TICKS ≥ 2 (never 0 or negative).
 */
#define RMT_STEP_MIN_TICKS      12U

/** Maximum interval in ticks: RMT 15-bit field limit → ~61 Hz floor */
#define RMT_STEP_MAX_TICKS      32767U

/** Default hold interval before any step has been consumed (= minimum interval).
 *  Prevents duration1 wraparound to ~65535 ticks on first ring-empty hold. */
#define RMT_STEP_DEFAULT_TICKS  RMT_STEP_MIN_TICKS

// ---------------------------------------------------------------------------
// RMT streaming constants (FastAccelStepper-style ping-pong)
// ---------------------------------------------------------------------------

/** Symbols per ping-pong half-buffer.  Must divide RMT_MEM_SYMBOLS evenly. */
#define PART_SIZE               32U

/** Total RMT hardware memory per channel (2 × PART_SIZE for ping-pong). */
#define RMT_MEM_SYMBOLS         (2U * PART_SIZE)

/**
 * Minimum command duration in ticks (200 µs at 2 MHz = 400 ticks).
 * Used as pause filler when the ring buffer empties or on direction changes.
 */
#define MIN_CMD_TICKS           400U

// ---------------------------------------------------------------------------
// Ring buffer sizing
// ---------------------------------------------------------------------------

/** Ring buffer size — must be a power of 2.
 *  2048 entries = 38.6ms at 53kHz (500 RPM) / 19.3ms at 106kHz (1000 RPM).
 *  Sized so that even a 10ms FreeRTOS scheduling gap cannot drain the buffer. */
#define STEP_RING_SIZE          2048U

/** Bit mask for ring buffer index wrap-around. */
#define STEP_RING_MASK          (STEP_RING_SIZE - 1U)

// ---------------------------------------------------------------------------
// Block / queue sizing
// ---------------------------------------------------------------------------

/** Number of step commands per block. */
#define STEP_BLOCK_SIZE         64

/**
 * Depth of the FreeRTOS step-block queue (per motor).
 * Provides ~(STEPPER_QUEUE_DEPTH × STEP_BLOCK_SIZE) steps of look-ahead.
 */
#define STEPPER_QUEUE_DEPTH     16

/**
 * Flow-control threshold: the comm layer sends a NACK / buffer-full warning
 * to the host when the number of free queue slots drops below this value.
 */
#define FLOW_CONTROL_THRESHOLD  4

/** Microstep denominator — override with -DMICROSTEPS=N in build flags. */
#ifndef MICROSTEPS
#define MICROSTEPS 32
#endif

// ---------------------------------------------------------------------------
// Core data types
// ---------------------------------------------------------------------------

/**
 * @brief A single pre-timed step command.
 *
 * @note  interval_ticks == 0  →  stop marker (end of move).
 * @note  All steps within one block MUST share the same direction.
 *        The host is responsible for splitting trajectories at direction
 *        reversals; crossing a reversal within a block is undefined behaviour.
 */
typedef struct {
    uint32_t interval_ticks; /**< RMT ticks until the NEXT step (0 = stop)          */
    bool     direction;      /**< true = forward / CW, false = reverse / CCW         */
} step_cmd_t;

/**
 * @brief A pre-computed block of step commands.
 *
 * The host fills steps[0..count-1] and sets count.  The executor iterates
 * only up to count; remaining entries are ignored.
 */
typedef struct {
    step_cmd_t steps[STEP_BLOCK_SIZE]; /**< Pre-timed step commands              */
    uint32_t   count;                  /**< Number of valid entries in steps[]   */
} step_block_t;

/**
 * @brief Ring buffer entry consumed by the RMT encoder callback (ISR context).
 *
 * Produced by the executor task, consumed by the simple_encoder callback.
 * SPSC: one writer (task), one reader (ISR).
 */
typedef struct {
    uint16_t ticks;      /**< Total step period in RMT ticks (2 MHz).  0 = invalid. */
    uint8_t  toggle_dir; /**< 1 = toggle DIR pin before this step.                   */
    uint8_t  pad;        /**< Padding for 4-byte alignment.                          */
} ring_entry_t;

/**
 * @brief Wire packet received from the host over UART/SPI.
 *
 * The comm layer deserialises this from the byte stream and routes it to
 * the appropriate motor queue based on motor_id.
 */
typedef struct {
    uint8_t    motor_id;               /**< Target motor: 0 or 1                 */
    uint8_t    block_seq;              /**< Rolling sequence number (loss detect) */
    uint32_t   count;                  /**< Number of valid steps in payload      */
    step_cmd_t steps[STEP_BLOCK_SIZE]; /**< Step payload                          */
} comm_packet_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
