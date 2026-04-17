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

/** Number of compressed motion segments per transport block. */
#define SEGMENT_BLOCK_SIZE      60

/** Buffered step target before starting/restarting the RMT stream. */
#define STEP_STREAM_START_FILL  512U

/**
 * Depth of the FreeRTOS step-block queue (per motor).
 * Increased to 24 to handle slower host SPI latency and maintain RMT fill rate.
 * Provides ~(STEPPER_QUEUE_DEPTH × STEP_BLOCK_SIZE) steps of look-ahead.
 */
#define STEPPER_QUEUE_DEPTH     24

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
 * @brief A compressed motion segment.
 *
 * Represents `step_count` successive steps where the interval evolves as:
 *
 *   ticks[n] = start_ticks + n * add_ticks
 *
 * This is the same basic representation used by Klipper-style trapezoid
 * segments: the host sends a compact arithmetic description, the MCU expands
 * it locally into concrete step timings.
 */
typedef struct {
    uint16_t step_count;   /**< Number of steps encoded by the segment          */
    uint16_t start_ticks;  /**< Interval for the first step in RMT ticks        */
    int16_t  add_ticks;    /**< Delta applied after each emitted step           */
    uint8_t  direction;    /**< true = forward / CW, false = reverse / CCW      */
    uint8_t  reserved;     /**< Padding / future flags                          */
} motion_segment_t;

/**
 * @brief A transport block of compressed motion segments.
 */
typedef struct {
    motion_segment_t segments[SEGMENT_BLOCK_SIZE];
    uint32_t         count;   /**< Number of valid segments[] entries             */
} segment_block_t;

typedef enum {
    MOTION_BLOCK_KIND_STEP = 0,
    MOTION_BLOCK_KIND_SEGMENT = 1,
} motion_block_kind_t;

/**
 * @brief Queue item exchanged between the comm task and the executor task.
 *
 * The comm task can enqueue either legacy per-step blocks or compressed
 * segment blocks. The executor expands segment blocks into `step_block_t`
 * chunks locally before feeding the driver ring.
 */
typedef struct {
    uint8_t kind;
    uint8_t reserved[3];
    union {
        step_block_t    step;
        segment_block_t segment;
    } payload;
} motion_block_t;

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

// ---------------------------------------------------------------------------
// Multi-axis synchronised segment block (MULTI_AXIS_SEGMENT_BLOCK = 0x13)
// ---------------------------------------------------------------------------

/** Maximum number of axes in a multi-axis segment block. */
#define MULTI_AXIS_MAX_AXES  4

/** Maximum number of segments per multi-axis block. */
#define MULTI_AXIS_BLOCK_SIZE  60

/**
 * @brief One synchronised multi-axis time-based segment.
 *
 * The host sends one of these records per segment.  All axes execute their
 * respective step_counts over the shared duration_us, with steps distributed
 * evenly in time by the MCU.  direction_mask bit i = 1 means axis i reverses.
 */
typedef struct {
    uint16_t motion_sequence; /**< Globally increasing motion identifier       */
    uint16_t duration_us;     /**< Wall-clock duration of this segment in µs   */
    uint16_t direction_mask;  /**< Bit i=1: axis i runs in reverse direction   */
    uint16_t step_counts[MULTI_AXIS_MAX_AXES]; /**< Steps per axis             */
} multi_axis_segment_t;

/**
 * @brief Block of synchronised multi-axis segments received from the host.
 *
 * Enqueued into the global multi-axis queue and consumed by the executor.
 */
typedef struct {
    uint8_t              axis_ids[MULTI_AXIS_MAX_AXES]; /**< Logical axis IDs  */
    uint8_t              axis_count;                    /**< Valid entries      */
    uint8_t              segment_count;                 /**< Valid segments     */
    multi_axis_segment_t segments[MULTI_AXIS_BLOCK_SIZE];
} multi_axis_block_t;

/**
 * @brief Flush request: discard all segments with motion_sequence > threshold.
 */
typedef struct {
    uint16_t flush_sequence; /**< Keep segments ≤ this; discard the rest       */
} flush_request_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
