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
 *   Resolution : 40 MHz  (1 tick = 25 ns)
 *   Step shape : one RMT symbol per step; HIGH = PULSE_TICKS (4), LOW = remainder
 *
 *   5 MHz max  : interval =   8 ticks   (200 ns period) — hardware ceiling
 *   160 kHz    : interval = 250 ticks   (6.25 µs)
 *   100 Hz min : interval = 400 000 ticks  → use RMT_STEP_MAX_TICKS (65535) in practice
 *    15 Hz abs : interval =  65535 ticks = 1.638 ms  (16-bit RMT field limit)
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

/** RMT TX channel resolution: 80 MHz  (1 tick = 12.5 ns) */
#define RMT_STEP_RESOLUTION_HZ  80000000UL

/** Ticks per microsecond derived from RMT_STEP_RESOLUTION_HZ (80 at 80 MHz). */
#define RMT_TICKS_PER_US        (RMT_STEP_RESOLUTION_HZ / 1000000UL)

/** Minimum half-period guard in RMT ticks (100 ns — 8 × 12.5 ns at 80 MHz;
 *  meets DRV8825/A4988 STEP pulse width minimum of 1 µs when low half is added). */
#define RMT_STEP_PULSE_TICKS    8U

/**
 * Minimum total interval in ticks.
 * 16 ticks × 12.5 ns = 200 ns → 5 MHz step rate ceiling at hardware level.
 * Ensures duration1 = interval_ticks − PULSE_TICKS ≥ 8 ticks.
 */
#define RMT_STEP_MIN_TICKS      16U

/** Maximum interval in ticks: 16-bit RMT field → 65535 ticks = ~819 µs → ~1.2 kHz floor */
#define RMT_STEP_MAX_TICKS      0xFFFFU

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
 *  4096 entries = 58ms at 70kHz (cruise speed, ~660 RPM).
 *  Must cover the worst-case host re-fill time: 25 segments × 1ms SPI = 25ms,
 *  plus OS jitter.  4096 gives 58ms >> 25ms, preventing ring starvation when
 *  all deferred notifications fire simultaneously. */
#define STEP_RING_SIZE          4096U

/** Bit mask for ring buffer index wrap-around. */
#define STEP_RING_MASK          (STEP_RING_SIZE - 1U)

// ---------------------------------------------------------------------------
// Block / queue sizing
// ---------------------------------------------------------------------------

/** Number of step commands per block. */
#define STEP_BLOCK_SIZE         64

/** Number of compressed motion segments per transport block. */
#define SEGMENT_BLOCK_SIZE      60

/** Buffered step target before starting/restarting the RMT stream.
 *  64 = 2 × PART_SIZE: the minimum safe value for the static_assert, and
 *  low enough that the auto-start path fires during low-speed acceleration
 *  (where segments may contain only 2-64 steps each).
 *  The primary start path at low speed is the explicit kickStart() called
 *  by multiAxisExecutorTask after draining each block (see comm_interface.cpp).
 *  Must satisfy: STEP_STREAM_START_FILL >= 2 * PART_SIZE. */
#define STEP_STREAM_START_FILL  64U

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
