Stepper queue / RMT implementation

Overview

- The active pulse-output path in this branch is the custom queue + RMT stack:
  - `src/esp32/src/stepper_queue.h`
  - `src/esp32/src/stepper_queue.cpp`
  - `src/esp32/src/stepper_driver.h`
  - `src/esp32/src/stepper_driver.cpp`

- `resources/FastAccelStepper` is kept in the repository as the behavioral
  reference used for the ESP32 RMT implementation.

Design

- `StepperQueue` receives `step_block_t` packets from the host-facing layer.
- A dedicated executor task on Core 1 drains all pending blocks into the
  driver's software ring buffer.
- `StepperDriver` owns the RMT channel, the DIR/EN pins, and the
  `simple_encoder` callback.
- The callback is the only code that converts queued timing entries into
  physical `rmt_symbol_word_t` step pulses.

FastAccelStepper-aligned runtime rules:

- one `rmt_transmit()` per continuous run,
- `simple_encoder` refills in `PART_SIZE` chunks,
- `trans_queue_depth = 1`,
- one balanced 50/50 HIGH/LOW RMT symbol per normal step,
- fixed LOW-level pause chunk before a DIR toggle if the previous chunk still
  contained steps,
- one LOW-level pause chunk plus stop on starvation,
- no task-side busy-spin while waiting for ring space.

Lifecycle

1. Host transport enqueues `step_block_t` packets into `StepperQueue`.
2. The executor task drains the block queue into the driver's software ring.
3. If idle, `StepperDriver` starts one RMT transmission.
4. The RMT `simple_encoder` callback streams `PART_SIZE` symbols per refill.
5. On starvation, one LOW-level pause chunk is emitted and the transaction
  ends on the next callback.

Notes

- This document covers the queue/RMT pulse-output path only.
- Motion planning and winding geometry remain host-driven.
- The custom implementation does not embed the FastAccelStepper library, but
  its queue/RMT behavior is intentionally matched to the ESP32 IDF5 backend in
  `resources/FastAccelStepper`.
