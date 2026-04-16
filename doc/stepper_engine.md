# Stepper queue / RMT implementation

## Active stack

- `src/esp32/src/messages.h`
- `src/esp32/src/comm_interface.h`
- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/step_types.h`
- `src/esp32/src/stepper_queue.h`
- `src/esp32/src/stepper_queue.cpp`
- `src/esp32/src/stepper_driver.h`
- `src/esp32/src/stepper_driver.cpp`
- `src/rpi/messages.py`
- `src/rpi/ramp.py`
- `src/rpi/spi_transport.py`
- `src/rpi/streamer.py`
- `src/rpi/demo_spi.py`

## Design (current)

- Host sends **compressed motion** through `SEGMENT_BLOCK`.
- `CommInterface` validates CRC16 and enqueues `motion_block_t`.
- `StepperQueue` executor expands segments into concrete `step_block_t` chunks.
- Expanded steps are pushed into `StepperDriver` software ring.
- RMT `simple_encoder` callback is the only layer emitting STEP symbols.

## Runtime rules

- one `rmt_transmit()` per continuous run,
- `simple_encoder` refill in `PART_SIZE` chunks,
- `trans_queue_depth = 1`,
- balanced 50/50 HIGH/LOW symbol for normal steps,
- fixed LOW pause before DIR toggle when required,
- one LOW pause + stop on starvation,
- no task-side busy-spin while waiting for ring space.

## Execution lifecycle

1. Host planner generates arithmetic segments (`step_count`, `start_ticks`, `add_ticks`, dir).
2. Host streamer sends `SEGMENT_BLOCK` messages over SPI.
3. ESP32 Core0 task validates and enqueues motion blocks.
4. ESP32 Core1 task expands segments and fills driver ring.
5. Driver starts/maintains stream when ring fill threshold is reached.
6. RMT ISR emits step symbols until ring drains.

## Notes

- Legacy explicit `STEP_BLOCK` is still accepted for debug.
- Production path should use `SEGMENT_BLOCK`.
- `resources/FastAccelStepper` remains reference-only; library is not linked.
