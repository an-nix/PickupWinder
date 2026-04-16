StepperEngine (FastAccelStepper) implementation

Overview

- New implementation `StepperEngine` uses the `FastAccelStepper` library to
  generate hardware-driven step pulses. It lives in:
  - `src/esp32/src/stepper_engine.h`
  - `src/esp32/src/stepper_engine.cpp`

- The previous RMT-based driver (`rmt_stepper.*`) is left in the tree as a
  reference and is not removed.

Design

- The engine exposes a per-axis wrapper `StepperAxis` that provides a small
  subset of the API used by higher-level code (position read/write, move to
  absolute position, reset position, emergency stop, speed/accel configuration
  and endstop helpers).

- A single global instance `g_stepper_engine` is provided and used by the
  SPI task and the endstop handler.  The SPI task now snapshots status via
  `g_stepper_engine.get_status()`.

Commands supported

- MOVE_ABS (Opcode 0x02): move axis to absolute position (data = int32 steps)
- MOVE_REL (Opcode 0x03): move axis by relative steps (data = int32 steps)
- RESET_POS (Opcode 0x0B): reset axis position counter to 0
- SET_SPEED (Opcode 0x01): set speed (Hz)
- SET_ACCEL (Opcode 0x08): set acceleration (steps/s^2)
- STOP (Opcode 0x04): controlled stop
- ESTOP (Opcode 0x05): emergency stop (all axes)
- ENABLE (Opcode 0x06): enable/disable driver (data[0] = 1/0)

Notes & limitations

- The `endstop` ISR marks the axis as in a fault/home state and requests an
  emergency stop. The actual force-stop work is performed in task context for
  safety; the ISR also disables the driver pin immediately to reduce risk.

- This initial wrapper implements a minimal, compatible subset of the RMT
  engine API.  It focuses on the "move to position" and "reset position"
  commands requested. Additional features (limits, complex ramping, tightly
  synchronized multi-axis moves) can be added on top of `StepperAxis` if
  needed.

Usage

- Initialization in `app_main()` (changed):

  - `g_stepper_engine.init(AXIS_PINS);`
  - `g_stepper_engine.start(g_cmd_queue);`

- SPI transactions still carry the 8-byte `CmdFrame`; the engine responds to
  the same opcodes and fills the 44-byte `StatusFrame` as before.

Next steps / suggestions

- Add unit tests that exercise `MOVE_ABS` and `RESET_POS` via the mock SPI
  transport (existing test harnesses in `src/rpi/tests/` can be adapted).

- If ISR-to-hardware latency is critical for your use-case, consider
  implementing a faster ISR path (e.g., direct register toggles) for the
  enable/disable path or a small RMT fallback for safety-critical emergency
  stops.
