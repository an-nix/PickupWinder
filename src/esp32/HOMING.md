# Lateral homing contract

This firmware keeps trajectory generation on the host and keeps the ESP32 as a deterministic executor.
The homing flow must therefore be split cleanly between host orchestration and firmware enforcement.

## Design goals

- stop on a definite endstop hit with bounded latency
- never resume old queued motion after a hit
- allow guarded backoff without requiring a temporary unguarded phase
- tolerate short NO/NC crossover glitches during contact changeover
- surface persistent sensor faults as a clear failure condition

## Firmware responsibilities

The firmware now implements these rules:

1. `ENABLE_ENDSTOP(arm=1)` clears the previous homing latch.
2. A valid CLOSED sample (`NO=0`, `NC=1`) while armed latches a hit immediately.
3. The executor posts an internal `FLUSH` request on endstop recovery paths so stale queued motion is discarded.
4. ~~After a hit, only the latched clearance direction is accepted while the switch remains closed.~~ **Removed** — direction gating on `CLOSED` was removed to align with host-driven sequencing (see `stepper_driver.cpp` `isEndstopMoveAllowed`). Only `ABSENT` now vetoes movement when armed.
5. Short INVALID (`NO==NC`) crossover windows are masked using the last stable state.
6. Persistent INVALID state is reported as `ABSENT` and must be treated as a homing fault.

## Host responsibilities

The host must use the following sequence for the lateral axis.

### Phase 1: fast seek

1. Enable the lateral driver.
2. Send `ENABLE_ENDSTOP(axis=1, arm=1)`.
3. Send fast seek motion toward the switch.
4. Poll `STATUS` until either:
   - `endstop_hit_mask & (1 << 1)` becomes non-zero, or
   - `lateral_endstop_state == PRESENT_CLOSED`.
5. Stop enqueueing more forward motion immediately.
6. Wait until the MCU has drained the hit and stopped the axis.

### Phase 2: guarded backoff

1. Keep the endstop armed.
2. Send short reverse blocks away from the switch.
3. Continue until `lateral_endstop_state == PRESENT_OPEN`.
4. If `lateral_endstop_state == ABSENT`, abort homing and raise a fault.

### Phase 3: slow seek

1. Send `ENABLE_ENDSTOP(axis=1, arm=1)` again.
2. Send slow seek motion toward the switch.
3. Wait for the next hit using the same status conditions as phase 1.
4. Stop enqueueing forward motion.

### Phase 4: final settle

Choose one of these strategies:

- keep the carriage on the switch if that is the mechanical zero, or
- send a small guarded reverse offset to land on the desired final zero, then
  `ENABLE_ENDSTOP(axis=1, arm=0)` if subsequent motion should be unguarded.

## Status fields to monitor

The host should treat these fields as authoritative during homing:

- `lateral_endstop_state`
- `endstop_armed_mask`
- `endstop_hit_mask`
- `running_mask`
- `last_result`
- `last_executed_sequence`

## Notes

- The firmware does not implement a full autonomous homing state machine.
  It implements the protection, latching, and queue recovery needed for a
  reliable host-driven homing sequence.
- The host should keep `block_seq` and `motion_sequence` monotonic across the
  full homing cycle.
