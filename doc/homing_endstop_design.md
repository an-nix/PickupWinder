# Homing and Endstop Design for PickupWinder

> Status: implementation guidance based on the current production architecture.
> Scope: lateral homing and endstop handling with the SPI transport left unchanged.
> Audience: maintainers of the Raspberry Pi host, ESP32 firmware, and Wendy / JSON-RPC surface.

---

## 1. Executive summary

### Direct answer

Yes: in this project, the endstop should be **explicitly armed and explicitly disarmed** by the host.

This should not be an always-armed system.

The recommended model is:

- arm the endstop only for the homing phases that must stop on contact,
- disarm it for preclear, backoff, and ordinary production motion,
- let the firmware perform the immediate real-time stop when the armed endstop triggers,
- let the host decide what the next homing phase is.

This matches the existing architecture:

- the Raspberry Pi owns motion policy, homing sequencing, retry logic, and session behavior,
- the ESP32 owns SPI slave dispatch, queueing, sensor state publication, and immediate motion stop.

That split is especially important here because the SPI settings must remain unchanged for the normal spindle production path, which already works correctly at 1750 rpm.

---

## 2. Why this document exists

Recent debugging showed two separate realities:

1. **Normal winding / spindle production motion works at the current SPI settings.**
2. **Homing and endstop handling became fragile when they generated too much control traffic and too many status polls around endstop transitions.**

That means the correct response is **not** to redesign the SPI link or to slow the bus globally.

Instead, homing and endstop must be implemented in a way that:

- preserves the proven production path,
- uses the firmware only for the strict real-time safety boundary,
- keeps homing state transitions mostly on the host,
- minimizes extra SPI chatter during homing,
- keeps request confirmation and motion status semantically correct.

---

## 3. Analysis findings

## 3.1 What was observed

During homing and related endstop activity, the host reported frames such as:

- `bad magic: 0x0000`
- `bad magic: 0x0350`
- `bad magic: 0x24F5`
- `bad magic: 0x2DF5`

The host also reported:

- `zero_rx`
- `host/firmware buffer desync suspected`

Two main homing scenarios were observed:

1. the axis moved toward the endstop and the endstop triggered during motion,
2. the axis started with the endstop already closed and had to run a preclear first.

## 3.2 What these errors most likely mean

These errors do **not** point primarily to a wrong endstop policy.

They point to a transport pressure problem triggered by homing behavior:

- `0x0000` strongly suggests an invalid or empty receive window,
- `0x0350` strongly suggests a shifted frame where the first byte was lost,
- the other values are consistent with corrupted or misaligned leading bytes.

In other words, the SPI link is good enough for the steady production motion path, but homing can create a different traffic pattern that stresses it more:

- arm/disarm confirmation,
- repeated state checks,
- endstop-triggered stop/flush/recovery,
- preclear verification when the switch starts closed.

## 3.3 Architectural conclusion

The solution is **not** to move homing into a big firmware macro-command.

The correct solution is:

- keep homing **host-driven**,
- keep the firmware stop path **minimal and deterministic**,
- reduce unnecessary polling around phase transitions,
- keep arm/disarm **phase-scoped**,
- use status data already returned by normal request/stream traffic whenever possible.

---

## 4. Recommended architecture

## 4.1 Responsibility split

### Host responsibilities

The Raspberry Pi should own:

- homing phase sequencing,
- choice of approach speed, search speed, and backoff distance,
- retry policy,
- success/failure interpretation,
- publication of homing lifecycle events,
- home position finalization,
- the decision of when the endstop must be armed or disarmed.

Relevant code areas:

- `src/rpi/core/lateral.py`
- `src/rpi/motion/command_service.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/jsonrpc/winding_handler.py`

Current hardware mapping for the dual-contact lateral home sensor is:

- `GPIO22` = NO contact
- `GPIO21` = NC contact

The firmware interprets the NO/NC levels explicitly, so these semantic labels
must stay aligned with the real wiring.

### Firmware responsibilities

The ESP32 should own:

- applying `ENABLE_ENDSTOP`,
- reading and publishing endstop state,
- stopping motion immediately when an armed endstop triggers,
- refusing guarded motion when the endstop state is not safe,
- exposing canonical hit/armed/open/closed information in `StatusPayload`,
- recovery and queue protection after a hit.

Relevant code areas:

- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/stepper_driver.cpp`
- `src/esp32/src/messages.h`

## 4.2 Why the host must remain the homing orchestrator

This project already places planning and session logic on the host.

That is the right design because homing is not just a low-level stop condition. It is a policy sequence:

- possibly clear an already-closed switch,
- approach until first contact,
- back off,
- search again more slowly,
- declare the final home position,
- emit completion or failure events.

That logic is high-level and belongs on the Pi.

The firmware should **not** try to infer the whole homing process from a sensor edge.

---

## 5. Direct recommendation: explicit arm/disarm model

## 5.1 Yes, arm/disarm should be explicit

The host should explicitly send the `ENABLE_ENDSTOP` message to arm and disarm the endstop.

This is the recommended contract because it makes intent explicit and prevents endstop logic from interfering with non-homing motion.

That means:

- the endstop is **not** globally active forever,
- the host decides when contact should stop motion,
- the firmware only enforces what is currently armed.

## 5.2 Why not always armed

An always-armed model creates several problems:

- normal winding or manual moves could be stopped by a switch state that is only relevant during homing,
- preclear and backoff moves would require special exceptions anyway,
- more firmware-side policy would leak into the production motion path,
- it becomes much harder to reason about `endstop_hit_mask` and stale hits between phases.

## 5.3 What “explicit arm/disarm” means in practice

It does **not** mean spamming arm/disarm messages continuously.

It means sending arm/disarm **only at phase boundaries**.

Recommended frequency:

- one arm before a guarded phase starts,
- one disarm before an unguarded phase starts,
- one final disarm at the end or on failure cleanup.

That is cheap and stable.

What should be avoided is:

- per-segment arm/disarm,
- per-poll arm confirmation loops beyond the minimum required ACK/state confirmation,
- frequent additional `GET_STATUS` requests when a recent request or stream response already contains fresh status.

---

## 6. Recommended homing state machine

The recommended homing behavior for the lateral axis is a four-state pattern with phase-scoped arm/disarm.

## 6.1 Startup check

Before homing starts, the host reads the current lateral endstop state.

Possible startup states:

- `PRESENT_OPEN`
- `PRESENT_CLOSED`
- `ABSENT`

Recommended behavior:

- if `ABSENT`: fail immediately,
- if `PRESENT_OPEN`: start the normal homing sequence,
- if `PRESENT_CLOSED`: run a preclear first.

## 6.2 Phase A: preclear if the endstop starts closed

Goal:

- move away from the switch until the sensor becomes open.

Rules:

- **disarm** the endstop before moving,
- run a controlled backoff move away from the switch,
- wait until the sensor reads `PRESENT_OPEN`,
- add a short debounce / stabilization delay,
- re-read to confirm `PRESENT_OPEN`.

Why disarmed:

- the system is intentionally moving away from the switch,
- contact during that motion must not be treated as the guarded event,
- preclear is a corrective move, not a final homing approach.

## 6.3 Phase B: fast approach

Goal:

- move toward the switch until the first contact.

Rules:

- **arm** the endstop before starting,
- stream the approach move,
- let the firmware stop immediately if contact occurs,
- let the host interpret the stop as the successful end of this phase.

Expected result:

- first contact is a normal phase completion, not an error.

## 6.4 Phase C: backoff

Goal:

- move off the switch to reopen the sensor.

Rules:

- **disarm** the endstop before moving,
- move in the reverse direction a configured number of steps,
- confirm the sensor is open before starting the final search.

Why disarmed:

- this phase intentionally moves through the contact-release region,
- the stop criterion is not “contact happened”,
- this is a positioning reset phase.

## 6.5 Phase D: slow search

Goal:

- move toward the switch again at lower speed to get the final reference point.

Rules:

- **arm** the endstop,
- run a slow guarded move,
- stop on the first valid contact,
- mark the axis home position,
- **disarm** on exit.

Expected result:

- the final contact defines the home reference.

---

## 7. Recommended arm/disarm policy table

| Homing phase | Arm endstop | Why |
|---|:---:|---|
| Startup state read | No | Read-only check |
| Preclear | No | Move away from a closed switch |
| Fast approach | Yes | Stop on first contact |
| Backoff | No | Move off the switch |
| Slow search | Yes | Stop on final contact |
| Normal winding | No | Must not perturb the production path |
| Manual jog / free move | Usually no | Only arm when the move is intentionally endstop-guarded |

This is the core recommendation of this document.

---

## 8. Protocol and status contract

## 8.1 Control message

The host uses `ENABLE_ENDSTOP` to change the armed state.

Relevant protocol definitions:

- `EnableEndstopPayload` in `src/esp32/src/messages.h`
- host mirror in `src/rpi/transport/messages.py`

This message should be treated like any other control request:

- send it,
- confirm it through `wait_for_request_result()`,
- then verify the published armed state from status.

## 8.2 Do not trust the immediate duplex response as the ACK

The SPI status is pipelined by one transfer.

Therefore the host must not assume the immediate full-duplex reply is the definitive result for the just-sent arm/disarm request.

The correct rule is:

1. send `ENABLE_ENDSTOP`,
2. wait until the status publishes the matching request sequence,
3. verify `last_result`,
4. verify `endstop_armed_mask`.

Relevant implementation and documentation:

- `src/rpi/transport/spi_transport.py`
- `src/esp32/src/comm_interface.cpp`
- `doc/spi_protocol.md`
- `doc/sequencing.md`

## 8.3 Status fields that matter

The endstop contract depends on these status fields:

- `lateral_endstop_state`
- `endstop_armed_mask`
- `endstop_hit_mask`
- `last_result`
- `last_executed_sequence`

### `lateral_endstop_state`

This answers the physical sensor interpretation:

- `PRESENT_OPEN`
- `PRESENT_CLOSED`
- `ABSENT`

This field is used by the host to validate startup conditions and phase transitions.

### `endstop_armed_mask`

This answers whether the firmware currently considers the axis armed.

This is the canonical way for the host to verify that the arm/disarm transition has actually taken effect.

### `endstop_hit_mask`

This answers whether an endstop hit has been latched since the last arm cycle.

This is the preferred signal for guarded-phase detection.

It should only be interpreted for axes that the host considers locally armed.

### `last_executed_sequence`

This must only advance for motion that was truly executed.

It must **not** be advanced by:

- segments blocked before execution,
- segments drained during recovery,
- segments stopped before they became real executed motion.

That rule is essential to keep the host and firmware motion models aligned after an endstop hit.

---

## 9. Firmware behavior that should remain true

## 9.1 Immediate stop belongs in firmware

The endstop-triggered stop must happen on the ESP32 side.

The Pi must never be the primary real-time safety boundary, because:

- SPI status is pipelined,
- the host is not real-time,
- an endstop-triggered motion stop is too latency-sensitive.

The firmware must therefore:

- latch the endstop event,
- stop the motion path immediately when armed,
- expose that hit to the host through status.

## 9.2 Gating of guarded motion

When the lateral endstop is armed, the firmware should reject or block lateral motion if the physical sensor state is not safe for the guarded direction.

This protects against:

- moving while the sensor is already closed,
- sensor absence while armed,
- accepting motion that violates the host’s guarded assumption.

Relevant logic lives in:

- `src/esp32/src/comm_interface.cpp`

## 9.3 ABSENT must remain fail-safe

If the endstop is armed and the sensor becomes `ABSENT`, that must be treated as a hard safety fault.

Expected behavior:

- stop motion,
- publish fault-compatible status/result,
- let the host fail the homing attempt.

This is not a normal homing completion.

## 9.4 Recovery after hit must not fabricate execution progress

After an endstop-triggered recovery, the firmware may drain queued work internally, but it must not publish those discarded segments as executed.

Otherwise the host will believe the machine progressed farther than it really did.

---

## 10. Host behavior that should remain true

## 10.1 Homing remains host-driven

The host should continue to build a homing move as a sequence of sub-moves and phases.

Relevant code areas:

- `src/rpi/core/lateral.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/motion/command_service.py`

## 10.2 Guarded-phase contact is not a generic error

In a guarded homing phase, an endstop trigger is usually the **expected success condition**.

That means the host must distinguish between:

- an expected contact during `approach` or `search`,
- a true transport fault,
- a sensor absent fault,
- a phase timeout with no contact,
- an externally requested stop.

This distinction is crucial.

A large part of previous instability came from letting these cases collapse into the same generic failure path.

## 10.3 Preclear and backoff must be unguarded

The host must explicitly disarm before preclear and backoff.

This is not optional.

If these phases are left armed, the system becomes self-contradictory:

- it is told to stop on contact,
- while simultaneously being asked to move out of a contact region.

## 10.4 Use fresh status from existing traffic first

The host should prefer status information already returned by:

- the control request confirmation path,
- the motion streaming responses.

Extra standalone `GET_STATUS` loops should be minimized.

This is especially important because the SPI settings must remain untouched for production.

---

## 11. How to implement this in the current codebase

## 11.1 Recommended host implementation pattern

### In `src/rpi/core/lateral.py`

Keep `LateralAxisController` as the owner of:

- homing state,
- `HOMING_STARTED`, `HOMING_COMPLETED`, `HOMING_FAILED` event publication,
- final home-state persistence in memory,
- finalization of success/failure.

### In `src/rpi/motion/command_service.py`

Keep `home_lateral()` asynchronous:

- return quickly to RPC,
- let background monitoring publish completion/failure through events.

This avoids long RPC timeouts and matches the fact that homing can legitimately take time.

### In `src/rpi/motion/move_queue.py`

This should remain the central place for:

- startup state validation,
- preclear if the switch starts closed,
- phase transitions,
- explicit `_set_endstop_armed(True/False)` at phase boundaries,
- interpretation of phase result,
- final cleanup disarm.

This file is the natural home of the operational homing state machine.

## 11.2 Recommended phase sequence

The effective host flow should be:

1. Read current status.
2. If `ABSENT`, fail.
3. If `CLOSED`, run preclear with endstop disarmed.
4. Confirm the sensor is open.
5. Arm endstop.
6. Run fast approach.
7. Treat contact as normal phase completion.
8. Disarm endstop.
9. Run backoff.
10. Confirm open.
11. Arm endstop.
12. Run slow search.
13. Treat contact as successful final home.
14. Disarm endstop.
15. Mark axis homed and publish completion.

## 11.3 Minimal control traffic rule

The implementation should send control traffic only when necessary:

- one arm request before approach,
- one disarm request before backoff,
- one arm request before final search,
- one disarm request on completion or failure.

It should **not**:

- repeatedly re-arm during the same phase,
- flood `GET_STATUS` between every micro-step of the workflow,
- use arm/disarm as a streaming-time heartbeat.

## 11.4 Manual API surface

Manual RPC methods like `arm_endstop` and `disarm_endstop` can still exist, but they should be documented as:

- service/debug helpers,
- not the primary production homing path.

The normal `home_lateral` command should manage arm/disarm internally.

---

## 12. Fault model

## 12.1 Conditions that should fail homing immediately

The following should fail the homing attempt:

- sensor state is `ABSENT` when a guarded phase is about to begin,
- preclear cannot open the switch,
- approach/search exceeds its allowed travel without contact,
- firmware reports guarded-motion rejection incompatible with the phase expectation,
- transport cannot confirm the required control request after bounded retries,
- a non-homing external stop interrupts the move.

## 12.2 Conditions that are normal, not faults

These are normal homing conditions and must not be promoted to generic errors:

- first contact at the end of `approach`,
- switch reopening during backoff,
- final contact during `search`,
- startup with the switch already closed, followed by a successful preclear.

## 12.3 Conditions that are safety faults

These should be treated as genuine safety problems:

- sensor becomes `ABSENT` while armed,
- firmware reports blocked motion when the host expected a safe guarded start,
- inconsistent status after arm/disarm confirmation,
- repeated unrecoverable control confirmation failures.

---

## 13. Why this design preserves the production spindle path

This design deliberately avoids changing the behavior that already works at 1750 rpm.

It preserves the production path because:

- the SPI frequency and framing stay unchanged,
- normal winding does not become permanently endstop-guarded,
- homing adds only a small number of explicit control transitions,
- the firmware hot path remains focused on execution and safety,
- the host handles high-level homing policy outside the steady winding stream.

In short:

- production winding remains optimized for throughput,
- homing remains optimized for correctness and safety,
- neither concern distorts the other.

---

## 14. Implementation checklist

Use this as the target contract for code review.

### Host

- homing is asynchronous at the RPC layer,
- `home_lateral` internally manages all arm/disarm transitions,
- preclear runs disarmed,
- backoff runs disarmed,
- approach and search run armed,
- guarded-phase contact is treated as phase success,
- home is only marked after the final slow search contact,
- final cleanup always disarms.

### Firmware

- `ENABLE_ENDSTOP` changes the armed state deterministically,
- armed state is published in `endstop_armed_mask`,
- hit state is published in `endstop_hit_mask`,
- sensor state is published in `lateral_endstop_state`,
- an armed endstop triggers an immediate stop,
- `ABSENT` while armed remains fail-safe,
- recovery does not publish discarded motion as executed.

### Transport

- control requests are confirmed via `wait_for_request_result()`,
- status is reused from existing traffic where possible,
- extra poll loops are minimized,
- motion/execution sequence semantics remain wrap-aware.

---

## 15. Files that define this design

### Host

- `src/rpi/core/lateral.py`
- `src/rpi/motion/command_service.py`
- `src/rpi/motion/move_queue.py`
- `src/rpi/transport/streamer.py`
- `src/rpi/transport/spi_transport.py`
- `src/rpi/jsonrpc/winding_handler.py`

### Firmware

- `src/esp32/src/comm_interface.cpp`
- `src/esp32/src/stepper_driver.cpp`
- `src/esp32/src/messages.h`

### Documentation

- `doc/homing.md`
- `doc/spi_protocol.md`
- `doc/sequencing.md`
- `doc/architecture.md`

---

## 16. Final recommendation

The correct implementation for PickupWinder is:

- **host-driven homing**,
- **firmware-enforced immediate stop**,
- **explicit endstop arm/disarm**,
- **phase-scoped guarding**,
- **minimal extra SPI traffic**,
- **no SPI setting changes for production**.

If a single sentence is needed for the design review, it is this:

> The endstop is a phase-scoped real-time safety guard controlled explicitly by the host, not a permanently armed global behavior and not a full homing state machine inside the firmware.
