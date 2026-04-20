# Sequencing, Retries, and Wrap-around

This note consolidates the sequence audit for PickupWinder after the transport and planner fixes.

## Sequence domains

The runtime uses three distinct 16-bit sequence spaces.

### 1. Transport request sequence

- Field: `SpiMessageHeader.sequence`
- Producer: host transport in `src/rpi/transport/spi_transport.py`
- Consumer: `CommInterface` status publishing through `last_rx_sequence`
- Purpose: correlate a request with its real ACK/result

This is not a motion sequence. It only answers: "which request did the ESP32 actually process?"

### 2. Motion block sequence

- Field: `MultiAxisSegmentBlockHeader.block_seq`
- Producer: host streamer
- Consumer: `CommInterface::handleMultiAxisSegmentBlock()`
- Purpose: drop already accepted multi-axis block retries

This protects against the host resending the same block after a timeout or a stale immediate status frame.

### 3. Motion segment sequence

- Field: `multi_axis_segment_t.motion_sequence`
- Producer: host generators / `MoveQueue`
- Consumers:
  - host streamer in-flight tracking
  - firmware planner monotonic guard
  - firmware executor completion reporting
- Purpose: preserve execution order across the full motion pipeline

## Pipelined ACK semantics

SPI is full-duplex, but the status payload returned during transfer `N` reflects the state after transfer `N-1` was handled.

Operational rule:

- send request
- keep the returned transport sequence
- call `wait_for_request_result(sequence)`
- only then trust `last_result`

The immediate response from `transfer_request()` is useful as a fresh status snapshot, but not as the definitive ACK for the just-sent request.

## Active protection layers

### Exact request retry dedupe

`CommInterface` caches the last processed request by:

- transport sequence
- message type
- payload length
- CRC

An exact retry reuses the prior result instead of re-executing side effects.

### Accepted block dedupe

Once a `MULTI_AXIS_SEGMENT_BLOCK` is accepted, `block_seq` is compared against `last_accepted_block_seq_` using wrap-aware helpers.

This prevents a block already consumed by the firmware from being injected twice after a retry.

### Planner monotonic guard

`MotionPlanner` keeps `last_planned_motion_seq_` and drops stale or out-of-order `motion_sequence` values before they reach the executor queue.

### Executor completion state

`last_executed_sequence_` is published back to the host through `StatusPayload` so the streamer can retire confirmed in-flight segments.

## Flush semantics

`flush_until(sequence)` now means:

1. the host requests a flush threshold
2. the firmware drains queued command and planned-segment buffers
3. the planner posts a flush sentinel
4. the planner floor moves to `flush_sequence`
5. newly stale segments are ignored

This prevents old queued segments from surviving a trajectory cancellation and then resurfacing later.

## Wrap-around safety

Python and firmware both compare sequences with signed 16-bit distance helpers.

Python:

```python
((a - b + 0x8000) & 0xFFFF) - 0x8000
```

Firmware:

```cpp
static_cast<int16_t>(static_cast<uint16_t>(lhs - rhs))
```

This is wrap-safe if the compared values never drift by `>= 32768`.

That condition holds in this project because the real windows are small:

- transport outstanding ACKs: 1 request at a time
- accepted block queue depth: 64
- planner queue depth: 128
- host inflight segments: speed-dependent `24 / 48 / 96`

These are all far below half of the 16-bit space.

## Boundary case: `0xFFFF`

`StatusPayload.last_executed_sequence` uses `0xFFFF` as the startup sentinel for "nothing executed yet".

That same value also exists naturally at wrap-around.

In practice this is acceptable because:

- the next valid sequence after `0xFFFF` is `0`
- the host already restarts new motion at `0` when seeding from `0xFFFF`
- the next reported value (`0`) re-establishes monotonic order cleanly

This means the exact wrap boundary is intentionally treated as a special case, but it does not break ordering.

## Host-side follow-up fixes included in this audit

Two additional host issues were corrected during the review.

### Batch-local monotonic validation

`MultiAxisRampStreamer` now validates each new segment against the previous segment in the current batch, not only against the last previously acknowledged send.

This catches a generator bug such as:

- `10, 12, 11`

while still allowing a legal wrap sequence such as:

- `65534, 65535, 0, 1`

### Mock transport wrap behavior

`MockSpiTransport` now preserves wrapped execution order with an explicit pending sequence queue instead of raw integer `max()` and `<` comparisons.

That makes mock-based tests consistent with the real wrap-aware transport semantics.

## Conclusion

With the current implementation, the production host and firmware path is overflow-safe for the intended operating envelope.

The remaining practical constraints are:

- do not compare sequence values separated by `>= 32768`
- do not reintroduce raw integer ordering for 16-bit sequence fields
- always confirm requests through `wait_for_request_result()`
- keep new generated diagnostics outside the repository root