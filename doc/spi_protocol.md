# SPI Protocol — PickupWinder

This document describes the SPI frame format, the pipelined ACK semantics,
and the deduplication layers used between the Raspberry Pi host and the ESP32
firmware.

---

## 1. Physical layer

| Parameter | Value |
|-----------|-------|
| Frame size | **512 bytes** (fixed, both directions) |
| SPI mode | **Mode 1** (CPOL=0, CPHA=1) |
| Bit order | MSB first |
| Endianness | Little-endian |
| CRC | **CRC16-CCITT-FALSE** (poly 0x1021, init 0xFFFF) |
| READY sideband | GPIO17 (active-low: ESP32 drives low when ready to receive) |

Every SPI transaction exchanges exactly one 512-byte frame in each direction
simultaneously. The host never sends a partial frame.

SPI Mode 1 is required on both sides. Using Mode 0 reintroduces a timing
fragility that has been observed with the ESP32 SPI slave DMA peripheral.

---

## 2. Frame layout

```
 Byte offset   Field
 ────────────────────────────────────────────────
 0–11          SpiMessageHeader (12 bytes, fixed)
 12–511        Payload (0–500 bytes, variable)
 (unused payload bytes are zeroed)
```

### SpiMessageHeader (12 bytes)

```c
struct __attribute__((packed)) SpiMessageHeader {
    uint16_t magic;           // 0x5057 ('P','W')
    uint8_t  version;         // must be 3
    uint8_t  msg_type;        // SpiMessageType enum
    uint16_t sequence;        // 16-bit wrapping counter (host side)
    uint16_t payload_length;  // bytes of payload that follow
    uint16_t flags;           // reserved, set to 0
    uint16_t crc16;           // CRC over header (crc16=0) + payload
};
```

The CRC covers the entire header (with `crc16` zeroed) plus the payload bytes
indicated by `payload_length`. Trailing bytes beyond `payload_length` are not
included.

Reference files:
- Firmware definition: `src/esp32/src/comm/messages.h`
- Python mirror: `src/windy/transport/messages.py`

---

## 3. Message types

```
Value  Name                       Direction   Description
─────────────────────────────────────────────────────────────────────
0x00   NOP                        host→esp    No-op heartbeat
0x06   GET_STATUS                 host→esp    Request a fresh status frame
0x12   FLUSH                      host→esp    Flush motion queue past a sequence floor
0x13   MULTI_AXIS_SEGMENT_BLOCK   host→esp    Batch of multi-axis motion segments (production)
0x14   ENABLE_ENDSTOP             host→esp    Arm or disarm an endstop on an axis
0x7F   PING                       host→esp    Connectivity check
0x80   STATUS                     esp→host    Status/telemetry frame (always returned)
```

Legacy message types (`ENABLE_AXIS`, `ESTOP`, `STOP_AXIS`, `DISABLE_ALL`,
`RESET_STATS`, `STEP_BLOCK`, `SEGMENT_BLOCK`) remain defined in `messages.h`
but are not used by the active Python host code.

---

## 4. MULTI_AXIS_SEGMENT_BLOCK (0x13)

This is the production motion path. Every winding move is broken into
`MultiAxisSegment` objects by the host and streamed as blocks.

### Block header (4 bytes, immediately follows SpiMessageHeader)

```c
struct __attribute__((packed)) MultiAxisSegmentBlockHeader {
    uint16_t block_seq;      // wrapping block sequence (for retry dedup)
    uint8_t  segment_count;  // number of multi-axis segments in this block
    uint8_t  axis_count;     // number of axes per segment
};
```

### Per-segment layout

Each multi-axis segment contains one `MotionSegmentMessage` per axis:

```c
struct __attribute__((packed)) MotionSegmentMessage {
    uint16_t step_count;   // number of steps in this segment
    uint16_t start_ticks;  // initial interval between steps (RMT ticks)
    int16_t  add_ticks;    // linear rate adjustment per step (Bresenham-style)
    uint8_t  flags;        // bit 0: DIR_REVERSE
    uint8_t  reserved;
};  // 8 bytes
```

The segment also carries a `motion_sequence` field (16-bit, one per
`MultiAxisSegmentBlockHeader`) used for ordering enforcement.

### Example block (2 axes, 3 segments)

```
[SpiMessageHeader 12 B]
[MultiAxisSegmentBlockHeader 4 B]
  [motion_sequence_0 2 B][seg_axis0 8 B][seg_axis1 8 B]   ← segment 0
  [motion_sequence_1 2 B][seg_axis0 8 B][seg_axis1 8 B]   ← segment 1
  [motion_sequence_2 2 B][seg_axis0 8 B][seg_axis1 8 B]   ← segment 2
[zero-padding to 512 B]
```

---

## 5. FLUSH (0x12)

```c
struct __attribute__((packed)) FlushPayload {
    uint16_t flush_sequence;  // new minimum motion_sequence floor
    uint8_t  reserved[2];
};
```

Instructs the firmware to discard any queued segments whose `motion_sequence`
is less than `flush_sequence`, then drain any partially-filled pipeline stages.
Used by the host before a program restart or stop.

---

## 6. ENABLE_ENDSTOP (0x14)

```c
struct __attribute__((packed)) EnableEndstopPayload {
    uint8_t axis_id;   // 0 = spindle, 1 = lateral
    uint8_t arm;       // 1 = arm, 0 = disarm
    uint8_t reserved[2];
};
```

Arms or disarms the endstop for the given axis. The firmware records the state
in `endstop_armed_mask` within the `StatusPayload`.

---

## 7. StatusPayload (0x80) — always returned

The ESP32 always returns a 512-byte frame. The payload is a `StatusPayload`
(54 bytes):

```c
struct __attribute__((packed)) StatusPayload {
    uint32_t uptime_ms;
    uint16_t queue_free_slots[4];      // per-axis step queue headroom
    uint16_t ring_free_slots[4];       // per-axis RMT ring headroom
    uint32_t underrun_count[4];        // total RMT underruns per axis
    uint16_t last_rx_sequence;         // sequence of last non-telemetry request processed
    uint8_t  last_rx_type;             // message type of that request
    uint8_t  last_result;              // SpiMessageResult for that request
    uint8_t  protocol_version;         // mirrors SPI_MSG_VERSION
    uint8_t  enabled_mask;             // bitmask of enabled axes
    uint8_t  running_mask;             // bitmask of axes currently stepping
    uint8_t  lateral_endstop_state;    // LateralEndstopState enum
    uint8_t  endstop_armed_mask;       // bitmask of armed endstops
    uint8_t  endstop_hit_mask;         // bitmask of triggered endstops (sticky until re-armed)
    uint16_t last_executed_sequence;   // motion_sequence of last completed segment
    uint8_t  multi_axis_queue_free;    // free slots in the multi-axis planner queue
    uint8_t  planner_queue_free;       // free slots in the motion planner queue
    uint16_t last_planned_sequence;    // motion_sequence of last planned segment
    uint16_t segments_dropped;         // total out-of-order segments dropped
};
```

### Key fields for the host

| Field | How the host uses it |
|-------|----------------------|
| `last_rx_sequence` + `last_result` | ACK confirmation for control requests. |
| `endstop_hit_mask` | Primary homing-complete signal. Bit N stays set after axis N triggers until re-armed. |
| `multi_axis_queue_free` | Backpressure gate: host waits before sending more segments. |
| `last_executed_sequence` | Progress tracking: how far the firmware has actually stepped. |
| `segments_dropped` | Diagnostic counter; non-zero indicates sequence ordering violation. |

### What does NOT update the ACK triple

`GET_STATUS`, `PING`, and `NOP` are telemetry-only. They do not overwrite
`last_rx_sequence` / `last_rx_type` / `last_result`. Transient parse errors
(`BAD_MAGIC`, `BAD_VERSION`, `BAD_LENGTH`, `BAD_CRC`) also do not overwrite
the ACK triple.

---

## 8. Pipelined ACK rule

SPI is full-duplex. The status frame received **during transfer N** reflects the
processing result of transfer **N-1**:

```
Host TX:  [request N  ]   [request N+1]   [GET_STATUS  ]
Host RX:  [status N-1 ]   [status N   ]   [status N+1 ]
                                               ↑
                              ACK for request N visible here
```

**The host must never interpret the immediate return value of a send as the ACK
for that send.**

The correct pattern is:

```python
seq = transport.send_multi_axis_block(segments)
# seq is the sequence number just sent
# but the ACK arrives in the next transfer
transport.wait_for_request_result(seq)   # sends a GET_STATUS internally; blocks until confirmed
```

`wait_for_request_result()` is implemented in `src/windy/transport/spi_transport.py`.

---

## 9. Deduplication and ordering

Three independent deduplication layers protect motion correctness:

### Layer 1 — Exact transport retry deduplification (CommInterface)

Applies to all control requests. Key: `(sequence, msg_type, payload_length, crc16)`.
If a retried frame is byte-for-byte identical to the last accepted request, the
firmware silently accepts it without re-processing.

### Layer 2 — Block sequence deduplication (MotionPlanner)

Applies only to `MULTI_AXIS_SEGMENT_BLOCK`. Key: `block_seq` (16-bit, wrapping).
If the firmware has already accepted a block with the same `block_seq`, it ignores
the duplicate.

### Layer 3 — Motion sequence ordering (MotionPlanner)

Each `MultiAxisSegment` carries a `motion_sequence`. The planner enforces strictly
monotonic delivery: any segment whose `motion_sequence` is older than the current
floor is dropped (counted in `segments_dropped`).

### All sequence comparisons are wrap-aware

16-bit counters wrap at 65535. Every comparison uses signed-distance arithmetic
(safe for gaps smaller than 32768):

```python
# src/windy/transport/messages.py
def sequence_signed_distance(a: int, b: int) -> int:
    """Returns b - a as a signed 16-bit value."""
    diff = (b - a) & 0xFFFF
    return diff if diff < 0x8000 else diff - 0x10000
```

```c
// src/esp32/src/motion/step_types.h
static inline int16_t sequence_signed_distance_u16(uint16_t a, uint16_t b) {
    return (int16_t)(uint16_t)(b - a);
}
```

See [sequencing.md](sequencing.md) for the full treatment.

---

## 10. Result codes

| Code | Name | Meaning |
|------|------|---------|
| 0x00 | `OK` | Request accepted |
| 0x01 | `BAD_MAGIC` | Wrong magic bytes |
| 0x02 | `BAD_VERSION` | Unsupported protocol version |
| 0x03 | `BAD_LENGTH` | `payload_length` out of range |
| 0x04 | `BAD_CRC` | CRC mismatch |
| 0x05 | `UNKNOWN_TYPE` | Unrecognised `msg_type` |
| 0x06 | `BAD_AXIS` | `axis_id` out of range |
| 0x07 | `QUEUE_FULL` | Motion queue has no room; retry |
| 0x08 | `INTERNAL_ERROR` | Firmware-side error |
| 0x09 | `ENDSTOP_BLOCKED` | Requested direction blocked by armed endstop |

---

## 11. Adding a new message type

1. Add the type constant to `SpiMessageType` in `src/esp32/src/comm/messages.h`.
2. Add the matching constant to `src/windy/transport/messages.py`.
3. Define the packed payload struct in `messages.h` and its Python mirror (`struct.Struct` + dataclass) in `messages.py`.
4. Add a dispatch case in `src/esp32/src/comm/comm_interface.cpp` (and `comm_request_dispatcher.cpp` if applicable).
5. Add an emit helper in `src/windy/transport/spi_transport.py`.
6. Verify the binary size: `static_assert(sizeof(NewPayload) <= SPI_MAX_PAYLOAD_SIZE)`.
7. Test with CRC injection to confirm the firmware rejects bad frames correctly.

---

## 12. Quick validation

Python:

```powershell
python -m py_compile src\windy\transport\messages.py src\windy\transport\spi_transport.py
```

Firmware:

```powershell
cd src\esp32
platformio run
```
