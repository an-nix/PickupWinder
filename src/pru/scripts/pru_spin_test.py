#!/usr/bin/env python3
"""Small PRU spindle spin test using direct shared-RAM IPC.

Run on the BeagleBone Black as root after the PRU firmware is loaded.
It sends one constant-speed move to the spindle axis, waits for completion,
and prints telemetry snapshots.
"""

import argparse
import mmap
import os
import struct
import sys
import time

PRU_SRAM_PHYS_BASE = 0x4A310000
PRU_SRAM_SIZE = 0x3000
PRU_CLOCK_HZ = 200_000_000

IPC_CMD_RING_SLOTS = 32
IPC_CMD_RING_OFFSET = 0x0000
IPC_CMD_WHEAD_OFFSET = 0x0200
IPC_CMD_RHEAD_OFFSET = 0x0204
IPC_SPINDLE_TELEM_OFFSET = 0x0210
IPC_SP_MOVE_WHEAD_OFFSET = 0x0280
IPC_SP_MOVE_RHEAD_OFFSET = 0x0284
IPC_SP_MOVE_RING_OFFSET = 0x0288
IPC_SP_MOVE_SLOTS = 128
IPC_LATERAL_TELEM_OFFSET = 0x0240
IPC_SYNC_OFFSET = 0x0270

AXIS_SPINDLE = 0
CMD_ENABLE = 1
CMD_RESET_POSITION = 3
CMD_QUEUE_FLUSH = 5

STATE_RUNNING = 1 << 1
STATE_ENABLED = 1 << 6

CMD_FMT = struct.Struct("<BBBBIII")
MOVE_FMT = struct.Struct("<IIiBBH")
TELEM_FMT = struct.Struct("<IIIIiHH6I")
U32_FMT = struct.Struct("<I")

STEPS_PER_REV = 6400
MIN_HZ = 1067
MAX_HZ = 160000


def read_u32(buf, offset):
    return U32_FMT.unpack_from(buf, offset)[0]


def write_u32(buf, offset, value):
    U32_FMT.pack_into(buf, offset, value & 0xFFFFFFFF)


def wait_until(predicate, timeout_s, poll_s=0.002):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll_s)
    return False


def send_command(buf, cmd, axis, value_a=0, value_b=0, timeout_s=1.0):
    def has_room():
        wh = read_u32(buf, IPC_CMD_WHEAD_OFFSET) % IPC_CMD_RING_SLOTS
        rh = read_u32(buf, IPC_CMD_RHEAD_OFFSET) % IPC_CMD_RING_SLOTS
        return ((wh + 1) % IPC_CMD_RING_SLOTS) != rh

    if not wait_until(has_room, timeout_s):
        raise RuntimeError("command ring full")

    wh = read_u32(buf, IPC_CMD_WHEAD_OFFSET) % IPC_CMD_RING_SLOTS
    CMD_FMT.pack_into(buf, IPC_CMD_RING_OFFSET + wh * CMD_FMT.size,
                      cmd, axis, 0, 0, value_a, value_b, 0)
    write_u32(buf, IPC_CMD_WHEAD_OFFSET, (wh + 1) % IPC_CMD_RING_SLOTS)


def send_move(buf, interval, count, add, direction, timeout_s=1.0):
    def has_room():
        wh = read_u32(buf, IPC_SP_MOVE_WHEAD_OFFSET) % IPC_SP_MOVE_SLOTS
        rh = read_u32(buf, IPC_SP_MOVE_RHEAD_OFFSET) % IPC_SP_MOVE_SLOTS
        return ((wh + 1) % IPC_SP_MOVE_SLOTS) != rh

    if not wait_until(has_room, timeout_s):
        raise RuntimeError("spindle move ring full")

    wh = read_u32(buf, IPC_SP_MOVE_WHEAD_OFFSET) % IPC_SP_MOVE_SLOTS
    MOVE_FMT.pack_into(buf, IPC_SP_MOVE_RING_OFFSET + wh * MOVE_FMT.size,
                       interval, count, add, direction, 0, 0)
    write_u32(buf, IPC_SP_MOVE_WHEAD_OFFSET, (wh + 1) % IPC_SP_MOVE_SLOTS)


def reset_ipc_state(buf):
    write_u32(buf, IPC_CMD_WHEAD_OFFSET, 0)
    write_u32(buf, IPC_CMD_RHEAD_OFFSET, 0)
    write_u32(buf, IPC_SP_MOVE_WHEAD_OFFSET, 0)
    write_u32(buf, IPC_SP_MOVE_RHEAD_OFFSET, 0)

    for offset in (IPC_SPINDLE_TELEM_OFFSET, IPC_LATERAL_TELEM_OFFSET):
        buf[offset:offset + TELEM_FMT.size] = b"\x00" * TELEM_FMT.size
    buf[IPC_SYNC_OFFSET:IPC_SYNC_OFFSET + 16] = b"\x00" * 16


def ring_snapshot(buf):
    return {
        "cmd_w": read_u32(buf, IPC_CMD_WHEAD_OFFSET),
        "cmd_r": read_u32(buf, IPC_CMD_RHEAD_OFFSET),
        "sp_w": read_u32(buf, IPC_SP_MOVE_WHEAD_OFFSET),
        "sp_r": read_u32(buf, IPC_SP_MOVE_RHEAD_OFFSET),
    }


def read_telem(buf):
    return TELEM_FMT.unpack_from(buf, IPC_SPINDLE_TELEM_OFFSET)


def interval_from_hz(hz):
    hz = max(MIN_HZ, min(MAX_HZ, hz))
    return PRU_CLOCK_HZ // (2 * hz)


def rpm_to_hz(rpm):
    hz = (rpm * STEPS_PER_REV) // 60
    return max(MIN_HZ, min(MAX_HZ, hz))


def main():
    parser = argparse.ArgumentParser(description="PRU spindle spin test")
    parser.add_argument("--rpm", type=int, default=20)
    parser.add_argument("--seconds", type=int, default=6)
    parser.add_argument("--forward", type=int, choices=(0, 1), default=1)
    parser.add_argument("--settle-ms", type=int, default=250)
    args = parser.parse_args()

    if os.geteuid() != 0:
        print("Must run as root", file=sys.stderr)
        return 1

    rpm = max(1, args.rpm)
    seconds = max(1, args.seconds)
    step_hz = rpm_to_hz(rpm)
    interval = interval_from_hz(step_hz)
    edges = 2 * step_hz * seconds

    print(f"[spin] rpm={rpm} seconds={seconds} forward={args.forward} step_hz={step_hz} interval={interval} edges={edges}")

    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    try:
        buf = mmap.mmap(fd, PRU_SRAM_SIZE, mmap.MAP_SHARED,
                        mmap.PROT_READ | mmap.PROT_WRITE,
                        offset=PRU_SRAM_PHYS_BASE)
        try:
            reset_ipc_state(buf)
            print(f"[ipc] reset heads {ring_snapshot(buf)}")

            send_command(buf, CMD_QUEUE_FLUSH, AXIS_SPINDLE)
            send_command(buf, CMD_ENABLE, AXIS_SPINDLE, 0)
            send_command(buf, CMD_RESET_POSITION, AXIS_SPINDLE)
            time.sleep(max(0.0, args.settle_ms / 1000.0))

            send_command(buf, CMD_ENABLE, AXIS_SPINDLE, 1)
            send_move(buf, interval, edges, 0, args.forward)
            print(f"[ipc] queued move {ring_snapshot(buf)}")

            t0 = time.monotonic()
            next_print = t0
            while True:
                now = time.monotonic()
                seq, step_count, current_interval, pending, pos, state, faults, *_ = read_telem(buf)
                if now >= next_print:
                    running = 1 if (state & STATE_RUNNING) else 0
                    enabled = 1 if (state & STATE_ENABLED) else 0
                    print(f"[telem] seq={seq} steps={step_count} interval={current_interval} pending={pending} running={running} enabled={enabled} faults=0x{faults:04x} heads={ring_snapshot(buf)}")
                    next_print = now + 0.5
                if now - t0 >= seconds + 1.0:
                    break
                time.sleep(0.01)

            send_command(buf, CMD_QUEUE_FLUSH, AXIS_SPINDLE)
            send_command(buf, CMD_ENABLE, AXIS_SPINDLE, 0)
            seq, step_count, current_interval, pending, pos, state, faults, *_ = read_telem(buf)
            print(f"[done] steps={step_count} interval={current_interval} pending={pending} state=0x{state:04x} faults=0x{faults:04x}")
        finally:
            buf.close()
    finally:
        os.close(fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
