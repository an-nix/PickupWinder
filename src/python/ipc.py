"""PRU IPC helper using /dev/mem and mmap.

Direct mapping of the PRU shared RAM layout defined in `pru_ipc.h`.
Requires root to open `/dev/mem` on the target.
"""
import os
import mmap
import struct

# Physical PRU shared RAM (AM335x)
PRU_SRAM_PHYS_BASE = 0x4A310000
PRU_SRAM_SIZE = 0x3000

# Offsets (must match pru_ipc.h)
IPC_CMD_RING_OFFSET      = 0x0000
IPC_CMD_RING_SLOTS       = 32
IPC_CMD_WHEAD_OFFSET     = 0x0200
IPC_CMD_RHEAD_OFFSET     = 0x0204
IPC_SPINDLE_TELEM_OFFSET = 0x0210
IPC_LATERAL_TELEM_OFFSET = 0x0240
IPC_SYNC_OFFSET          = 0x0270
IPC_SP_MOVE_WHEAD_OFFSET = 0x0280
IPC_SP_MOVE_RHEAD_OFFSET = 0x0284
IPC_SP_MOVE_RING_OFFSET  = 0x0288
IPC_SP_MOVE_SLOTS        = 128
IPC_LAT_MOVE_WHEAD_OFFSET = 0x0A88
IPC_LAT_MOVE_RHEAD_OFFSET = 0x0A8C
IPC_LAT_MOVE_RING_OFFSET  = 0x0A90
IPC_LAT_MOVE_SLOTS        = 128

# Axis identifiers (pru_ipc.h: AXIS_*)
AXIS_SPINDLE = 0
AXIS_LATERAL = 1
AXIS_ALL     = 0xFF

# Command opcodes (pru_ipc.h: CMD_*) — 6 opcodes only
CMD_NOP               = 0
CMD_ENABLE            = 1   # value_a = 1:enable  0:disable driver
CMD_EMERGENCY_STOP    = 2   # both axes: immediate stop, flush ring
CMD_RESET_POSITION    = 3   # reset step_count and position_steps to 0
CMD_HOME_START        = 4   # PRU1: enter homing_mode
CMD_QUEUE_FLUSH       = 5   # flush move ring without emergency stop

# State flag bits (state field of pru_axis_telem_t) — must match pru_ipc.h
STATE_IDLE    = (1 << 0)
STATE_RUNNING = (1 << 1)
STATE_AT_HOME = (1 << 4)
STATE_FAULT   = (1 << 5)
STATE_ENABLED = (1 << 6)
STATE_HOMING  = (1 << 7)

# Fault flag bits (faults field of pru_axis_telem_t)
FAULT_HOME_SENSOR    = (1 << 0)
FAULT_OVERRUN        = (1 << 1)
FAULT_WATCHDOG       = (1 << 2)
FAULT_MOVE_UNDERRUN  = (1 << 3)  # ring drained while stepper was running


class IpcChannel:
    """Memory-map PRU shared RAM and provide basic send/read helpers."""

    def __init__(self, dev='/dev/mem', phys_base=PRU_SRAM_PHYS_BASE, size=PRU_SRAM_SIZE):
        self.dev = dev
        self.phys_base = phys_base
        self.size = size
        self.fd = None
        self.mm = None

    def open(self):
        """Open and map /dev/mem.  Raises OSError if not root or hardware missing."""
        self.fd = os.open(self.dev, os.O_RDWR | os.O_SYNC)
        self.mm = mmap.mmap(
            self.fd, self.size, mmap.MAP_SHARED,
            mmap.PROT_READ | mmap.PROT_WRITE,
            offset=self.phys_base
        )

    def isOpen(self) -> bool:
        return self.mm is not None

    def close(self):
        try:
            if self.mm: self.mm.close()
        except Exception:
            pass
        try:
            if self.fd is not None: os.close(self.fd)
        except Exception:
            pass
        self.mm = None
        self.fd = None

    # ── Commands ──────────────────────────────────────────────────────────────
    def sendCommand(self, cmd: int, axis: int, value_a: int = 0, value_b: int = 0) -> bool:
        """Write a pru_cmd_t (16 B) into the ring buffer.  Returns True on success."""
        if not self.mm:
            return False
        whead = struct.unpack_from('<I', self.mm, IPC_CMD_WHEAD_OFFSET)[0]
        rhead = struct.unpack_from('<I', self.mm, IPC_CMD_RHEAD_OFFSET)[0]
        next_slot = (whead + 1) % IPC_CMD_RING_SLOTS
        if next_slot == rhead:
            return False   # ring full
        slot_off = IPC_CMD_RING_OFFSET + whead * 16
        # pru_cmd_t: u8 cmd, u8 axis, u8 flags, u8 _pad0, u32 value_a, u32 value_b, u32 _reserved
        packed = struct.pack('<4B3I',
                             int(cmd) & 0xFF, int(axis) & 0xFF, 0, 0,
                             int(value_a) & 0xFFFFFFFF,
                             int(value_b) & 0xFFFFFFFF, 0)
        self.mm[slot_off:slot_off + 16] = packed
        struct.pack_into('<I', self.mm, IPC_CMD_WHEAD_OFFSET, next_slot)
        try:
            self.mm.flush(IPC_CMD_WHEAD_OFFSET, 4)
        except Exception:
            pass
        return True

    def sendMove(self, axis: int, interval: int, count: int, add: int, direction: int) -> bool:
        """Push one Klipper-style move triple into the per-axis move ring."""
        if not self.mm:
            return False
        if axis == AXIS_SPINDLE:
            whead_off = IPC_SP_MOVE_WHEAD_OFFSET
            rhead_off = IPC_SP_MOVE_RHEAD_OFFSET
            ring_off  = IPC_SP_MOVE_RING_OFFSET
            slots     = IPC_SP_MOVE_SLOTS
        elif axis == AXIS_LATERAL:
            whead_off = IPC_LAT_MOVE_WHEAD_OFFSET
            rhead_off = IPC_LAT_MOVE_RHEAD_OFFSET
            ring_off  = IPC_LAT_MOVE_RING_OFFSET
            slots     = IPC_LAT_MOVE_SLOTS
        else:
            return False
        whead = struct.unpack_from('<I', self.mm, whead_off)[0]
        rhead = struct.unpack_from('<I', self.mm, rhead_off)[0]
        if ((whead + 1) % slots) == rhead:
            return False   # ring full
        entry_off = ring_off + whead * 16
        # pru_move_t: u32 interval, u32 count, i32 add, u8 direction, u8 _pad[3]
        packed = struct.pack('<IIiB3x',
                             int(interval) & 0xFFFFFFFF,
                             int(count) & 0xFFFFFFFF,
                             int(add),
                             int(direction) & 0xFF)
        self.mm[entry_off:entry_off + 16] = packed
        struct.pack_into('<I', self.mm, whead_off, (whead + 1) % slots)
        try:
            self.mm.flush(whead_off, 4)
        except Exception:
            pass
        return True

    def moveQueueFreeSlots(self, axis: int) -> int:
        """Return number of free slots in an axis move ring."""
        if not self.mm:
            return 0
        if axis == AXIS_SPINDLE:
            wh = struct.unpack_from('<I', self.mm, IPC_SP_MOVE_WHEAD_OFFSET)[0]
            rh = struct.unpack_from('<I', self.mm, IPC_SP_MOVE_RHEAD_OFFSET)[0]
            slots = IPC_SP_MOVE_SLOTS
        else:
            wh = struct.unpack_from('<I', self.mm, IPC_LAT_MOVE_WHEAD_OFFSET)[0]
            rh = struct.unpack_from('<I', self.mm, IPC_LAT_MOVE_RHEAD_OFFSET)[0]
            slots = IPC_LAT_MOVE_SLOTS
        used = (wh - rh) % slots
        return slots - 1 - used

    # snake_case aliases
    def send_command(self, cmd, axis, value_a=0, value_b=0):
        return self.sendCommand(cmd, axis, value_a, value_b)

    def send_move(self, axis, interval, count, add, direction):
        return self.sendMove(axis, interval, count, add, direction)

    # ── Telemetry ─────────────────────────────────────────────────────────────
    def _readAxisTelem(self, offset: int) -> dict:
        """Parse pru_axis_telem_t (48 B) from shared RAM."""
        if not self.mm:
            return {}
        data = self.mm[offset:offset + 48]
        try:
            # seq(u32), step_count(u32), current_interval(u32), moves_pending(u32),
            # position_steps(i32), state(u16), faults(u16), _reserved[2](u32 x2)
            seq, step_count, current_interval, moves_pending, \
                position_steps, state, faults, _, _ = \
                struct.unpack_from('<IIIIiHHII', data, 0)
        except struct.error:
            return {}
        # Compute Hz from interval for convenience
        PRU_CLOCK_HZ = 200_000_000
        current_hz = PRU_CLOCK_HZ // (2 * current_interval) if current_interval else 0
        return {
            'seq':              seq,
            'step_count':       step_count,
            'current_interval': current_interval,
            'current_hz':       current_hz,     # derived
            'moves_pending':    moves_pending,
            'position_steps':   position_steps,
            'state':            state,
            'faults':           faults,
        }

    def readSpindleTelem(self) -> dict:
        return self._readAxisTelem(IPC_SPINDLE_TELEM_OFFSET)

    def readLateralTelem(self) -> dict:
        return self._readAxisTelem(IPC_LATERAL_TELEM_OFFSET)

    # Legacy aliases
    def read_spindle_telem(self): return self.readSpindleTelem()
    def read_lateral_telem(self): return self.readLateralTelem()
    def read_axis_telem(self, offset): return self._readAxisTelem(offset)

    def readSync(self) -> dict:
        if not self.mm:
            return {}
        data = self.mm[IPC_SYNC_OFFSET:IPC_SYNC_OFFSET + 16]
        try:
            spindle_interval, lateral_interval, control_flags, _ = \
                struct.unpack_from('<4I', data, 0)
        except struct.error:
            return {}
        PRU_CLOCK_HZ = 200_000_000
        spindle_hz = PRU_CLOCK_HZ // (2 * spindle_interval) if spindle_interval else 0
        lateral_hz = PRU_CLOCK_HZ // (2 * lateral_interval) if lateral_interval else 0
        return {
            'spindle_interval': spindle_interval,
            'lateral_interval': lateral_interval,
            'spindle_hz':       spindle_hz,    # derived
            'lateral_hz':       lateral_hz,    # derived
            'control_flags':    control_flags,
        }

    def read_sync(self): return self.readSync()