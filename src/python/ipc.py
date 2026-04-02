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

# Axis identifiers (pru_ipc.h: AXIS_*)
AXIS_SPINDLE = 0
AXIS_LATERAL = 1
AXIS_ALL     = 0xFF

# Command opcodes (pru_ipc.h: CMD_*)
CMD_NOP               = 0
CMD_SPINDLE_SET_HZ    = 1   # value_a = target Hz
CMD_SPINDLE_SET_ACCEL = 2   # value_a = accel Hz/ms
CMD_SPINDLE_START     = 3   # value_a = 1:forward  0:backward
CMD_SPINDLE_STOP      = 4   # controlled decel to 0
CMD_SPINDLE_FORCE     = 5   # immediate stop, keep enabled
CMD_SPINDLE_ENABLE    = 6   # value_a = 1:enable  0:disable driver
CMD_LATERAL_SET_HZ    = 7   # value_a = target Hz
CMD_LATERAL_SET_ACCEL = 8   # value_a = accel Hz/ms
CMD_LATERAL_START     = 9   # value_a = 1:forward  0:backward
CMD_LATERAL_STOP      = 10
CMD_LATERAL_FORCE     = 11
CMD_LATERAL_ENABLE    = 12  # value_a = 1:enable  0:disable
CMD_EMERGENCY_STOP    = 13  # both axes: immediate, disable drivers
CMD_RESET_POSITION    = 14  # value_a = axis: reset step counter
CMD_SET_NOMINAL_LAT   = 15  # value_a = nominal lateral Hz for compensation

# State flag bits (state field of pru_axis_telem_t)
STATE_IDLE    = (1 << 0)
STATE_RUNNING = (1 << 1)
STATE_ACCEL   = (1 << 2)
STATE_DECEL   = (1 << 3)
STATE_AT_HOME = (1 << 4)
STATE_FAULT   = (1 << 5)
STATE_ENABLED = (1 << 6)

# Fault flag bits (faults field of pru_axis_telem_t)
FAULT_HOME_SENSOR  = (1 << 0)
FAULT_OVERRUN      = (1 << 1)
FAULT_WATCHDOG     = (1 << 2)
FAULT_CMD_OVERFLOW = (1 << 3)


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

    # snake_case alias
    def send_command(self, cmd, axis, value_a=0, value_b=0):
        return self.sendCommand(cmd, axis, value_a, value_b)

    def setNominalLateralHz(self, lat_hz: int):
        self.sendCommand(CMD_SET_NOMINAL_LAT, AXIS_LATERAL, lat_hz)

    # ── Telemetry ─────────────────────────────────────────────────────────────
    def _readAxisTelem(self, offset: int) -> dict:
        """Parse pru_axis_telem_t (48 B) from shared RAM."""
        if not self.mm:
            return {}
        data = self.mm[offset:offset + 48]
        try:
            # seq(u32), step_count(u32), current_hz(u32), target_hz(u32),
            # position_steps(i32), state(u16), faults(u16), _reserved[2](u32 x2)
            seq, step_count, current_hz, target_hz, position_steps, state, faults, _, _ = \
                struct.unpack_from('<IIIIiHHII', data, 0)
        except struct.error:
            return {}
        return {
            'seq':            seq,
            'step_count':     step_count,
            'current_hz':     current_hz,
            'target_hz':      target_hz,
            'position_steps': position_steps,
            'state':          state,
            'faults':         faults,
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
            spindle_hz, lateral_hz, nominal_lat_hz, control_flags = \
                struct.unpack_from('<4I', data, 0)
        except struct.error:
            return {}
        return {
            'spindle_hz':     spindle_hz,
            'lateral_hz':     lateral_hz,
            'nominal_lat_hz': nominal_lat_hz,
            'control_flags':  control_flags,
        }

    def read_sync(self): return self.readSync()