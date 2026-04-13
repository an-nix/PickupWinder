"""esp32_controller.py — Async ESP32 controller via SPI.

MIGRATION: Replaces the BBB PickupController + DaemonClient + PruClient
chain (Python → Unix socket → C daemon → PRU shared RAM).
Now the path is: Python → SPI → ESP32 directly.

The controller is async-first (asyncio) but SPI transfers are blocking
(~10 µs per 32-byte transfer at 4 MHz).  We use asyncio.to_thread()
for the SPI calls to avoid blocking the event loop.

Usage:
    ctrl = ESP32Controller(transport, axes)
    await ctrl.enable(AxisId.ALL)
    await ctrl.set_speed(AxisId.BOBBIN, rpm=1000)
    status = await ctrl.get_status()
"""

from __future__ import annotations

import asyncio
import logging
from typing import Callable, Optional, Awaitable

from .protocol import (
    AxisId,
    CmdFlags,
    CmdOpcode,
    EventType,
    MachineStatus,
    encode_cmd,
    encode_cmd_u32,
)
from .spi_transport import SpiTransport
from .axis import Axis

logger = logging.getLogger(__name__)

# Type alias for event callbacks
EventCallback = Callable[[EventType, int, MachineStatus], Awaitable[None]]


class ESP32Controller:
    """High-level async controller for the ESP32 stepper engine.

    MIGRATION: This class unifies the BBB PickupController + DaemonClient.
    On BBB, the daemon owned state (homing_active, winding_mode, accel params).
    Here, the ESP32 owns that state; the controller is stateless.

    All commands are fire-and-forget via SPI; status is polled or pushed
    via periodic get_status() calls.

    Args:
        transport: Open SpiTransport instance.
        axes: Dict of axis_id → Axis for unit conversions.
        poll_interval: Status polling interval in seconds.
    """

    def __init__(
        self,
        transport: SpiTransport,
        axes: dict[int, Axis],
        poll_interval: float = 0.01,
    ) -> None:
        self._spi = transport
        self._axes = axes
        self._poll_interval = poll_interval
        self._event_callbacks: list[EventCallback] = []
        self._polling_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._last_status: Optional[MachineStatus] = None

    # ── Lifecycle ────────────────────────────────────────────────────────────

    async def start_polling(self) -> None:
        """Start background status polling task."""
        if self._polling_task is not None:
            return
        self._polling_task = asyncio.create_task(self._poll_loop())
        logger.info("Status polling started (%.0f ms interval)",
                     self._poll_interval * 1000)

    async def stop_polling(self) -> None:
        """Stop background status polling."""
        if self._polling_task is not None:
            self._polling_task.cancel()
            try:
                await self._polling_task
            except asyncio.CancelledError:
                pass
            self._polling_task = None

    def on_event(self, callback: EventCallback) -> None:
        """Register an event callback.

        Called when an event is detected in the status frame.
        Signature: async callback(event_type, event_axis, full_status)
        """
        self._event_callbacks.append(callback)

    # ── Commands ─────────────────────────────────────────────────────────────

    async def enable(self, axis: AxisId = AxisId.ALL, enabled: bool = True) -> MachineStatus:
        """Enable or disable stepper drivers.

        MIGRATION: On BBB, enable sent axis=0xFF for all.  Same here.
        """
        cmd = encode_cmd_u32(CmdOpcode.ENABLE, axis, 1 if enabled else 0)
        return await self._send(cmd)

    async def disable(self, axis: AxisId = AxisId.ALL) -> MachineStatus:
        """Disable stepper drivers."""
        return await self.enable(axis, False)

    async def emergency_stop(self) -> MachineStatus:
        """Emergency stop — immediate halt of all axes.

        MIGRATION: Identical to BBB e_stop command.
        """
        cmd = encode_cmd(CmdOpcode.ESTOP)
        return await self._send(cmd)

    async def set_speed(
        self,
        axis: AxisId,
        *,
        hz: Optional[int] = None,
        rpm: Optional[float] = None,
        reverse: bool = False,
    ) -> MachineStatus:
        """Set axis speed.

        MIGRATION: On BBB, set_speed sent Hz and the daemon converted to
        IEP intervals with auto-ramping.  Here the ESP32 handles ramping
        internally; we just send the target Hz.

        Args:
            axis: Target axis.
            hz: Step frequency (direct).
            rpm: RPM (converted to Hz via axis config).
            reverse: Direction flag.
        """
        if rpm is not None:
            ax = self._axes.get(int(axis))
            if ax is None:
                raise ValueError(f"Unknown axis {axis}")
            target_hz = ax.rpm_to_hz(rpm)
        elif hz is not None:
            target_hz = hz
        else:
            raise ValueError("Must specify hz or rpm")

        flags = CmdFlags.DIR_REVERSE if reverse else CmdFlags.NONE
        cmd = encode_cmd_u32(CmdOpcode.SET_SPEED, axis, target_hz, flags)
        return await self._send(cmd)

    async def stop(self, axis: AxisId = AxisId.ALL) -> MachineStatus:
        """Controlled deceleration stop."""
        cmd = encode_cmd(CmdOpcode.STOP, axis)
        return await self._send(cmd)

    async def move_to(
        self,
        axis: AxisId,
        position: int,
    ) -> MachineStatus:
        """Move axis to absolute position (steps).

        MIGRATION: On BBB, move_to accepted start_hz/max_hz/accel_steps.
        Here the ESP32 uses its internal accel profile.
        """
        cmd = encode_cmd(CmdOpcode.MOVE_ABS, axis, position)
        return await self._send(cmd)

    async def move_to_mm(
        self,
        axis: AxisId,
        mm: float,
    ) -> MachineStatus:
        """Move axis to absolute position (millimeters)."""
        ax = self._axes.get(int(axis))
        if ax is None:
            raise ValueError(f"Unknown axis {axis}")
        steps = ax.mm_to_steps(mm)
        return await self.move_to(axis, steps)

    async def move_relative(
        self,
        axis: AxisId,
        delta: int,
    ) -> MachineStatus:
        """Move axis by relative distance (steps)."""
        cmd = encode_cmd(CmdOpcode.MOVE_REL, axis, delta)
        return await self._send(cmd)

    async def home(self, axis: AxisId) -> MachineStatus:
        """Start homing sequence for axis.

        MIGRATION: On BBB, home_start triggered the PRU0 homing FSM.
        Here the ESP32 handles the full sequence internally.
        """
        cmd = encode_cmd(CmdOpcode.HOME, axis)
        return await self._send(cmd)

    async def set_accel(self, axis: AxisId, steps_per_s2: int) -> MachineStatus:
        """Set acceleration profile.

        MIGRATION: On BBB, set_accel tuned daemon-side ramp params.
        Here it sets the ESP32's per-axis accel directly.
        """
        cmd = encode_cmd_u32(CmdOpcode.SET_ACCEL, axis, steps_per_s2)
        return await self._send(cmd)

    async def set_mode(self, winding: bool = False) -> MachineStatus:
        """Set winding mode (axis synchronization).

        MIGRATION: On BBB, set_mode was daemon-only (no IPC to PRU).
        Here it's sent to ESP32 which enables bobbin-lateral sync.
        """
        cmd = encode_cmd_u32(CmdOpcode.SET_MODE, AxisId.ALL, 1 if winding else 0)
        return await self._send(cmd)

    async def set_limits(
        self,
        axis: AxisId,
        min_steps: Optional[int] = None,
        max_steps: Optional[int] = None,
    ) -> None:
        """Set software position limits.

        MIGRATION: On BBB, set_limits sent both min/max in one command.
        Here we send them separately (one per SPI frame).
        """
        if min_steps is not None:
            cmd = encode_cmd(CmdOpcode.SET_LIMITS, axis, min_steps, CmdFlags.NONE)
            await self._send(cmd)
        if max_steps is not None:
            cmd = encode_cmd(CmdOpcode.SET_LIMITS, axis, max_steps, CmdFlags.LIMIT_MAX)
            await self._send(cmd)

    async def reset_position(self, axis: AxisId = AxisId.ALL) -> MachineStatus:
        """Reset position counters to zero.

        MIGRATION: Identical to BBB reset_pos.
        """
        cmd = encode_cmd(CmdOpcode.RESET_POS, axis)
        return await self._send(cmd)

    async def ack_event(self, axis: AxisId = AxisId.ALL) -> MachineStatus:
        """Acknowledge pending event.

        MIGRATION: Identical to BBB ack_event.
        """
        cmd = encode_cmd(CmdOpcode.ACK_EVENT, axis)
        return await self._send(cmd)

    async def get_status(self) -> MachineStatus:
        """Poll current machine status."""
        return await self._send_nop()

    @property
    def last_status(self) -> Optional[MachineStatus]:
        """Most recent status from polling."""
        return self._last_status

    # ── Wait helpers ─────────────────────────────────────────────────────────

    async def wait_for_event(
        self,
        event: EventType,
        axis: Optional[AxisId] = None,
        timeout: float = 30.0,
    ) -> MachineStatus:
        """Wait for a specific event, polling status.

        MIGRATION: On BBB, events came as JSON over the socket.
        Here we poll SPI status frames and check event_type.
        """
        deadline = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < deadline:
            status = await self.get_status()
            if (
                status.event_type == event
                and (axis is None or status.event_axis == int(axis))
            ):
                return status
            await asyncio.sleep(self._poll_interval)
        raise TimeoutError(
            f"Timeout waiting for event {event.name} "
            f"(axis={axis}, timeout={timeout}s)"
        )

    async def wait_idle(
        self,
        axis: Optional[AxisId] = None,
        timeout: float = 30.0,
    ) -> MachineStatus:
        """Wait for axis to stop moving."""
        deadline = asyncio.get_event_loop().time() + timeout
        while asyncio.get_event_loop().time() < deadline:
            status = await self.get_status()
            if axis is not None:
                ax_status = status.axes[int(axis)]
                if not ax_status.is_moving:
                    return status
            elif not status.any_moving:
                return status
            await asyncio.sleep(self._poll_interval)
        raise TimeoutError(f"Timeout waiting for idle (timeout={timeout}s)")

    # ── Internal ─────────────────────────────────────────────────────────────

    async def _send(self, cmd: bytes) -> MachineStatus:
        """Send command via SPI (in thread pool to avoid blocking)."""
        status = await asyncio.to_thread(self._spi.send_command, cmd)
        self._last_status = status
        return status

    async def _send_nop(self) -> MachineStatus:
        """Send NOP to read status."""
        status = await asyncio.to_thread(self._spi.get_status)
        self._last_status = status
        return status

    async def _poll_loop(self) -> None:
        """Background polling loop — detects events and calls callbacks."""
        prev_event = EventType.NONE
        while True:
            try:
                status = await self._send_nop()

                # Detect new events
                if status.has_event and status.event_type != prev_event:
                    prev_event = status.event_type
                    for cb in self._event_callbacks:
                        try:
                            await cb(
                                status.event_type,
                                status.event_axis,
                                status,
                            )
                        except Exception:
                            logger.exception("Event callback error")
                elif not status.has_event:
                    prev_event = EventType.NONE

            except Exception:
                logger.exception("Polling error")

            await asyncio.sleep(self._poll_interval)
