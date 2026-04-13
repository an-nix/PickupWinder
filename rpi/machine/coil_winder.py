"""coil_winder.py — High-level winding logic.

Orchestrates a complete pickup winding program: turns count, layers,
wire pitch, lateral traverse synchronization.

MIGRATION: This module replaces the planned (but not yet implemented)
BBB classes: WinderStateMachine, WindingLayer, WindingPattern, RecipeRunner.
The BBB had only stubs for these in core/__init__.py exports.
This is a clean implementation using the new ESP32 HAL.

State machine:
    IDLE → HOMING → READY → WINDING → PAUSED → WINDING → ... → COMPLETE → IDLE
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Callable, Awaitable

from ..hal import ESP32Controller, AxisId, EventType, MachineStatus

logger = logging.getLogger(__name__)


class WindingState(Enum):
    """Winding state machine states."""
    IDLE = auto()
    HOMING = auto()
    READY = auto()
    WINDING = auto()
    PAUSED = auto()
    STOPPING = auto()
    COMPLETE = auto()
    ERROR = auto()


@dataclass
class WindingProgram:
    """Parameters for a winding program.

    MIGRATION: On BBB, these would have been in a Recipe JSON.
    Here we use a typed dataclass.

    Attributes:
        target_turns: Total number of bobbin turns to wind.
        wire_gauge_mm: Wire diameter in millimeters.
        bobbin_width_mm: Usable winding width (total - flanges).
        winding_rpm: Bobbin rotation speed during winding.
        approach_rpm: Slow approach speed for initial positioning.
        layers: Expected number of layers (auto-computed if 0).
        start_offset_mm: Distance from home to start winding.
    """
    target_turns: int = 8000
    wire_gauge_mm: float = 0.071    # AWG 42
    bobbin_width_mm: float = 14.0   # Strat: 17 - 1.5 - 1.5
    winding_rpm: float = 1000.0
    approach_rpm: float = 50.0
    layers: int = 0                  # 0 = auto-compute
    start_offset_mm: float = 1.5    # flange offset

    @property
    def turns_per_layer(self) -> float:
        """Compute turns per layer from bobbin width and wire gauge."""
        if self.wire_gauge_mm <= 0:
            return 0
        return self.bobbin_width_mm / self.wire_gauge_mm

    @property
    def computed_layers(self) -> int:
        """Auto-compute layer count from target turns."""
        tpl = self.turns_per_layer
        if tpl <= 0:
            return 1
        return max(1, int(self.target_turns / tpl) + 1)

    @property
    def lateral_pitch_mm(self) -> float:
        """Wire pitch = wire diameter (tight winding)."""
        return self.wire_gauge_mm


# Type alias for progress callbacks
ProgressCallback = Callable[[int, int, WindingState], Awaitable[None]]


class CoilWinder:
    """Orchestrates a complete winding program.

    MIGRATION: On BBB, this logic was split across multiple planned modules
    (WinderStateMachine, WindingLayer, etc.) that were never implemented.
    This is a complete implementation.

    The CoilWinder:
    1. Homes the lateral axis
    2. Positions at start offset
    3. Runs the winding loop (synchronizing bobbin + lateral)
    4. Reverses lateral at layer boundaries
    5. Counts turns and layers
    6. Stops at target

    Args:
        controller: ESP32Controller for hardware commands.
        bobbin_steps_per_rev: Full steps per bobbin revolution.
        lateral_steps_per_mm: Steps per millimeter of lateral travel.
    """

    def __init__(
        self,
        controller: ESP32Controller,
        bobbin_steps_per_rev: int = 6400,
        lateral_steps_per_mm: float = 3072.0,
    ) -> None:
        self._ctrl = controller
        self._sp_steps_rev = bobbin_steps_per_rev
        self._lat_steps_mm = lateral_steps_per_mm
        self._state = WindingState.IDLE
        self._program: Optional[WindingProgram] = None
        self._turn_count = 0
        self._layer_count = 0
        self._lateral_direction = 1  # 1 = forward, -1 = reverse
        self._progress_callbacks: list[ProgressCallback] = []
        self._stop_requested = False

    # ── Properties ───────────────────────────────────────────────────────────

    @property
    def state(self) -> WindingState:
        return self._state

    @property
    def turn_count(self) -> int:
        return self._turn_count

    @property
    def layer_count(self) -> int:
        return self._layer_count

    @property
    def program(self) -> Optional[WindingProgram]:
        return self._program

    def on_progress(self, callback: ProgressCallback) -> None:
        """Register a progress callback.

        Called after each turn with (current_turns, target_turns, state).
        """
        self._progress_callbacks.append(callback)

    # ── Main winding sequence ────────────────────────────────────────────────

    async def run(self, program: WindingProgram) -> bool:
        """Execute a complete winding program.

        Returns True if winding completed successfully, False if interrupted.

        MIGRATION: On BBB, the winding loop would have been in WinderApp
        (never implemented).  This is a complete asyncio implementation.
        """
        self._program = program
        self._turn_count = 0
        self._layer_count = 0
        self._lateral_direction = 1
        self._stop_requested = False

        try:
            # Step 1: Home lateral axis
            self._state = WindingState.HOMING
            logger.info("Homing lateral axis...")
            await self._ctrl.home(AxisId.LATERAL)
            await self._ctrl.wait_for_event(EventType.HOME_COMPLETE,
                                            AxisId.LATERAL, timeout=30)
            await self._ctrl.ack_event(AxisId.LATERAL)
            logger.info("Lateral axis homed")

            # Step 2: Enable all axes
            await self._ctrl.enable(AxisId.ALL)

            # Step 3: Position at start offset
            start_steps = int(program.start_offset_mm * self._lat_steps_mm)
            logger.info("Moving to start position: %.1f mm (%d steps)",
                       program.start_offset_mm, start_steps)
            await self._ctrl.move_to(AxisId.LATERAL, start_steps)
            await self._ctrl.wait_for_event(EventType.MOVE_COMPLETE,
                                            AxisId.LATERAL, timeout=10)
            await self._ctrl.ack_event(AxisId.LATERAL)

            # Step 4: Set winding mode (enables axis sync on ESP32)
            await self._ctrl.set_mode(winding=True)

            # Step 5: Start winding loop
            self._state = WindingState.WINDING
            logger.info("Starting winding: %d turns at %.0f RPM, "
                       "wire %.3f mm, bobbin width %.1f mm",
                       program.target_turns, program.winding_rpm,
                       program.wire_gauge_mm, program.bobbin_width_mm)

            success = await self._winding_loop(program)

            if success:
                self._state = WindingState.COMPLETE
                logger.info("Winding complete: %d turns, %d layers",
                           self._turn_count, self._layer_count)
            else:
                self._state = WindingState.IDLE
                logger.info("Winding stopped at %d turns", self._turn_count)

            return success

        except Exception as e:
            self._state = WindingState.ERROR
            logger.exception("Winding error: %s", e)
            await self._ctrl.emergency_stop()
            return False

        finally:
            await self._ctrl.set_mode(winding=False)
            await self._ctrl.stop(AxisId.BOBBIN)

    async def pause(self) -> None:
        """Pause winding (decelerate to stop)."""
        if self._state == WindingState.WINDING:
            self._state = WindingState.PAUSED
            await self._ctrl.stop(AxisId.BOBBIN)
            await self._ctrl.stop(AxisId.LATERAL)
            logger.info("Winding paused at turn %d", self._turn_count)

    async def resume(self) -> None:
        """Resume winding from paused state."""
        if self._state == WindingState.PAUSED:
            self._state = WindingState.WINDING
            logger.info("Winding resumed")

    def request_stop(self) -> None:
        """Request graceful stop (completes current turn, then stops)."""
        self._stop_requested = True

    # ── Internal winding loop ────────────────────────────────────────────────

    async def _winding_loop(self, program: WindingProgram) -> bool:
        """Core winding loop: one iteration per layer traverse."""
        turns_per_layer = int(program.turns_per_layer)
        traverse_mm = program.bobbin_width_mm
        traverse_steps = int(traverse_mm * self._lat_steps_mm)

        # Compute lateral speed from bobbin RPM and wire pitch
        # lateral_hz = bobbin_hz * wire_pitch_mm * lat_steps_mm / sp_steps_rev
        bobbin_hz = int(program.winding_rpm * self._sp_steps_rev / 60)
        lateral_hz = int(
            bobbin_hz * program.lateral_pitch_mm
            * self._lat_steps_mm / self._sp_steps_rev
        )
        lateral_hz = max(100, min(lateral_hz, 160_000))

        logger.info("Bobbin: %d Hz, Lateral: %d Hz, %d turns/layer",
                    bobbin_hz, lateral_hz, turns_per_layer)

        while self._turn_count < program.target_turns:
            if self._stop_requested:
                return False

            # Wait if paused
            while self._state == WindingState.PAUSED:
                await asyncio.sleep(0.1)
                if self._stop_requested:
                    return False

            # Start bobbin rotation
            await self._ctrl.set_speed(AxisId.BOBBIN, hz=bobbin_hz)

            # Start lateral traverse
            status = await self._ctrl.get_status()
            current_lat_pos = status.axes[int(AxisId.LATERAL)].position

            if self._lateral_direction > 0:
                target = current_lat_pos + traverse_steps
            else:
                target = current_lat_pos - traverse_steps

            await self._ctrl.set_speed(
                AxisId.LATERAL,
                hz=lateral_hz,
                reverse=(self._lateral_direction < 0),
            )
            await self._ctrl.move_to(AxisId.LATERAL, target)

            # Wait for traverse to complete (one layer)
            await self._ctrl.wait_for_event(
                EventType.MOVE_COMPLETE, AxisId.LATERAL, timeout=60
            )
            await self._ctrl.ack_event(AxisId.LATERAL)

            # Update counters
            self._turn_count += turns_per_layer
            self._layer_count += 1
            self._lateral_direction *= -1  # reverse for next layer

            # Notify progress
            for cb in self._progress_callbacks:
                try:
                    await cb(self._turn_count, program.target_turns, self._state)
                except Exception:
                    logger.exception("Progress callback error")

            logger.info("Layer %d complete: %d/%d turns",
                       self._layer_count, self._turn_count,
                       program.target_turns)

        return True
