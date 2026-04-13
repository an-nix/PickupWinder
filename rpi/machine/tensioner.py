"""tensioner.py — Wire tension control for axis 2 (Tensioner).

MIGRATION: The BBB had no tensioner axis. This is new hardware
on the RPi+ESP32 platform. Axis 2 runs a small stepper to maintain
constant tension via a dancer arm / load cell feedback loop.

Simple PID-like control with configurable setpoint and limits.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

from ..hal import ESP32Controller, AxisId

logger = logging.getLogger(__name__)


@dataclass
class TensionConfig:
    """Tension control parameters.

    Attributes:
        target_grams: Desired wire tension in grams.
        max_grams: Emergency stop threshold.
        kp: Proportional gain (Hz per gram error).
        ki: Integral gain (Hz per gram·second error).
        max_hz: Maximum tensioner motor speed.
        min_hz: Minimum tensioner motor speed (0 = stopped).
        update_interval_s: Control loop period in seconds.
    """
    target_grams: float = 50.0
    max_grams: float = 200.0
    kp: float = 10.0
    ki: float = 0.5
    max_hz: int = 5000
    min_hz: int = 0
    update_interval_s: float = 0.05   # 20 Hz control loop


class Tensioner:
    """Wire tension controller.

    Runs a background asyncio task that reads tension feedback
    and adjusts tensioner motor speed to maintain setpoint.

    NOTE: Current implementation uses open-loop speed commands.
    Closed-loop (load cell or dancer position feedback) will be
    added when the HX711 or analog feedback path is implemented.
    """

    def __init__(
        self,
        controller: ESP32Controller,
        config: TensionConfig | None = None,
    ) -> None:
        self._ctrl = controller
        self._config = config or TensionConfig()
        self._task: asyncio.Task | None = None
        self._running = False
        self._integral = 0.0
        self._current_hz = 0

    @property
    def config(self) -> TensionConfig:
        return self._config

    @config.setter
    def config(self, value: TensionConfig) -> None:
        self._config = value

    @property
    def current_hz(self) -> int:
        return self._current_hz

    async def start(self) -> None:
        """Start the tension control loop."""
        if self._running:
            return
        self._running = True
        self._integral = 0.0
        await self._ctrl.enable(AxisId.TENSIONER)
        self._task = asyncio.create_task(self._control_loop())
        logger.info("Tensioner started (target=%.0f g)", self._config.target_grams)

    async def stop(self) -> None:
        """Stop the tension control loop and disable motor."""
        self._running = False
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        await self._ctrl.stop(AxisId.TENSIONER)
        self._current_hz = 0
        logger.info("Tensioner stopped")

    async def set_constant_speed(self, hz: int) -> None:
        """Set a constant tensioner speed (open-loop mode).

        Useful for testing or simple winding without feedback.
        """
        hz = max(self._config.min_hz, min(hz, self._config.max_hz))
        await self._ctrl.set_speed(AxisId.TENSIONER, hz=hz)
        self._current_hz = hz
        logger.debug("Tensioner constant speed: %d Hz", hz)

    async def _control_loop(self) -> None:
        """Background PID control loop.

        TODO: Read actual tension from HX711 load cell or analog
        dancer arm position sensor. Currently this is a placeholder
        that maintains a fixed speed proportional to target tension.
        """
        cfg = self._config

        try:
            while self._running:
                # Placeholder: compute speed from target tension
                # Real implementation will read sensor and apply PID
                target_hz = int(cfg.target_grams * cfg.kp)
                target_hz = max(cfg.min_hz, min(target_hz, cfg.max_hz))

                if target_hz != self._current_hz:
                    await self._ctrl.set_speed(AxisId.TENSIONER, hz=target_hz)
                    self._current_hz = target_hz

                await asyncio.sleep(cfg.update_interval_s)

        except asyncio.CancelledError:
            pass
        except Exception:
            logger.exception("Tensioner control loop error")
            await self._ctrl.emergency_stop()
