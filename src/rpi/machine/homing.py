"""homing.py — Homing sequences for all axes.

MIGRATION: On BBB, homing was implemented in PRU0 orchestrator as a
state machine (IDLE → APPROACH → HIT), triggered via HOST_CMD_HOME.
The C daemon forwarded the home_start JSON command and waited for
the home_complete event.

On RPi+ESP32, homing is executed on the ESP32 side (endstop.cpp)
via CMD_HOME. This module provides high-level orchestration around
the raw home command: sequencing, timeouts, error recovery.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass

from hal import ESP32Controller, AxisId, EventType

logger = logging.getLogger(__name__)


@dataclass
class HomingConfig:
    """Homing parameters per axis.

    Attributes:
        axis: Which axis to home.
        approach_hz: Speed during homing approach.
        backoff_hz: Speed during backoff move.
        backoff_mm: Distance to back off after hitting endstop.
        timeout_s: Maximum time to wait for home_complete event.
        steps_per_mm: Conversion factor for backoff distance.
    """
    axis: AxisId = AxisId.LATERAL
    approach_hz: int = 2000
    backoff_hz: int = 500
    backoff_mm: float = 2.0
    timeout_s: float = 30.0
    steps_per_mm: float = 3072.0


# Default homing configs
LATERAL_HOME = HomingConfig(
    axis=AxisId.LATERAL,
    approach_hz=2000,
    backoff_hz=500,
    backoff_mm=2.0,
    timeout_s=30.0,
    steps_per_mm=3072.0,
)


async def home_axis(
    controller: ESP32Controller,
    config: HomingConfig | None = None,
) -> bool:
    """Execute a homing sequence for one axis.

    The actual homing motion (approach → hit → backoff → reset position)
    is performed by the ESP32 firmware (endstop.cpp). This function:
    1. Sends CMD_HOME to the ESP32
    2. Waits for HOME_COMPLETE event
    3. Acknowledges the event
    4. Resets position to 0

    Returns True on success, False on timeout or error.

    MIGRATION: On BBB, the daemon sent HOME_START → PRU0 homing FSM ran
    → home_complete event → daemon broadcast. Here the ESP32 does it all.
    """
    cfg = config or LATERAL_HOME

    logger.info("Homing axis %s (approach=%d Hz, timeout=%.0fs)",
               cfg.axis.name, cfg.approach_hz, cfg.timeout_s)

    try:
        # Enable the axis first
        await controller.enable(cfg.axis)

        # Send home command — ESP32 firmware runs the full sequence
        await controller.home(cfg.axis)

        # Wait for home_complete event from ESP32
        event = await controller.wait_for_event(
            EventType.HOME_COMPLETE,
            cfg.axis,
            timeout=cfg.timeout_s,
        )

        if event is None:
            logger.error("Homing timeout on axis %s", cfg.axis.name)
            await controller.emergency_stop()
            return False

        # Acknowledge the event
        await controller.ack_event(cfg.axis)

        # Position is reset to 0 by ESP32 firmware after homing
        logger.info("Axis %s homed successfully", cfg.axis.name)
        return True

    except Exception as e:
        logger.exception("Homing failed on axis %s: %s", cfg.axis.name, e)
        await controller.emergency_stop()
        return False


async def home_all(
    controller: ESP32Controller,
    configs: list[HomingConfig] | None = None,
) -> bool:
    """Home all configured axes sequentially.

    Axes are homed one at a time in the order provided.
    If any axis fails, remaining axes are skipped.

    Returns True if all axes homed successfully.
    """
    if configs is None:
        # Default: only lateral needs homing
        configs = [LATERAL_HOME]

    for cfg in configs:
        success = await home_axis(controller, cfg)
        if not success:
            logger.error("Homing sequence aborted at axis %s", cfg.axis.name)
            return False

    logger.info("All %d axes homed successfully", len(configs))
    return True
