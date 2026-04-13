"""main.py — Asyncio entry point for the PickupWinder RPi application.

Usage:
    python -m main                     # Normal operation
    python -m main --config path.yaml  # Custom config
    python -m main --test              # Quick smoke test
    python -m main --dry-run           # Use mock SPI (no hardware)

MIGRATION: On BBB, the daemon was a C process and the Python layer
was pickup_test.py + pru_client.py over Unix socket.  Here Python
is the sole host application, talking to ESP32 over SPI.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from pathlib import Path
from typing import Any

import yaml

from hal import ESP32Controller, SpiTransport, Axis, AxisId, EventType
from hal.axis import BOBBIN_DEFAULTS, LATERAL_DEFAULTS, TENSIONER_DEFAULTS
from machine import (
    CoilWinder,
    WindingProgram,
    WindingState,
    Tensioner,
    TensionConfig,
    home_axis,
    HomingConfig,
    LATERAL_HOME,
)

logger = logging.getLogger("pickup_winder")

CONFIG_DIR = Path(__file__).parent / "config"
DEFAULT_CONFIG = CONFIG_DIR / "machine_config.yaml"


def load_config(path: Path) -> dict[str, Any]:
    """Load YAML machine configuration."""
    with open(path) as f:
        return yaml.safe_load(f)


def build_controller(cfg: dict[str, Any], dry_run: bool = False) -> ESP32Controller:
    """Build the ESP32Controller from config.

    In dry_run mode, uses a mock transport (no hardware needed).
    """
    spi_cfg = cfg.get("spi", {})

    if dry_run:
        from tests.mock_esp32 import MockSpiTransport
        transport = MockSpiTransport()
    else:
        transport = SpiTransport(
            bus=spi_cfg.get("bus", 0),
            device=spi_cfg.get("device", 0),
            speed_hz=spi_cfg.get("speed_hz", 4_000_000),
            mode=spi_cfg.get("mode", 0),
        )
        transport.open()

    axes = {
        AxisId.BOBBIN: Axis(BOBBIN_DEFAULTS),
        AxisId.LATERAL: Axis(LATERAL_DEFAULTS),
        AxisId.TENSIONER: Axis(TENSIONER_DEFAULTS),
    }

    controller = ESP32Controller(transport, axes)
    return controller


def build_winding_program(
    cfg: dict[str, Any],
    preset: str | None = None,
) -> WindingProgram:
    """Build a WindingProgram from config presets."""
    if preset is None:
        preset = "strat"

    presets = cfg.get("bobbin_presets", {})
    p = presets.get(preset, presets.get("strat", {}))

    total_mm = p.get("total_mm", 17.0)
    flange_l = p.get("flange_left_mm", 1.5)
    flange_r = p.get("flange_right_mm", 1.5)
    width = total_mm - flange_l - flange_r

    return WindingProgram(
        target_turns=p.get("default_turns", 8000),
        wire_gauge_mm=p.get("wire_mm", 0.071),
        bobbin_width_mm=width,
        winding_rpm=1000.0,
        start_offset_mm=flange_l,
    )


async def smoke_test(controller: ESP32Controller) -> None:
    """Quick smoke test: enable, read status, disable.

    MIGRATION: Equivalent to BBB pickup_test.py test_enable().
    """
    logger.info("=== Smoke Test ===")

    # Enable all axes
    await controller.enable(AxisId.ALL)
    await asyncio.sleep(0.1)

    # Read status
    status = await controller.get_status()
    logger.info("Status: flags=0x%02X, uptime=%d ms",
               status.global_flags, status.uptime_ms)
    for i, ax in enumerate(status.axes):
        logger.info("  Axis %d: state=%d pos=%d speed=%d",
                    i, ax.state, ax.position, ax.current_speed)

    # Emergency stop
    await controller.emergency_stop()
    await asyncio.sleep(0.1)

    # Verify stopped
    status = await controller.get_status()
    logger.info("After e-stop: flags=0x%02X", status.global_flags)

    # Disable
    await controller.disable(AxisId.ALL)
    logger.info("=== Smoke Test Complete ===")


async def run_winding(
    controller: ESP32Controller,
    cfg: dict[str, Any],
    preset: str = "strat",
) -> None:
    """Run a full winding session."""
    program = build_winding_program(cfg, preset)

    axes_cfg = cfg.get("axes", {})
    lat_cfg = axes_cfg.get("lateral", {})

    winder = CoilWinder(
        controller,
        bobbin_steps_per_rev=axes_cfg.get("bobbin", {}).get("steps_per_rev", 6400),
        lateral_steps_per_mm=lat_cfg.get("steps_per_mm", 3072.0),
    )

    # Progress callback
    async def on_progress(current: int, target: int, state: WindingState) -> None:
        pct = 100.0 * current / target if target > 0 else 0
        logger.info("Progress: %d/%d turns (%.1f%%) [%s]",
                    current, target, pct, state.name)

    winder.on_progress(on_progress)

    success = await winder.run(program)
    if success:
        logger.info("Winding completed successfully!")
    else:
        logger.info("Winding was interrupted.")


async def async_main(args: argparse.Namespace) -> int:
    """Async entry point."""
    cfg = load_config(Path(args.config))
    controller = build_controller(cfg, dry_run=args.dry_run)

    # Graceful shutdown on SIGINT/SIGTERM
    loop = asyncio.get_running_loop()
    shutdown_event = asyncio.Event()

    def signal_handler() -> None:
        logger.info("Shutdown requested")
        shutdown_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    try:
        # Start polling
        await controller.start_polling()

        if args.test:
            await smoke_test(controller)
            return 0

        await run_winding(controller, cfg, preset=args.preset)
        return 0

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 1
    finally:
        await controller.stop_polling()
        if hasattr(controller, '_spi'):
            controller._spi.close()


def main() -> None:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="PickupWinder — Guitar pickup coil winding controller"
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to machine_config.yaml",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run quick smoke test only",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Use mock SPI transport (no hardware)",
    )
    parser.add_argument(
        "--preset",
        default="strat",
        choices=["strat", "telecaster", "p90", "humbucker", "jazzmaster"],
        help="Bobbin preset (default: strat)",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable debug logging",
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )

    sys.exit(asyncio.run(async_main(args)))


if __name__ == "__main__":
    main()
