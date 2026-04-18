from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# Ensure the parent directory is added to sys.path for proper imports
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from motion.axis import Axis
from motion.homing import LateralHomingConfig, home_lateral_axis
from transport import Esp32SpiTransport
from transport import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Test du homing latéral via SPI")
    parser.add_argument("--bus", type=int, default=0, help="Bus SPI à utiliser (par défaut 0)")
    parser.add_argument("--device", type=int, default=0, help="Device SPI à utiliser (par défaut 0)")
    parser.add_argument("--speed-hz", type=int, default=4_000_000, help="Vitesse SPI en Hz")
    parser.add_argument("--steps-per-attempt", type=int, default=32, help="Nombre de pas envoyés par tentative de homing")
    parser.add_argument(
        "--duration-us",
        type=int,
        default=50_000,
        help="Durée d'une tentative de homing en microsecondes (max 65535)",
    )
    parser.add_argument("--max-attempts", type=int, default=200, help="Nombre maximal de tentatives de homing")
    parser.add_argument("--poll-interval-s", type=float, default=0.02, help="Intervalle de sondage SPI pendant le homing")
    parser.add_argument("--reverse", action="store_true", help="Inverser la direction de la tentative de homing")
    parser.add_argument("--no-status", action="store_true", help="Ne pas afficher le statut initial/final")
    return parser


def format_endstop(state: int) -> str:
    if state == LATERAL_ENDSTOP_ABSENT:
        return "ABSENT/FAULT"
    if state == LATERAL_ENDSTOP_PRESENT_CLOSED:
        return "CLOSED"
    if state == LATERAL_ENDSTOP_PRESENT_OPEN:
        return "OPEN"
    return f"UNKNOWN(0x{state:02X})"


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    config = LateralHomingConfig(
        axis=Axis(axis_id=1, name="lateral", can_move_without_homing=False),
        steps_per_attempt=args.steps_per_attempt,
        duration_us=args.duration_us,
        max_attempts=args.max_attempts,
        poll_interval_s=args.poll_interval_s,
        reverse=args.reverse,
    )

    with Esp32SpiTransport(bus=args.bus, device=args.device, speed_hz=args.speed_hz, mode=0) as transport:
        try:
            if not args.no_status:
                status = transport.get_status()
                print("Statut initial:")
                print(f"  endstop lateral = {format_endstop(status.lateral_endstop_state)}")
                print(f"  queue_free = {status.queue_free_slots}")
                print(f"  running_mask = 0x{status.running_mask:02X}")
                print(f"  enabled_mask = 0x{status.enabled_mask:02X}")

            print("Lancement du homing lateral...")
            home_lateral_axis(transport, config)
            print("Homing terminé avec succès.")

            if not args.no_status:
                status = transport.get_status()
                print("Statut final:")
                print(f"  endstop lateral = {format_endstop(status.lateral_endstop_state)}")
                print(f"  last_executed_sequence = {status.last_executed_sequence}")
        except Exception as exc:
            print(f"Homing échoué : {exc}")
            sys.exit(1)


if __name__ == "__main__":
    main()
