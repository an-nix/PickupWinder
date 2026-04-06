#!/usr/bin/env python3
"""Surveille l'état de la pin P9_23 sur BeagleBone Black via libgpiod.

Ce script utilise uniquement libgpiod et ne lit pas le debugfs pinctrl.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

DEFAULT_LINE_OFFSET = 17
DEFAULT_CHIP_ROOT = Path("/dev")


def find_gpiochip(chip_path: Path | None = None) -> Path | None:
    if chip_path is not None and chip_path.exists():
        return chip_path

    for dev in sorted(DEFAULT_CHIP_ROOT.glob("gpiochip*")):
        return dev
    return None


def read_gpio_value(chip_path: Path | None, line_offset: int) -> tuple[int | None, str | None]:
    try:
        import gpiod
    except ImportError:
        return None, "libgpiod not installed"

    chip = None
    try:
        if chip_path is None:
            chip_path = find_gpiochip(None)
            if chip_path is None:
                return None, "no gpiochip found"
        chip = gpiod.Chip(str(chip_path))
        line = chip.get_line(line_offset)
        line.request(consumer="p9_23_monitor", type=gpiod.LINE_REQ_DIR_IN)
        return line.get_value(), None
    except Exception as exc:
        return None, str(exc)
    finally:
        if chip is not None:
            chip.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Surveille l'état de la pin P9_23 via libgpiod.")
    parser.add_argument("--interval", type=float, default=1.0, help="Intervalle de rafraîchissement en secondes")
    parser.add_argument("--count", type=int, default=0, help="Nombre d'itérations (0 = infini)")
    parser.add_argument("--chip", type=Path, default=None, help="Chemin du gpiochip à utiliser (ex: /dev/gpiochip512)")
    parser.add_argument("--line", type=int, default=DEFAULT_LINE_OFFSET, help="Offset de ligne dans le gpiochip (par défaut 17 pour P9_23)")
    args = parser.parse_args()

    print("P9_23 libgpiod monitor started")
    print(f"gpio chip: {args.chip or 'auto-detect'}")
    print(f"gpio line offset: {args.line}")
    print()

    iteration = 0
    while args.count == 0 or iteration < args.count:
        gpio_value, gpio_err = read_gpio_value(args.chip, args.line)

        if gpio_value is None:
            print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] gpio libgpiod: unavailable ({gpio_err})")
        else:
            print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] P9_23 gpio value: {gpio_value}")

        print("---")
        iteration += 1
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
