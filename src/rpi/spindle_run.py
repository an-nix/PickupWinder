#!/usr/bin/env python3
"""spindle_run.py — Pilotage de l'axe spindle (bobbin) avec profil d'accélération.

Permet de définir une vitesse cible (RPM ou Hz) et une courbe d'accélération,
puis d'exécuter une montée en vitesse, un palier, et un arrêt contrôlé.

La rampe est calculée côté RPi (host) et envoyée à l'ESP32 sous forme de
commandes SET_SPEED successives toutes les RAMP_TICK_MS millisecondes.
L'ESP32 reçoit uniquement des SET_SPEED ; il ne connaît pas le profil global.

Utilisation :
    python3 spindle_run.py                          # défauts (1000 RPM, 200 RPM/s)
    python3 spindle_run.py --rpm 1500 --accel 500   # 1500 RPM, montée 500 RPM/s
    python3 spindle_run.py --rpm 1500 --accel 500 --duration 10
    python3 spindle_run.py --reverse                # sens inverse
    python3 spindle_run.py --dry-run                # affiche le profil sans SPI
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from dataclasses import dataclass
from typing import List

# ── Chemin vers le package src/rpi ───────────────────────────────────────────
sys.path.insert(0, '/home/nicolas/Documents/PlatformIO/Projects/PickupWinder/src/rpi')

from hal.esp32_controller import ESP32Controller
from hal.spi_transport import SpiTransport
from hal.axis import Axis, AxisConfig
from hal.protocol import AxisId

# ── Constantes machine (bobbin) ───────────────────────────────────────────────

BOBBIN_STEPS_PER_REV: int = 200     # full steps
BOBBIN_MICROSTEPS:    int = 32      # microstepping factor
BOBBIN_PPR:           int = BOBBIN_STEPS_PER_REV * BOBBIN_MICROSTEPS  # 6400

HZ_MIN: int =   100    # 0.94 RPM — limite basse ESP32
HZ_MAX: int = 160_000  # 1500 RPM

RPM_MIN: float = HZ_MIN  * 60 / BOBBIN_PPR   # ≈ 0.94 RPM
RPM_MAX: float = HZ_MAX  * 60 / BOBBIN_PPR   # = 1500 RPM

# Résolution de la rampe host-side : un SET_SPEED toutes les N ms
RAMP_TICK_MS: int = 20   # 50 Hz de mise à jour de consigne

# ── Conversion ────────────────────────────────────────────────────────────────

def rpm_to_hz(rpm: float) -> int:
    """RPM → fréquence de step (Hz), clamped dans [HZ_MIN, HZ_MAX]."""
    hz = rpm * BOBBIN_PPR / 60.0
    return max(HZ_MIN, min(HZ_MAX, int(round(hz))))


def hz_to_rpm(hz: int) -> float:
    """Fréquence de step (Hz) → RPM."""
    return hz * 60.0 / BOBBIN_PPR


# ── Profil trapezoïdal ────────────────────────────────────────────────────────

@dataclass
class RampPoint:
    """Un point du profil de vitesse."""
    t_ms:   int    # instant depuis le début de la rampe [ms]
    hz:     int    # consigne fréquence step [Hz]
    rpm:    float  # idem en RPM (affichage)


def build_trapezoid(
    start_rpm:  float,
    target_rpm: float,
    accel_rpm_s: float,
    cruise_duration_s: float,
) -> List[RampPoint]:
    """Construit un profil trapézoïdal de vitesse.

    Phases :
      1. Montée  : start_rpm → target_rpm à accel_rpm_s RPM/s
      2. Palier  : target_rpm pendant cruise_duration_s secondes
      3. Descente: target_rpm → start_rpm à accel_rpm_s RPM/s

    Retourne une liste de RampPoint échantillonnés à RAMP_TICK_MS.

    Args:
        start_rpm       : vitesse de départ (≥ RPM_MIN pour un motor en cours)
                          mettre 0 pour partir de l'arrêt (clamped à HZ_MIN)
        target_rpm      : vitesse de consigne (≤ RPM_MAX)
        accel_rpm_s     : taux d'accélération  [RPM/s]
        cruise_duration_s : durée du palier à vitesse max  [s]
    """
    # Clamp
    start_rpm  = max(0.0, min(start_rpm,  RPM_MAX))
    target_rpm = max(0.0, min(target_rpm, RPM_MAX))
    accel_rpm_s = max(1.0, accel_rpm_s)

    ramp_up_s   = abs(target_rpm - start_rpm) / accel_rpm_s
    ramp_down_s = abs(target_rpm - start_rpm) / accel_rpm_s
    total_s     = ramp_up_s + cruise_duration_s + ramp_down_s

    points: List[RampPoint] = []
    tick_s = RAMP_TICK_MS / 1000.0
    n_ticks = math.ceil(total_s / tick_s) + 1

    for i in range(n_ticks):
        t = i * tick_s
        if t <= ramp_up_s:
            # Phase montée
            rpm = start_rpm + accel_rpm_s * t
        elif t <= ramp_up_s + cruise_duration_s:
            # Phase palier
            rpm = target_rpm
        else:
            # Phase descente
            elapsed_decel = t - ramp_up_s - cruise_duration_s
            rpm = target_rpm - accel_rpm_s * elapsed_decel

        rpm = max(0.0, min(rpm, RPM_MAX))
        hz  = rpm_to_hz(rpm) if rpm >= RPM_MIN else 0
        points.append(RampPoint(t_ms=int(round(t * 1000)), hz=hz, rpm=rpm))

    return points


def print_profile(points: List[RampPoint], title: str = "Profil de vitesse") -> None:
    """Affiche un résumé du profil."""
    print(f"\n{'─' * 58}")
    print(f"  {title}")
    print(f"{'─' * 58}")
    print(f"  {'t [ms]':>8}  {'RPM':>8}  {'Hz':>8}  {'barre'}")
    print(f"{'─' * 58}")
    bar_max = max((p.rpm for p in points), default=1.0)
    # N'affiche qu'un point sur 5 pour ne pas inonder le terminal
    step = max(1, len(points) // 20)
    for p in points[::step]:
        bar_len = int(p.rpm / bar_max * 30) if bar_max > 0 else 0
        bar = '█' * bar_len
        print(f"  {p.t_ms:>8}  {p.rpm:>8.1f}  {p.hz:>8}  {bar}")
    last = points[-1]
    if last not in points[::step]:
        bar_len = int(last.rpm / bar_max * 30) if bar_max > 0 else 0
        bar = '█' * bar_len
        print(f"  {last.t_ms:>8}  {last.rpm:>8.1f}  {last.hz:>8}  {bar}")
    print(f"{'─' * 58}")
    total_s = points[-1].t_ms / 1000 if points else 0
    print(f"  Durée totale : {total_s:.2f} s   Points : {len(points)}")
    print(f"  Tick période : {RAMP_TICK_MS} ms   PPR : {BOBBIN_PPR}")
    print(f"{'─' * 58}\n")


# ── Exécution du profil ───────────────────────────────────────────────────────

async def run_spindle_profile(
    ctrl: ESP32Controller,
    points: List[RampPoint],
    reverse: bool = False,
    verbose: bool = True,
) -> None:
    """Envoie le profil de vitesse à l'ESP32 en temps réel.

    Pour chaque point, envoie un SET_SPEED puis attend jusqu'au prochain tick.
    L'ESP32 effectue sa propre gestion interne du step ISR — cette boucle
    n'est que la source de consigne de vitesse.

    Args:
        ctrl    : contrôleur ESP32 connecté
        points  : profil calculé par build_trapezoid()
        reverse : True = sens antihoraire
        verbose : affiche la vitesse mesurée pendant l'exécution
    """
    if not points:
        return

    t0 = time.monotonic()
    last_print_s = 0.0

    for i, pt in enumerate(points):
        # Heure cible de ce point
        t_target = t0 + pt.t_ms / 1000.0
        now = time.monotonic()

        # Attendre jusqu'à l'heure du point (sans dériver)
        if now < t_target:
            await asyncio.sleep(t_target - now)

        # Arrêt propre si la consigne tombe à 0
        if pt.hz == 0:
            await ctrl.stop(AxisId.BOBBIN)
            if verbose:
                print(f"  [{pt.t_ms:>6} ms]  STOP → 0 RPM")
            continue

        # Envoi SET_SPEED (non bloquant côté asyncio — ~50 µs en thread)
        await ctrl.set_speed(AxisId.BOBBIN, hz=pt.hz, reverse=reverse)

        # Affichage de statut toutes les 500 ms
        if verbose:
            elapsed = time.monotonic() - t0
            if elapsed - last_print_s >= 0.5:
                last_print_s = elapsed
                try:
                    status = await ctrl.get_status()
                    ax = status.axes[int(AxisId.BOBBIN)]
                    actual_rpm = hz_to_rpm(ax.current_hz)
                    print(
                        f"  [{pt.t_ms:>6} ms]  consigne: {pt.rpm:>7.1f} RPM"
                        f"  mesuré: {actual_rpm:>7.1f} RPM"
                        f"  pos: {ax.position:>8}"
                    )
                except Exception:
                    pass


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Pilotage axe spindle — vitesse max + courbe accélération",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--rpm", type=float, default=1000.0,
        help="Vitesse de consigne max [RPM]",
    )
    p.add_argument(
        "--accel", type=float, default=200.0,
        help="Taux d'accélération et décélération [RPM/s]",
    )
    p.add_argument(
        "--duration", type=float, default=5.0,
        help="Durée du palier à vitesse max [s]",
    )
    p.add_argument(
        "--start-rpm", type=float, default=0.0,
        help="Vitesse de départ [RPM]  (0 = démarrage depuis l'arrêt)",
    )
    p.add_argument(
        "--reverse", action="store_true",
        help="Sens de rotation inverse",
    )
    p.add_argument(
        "--spi-device", default="/dev/spidev0.0",
        help="Périphérique SPI",
    )
    p.add_argument(
        "--spi-speed", type=int, default=4_000_000,
        help="Fréquence SPI [Hz]",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="Calcule et affiche le profil sans communiquer avec l'ESP32",
    )
    p.add_argument(
        "--no-verbose", action="store_true",
        help="Désactive l'affichage de la vitesse mesurée",
    )
    return p.parse_args()


async def main() -> int:
    args = parse_args()

    # ── Validation ────────────────────────────────────────────────────────────
    if args.rpm > RPM_MAX:
        print(f"[!] --rpm {args.rpm} dépasse la limite {RPM_MAX:.0f} RPM, clampé.")
        args.rpm = RPM_MAX
    if args.rpm < RPM_MIN:
        print(f"[!] --rpm {args.rpm} sous la limite {RPM_MIN:.2f} RPM, clampé.")
        args.rpm = RPM_MIN

    # ── Construction du profil ────────────────────────────────────────────────
    points = build_trapezoid(
        start_rpm=args.start_rpm,
        target_rpm=args.rpm,
        accel_rpm_s=args.accel,
        cruise_duration_s=args.duration,
    )

    direction_str = "INVERSE (CCW)" if args.reverse else "NORMAL (CW)"
    print_profile(
        points,
        title=(
            f"Spindle — {args.rpm:.0f} RPM  accel {args.accel:.0f} RPM/s  "
            f"palier {args.duration:.1f} s  {direction_str}"
        ),
    )

    if args.dry_run:
        print("[dry-run] Profil calculé — aucune communication SPI.")
        return 0

    # ── Connexion SPI ─────────────────────────────────────────────────────────
    bobbin_cfg = AxisConfig(
        name="bobbin",
        steps_per_revolution=BOBBIN_STEPS_PER_REV,
        microstepping=BOBBIN_MICROSTEPS,
        max_hz=HZ_MAX,
        min_hz=HZ_MIN,
        max_rpm=RPM_MAX,
    )
    axes = {int(AxisId.BOBBIN): Axis(bobbin_cfg)}

    transport = SpiTransport(port=args.spi_device, max_speed_hz=args.spi_speed)
    ctrl = ESP32Controller(transport, axes)

    print(f"[*] Connexion SPI : {args.spi_device} @ {args.spi_speed // 1_000} kHz")

    # ── Séquence ──────────────────────────────────────────────────────────────
    try:
        # Statut initial
        status = await ctrl.get_status()
        ax = status.axes[int(AxisId.BOBBIN)]
        print(f"[+] Spindle — position: {ax.position}  vitesse actuelle: {hz_to_rpm(ax.current_hz):.1f} RPM")

        # Activation du driver
        print("[*] Activation du driver spindle...")
        await ctrl.enable(AxisId.BOBBIN, enabled=True)
        await asyncio.sleep(0.05)

        # Exécution du profil
        print(f"[*] Démarrage du profil ({len(points)} points, {points[-1].t_ms / 1000:.1f} s)...")
        await run_spindle_profile(
            ctrl, points,
            reverse=args.reverse,
            verbose=not args.no_verbose,
        )

        # Arrêt final
        print("[*] Arrêt contrôlé...")
        await ctrl.stop(AxisId.BOBBIN)
        await asyncio.sleep(0.3)

        # Statut final
        status = await ctrl.get_status()
        ax = status.axes[int(AxisId.BOBBIN)]
        print(f"[+] Spindle — position finale: {ax.position}  vitesse: {hz_to_rpm(ax.current_hz):.1f} RPM")

        # Désactivation (optionnel — commenter pour maintenir le couple)
        # await ctrl.enable(AxisId.BOBBIN, enabled=False)

    except KeyboardInterrupt:
        print("\n[!] Interruption — arrêt d'urgence")
        await ctrl.emergency_stop()
        return 1
    except Exception as e:
        print(f"\n[!] Erreur : {e}")
        await ctrl.emergency_stop()
        return 2

    print("[+] Terminé.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
