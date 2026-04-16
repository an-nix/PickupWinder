#!/usr/bin/env python3
"""rmt_stepper_demo.py — Host-side demo for the RMT-based stepper backend.

Demonstrates two scenarios:

  1. SPINDLE RAMP  (default)
     Drive the bobbin axis (Axis 0) through a trapezoidal velocity profile.
     The host computes the ramp and streams SET_SPEED commands every
     RAMP_TICK_MS milliseconds. The ESP32 receives only SET_SPEED; it has no
     knowledge of the overall ramp shape.

  2. DUAL-AXIS WINDING  (--mode winding)
     Spin bobbin and lateral axes simultaneously using the winding frequency
     ratio R = (wire_pitch / traverse_pitch) × (lateral_ppr / bobbin_ppr).
     Both axes receive individual SET_SPEED commands computed from R each tick.

The RMT backend on the ESP32 is transparent to the Python side: the same
ESP32Controller / SPI protocol is used regardless of whether the firmware
uses Timer Groups or RMT for step generation.

Usage:
    # Dry-run (no hardware) — print ASCII profile
    python3 rmt_stepper_demo.py --dry-run

    # Spindle ramp: 1200 RPM in 3 s, hold 5 s, ramp down
    python3 rmt_stepper_demo.py --rpm 1200 --accel 400 --duration 5

    # Dual-axis winding demo (0.3 mm wire on 2 mm leadscrew, 50 RPM)
    python3 rmt_stepper_demo.py --mode winding --rpm 50 --accel 30 --duration 10

    # Reverse direction, verbose
    python3 rmt_stepper_demo.py --reverse --verbose

    # Specify SPI device
    python3 rmt_stepper_demo.py --spi-device /dev/spidev0.0
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from dataclasses import dataclass, field
from typing import List, Sequence

# ── Path bootstrap ────────────────────────────────────────────────────────────
import os
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from hal.esp32_controller import ESP32Controller
from hal.spi_transport import SpiTransport
from hal.axis import Axis, AxisConfig
from hal.protocol import AxisId

# ═════════════════════════════════════════════════════════════════════════════
#  Machine constants  (must match machine_config.yaml and rmt_stepper.h)
# ═════════════════════════════════════════════════════════════════════════════

# Bobbin axis (Axis 0)
BOBBIN_STEPS_PER_REV: int   = 200
BOBBIN_MICROSTEPS:    int   = 32
BOBBIN_PPR:           int   = BOBBIN_STEPS_PER_REV * BOBBIN_MICROSTEPS   # 6400

# Lateral axis (Axis 1)
LATERAL_STEPS_PER_REV: int  = 96
LATERAL_MICROSTEPS:    int  = 32
LATERAL_PPR:           int  = LATERAL_STEPS_PER_REV * LATERAL_MICROSTEPS  # 3072
TRAVERSE_PITCH_MM:     float = 2.0    # leadscrew mm/rev (M6)

# ESP32 step-frequency limits (RMT_HZ_MIN / RMT_HZ_MAX in rmt_stepper.h)
HZ_MIN: int = 100
HZ_MAX: int = 160_000

# Host-side ramp resolution: one SET_SPEED command per TICK_MS milliseconds
RAMP_TICK_MS: int = 20   # 50 Hz command rate


# ═════════════════════════════════════════════════════════════════════════════
#  Unit conversions
# ═════════════════════════════════════════════════════════════════════════════

def rpm_to_hz(rpm: float, ppr: int = BOBBIN_PPR) -> int:
    """RPM → step frequency (Hz), clamped to firmware limits."""
    return max(HZ_MIN, min(HZ_MAX, round(rpm * ppr / 60.0)))


def hz_to_rpm(hz: int, ppr: int = BOBBIN_PPR) -> float:
    return hz * 60.0 / ppr


def lateral_hz(bobbin_hz: int, wire_mm: float, traverse_pitch_mm: float) -> int:
    """Compute lateral step frequency for synchronised winding.

    General formula (supports different ppr per axis):
        R = (wire_pitch / traverse_pitch) × (lateral_ppr / bobbin_ppr)
        hz_lateral = hz_bobbin × R

    See copilot-instructions.md §12 and winding_kinematics.py.
    """
    R = (wire_mm / traverse_pitch_mm) * (LATERAL_PPR / BOBBIN_PPR)
    hz = round(bobbin_hz * R)
    return max(1, min(HZ_MAX, hz))


# ═════════════════════════════════════════════════════════════════════════════
#  Ramp profile data
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class RampPoint:
    """One tick of the velocity profile (both axes)."""
    t_ms:       int    # time from profile start (ms)
    bobbin_hz:  int    # bobbin step frequency (Hz)
    lateral_hz: int    # lateral step frequency (Hz) — 0 if spindle-only mode
    bobbin_rpm: float  # for display


def build_trapezoid(
    start_rpm:       float,
    target_rpm:      float,
    accel_rpm_s:     float,
    cruise_s:        float,
    wire_mm:         float = 0.0,   # set > 0 to enable lateral axis
    traverse_pitch:  float = TRAVERSE_PITCH_MM,
) -> List[RampPoint]:
    """Generate a trapezoidal velocity profile sampled at RAMP_TICK_MS.

    Phases:
      1. Ramp-up:   start_rpm → target_rpm at accel_rpm_s
      2. Cruise:    target_rpm for cruise_s seconds
      3. Ramp-down: target_rpm → start_rpm at accel_rpm_s

    Returns a list of RampPoint, one per RAMP_TICK_MS tick.
    """
    dt_s   = RAMP_TICK_MS / 1000.0
    points: List[RampPoint] = []
    t_ms   = 0

    # ── Phase 1: ramp-up ─────────────────────────────────────────────────────
    ramp_ticks = max(1, math.ceil(abs(target_rpm - start_rpm) / accel_rpm_s / dt_s))
    for k in range(ramp_ticks):
        rpm = start_rpm + (target_rpm - start_rpm) * (k + 1) / ramp_ticks
        b_hz = rpm_to_hz(rpm)
        l_hz = lateral_hz(b_hz, wire_mm, traverse_pitch) if wire_mm > 0 else 0
        points.append(RampPoint(t_ms=t_ms, bobbin_hz=b_hz, lateral_hz=l_hz,
                                bobbin_rpm=hz_to_rpm(b_hz)))
        t_ms += RAMP_TICK_MS

    # ── Phase 2: cruise ───────────────────────────────────────────────────────
    cruise_ticks = max(1, round(cruise_s / dt_s))
    b_hz_cruise = rpm_to_hz(target_rpm)
    l_hz_cruise = lateral_hz(b_hz_cruise, wire_mm, traverse_pitch) if wire_mm > 0 else 0
    for _ in range(cruise_ticks):
        points.append(RampPoint(t_ms=t_ms, bobbin_hz=b_hz_cruise,
                                lateral_hz=l_hz_cruise,
                                bobbin_rpm=hz_to_rpm(b_hz_cruise)))
        t_ms += RAMP_TICK_MS

    # ── Phase 3: ramp-down ────────────────────────────────────────────────────
    for k in range(ramp_ticks):
        rpm = target_rpm + (start_rpm - target_rpm) * (k + 1) / ramp_ticks
        b_hz = rpm_to_hz(rpm)
        l_hz = lateral_hz(b_hz, wire_mm, traverse_pitch) if wire_mm > 0 else 0
        points.append(RampPoint(t_ms=t_ms, bobbin_hz=b_hz, lateral_hz=l_hz,
                                bobbin_rpm=hz_to_rpm(b_hz)))
        t_ms += RAMP_TICK_MS

    return points


# ═════════════════════════════════════════════════════════════════════════════
#  ASCII profile visualiser (--dry-run)
# ═════════════════════════════════════════════════════════════════════════════

def print_profile(points: List[RampPoint], title: str = "Velocity profile") -> None:
    """Print an ASCII bar chart of the bobbin velocity profile."""
    if not points:
        print("(empty profile)")
        return

    max_rpm = max(p.bobbin_rpm for p in points)
    bar_w   = 50
    print(f"\n{'─'*60}")
    print(f"  {title}")
    print(f"  {len(points)} ticks × {RAMP_TICK_MS} ms = "
          f"{len(points)*RAMP_TICK_MS/1000:.1f} s total")
    print(f"  Peak: {max_rpm:.1f} RPM  ({rpm_to_hz(max_rpm)} Hz)")
    if any(p.lateral_hz for p in points):
        max_lat = max(p.lateral_hz for p in points)
        print(f"  Lateral peak: {max_lat} Hz  "
              f"({max_lat/LATERAL_PPR*60:.2f} mm/s)")
    print(f"{'─'*60}")

    # Sub-sample for display (max 40 rows)
    step = max(1, len(points) // 40)
    for p in points[::step]:
        filled = round(p.bobbin_rpm / max_rpm * bar_w) if max_rpm > 0 else 0
        bar    = "█" * filled + "░" * (bar_w - filled)
        print(f"  {p.t_ms/1000:5.2f}s │{bar}│ {p.bobbin_rpm:7.1f} RPM")

    print(f"{'─'*60}\n")


# ═════════════════════════════════════════════════════════════════════════════
#  Real-time execution
# ═════════════════════════════════════════════════════════════════════════════

async def run_profile(
    ctrl:    ESP32Controller,
    points:  List[RampPoint],
    reverse: bool,
    winding: bool,
    verbose: bool,
) -> None:
    """Stream pre-computed velocity profile to the ESP32 via SET_SPEED commands.

    This function is the host-side counterpart of the RMT fill_rmt_buffer()
    loop on the ESP32: it governs WHAT speed the motor runs at, while the
    firmware governs HOW (pulse generation, micro-timing, ISR rate).

    Timing strategy:
        deadline = profile_start + tick_index × RAMP_TICK_MS
        Each iteration sleeps until the next deadline, then sends SET_SPEED.
        asyncio.sleep() provides ~1 ms accuracy on Linux (sufficient for a
        50 Hz command rate driving a motor with mechanical inertia >> 1 ms).
    """
    if not points:
        return

    # Enable axes
    await ctrl.enable(AxisId.BOBBIN, True)
    if winding:
        await ctrl.enable(AxisId.LATERAL, True)

    t0      = asyncio.get_event_loop().time()
    n_sent  = 0
    t_start = time.monotonic()

    try:
        for i, pt in enumerate(points):
            # Wait until this tick's deadline
            deadline = t0 + i * RAMP_TICK_MS / 1000.0
            now      = asyncio.get_event_loop().time()
            if deadline > now:
                await asyncio.sleep(deadline - now)

            # Send bobbin SET_SPEED
            await ctrl.set_speed(AxisId.BOBBIN, hz=pt.bobbin_hz, reverse=reverse)

            # Send lateral SET_SPEED in winding mode
            if winding and pt.lateral_hz > 0:
                await ctrl.set_speed(AxisId.LATERAL, hz=pt.lateral_hz,
                                     reverse=False)

            n_sent += 1
            if verbose and i % 10 == 0:
                elapsed = time.monotonic() - t_start
                status  = await ctrl.get_status()
                ax0     = status.axis[AxisId.BOBBIN]
                print(f"  t={elapsed:5.2f}s | bobbin {ax0.step_hz:6d} Hz "
                      f"({hz_to_rpm(ax0.step_hz):6.1f} RPM) "
                      f"pos={ax0.position:+8d}")

        # Controlled stop: ramp down is already encoded in the profile's last
        # phase; send explicit STOP after the last tick to release the motor.
        await ctrl.stop(AxisId.BOBBIN)
        if winding:
            await ctrl.stop(AxisId.LATERAL)

    finally:
        elapsed = time.monotonic() - t_start
        print(f"\n  Profile complete: {n_sent} commands in {elapsed:.2f} s")

    # Wait for axes to decelerate and go idle
    try:
        await asyncio.wait_for(ctrl.wait_idle(AxisId.BOBBIN), timeout=5.0)
    except asyncio.TimeoutError:
        print("  Warning: bobbin axis did not reach IDLE within 5 s")

    await ctrl.enable(AxisId.BOBBIN, False)
    if winding:
        await ctrl.enable(AxisId.LATERAL, False)


# ═════════════════════════════════════════════════════════════════════════════
#  ISR performance metrics (information only, requires --verbose)
# ═════════════════════════════════════════════════════════════════════════════

def print_rmt_analysis(points: List[RampPoint]) -> None:
    """Estimate RMT interrupt rates and buffer efficiency for the profile.

    At each speed, fill_rmt_buffer() packs N steps into the 63-slot buffer:
      - N = min(63, floor(63 / items_per_step(iv)))
      - ISR interval ≈ N × step_period
      - ISR rate ≈ Hz / N  (interrupts per second)

    This is informational: the firmware handles timing automatically.
    """
    from math import ceil

    RMT_MAX_ITEM_TICKS = 32767
    RMT_PULSE_TICKS    = 80
    RMT_BUF_SLOTS      = 63   # 64 - 1 (end marker)
    RMT_CLK_HZ         = 40_000_000

    print(f"\n{'─'*60}")
    print("  RMT performance analysis")
    print(f"  {'RPM':>8}  {'Hz':>8}  {'items/step':>10}  "
          f"{'steps/buf':>10}  {'ISR Hz':>8}")
    print(f"{'─'*60}")

    seen_hz: set = set()
    for pt in points:
        hz = pt.bobbin_hz
        if hz in seen_hz:
            continue
        seen_hz.add(hz)

        iv      = RMT_CLK_HZ // max(1, hz)
        gap     = max(0, iv - RMT_PULSE_TICKS)
        if gap <= RMT_MAX_ITEM_TICKS:
            n_items = 1
        else:
            remaining = gap - RMT_MAX_ITEM_TICKS
            n_items = 1 + ceil(remaining / (2 * RMT_MAX_ITEM_TICKS))

        steps_per_buf = max(1, RMT_BUF_SLOTS // n_items)
        isr_hz        = hz // steps_per_buf

        print(f"  {pt.bobbin_rpm:>8.1f}  {hz:>8d}  {n_items:>10d}  "
              f"{steps_per_buf:>10d}  {isr_hz:>8d}")

    print(f"{'─'*60}\n")


# ═════════════════════════════════════════════════════════════════════════════
#  Main entry point
# ═════════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="RMT stepper demo — drive bobbin (+ optional lateral) axis"
    )
    p.add_argument("--mode", choices=["spindle", "winding"], default="spindle",
                   help="spindle: bobbin only | winding: bobbin + lateral sync")
    p.add_argument("--rpm",      type=float, default=1000.0,
                   help="target RPM for bobbin (default: 1000)")
    p.add_argument("--accel",    type=float, default=200.0,
                   help="acceleration in RPM/s (default: 200)")
    p.add_argument("--duration", type=float, default=5.0,
                   help="cruise duration in seconds (default: 5)")
    p.add_argument("--start-rpm", type=float, default=None,
                   help="starting RPM (default: RPM_MIN)")
    p.add_argument("--wire-mm",  type=float, default=0.3,
                   help="wire diameter in mm for winding mode (default: 0.3)")
    p.add_argument("--traverse-pitch", type=float, default=TRAVERSE_PITCH_MM,
                   help=f"leadscrew pitch mm/rev (default: {TRAVERSE_PITCH_MM})")
    p.add_argument("--reverse",  action="store_true",
                   help="reverse bobbin direction")
    p.add_argument("--dry-run",  action="store_true",
                   help="compute and display profile without hardware")
    p.add_argument("--analysis", action="store_true",
                   help="print RMT ISR rate analysis table")
    p.add_argument("--verbose",  action="store_true",
                   help="print live axis status during execution")
    p.add_argument("--spi-device", default="/dev/spidev0.0",
                   help="SPI device path (default: /dev/spidev0.0)")
    p.add_argument("--spi-speed",  type=int, default=4_000_000,
                   help="SPI clock Hz (default: 4 000 000)")
    return p.parse_args()


async def async_main() -> None:
    args = parse_args()

    # RPM bounds
    rpm_min = hz_to_rpm(HZ_MIN)
    rpm_max = hz_to_rpm(HZ_MAX)
    target_rpm = max(rpm_min, min(rpm_max, args.rpm))
    start_rpm  = args.start_rpm if args.start_rpm is not None else rpm_min

    if target_rpm != args.rpm:
        print(f"  Note: RPM clamped from {args.rpm:.1f} to {target_rpm:.1f} "
              f"(HW range {rpm_min:.1f}–{rpm_max:.1f})")

    winding = (args.mode == "winding")

    # Build profile
    wire_mm = args.wire_mm if winding else 0.0
    points  = build_trapezoid(
        start_rpm      = start_rpm,
        target_rpm     = target_rpm,
        accel_rpm_s    = max(1.0, args.accel),
        cruise_s       = max(0.0, args.duration),
        wire_mm        = wire_mm,
        traverse_pitch = args.traverse_pitch,
    )

    title = (f"Winding demo — {target_rpm:.0f} RPM, "
             f"wire={args.wire_mm} mm, pitch={args.traverse_pitch} mm/rev"
             if winding else
             f"Spindle ramp — {target_rpm:.0f} RPM, "
             f"accel={args.accel:.0f} RPM/s, cruise={args.duration:.1f} s")

    print_profile(points, title)

    if args.analysis:
        print_rmt_analysis(points)

    if args.dry_run:
        print("  [dry-run] No hardware access. Use without --dry-run to execute.")
        return

    # ── Hardware execution ────────────────────────────────────────────────────
    print(f"  Connecting to ESP32 on {args.spi_device} at {args.spi_speed} Hz…")

    # Parse device path like /dev/spidevX.Y -> bus=X, device=Y
    bus = 0
    dev = 0
    dev_path = args.spi_device
    if isinstance(dev_path, str) and dev_path.startswith("/dev/spidev"):
        try:
            base = os.path.basename(dev_path)  # e.g. spidev0.0
            suffix = base[len("spidev"):]
            bus_str, dev_str = suffix.split(".")
            bus = int(bus_str)
            dev = int(dev_str)
        except Exception:
            print(f"  Warning: could not parse spi device '{dev_path}', using bus=0, device=0")
    else:
        print(f"  Warning: unexpected spi device '{dev_path}', using bus=0, device=0")

    transport = SpiTransport(bus=bus, device=dev, speed_hz=args.spi_speed)
    try:
        transport.open()
    except Exception as e:
        print(f"  Error opening SPI device {args.spi_device}: {e}")
        raise
    axes = {
        AxisId.BOBBIN:   AxisConfig(name='bobbin', steps_per_revolution=BOBBIN_STEPS_PER_REV,
                                    microstepping=BOBBIN_MICROSTEPS,
                                    max_hz=HZ_MAX, min_hz=HZ_MIN,
                                    max_rpm=rpm_max, default_accel=100_000),
        AxisId.LATERAL:  AxisConfig(name='lateral', steps_per_revolution=LATERAL_STEPS_PER_REV,
                                    microstepping=LATERAL_MICROSTEPS,
                                    max_hz=HZ_MAX, min_hz=HZ_MIN,
                                    max_rpm=hz_to_rpm(HZ_MAX, LATERAL_PPR),
                                    default_accel=50_000,
                                    steps_per_mm=LATERAL_PPR),
    }

    ctrl = ESP32Controller(transport=transport, axes=axes)
    await ctrl.start_polling()

    try:
        print(f"\n  Starting {'winding' if winding else 'spindle'} profile "
              f"({'REVERSE ' if args.reverse else ''}"
              f"{target_rpm:.0f} RPM, {args.duration:.1f} s cruise)…\n")

        await run_profile(
            ctrl    = ctrl,
            points  = points,
            reverse = args.reverse,
            winding = winding,
            verbose = args.verbose,
        )

    except KeyboardInterrupt:
        print("\n  ^C — issuing emergency stop…")
        await ctrl.emergency_stop()

    finally:
        await ctrl.stop_polling()
        transport.close()
        print("  Done.")


def main() -> None:
    """Entry point — wraps async_main() in asyncio.run()."""
    asyncio.run(async_main())


if __name__ == "__main__":
    main()
