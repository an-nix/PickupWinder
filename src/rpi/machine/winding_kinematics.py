"""winding_kinematics.py — Host-side winding geometry and velocity segment generation.

Computes layer geometry, lateral-to-bobbin frequency ratios, and trapezoidal
velocity profiles for the coil winding process.  All kinematics run on the
Raspberry Pi host; the ESP32 receives SET_SPEED commands via SPI.

Frequency ratio derivation
--------------------------
For one complete bobbin revolution:
  - Duration        = bobbin_ppr / hz_bobbin          [seconds]
  - Lateral travel  = pitch_mm                        [mm]
  - Lateral pulses  = pitch_mm × lateral_ppr / traverse_pitch_mm

Therefore:
  hz_lateral = lateral_pulses / duration
             = (pitch_mm × lateral_ppr / traverse_pitch_mm) × (hz_bobbin / bobbin_ppr)
             = hz_bobbin × (pitch_mm / traverse_pitch_mm) × (lateral_ppr / bobbin_ppr)

  R = hz_lateral / hz_bobbin = (pitch_mm / traverse_pitch_mm) × (lateral_ppr / bobbin_ppr)

When both axes share the same ppr (lateral_ppr == bobbin_ppr) the ppr terms cancel:
  R = pitch_mm / traverse_pitch_mm   (simplified form, equal-motor case)

Numerical example (from hardware constants — equal motors):
  d_wire         = 0.3 mm
  D0             = 20 mm   (runtime — NOT hardcoded)
  bobbin_width   = 30 mm   → turns_per_layer = floor(30/0.3) = 100
  traverse_pitch = 2.0 mm/rev (M6 leadscrew)
  bobbin motor:  200 full × 32 µstep → bobbin_ppr  = 6 400
  lateral motor: 200 full × 32 µstep → lateral_ppr = 6 400  (equal → simplified)
  bobbin_cruise  = 160 000 Hz  → 160000/6400 = 25 rev/s = 1 500 RPM
  R              = (0.3/2) × (6400/6400) = 0.15
  hz_lateral     = 160 000 × 0.15 = 24 000 Hz
  layer_duration = 100 turns / 25 rev/s = 4 s
  lateral_travel = 100 × 0.3 = 30 mm  ✓  (= bobbin_width)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List

# ── Constants ────────────────────────────────────────────────────────────────

ORTHO_PACK: float = math.sin(math.radians(60))  # sin 60° ≈ 0.86603

WK_HZ_MIN: int = 100
WK_HZ_MAX: int = 160_000


# ── Enumerations ─────────────────────────────────────────────────────────────

class WindingMode(IntEnum):
    FIXED_PITCH  = 0
    ORTHOCYCLIC  = 1
    CUSTOM_PITCH = 2


# ── Velocity segment ─────────────────────────────────────────────────────────

@dataclass
class VelocitySegment:
    """One trapezoidal velocity segment sent to the ESP32 as SET_SPEED commands.

    step_hz[0] — Bobbin axis frequency in Hz.
    step_hz[1] — Lateral axis frequency in Hz (= hz_bobbin × R).
    duration_ticks — Duration in 1 ms ticks.
    dir_mask — bit 0: bobbin direction (1=reverse), bit 1: lateral direction.
    flags — SEG_LAST | SEG_LAYER_END bitmask.
    """
    step_hz:        List[int] = field(default_factory=lambda: [0, 0])
    duration_ticks: int       = 0
    dir_mask:       int       = 0
    flags:          int       = 0


SEG_LAST:      int = 0x01   # Final segment of the profile
SEG_LAYER_END: int = 0x02   # End of a winding layer


# ── Geometry ─────────────────────────────────────────────────────────────────

@dataclass
class WindingGeometry:
    """Per-winding physical parameters and motor configuration.

    Wire and coil dimensions:
      wire_diameter_mm    — Wire diameter d  [mm]
      bobbin_width_mm     — Bobbin traverse width W  [mm]
      traverse_pitch_mm   — Lateral leadscrew pitch p_lead  [mm/rev]
      mandrel_diameter_mm — Bare mandrel outer diameter D0  [mm]  (runtime)

    Bobbin axis motor (axis 0):
      bobbin_steps_per_rev — Full steps per motor revolution  (default 200)
      bobbin_microsteps    — Microstepping factor             (default 32)

    Lateral axis motor (axis 1) — may differ from bobbin motor:
      lateral_steps_per_rev — Full steps per motor revolution  (default 200)
      lateral_microsteps    — Microstepping factor             (default 32)
    """
    wire_diameter_mm:     float
    bobbin_width_mm:      float
    traverse_pitch_mm:    float
    mandrel_diameter_mm:  float          # D0 — never hardcoded

    # Bobbin axis (axis 0) motor
    bobbin_steps_per_rev: int   = 200
    bobbin_microsteps:    int   = 32

    # Lateral axis (axis 1) motor — may have a different motor or drive setting
    lateral_steps_per_rev: int  = 200
    lateral_microsteps:    int  = 32

    def bobbin_ppr(self) -> int:
        """Pulses per revolution of the bobbin motor."""
        return self.bobbin_steps_per_rev * self.bobbin_microsteps

    def lateral_ppr(self) -> int:
        """Pulses per revolution of the lateral drive (leadscrew motor)."""
        return self.lateral_steps_per_rev * self.lateral_microsteps

    def traverse_mm_per_pulse(self) -> float:
        """Lateral axis: mm of traverse travel per step pulse.

        traverse_mm_per_pulse = traverse_pitch_mm / lateral_ppr
        """
        return self.traverse_pitch_mm / self.lateral_ppr()


# ── State ────────────────────────────────────────────────────────────────────

@dataclass
class WindingState:
    current_layer:    int         = 0
    total_layers:     int         = 0
    traverse_forward: bool        = True
    current_diameter: float       = 0.0
    ratio:            float       = 0.0
    bobbin_position:  int         = 0
    lateral_position: int         = 0
    total_turns:      int         = 0
    mode:             WindingMode = WindingMode.ORTHOCYCLIC
    custom_pitch_mm:  float       = 0.0


# ── Core kinematic functions ──────────────────────────────────────────────────

def compute_diameter(g: WindingGeometry, mode: WindingMode, layer: int) -> float:
    """Return the coil outer diameter D(n) for the given layer.

    D(n) = D0 + d_wire × (1 + 2n × PACK)
      PACK = 1.0     for FIXED_PITCH / CUSTOM_PITCH
      PACK = 0.86603 for ORTHOCYCLIC  (sin 60°)
    """
    if layer < 0:
        layer = 0
    if mode == WindingMode.ORTHOCYCLIC:
        radial_offset = g.wire_diameter_mm * (1.0 + 2.0 * layer * ORTHO_PACK)
    else:
        radial_offset = g.wire_diameter_mm * (1.0 + 2.0 * layer)
    return g.mandrel_diameter_mm + radial_offset


def compute_pitch(g: WindingGeometry, mode: WindingMode,
                  custom_pitch_mm: float = 0.0) -> float:
    """Return the effective wire pitch for the given winding mode."""
    if mode == WindingMode.CUSTOM_PITCH:
        return custom_pitch_mm if custom_pitch_mm > 0 else g.wire_diameter_mm
    return g.wire_diameter_mm


def ratio_lateral_per_bobbin(g: WindingGeometry, mode: WindingMode,
                              custom_pitch_mm: float = 0.0) -> float:
    """Return R = hz_lateral / hz_bobbin.

    R = (pitch_mm / traverse_pitch_mm) × (lateral_ppr / bobbin_ppr)

    When both axes share the same ppr the second factor is 1 and the formula
    reduces to the simpler form R = pitch_mm / traverse_pitch_mm.

    Typical values: 0 < R < 1  (lateral runs much slower than bobbin).
    """
    pitch = compute_pitch(g, mode, custom_pitch_mm)
    return (pitch / g.traverse_pitch_mm) * (g.lateral_ppr() / g.bobbin_ppr())


def turns_per_layer(g: WindingGeometry) -> int:
    """Number of wire turns that fit across the bobbin width."""
    if g.wire_diameter_mm <= 0:
        return 0
    return int(math.floor(g.bobbin_width_mm / g.wire_diameter_mm))


def total_pulses_per_layer(g: WindingGeometry) -> int:
    """Total bobbin motor pulses for one complete layer."""
    return turns_per_layer(g) * g.bobbin_ppr()


def on_layer_complete(state: WindingState, g: WindingGeometry) -> None:
    """Update winding state after one layer finishes."""
    state.current_layer    += 1
    state.traverse_forward  = not state.traverse_forward
    state.current_diameter  = compute_diameter(g, state.mode, state.current_layer)
    state.ratio             = ratio_lateral_per_bobbin(g, state.mode,
                                                        state.custom_pitch_mm)
    state.total_turns      += turns_per_layer(g)


# ── Segment generation ───────────────────────────────────────────────────────

def generate_layer(
    g: WindingGeometry,
    mode: WindingMode,
    layer: int,
    custom_pitch_mm: float,
    bobbin_cruise_hz: int,
    accel_hz_per_ms: int,
    traverse_forward: bool,
) -> List[VelocitySegment]:
    """Generate trapezoidal velocity segments for one winding layer.

    Parameters
    ----------
    g                : winding geometry
    mode             : FIXED_PITCH | ORTHOCYCLIC | CUSTOM_PITCH
    layer            : current layer index (0-based)
    custom_pitch_mm  : pitch override for CUSTOM_PITCH mode (ignored otherwise)
    bobbin_cruise_hz : target bobbin cruise speed  [Hz, clamped to WK_HZ range]
    accel_hz_per_ms  : acceleration rate  [Hz per 1 ms tick]; 0 = no ramp
    traverse_forward : True → lateral forward (bit1 of dir_mask = 0)

    Returns a list of VelocitySegment objects; the last segment carries
    SEG_LAST | SEG_LAYER_END.  Returns an empty list if the geometry is
    degenerate (zero wire diameter or zero ratio).
    """
    bobbin_cruise_hz = max(WK_HZ_MIN, min(bobbin_cruise_hz, WK_HZ_MAX))
    dir_val = 0x00 if traverse_forward else 0x02
    R = ratio_lateral_per_bobbin(g, mode, custom_pitch_mm)
    total_pulses = total_pulses_per_layer(g)

    if total_pulses <= 0 or R <= 0:
        return []

    def make_seg(hz_bob: int, dur_ms: int) -> VelocitySegment:
        hz_lat = max(1, round(hz_bob * R))
        hz_lat = min(hz_lat, WK_HZ_MAX)
        return VelocitySegment(
            step_hz=[hz_bob, hz_lat],
            duration_ticks=dur_ms,
            dir_mask=dir_val,
            flags=0,
        )

    segs: List[VelocitySegment] = []

    # ── No acceleration: single cruise segment ────────────────────────────
    if accel_hz_per_ms == 0:
        dur_ms = max(1, round(total_pulses / bobbin_cruise_hz * 1000))
        s = make_seg(bobbin_cruise_hz, dur_ms)
        s.flags = SEG_LAST | SEG_LAYER_END
        segs.append(s)
        return segs

    # ── Build ramp-up table (one entry per 1 ms tick) ─────────────────────
    ramp_hz: List[int] = []
    hz = WK_HZ_MIN
    while hz < bobbin_cruise_hz:
        ramp_hz.append(hz)
        hz += accel_hz_per_ms
    ramp_hz.append(bobbin_cruise_hz)

    ramp_pulses = sum(round(h * 0.001) for h in ramp_hz)

    # If ramp would consume more than half the layer, skip ramp entirely.
    if ramp_pulses * 2 >= total_pulses:
        dur_ms = max(1, round(total_pulses / bobbin_cruise_hz * 1000))
        s = make_seg(bobbin_cruise_hz, dur_ms)
        s.flags = SEG_LAST | SEG_LAYER_END
        segs.append(s)
        return segs

    pulses_remaining = total_pulses

    # Phase 1: Ramp-up
    for hz in ramp_hz:
        segs.append(make_seg(hz, 1))
        pulses_remaining -= round(hz * 0.001)

    # Phase 2: Cruise
    decel_pulses  = ramp_pulses
    cruise_pulses = max(0, pulses_remaining - decel_pulses)
    if cruise_pulses > 0:
        dur_ms = max(1, round(cruise_pulses / bobbin_cruise_hz * 1000))
        segs.append(make_seg(bobbin_cruise_hz, dur_ms))
        pulses_remaining -= cruise_pulses

    # Phase 3: Ramp-down (reverse ramp table)
    for hz in reversed(ramp_hz):
        segs.append(make_seg(hz, 1))
        pulses_remaining -= round(hz * 0.001)

    if segs:
        segs[-1].flags |= SEG_LAST | SEG_LAYER_END

    return segs
