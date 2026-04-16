"""test_winding_kinematics.py — Unit tests for winding geometry and segment generation.

All kinematic computation runs on the Raspberry Pi host (machine/winding_kinematics.py).
The ESP32 receives only SET_SPEED commands; it has no knowledge of wire geometry.

Numerical example (from hardware constants — equal motors on both axes):
  d_wire          = 0.3 mm
  D0              = 20 mm   (runtime — NOT hardcoded)
  bobbin_width    = 30 mm   → turns_per_layer = floor(30/0.3) = 100
  traverse_pitch  = 2 mm/rev (M6 leadscrew)
  bobbin motor:   200 full × 32 µstep → bobbin_ppr  = 6 400
  lateral motor:  200 full × 32 µstep → lateral_ppr = 6 400  (equal → simplified)
  R               = (0.3/2) × (6400/6400) = 0.15
  hz_lateral      = 160 000 × 0.15 = 24 000 Hz
  layer_duration  = 100 / 25 = 4 s
  lateral_travel  = 100 × 0.3 = 30 mm  ✓ (= bobbin_width)
"""

from __future__ import annotations

import pytest

from machine.winding_kinematics import (
    ORTHO_PACK,
    SEG_LAST,
    SEG_LAYER_END,
    WK_HZ_MAX,
    WK_HZ_MIN,
    VelocitySegment,
    WindingGeometry,
    WindingMode,
    WindingState,
    compute_diameter,
    compute_pitch,
    generate_layer,
    on_layer_complete,
    ratio_lateral_per_bobbin,
    total_pulses_per_layer,
    turns_per_layer,
)


# ── Reference fixture ─────────────────────────────────────────────────────────

@pytest.fixture
def ref_geom() -> WindingGeometry:
    """Reference geometry from the spec (equal motors on both axes)."""
    return WindingGeometry(
        wire_diameter_mm=0.3,
        bobbin_width_mm=30.0,
        traverse_pitch_mm=2.0,
        mandrel_diameter_mm=20.0,     # D0 — runtime parameter, NOT hardcoded
        bobbin_steps_per_rev=200,
        bobbin_microsteps=32,
        lateral_steps_per_rev=200,
        lateral_microsteps=32,
    )


# ── Motor parameter tests ─────────────────────────────────────────────────────

class TestMotorParams:
    def test_bobbin_ppr_standard(self, ref_geom: WindingGeometry) -> None:
        assert ref_geom.bobbin_ppr() == 6400

    def test_lateral_ppr_standard(self, ref_geom: WindingGeometry) -> None:
        assert ref_geom.lateral_ppr() == 6400

    def test_different_lateral_microsteps(self) -> None:
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=200, lateral_microsteps=16)
        assert g.bobbin_ppr() == 6400
        assert g.lateral_ppr() == 3200

    def test_different_lateral_motor(self) -> None:
        """Different steps/rev on lateral axis (e.g. 96 full × 32 µstep = 3072)."""
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=96, lateral_microsteps=32)
        assert g.bobbin_ppr() == 6400
        assert g.lateral_ppr() == 3072

    def test_traverse_mm_per_pulse(self, ref_geom: WindingGeometry) -> None:
        # 2 mm/rev / 6400 ppr = 0.0003125 mm/pulse
        assert abs(ref_geom.traverse_mm_per_pulse() - 0.0003125) < 1e-9

    def test_traverse_mm_per_pulse_with_different_lateral(self) -> None:
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=96, lateral_microsteps=32)
        # 2 mm/rev / 3072 ppr
        assert abs(g.traverse_mm_per_pulse() - 2.0 / 3072) < 1e-9


# ── Turns per layer ───────────────────────────────────────────────────────────

class TestTurnsPerLayer:
    def test_reference(self, ref_geom: WindingGeometry) -> None:
        # floor(30 / 0.3) = 100
        assert turns_per_layer(ref_geom) == 100

    def test_non_integer(self) -> None:
        g = WindingGeometry(0.3, 31.0, 2, 20)
        # floor(31 / 0.3) = floor(103.33) = 103
        assert turns_per_layer(g) == 103

    def test_tight_fit(self) -> None:
        g = WindingGeometry(0.3, 0.3, 2, 20)
        assert turns_per_layer(g) == 1

    def test_zero_wire(self) -> None:
        g = WindingGeometry(0.0, 30.0, 2, 20)
        assert turns_per_layer(g) == 0


# ── Total pulses per layer ────────────────────────────────────────────────────

class TestTotalPulsesPerLayer:
    def test_reference(self, ref_geom: WindingGeometry) -> None:
        # 100 turns × bobbin_ppr = 100 × 6400 = 640 000
        assert total_pulses_per_layer(ref_geom) == 640_000

    def test_uses_bobbin_ppr(self) -> None:
        """total_pulses counts bobbin axis pulses, not lateral."""
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=96, lateral_microsteps=32)
        # 100 turns × 6400 bobbin_ppr = 640 000 (lateral_ppr 3072 is irrelevant)
        assert total_pulses_per_layer(g) == 640_000


# ── Ratio ─────────────────────────────────────────────────────────────────────

class TestRatio:
    def test_reference_orthocyclic(self, ref_geom: WindingGeometry) -> None:
        # Equal motors → R = pitch / p_lead = 0.3 / 2 = 0.15
        R = ratio_lateral_per_bobbin(ref_geom, WindingMode.ORTHOCYCLIC)
        assert abs(R - 0.15) < 1e-9

    def test_reference_fixed_pitch(self, ref_geom: WindingGeometry) -> None:
        R = ratio_lateral_per_bobbin(ref_geom, WindingMode.FIXED_PITCH)
        assert abs(R - 0.15) < 1e-9

    def test_lateral_hz_at_cruise(self, ref_geom: WindingGeometry) -> None:
        # hz_lateral = 160 000 × 0.15 = 24 000 Hz
        R = ratio_lateral_per_bobbin(ref_geom, WindingMode.ORTHOCYCLIC)
        hz_lat = round(160_000 * R)
        assert abs(hz_lat - 24000) <= 1

    def test_custom_pitch(self, ref_geom: WindingGeometry) -> None:
        # Custom pitch 0.6 mm → R = 0.6/2 = 0.3 → hz_lat = 160000*0.3 = 48000
        R = ratio_lateral_per_bobbin(ref_geom, WindingMode.CUSTOM_PITCH,
                                     custom_pitch_mm=0.6)
        assert abs(R - 0.30) < 1e-9

    def test_ratio_is_constant_across_layers_orthocyclic(
            self, ref_geom: WindingGeometry) -> None:
        """Orthocyclic R is layer-independent (pitch = d_wire always)."""
        R0 = ratio_lateral_per_bobbin(ref_geom, WindingMode.ORTHOCYCLIC)
        R5 = ratio_lateral_per_bobbin(ref_geom, WindingMode.ORTHOCYCLIC)
        assert abs(R0 - R5) < 1e-9

    def test_ratio_is_less_than_one(self, ref_geom: WindingGeometry) -> None:
        """For typical coil winding, R < 1 (lateral much slower than bobbin)."""
        R = ratio_lateral_per_bobbin(ref_geom, WindingMode.ORTHOCYCLIC)
        assert 0 < R < 1.0

    def test_different_lateral_ppr_scales_ratio(self) -> None:
        """With lateral_ppr = bobbin_ppr/2, R is also halved.

        R = (pitch/p_lead) × (lateral_ppr/bobbin_ppr)
          = (0.3/2) × (3200/6400)
          = 0.15 × 0.5 = 0.075
        """
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=200, lateral_microsteps=16)
        R = ratio_lateral_per_bobbin(g, WindingMode.ORTHOCYCLIC)
        assert abs(R - 0.075) < 1e-9

    def test_different_lateral_steps_per_rev(self) -> None:
        """lateral_steps_per_rev=96 (e.g. special motor or gearing).

        bobbin_ppr  = 200×32 = 6400
        lateral_ppr =  96×32 = 3072
        R = (0.3/2) × (3072/6400) = 0.15 × 0.48 = 0.072
        """
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=96, lateral_microsteps=32)
        R = ratio_lateral_per_bobbin(g, WindingMode.ORTHOCYCLIC)
        expected = (0.3 / 2.0) * (3072 / 6400)
        assert abs(R - expected) < 1e-9


# ── Diameter ──────────────────────────────────────────────────────────────────

class TestComputeDiameter:
    def test_layer0_fixed(self, ref_geom: WindingGeometry) -> None:
        # D(0) = D0 + d_wire = 20 + 0.3 = 20.3
        D = compute_diameter(ref_geom, WindingMode.FIXED_PITCH, 0)
        assert abs(D - 20.3) < 1e-6

    def test_layer1_fixed(self, ref_geom: WindingGeometry) -> None:
        # D(1) = D0 + 3 * d = 20 + 0.9 = 20.9
        D = compute_diameter(ref_geom, WindingMode.FIXED_PITCH, 1)
        assert abs(D - 20.9) < 1e-6

    def test_layer10_fixed(self, ref_geom: WindingGeometry) -> None:
        # D(10) = D0 + 21 * d = 20 + 6.3 = 26.3
        D = compute_diameter(ref_geom, WindingMode.FIXED_PITCH, 10)
        assert abs(D - 26.3) < 1e-4

    def test_layer0_orthocyclic(self, ref_geom: WindingGeometry) -> None:
        D = compute_diameter(ref_geom, WindingMode.ORTHOCYCLIC, 0)
        assert abs(D - 20.3) < 1e-6

    def test_layer1_orthocyclic(self, ref_geom: WindingGeometry) -> None:
        expected = 20.0 + 0.3 * (1.0 + 2.0 * ORTHO_PACK)
        D = compute_diameter(ref_geom, WindingMode.ORTHOCYCLIC, 1)
        assert abs(D - expected) < 1e-6

    def test_d0_is_runtime(self) -> None:
        """D0 is never hardcoded — two geoms with different D0 must give different results."""
        g1 = WindingGeometry(0.3, 30, 2, mandrel_diameter_mm=20.0)
        g2 = WindingGeometry(0.3, 30, 2, mandrel_diameter_mm=15.0)
        D1 = compute_diameter(g1, WindingMode.FIXED_PITCH, 0)
        D2 = compute_diameter(g2, WindingMode.FIXED_PITCH, 0)
        assert abs(D1 - D2 - 5.0) < 1e-6


# ── Layer state transitions ───────────────────────────────────────────────────

class TestOnLayerComplete:
    def test_layer_increments(self, ref_geom: WindingGeometry) -> None:
        state = WindingState(mode=WindingMode.ORTHOCYCLIC)
        on_layer_complete(state, ref_geom)
        assert state.current_layer == 1

    def test_direction_reverses(self, ref_geom: WindingGeometry) -> None:
        state = WindingState(traverse_forward=True, mode=WindingMode.ORTHOCYCLIC)
        on_layer_complete(state, ref_geom)
        assert state.traverse_forward is False
        on_layer_complete(state, ref_geom)
        assert state.traverse_forward is True

    def test_turns_accumulate(self, ref_geom: WindingGeometry) -> None:
        state = WindingState(mode=WindingMode.ORTHOCYCLIC)
        on_layer_complete(state, ref_geom)
        assert state.total_turns == 100
        on_layer_complete(state, ref_geom)
        assert state.total_turns == 200

    def test_diameter_updates(self, ref_geom: WindingGeometry) -> None:
        state = WindingState(mode=WindingMode.FIXED_PITCH)
        on_layer_complete(state, ref_geom)
        expected = compute_diameter(ref_geom, WindingMode.FIXED_PITCH, 1)
        assert abs(state.current_diameter - expected) < 1e-6

    def test_ratio_updates_custom_pitch(self, ref_geom: WindingGeometry) -> None:
        state = WindingState(mode=WindingMode.CUSTOM_PITCH, custom_pitch_mm=0.6)
        on_layer_complete(state, ref_geom)
        expected_R = ratio_lateral_per_bobbin(
            ref_geom, WindingMode.CUSTOM_PITCH, custom_pitch_mm=0.6)
        assert abs(state.ratio - expected_R) < 1e-6


# ── Segment generation ────────────────────────────────────────────────────────

class TestGenerateLayer:
    def test_no_accel_single_segment(self, ref_geom: WindingGeometry) -> None:
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 0, True)
        assert len(segs) == 1
        assert segs[-1].flags & SEG_LAST
        assert segs[-1].flags & SEG_LAYER_END

    def test_last_segment_flagged(self, ref_geom: WindingGeometry) -> None:
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 10_000, True)
        assert len(segs) > 0
        assert segs[-1].flags & SEG_LAST
        assert segs[-1].flags & SEG_LAYER_END

    def test_hz_lateral_at_cruise_matches_spec(self, ref_geom: WindingGeometry) -> None:
        """Cruise segment must have hz_lateral = 24000 Hz at 160 kHz bobbin.

        R = (pitch/p_lead) × (lateral_ppr/bobbin_ppr) = (0.3/2) × 1 = 0.15
        hz_lat = 160 000 × 0.15 = 24 000 Hz
        """
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 10_000, True)
        cruise_segs = [s for s in segs if s.step_hz[0] == 160_000]
        assert len(cruise_segs) > 0, "No cruise segment at 160 kHz"
        hz_lat = cruise_segs[0].step_hz[1]
        assert abs(hz_lat - 24000) <= 1, f"Expected 24000 Hz lateral, got {hz_lat}"

    def test_lateral_hz_minimum_is_one(self, ref_geom: WindingGeometry) -> None:
        """No lateral segment should have hz < 1."""
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 10_000, True)
        for seg in segs:
            assert seg.step_hz[1] >= 1, \
                f"Lateral hz {seg.step_hz[1]} is below minimum"

    def test_direction_forward(self, ref_geom: WindingGeometry) -> None:
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 0, True)
        for seg in segs:
            assert (seg.dir_mask & 0x02) == 0x00, "Forward → bit1 must be 0"

    def test_direction_reverse(self, ref_geom: WindingGeometry) -> None:
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 0, False)
        for seg in segs:
            assert (seg.dir_mask & 0x02) == 0x02, "Reverse → bit1 must be 1"

    def test_all_bobbin_hz_in_range(self, ref_geom: WindingGeometry) -> None:
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 10_000, True)
        for seg in segs:
            assert WK_HZ_MIN <= seg.step_hz[0] <= WK_HZ_MAX, \
                f"Bobbin hz {seg.step_hz[0]} out of [{WK_HZ_MIN}, {WK_HZ_MAX}]"

    def test_custom_pitch_higher_ratio(self, ref_geom: WindingGeometry) -> None:
        """Custom pitch 0.6 mm → R = 0.6/2 = 0.3 → hz_lateral = 160000*0.3 = 48000 Hz."""
        segs = generate_layer(ref_geom, WindingMode.CUSTOM_PITCH, 0, 0.6,
                               160_000, 0, True)
        hz_lat = segs[-1].step_hz[1]
        assert abs(hz_lat - 48000) <= 1, f"Expected 48000 Hz lateral, got {hz_lat}"

    def test_empty_geometry_returns_no_segments(self) -> None:
        g = WindingGeometry(0.0, 30.0, 2, 20)
        segs = generate_layer(g, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 0, True)
        assert len(segs) == 0

    def test_layer_duration_approximately_4s(self, ref_geom: WindingGeometry) -> None:
        """Spec: layer_duration = 100 turns / 25 rev/s = 4 s (no accel)."""
        segs = generate_layer(ref_geom, WindingMode.ORTHOCYCLIC, 0, 0.0,
                               160_000, 0, True)
        total_ms = sum(s.duration_ticks for s in segs)
        assert abs(total_ms - 4000) <= 400, \
            f"Layer duration {total_ms} ms, expected ~4000 ms"

    def test_lateral_travel_matches_bobbin_width(self, ref_geom: WindingGeometry) -> None:
        """Lateral travel = total_bobbin_pulses × R × traverse_mm_per_pulse = bobbin_width.

        At constant speed:
          layer_time_s  = total_bobbin_pulses / hz_bobbin
          hz_lat        = hz_bobbin × R
          lateral_mm    = hz_lat × layer_time_s × traverse_mm_per_pulse
                        = hz_bobbin × R × (total_bobbin_pulses/hz_bobbin)
                                        × (p_lead/lateral_ppr)
                        = R × total_bobbin_pulses × (p_lead/lateral_ppr)
                        = (pitch/p_lead) × (lateral_ppr/bobbin_ppr)
                          × turns×bobbin_ppr × (p_lead/lateral_ppr)
                        = pitch × turns  = d_wire × floor(width/d_wire) ≈ width  ✓
        """
        R = ratio_lateral_per_bobbin(ref_geom, WindingMode.ORTHOCYCLIC)
        total_bobbin = total_pulses_per_layer(ref_geom)
        lateral_mm = R * total_bobbin * ref_geom.traverse_mm_per_pulse()
        assert abs(lateral_mm - ref_geom.bobbin_width_mm) < 1e-6, \
            f"Lateral travel {lateral_mm:.6f} mm, expected {ref_geom.bobbin_width_mm} mm"

    def test_lateral_travel_different_ppr(self) -> None:
        """Lateral travel equation holds even when the two axes have different ppr."""
        g = WindingGeometry(0.3, 30, 2, 20,
                            bobbin_steps_per_rev=200, bobbin_microsteps=32,
                            lateral_steps_per_rev=96, lateral_microsteps=32)
        R = ratio_lateral_per_bobbin(g, WindingMode.ORTHOCYCLIC)
        total_bobbin = total_pulses_per_layer(g)
        lateral_mm = R * total_bobbin * g.traverse_mm_per_pulse()
        assert abs(lateral_mm - g.bobbin_width_mm) < 1e-6, \
            f"Lateral travel {lateral_mm:.6f} mm, expected {g.bobbin_width_mm} mm"


# ── End-to-end numerical validation ──────────────────────────────────────────

class TestNumericalExample:
    """Full end-to-end validation of the spec numerical example."""

    def test_full_spec_example(self) -> None:
        """Validate every derived value from the spec numerical example (equal motors)."""
        g = WindingGeometry(
            wire_diameter_mm=0.3,
            bobbin_width_mm=30.0,
            traverse_pitch_mm=2.0,
            mandrel_diameter_mm=20.0,   # runtime — must NOT be hardcoded
            bobbin_steps_per_rev=200,
            bobbin_microsteps=32,
            lateral_steps_per_rev=200,
            lateral_microsteps=32,
        )

        assert g.bobbin_ppr()  == 6400
        assert g.lateral_ppr() == 6400
        assert abs(g.traverse_mm_per_pulse() - 0.0003125) < 1e-9

        assert turns_per_layer(g)        == 100
        assert total_pulses_per_layer(g) == 640_000

        # R = (pitch/p_lead) × (lateral_ppr/bobbin_ppr) = (0.3/2) × 1 = 0.15
        R = ratio_lateral_per_bobbin(g, WindingMode.ORTHOCYCLIC)
        assert abs(R - 0.15) < 1e-9

        # hz_lateral at cruise = 160 000 × 0.15 = 24 000 Hz
        hz_lat = round(160_000 * R)
        assert hz_lat == 24_000

        # Layer duration at cruise: 100 turns / (160000/6400) rev/s = 4 s
        layer_duration_s = total_pulses_per_layer(g) / 160_000
        assert abs(layer_duration_s - 4.0) < 1e-3

        # Lateral travel: R × total_bobbin_pulses × traverse_mm_per_pulse = 30 mm ✓
        lat_travel = R * total_pulses_per_layer(g) * g.traverse_mm_per_pulse()
        assert abs(lat_travel - 30.0) < 1e-6

        # Alternative: turns × d_wire = 100 × 0.3 = 30 mm ✓
        assert abs(turns_per_layer(g) * g.wire_diameter_mm - 30.0) < 1e-6

        # Diameter growth
        assert abs(compute_diameter(g, WindingMode.FIXED_PITCH, 0)  - 20.3) < 1e-5
        assert abs(compute_diameter(g, WindingMode.FIXED_PITCH, 1)  - 20.9) < 1e-4
        assert abs(compute_diameter(g, WindingMode.FIXED_PITCH, 10) - 26.3) < 1e-4

    def test_unequal_motors_lateral_travel_invariant(self) -> None:
        """Lateral travel identity holds regardless of the motor ppr ratio."""
        for lat_steps, lat_us in [(200, 16), (96, 32), (200, 32)]:
            g = WindingGeometry(0.3, 30, 2, 20,
                                bobbin_steps_per_rev=200, bobbin_microsteps=32,
                                lateral_steps_per_rev=lat_steps,
                                lateral_microsteps=lat_us)
            R = ratio_lateral_per_bobbin(g, WindingMode.ORTHOCYCLIC)
            lat_mm = R * total_pulses_per_layer(g) * g.traverse_mm_per_pulse()
            assert abs(lat_mm - g.bobbin_width_mm) < 1e-6, \
                f"lateral_ppr={g.lateral_ppr()}: travel={lat_mm:.6f} expected {g.bobbin_width_mm}"

    def test_d0_runtime_not_hardcoded(self) -> None:
        """Changing D0 at runtime must change the output — proves it is not hardcoded."""
        for d0 in [10.0, 15.0, 20.0, 25.0, 30.0]:
            g = WindingGeometry(0.3, 30.0, 2.0, mandrel_diameter_mm=d0)
            D = compute_diameter(g, WindingMode.FIXED_PITCH, 0)
            assert abs(D - (d0 + 0.3)) < 1e-6, \
                f"D0={d0}: expected {d0 + 0.3}, got {D}"
