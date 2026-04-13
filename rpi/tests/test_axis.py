"""test_axis.py — Unit tests for the Axis abstraction layer.

Tests RPM ↔ Hz conversions, mm ↔ steps conversions,
and clamping / validation behaviour.
"""

from __future__ import annotations

import pytest

from hal.axis import (
    Axis,
    AxisConfig,
    BOBBIN_DEFAULTS,
    LATERAL_DEFAULTS,
    TENSIONER_DEFAULTS,
)


class TestBobbinConversions:
    """Bobbin (spindle) axis unit conversions."""

    def setup_method(self) -> None:
        self.axis = Axis(BOBBIN_DEFAULTS)

    def test_rpm_to_hz_basic(self) -> None:
        # 1 RPM = 6400 steps/rev / 60s = 106.67 Hz
        hz = self.axis.rpm_to_hz(1.0)
        assert abs(hz - 106.67) < 1.0

    def test_rpm_to_hz_1000(self) -> None:
        # 1000 RPM = 1000 * 6400 / 60 ≈ 106666 Hz (truncated)
        hz = self.axis.rpm_to_hz(1000.0)
        assert abs(hz - 106667) < 2

    def test_hz_to_rpm_basic(self) -> None:
        rpm = self.axis.hz_to_rpm(6400)
        assert abs(rpm - 60.0) < 0.1  # 60 RPM

    def test_hz_to_rpm_roundtrip(self) -> None:
        for rpm in [10, 100, 500, 1000, 1500]:
            hz = self.axis.rpm_to_hz(rpm)
            back = self.axis.hz_to_rpm(hz)
            assert abs(back - rpm) < 0.1

    def test_rpm_zero(self) -> None:
        assert self.axis.rpm_to_hz(0.0) == 0.0

    def test_max_rpm(self) -> None:
        hz = self.axis.rpm_to_hz(1500)
        assert hz <= BOBBIN_DEFAULTS.max_hz


class TestLateralConversions:
    """Lateral axis unit conversions."""

    def setup_method(self) -> None:
        self.axis = Axis(LATERAL_DEFAULTS)

    def test_mm_to_steps_1mm(self) -> None:
        steps = self.axis.mm_to_steps(1.0)
        assert steps == 3072  # 3072 steps/mm

    def test_mm_to_steps_10mm(self) -> None:
        steps = self.axis.mm_to_steps(10.0)
        assert steps == 30720

    def test_steps_to_mm_basic(self) -> None:
        mm = self.axis.steps_to_mm(3072)
        assert abs(mm - 1.0) < 0.001

    def test_mm_roundtrip(self) -> None:
        for mm in [0.5, 1.0, 5.0, 10.0, 17.0]:
            steps = self.axis.mm_to_steps(mm)
            back = self.axis.steps_to_mm(steps)
            assert abs(back - mm) < 0.001

    def test_mm_to_steps_zero(self) -> None:
        assert self.axis.mm_to_steps(0.0) == 0

    def test_mm_to_steps_negative(self) -> None:
        steps = self.axis.mm_to_steps(-5.0)
        assert steps == -15360

    def test_steps_per_mm_default(self) -> None:
        assert LATERAL_DEFAULTS.steps_per_mm == 3072


class TestTensionerDefaults:
    """Tensioner axis configuration."""

    def setup_method(self) -> None:
        self.axis = Axis(TENSIONER_DEFAULTS)

    def test_full_steps_per_rev(self) -> None:
        assert TENSIONER_DEFAULTS.full_steps_per_rev == 3200

    def test_microstepping(self) -> None:
        assert TENSIONER_DEFAULTS.microstepping == 16

    def test_rpm_to_hz(self) -> None:
        # 1 RPM = 3200/60 ≈ 53.33 Hz, but clamped to min_hz=100
        hz = self.axis.rpm_to_hz(1.0)
        assert hz == TENSIONER_DEFAULTS.min_hz  # clamped


class TestAxisConfigValidation:
    """Edge cases and configuration validation."""

    def test_custom_config(self) -> None:
        cfg = AxisConfig(
            name="custom",
            steps_per_revolution=400,
            microstepping=8,
            max_hz=50000,
            min_hz=50,
        )
        axis = Axis(cfg)
        # 400 steps/rev × 8 µstep = 3200 steps/rev
        hz = axis.rpm_to_hz(60)
        assert abs(hz - 3200) < 1.0

    def test_steps_per_mm_none_raises(self) -> None:
        """mm_to_steps on axis without steps_per_mm should raise."""
        cfg = AxisConfig(name="no_mm", steps_per_revolution=200)
        axis = Axis(cfg)
        with pytest.raises((ValueError, TypeError, AttributeError)):
            axis.mm_to_steps(1.0)

    def test_default_configs_exist(self) -> None:
        assert BOBBIN_DEFAULTS.name == "bobbin"
        assert LATERAL_DEFAULTS.name == "lateral"
        assert TENSIONER_DEFAULTS.name == "tensioner"
