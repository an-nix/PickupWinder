"""axis.py — Axis abstraction with unit conversion.

MIGRATION: Replaces the BBB hal/hardware_definition.py and core/axis.py.
On BBB, unit conversion was done in the daemon (Hz→IEP interval) and
the Python Axis was a simple dataclass.  Here the Axis class owns
the full unit-aware API since the SPI protocol speaks in Hz and steps.

Usage:
    bobbin = Axis.from_config("bobbin", config["axes"]["bobbin"])
    hz = bobbin.rpm_to_hz(1500)
    steps = lateral.mm_to_steps(5.0)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class AxisConfig:
    """Axis hardware configuration (loaded from YAML)."""

    name: str
    steps_per_revolution: int = 200
    microstepping: int = 16
    steps_per_mm: Optional[float] = None  # None for rotary-only axes
    max_hz: int = 160_000
    min_hz: int = 100
    max_rpm: float = 1500.0
    default_accel: int = 10_000  # steps/s²
    home_hz: int = 2000
    home_direction_reverse: bool = True

    @property
    def full_steps_per_rev(self) -> int:
        """Total microsteps per revolution."""
        return self.steps_per_revolution * self.microstepping


class Axis:
    """Unit-aware axis abstraction.

    Provides conversion between physical units (RPM, mm) and the
    step-based units used by the ESP32 SPI protocol.

    MIGRATION: On BBB the daemon did Hz→interval conversion.  On ESP32
    the protocol uses Hz directly, so this class converts RPM→Hz and
    mm→steps only.  No interval math needed.
    """

    def __init__(self, config: AxisConfig) -> None:
        self._cfg = config

    @classmethod
    def from_config(cls, name: str, cfg_dict: dict) -> "Axis":
        """Create an Axis from a configuration dictionary.

        Expected keys match AxisConfig fields.
        """
        return cls(AxisConfig(name=name, **cfg_dict))

    @property
    def config(self) -> AxisConfig:
        return self._cfg

    @property
    def name(self) -> str:
        return self._cfg.name

    # ── RPM ↔ Hz conversion ──────────────────────────────────────────────────

    def rpm_to_hz(self, rpm: float) -> int:
        """Convert RPM to step frequency (Hz).

        MIGRATION: On BBB, this was `rpm * steps_per_rev / 60`.
        Identical formula.  Returns clamped integer Hz.
        """
        hz = rpm * self._cfg.full_steps_per_rev / 60.0
        return self._clamp_hz(int(hz))

    def hz_to_rpm(self, hz: int) -> float:
        """Convert step frequency (Hz) to RPM."""
        if self._cfg.full_steps_per_rev == 0:
            return 0.0
        return hz * 60.0 / self._cfg.full_steps_per_rev

    # ── mm ↔ steps conversion ────────────────────────────────────────────────

    def mm_to_steps(self, mm: float) -> int:
        """Convert millimeters to steps.

        Raises ValueError if this axis has no steps_per_mm configured.
        """
        if self._cfg.steps_per_mm is None:
            raise ValueError(f"Axis '{self._cfg.name}' has no steps_per_mm")
        return int(mm * self._cfg.steps_per_mm)

    def steps_to_mm(self, steps: int) -> float:
        """Convert steps to millimeters."""
        if self._cfg.steps_per_mm is None:
            raise ValueError(f"Axis '{self._cfg.name}' has no steps_per_mm")
        return steps / self._cfg.steps_per_mm

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _clamp_hz(self, hz: int) -> int:
        """Clamp Hz to valid range."""
        if hz <= 0:
            return 0
        return max(self._cfg.min_hz, min(self._cfg.max_hz, hz))

    def __repr__(self) -> str:
        return (
            f"Axis('{self._cfg.name}', "
            f"{self._cfg.full_steps_per_rev} steps/rev, "
            f"max {self._cfg.max_rpm} RPM)"
        )


# ── Predefined axis configurations ──────────────────────────────────────────
# MIGRATION: On BBB these were in HardwareDefinition dataclass.
# Here they serve as defaults; actual values come from machine_config.yaml.

BOBBIN_DEFAULTS = AxisConfig(
    name="bobbin",
    steps_per_revolution=200,
    microstepping=32,
    steps_per_mm=None,
    max_hz=160_000,
    min_hz=100,
    max_rpm=1500.0,
    default_accel=10_000,
)

LATERAL_DEFAULTS = AxisConfig(
    name="lateral",
    steps_per_revolution=96,
    microstepping=32,
    steps_per_mm=3072,
    max_hz=160_000,
    min_hz=100,
    max_rpm=500.0,
    default_accel=100_000,
    home_hz=2000,
    home_direction_reverse=True,
)

TENSIONER_DEFAULTS = AxisConfig(
    name="tensioner",
    steps_per_revolution=200,
    microstepping=16,
    steps_per_mm=None,
    max_hz=50_000,
    min_hz=100,
    max_rpm=200.0,
    default_accel=5_000,
)
