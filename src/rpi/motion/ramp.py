from __future__ import annotations

"""Compatibility shim for legacy motion imports.

This module exposes the old motion names while the real implementation
has moved into `multi_axis_segment_generator.py`.
"""

from .multi_axis_segment_generator import AxisMotionConfig, MultiAxisSegmentGenerator

__all__ = ["AxisMotionConfig", "MultiAxisSegmentGenerator"]
