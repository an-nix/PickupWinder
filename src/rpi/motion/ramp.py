from __future__ import annotations

"""Compatibility shim for legacy motion imports.

This module exposes the old motion names while the real implementation
has moved into `multi_axis_segment_generator.py`.

Deprecated: direct imports from motion.multi_axis_segment_generator
are preferred. This shim will be removed in a future version.
"""

import warnings

warnings.warn(
    "motion.ramp is deprecated, use motion.multi_axis_segment_generator",
    DeprecationWarning,
    stacklevel=2,
)

from .multi_axis_segment_generator import AxisMotionConfig, MultiAxisSegmentGenerator

__all__ = ["AxisMotionConfig", "MultiAxisSegmentGenerator"]
