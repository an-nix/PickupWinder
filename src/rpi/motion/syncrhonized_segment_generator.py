from __future__ import annotations

import warnings

warnings.warn(
    "motion.syncrhonized_segment_generator is deprecated, use motion.synchronized_segment_generator",
    DeprecationWarning,
    stacklevel=2,
)

from .synchronized_segment_generator import *