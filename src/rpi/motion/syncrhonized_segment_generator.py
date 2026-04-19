from __future__ import annotations

import warnings

warnings.warn(
    "syncrhonized_segment_generator is deprecated, use synchronized_segment_generator",
    DeprecationWarning,
    stacklevel=2,
)

from .synchronized_segment_generator import (  # noqa: F401
    SyncAxisConfig,
    SynchronizedSegmentGenerator,
)


