from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

from transport.messages import MultiAxisSegment

JSON_VERSION = 1


@dataclass(slots=True)
class SegmentJsonDocument:
    version: int
    axis_ids: list[int]
    metadata: dict[str, Any]
    segments: list[MultiAxisSegment]


def segment_to_dict(segment: MultiAxisSegment) -> dict[str, Any]:
    return {
        "sequence": int(segment.sequence),
        "duration_us": int(segment.duration_us),
        "steps": [int(step) for step in segment.steps],
        "directions": [int(direction) for direction in segment.directions],
    }


def segment_from_dict(data: dict[str, Any]) -> MultiAxisSegment:
    if not isinstance(data, dict):
        raise ValueError("segment record must be a JSON object")

    steps = list(data["steps"])
    directions = list(data["directions"])
    if len(steps) != len(directions):
        raise ValueError("segment steps and directions lengths differ")

    return MultiAxisSegment(
        sequence=int(data["sequence"]),
        duration_us=int(data["duration_us"]),
        steps=[int(step) for step in steps],
        directions=[int(direction) for direction in directions],
    )


def infer_axis_ids(segments: list[MultiAxisSegment]) -> list[int]:
    if not segments:
        return []
    axis_count = len(segments[0].steps)
    for segment in segments:
        if len(segment.steps) != axis_count:
            raise ValueError("all segments must have the same axis count")
    return list(range(axis_count))


def dump_segment_json(
    path: Path | str,
    segments: Iterator[MultiAxisSegment],
    *,
    axis_ids: list[int] | None = None,
    metadata: dict[str, Any] | None = None,
) -> None:
    path = Path(path)
    segment_list = list(segments)
    if axis_ids is None:
        axis_ids = infer_axis_ids(segment_list)
    if metadata is None:
        metadata = {}

    document = {
        "version": JSON_VERSION,
        "axis_ids": axis_ids,
        "metadata": metadata,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "segments": [segment_to_dict(segment) for segment in segment_list],
    }
    with path.open("w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2)


def load_segment_json(path: Path | str) -> tuple[list[int], dict[str, Any], Iterator[MultiAxisSegment]]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        document = json.load(handle)

    version = int(document.get("version", 0))
    if version != JSON_VERSION:
        raise ValueError(f"unsupported segment JSON version: {version}")

    axis_ids = list(document.get("axis_ids", []))
    metadata = dict(document.get("metadata", {}))
    segments_raw = document.get("segments")
    if not isinstance(segments_raw, list):
        raise ValueError("segment JSON file is missing 'segments' list")

    segments = [segment_from_dict(item) for item in segments_raw]
    if not axis_ids:
        axis_ids = infer_axis_ids(segments)
    if segments and len(axis_ids) != len(segments[0].steps):
        raise ValueError("axis_ids length does not match segment step count")

    return axis_ids, metadata, iter(segments)


def load_segments(path: Path | str) -> list[MultiAxisSegment]:
    _axis_ids, _metadata, iterator = load_segment_json(path)
    return list(iterator)
