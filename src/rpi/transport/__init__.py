from __future__ import annotations

from .messages import (
    SPI_FRAME_SIZE,
    SPI_MAX_AXES,
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    SpiMessageType,
    SpiMessageResult,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_ABSENT,
    MessageHeader,
    EnableAxisPayload,
    EmergencyStopPayload,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    FlushPayload,
    EnableEndstopPayload,
    StatusPayload,
    crc16_ccitt,
    build_frame,
    parse_status_frame,
    make_nop,
    make_get_status,
    make_enable_axis,
    make_estop,
    make_stop_axis,
    make_disable_all,
    make_reset_stats,
    make_multi_axis_segment_block,
    make_flush,
    make_enable_endstop,
)
from .spi_transport import Esp32SpiTransport
from .mock_spi_transport import MockSpiTransport

# Lazy import of streamer to break circular dependency
def __getattr__(name: str):
    if name == "StreamAxisConfig":
        from .streamer import StreamAxisConfig
        return StreamAxisConfig
    if name == "MultiAxisRampStreamer":
        from .streamer import MultiAxisRampStreamer
        return MultiAxisRampStreamer
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return [
        "SPI_FRAME_SIZE",
        "SPI_MAX_AXES",
        "MULTI_AXIS_SEGMENT_BLOCK_SIZE",
        "SpiMessageType",
        "SpiMessageResult",
        "LATERAL_ENDSTOP_PRESENT_OPEN",
        "LATERAL_ENDSTOP_PRESENT_CLOSED",
        "LATERAL_ENDSTOP_ABSENT",
        "MessageHeader",
        "EnableAxisPayload",
        "EmergencyStopPayload",
        "MultiAxisSegment",
        "MultiAxisSegmentBlockPayload",
        "FlushPayload",
        "EnableEndstopPayload",
        "StatusPayload",
        "crc16_ccitt",
        "build_frame",
        "parse_status_frame",
        "make_nop",
        "make_get_status",
        "make_enable_axis",
        "make_estop",
        "make_stop_axis",
        "make_disable_all",
        "make_reset_stats",
        "make_multi_axis_segment_block",
        "make_flush",
        "make_enable_endstop",
        "Esp32SpiTransport",
        "StreamAxisConfig",
        "MultiAxisRampStreamer",
    ]
