"""Top-level `transport` shim to support both:
- Local execution: `python3 demo_spi.py` (direct copy of transport/ folder)
- Package execution: `sys.path.insert(0, 'src'); import rpi.demo_spi`
"""

# Lazy imports to avoid circular dependencies
def __getattr__(name: str):
    if name == "Esp32SpiTransport":
        from rpi.transport.spi_transport import Esp32SpiTransport
        return Esp32SpiTransport
    if name == "StreamAxisConfig":
        from rpi.transport.streamer import StreamAxisConfig
        return StreamAxisConfig
    if name == "MultiAxisRampStreamer":
        from rpi.transport.streamer import MultiAxisRampStreamer
        return MultiAxisRampStreamer
    # Message types
    if name == "SPI_FRAME_SIZE":
        from rpi.transport.messages import SPI_FRAME_SIZE
        return SPI_FRAME_SIZE
    if name == "StatusPayload":
        from rpi.transport.messages import StatusPayload
        return StatusPayload
    if name == "MultiAxisSegmentBlockPayload":
        from rpi.transport.messages import MultiAxisSegmentBlockPayload
        return MultiAxisSegmentBlockPayload
    if name == "LATERAL_ENDSTOP_ABSENT":
        from rpi.transport.messages import LATERAL_ENDSTOP_ABSENT
        return LATERAL_ENDSTOP_ABSENT
    if name == "LATERAL_ENDSTOP_PRESENT_CLOSED":
        from rpi.transport.messages import LATERAL_ENDSTOP_PRESENT_CLOSED
        return LATERAL_ENDSTOP_PRESENT_CLOSED
    if name == "LATERAL_ENDSTOP_PRESENT_OPEN":
        from rpi.transport.messages import LATERAL_ENDSTOP_PRESENT_OPEN
        return LATERAL_ENDSTOP_PRESENT_OPEN
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return [
        "Esp32SpiTransport",
        "StreamAxisConfig",
        "MultiAxisRampStreamer",
        "SPI_FRAME_SIZE",
        "StatusPayload",
        "MultiAxisSegmentBlockPayload",
        "LATERAL_ENDSTOP_ABSENT",
        "LATERAL_ENDSTOP_PRESENT_CLOSED",
        "LATERAL_ENDSTOP_PRESENT_OPEN",
    ]
