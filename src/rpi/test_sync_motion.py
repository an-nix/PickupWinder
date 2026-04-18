#!/usr/bin/env python3
"""
Test simple de motion synchrone (sans async) pour vérifier si c'est un problème
de thread ou un problème de hardware/SPI.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from domain import AppConfiguration
from motion.axis import Axis
from motion.axis_controller import AxisController
from transport import Esp32SpiTransport


def main() -> int:
    print("=" * 70)
    print("SYNC STREAMING TEST (no background threads)")
    print("=" * 70)
    
    try:
        # Initialize hardware
        print("\n1. Initializing SPI transport...")
        config = AppConfiguration()
        transport = Esp32SpiTransport(
            bus=config.spi_bus,
            device=config.spi_device,
            speed_hz=config.spi_speed_hz,
            mode=0,
        )
        print("   ✓ SPI transport initialized")
        
        # Create axis
        print("\n2. Creating spindle axis...")
        spindle = Axis(
            axis_id=config.spindle_axis_id,
            name="spindle",
            can_move_without_homing=True,
        )
        print(f"   ✓ Axis created (id={spindle.axis_id})")
        
        # Create controller WITHOUT streaming_manager (disable async)
        print("\n3. Creating axis controller (sync mode)...")
        controller = AxisController(spindle, transport)
        print("   ✓ Controller created")
        
        # Test motion WITHOUT async (directly call stream_all)
        print("\n4. Testing motion (SYNC, no background threads)...")
        from motion.ramp import RampConfig
        from transport import StreamAxisConfig, MultiAxisRampStreamer
        
        duration_s = 3.0
        target_rpm = 100.0
        
        accel_s = min(0.5, duration_s * 0.25)
        decel_s = accel_s
        cruise_s = max(duration_s - accel_s - decel_s, 0.0)
        
        ramp = RampConfig(
            axis_id=spindle.axis_id,
            target_rpm=target_rpm,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
        )
        
        stream_axis = StreamAxisConfig(axis_id=spindle.axis_id, ramp=ramp)
        streamer = MultiAxisRampStreamer(
            transport,
            [stream_axis],
            poll_interval_s=0.01,
            print_every=1,
        )
        
        print(f"   Starting 3-second spindle motion at {target_rpm} RPM...")
        import time
        start = time.monotonic()
        block_count = streamer.stream_all()
        elapsed = time.monotonic() - start
        
        print(f"   ✓ Motion completed in {elapsed:.2f}s")
        print(f"   ✓ {block_count} blocks sent")
        
        if elapsed > duration_s * 1.5:
            print(f"   ⚠️  WARNING: Took longer than expected ({elapsed:.2f}s vs {duration_s}s)")
        
        print("\n" + "=" * 70)
        print("✅ SYNC TEST PASSED - Hardware is working!")
        print("=" * 70)
        print("\nIf motor turned, the issue is with async threading.")
        print("If motor didn't turn, check SPI connection.")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        try:
            transport.close()
        except:
            pass


if __name__ == "__main__":
    sys.exit(main())
