#!/usr/bin/env python3
"""
Test spindle axis smooth motion.

This script tests if the spindle axis maintains smooth motion when:
1. Speed is set
2. A target is given
3. More SET_SPEED commands are sent while motor is moving
4. A new target is given before old one completes

Expected ESP32 log output should show:
- → SET_SPEED axis=0 hz=...
- → MOVE_ABS axis=0 target=...
- Axis 0 run: idle->running hz=... (or similar)
- Axis 0 move_to: target=..., running=1
"""

import asyncio
import sys
sys.path.insert(0, '/home/nicolas/Documents/PlatformIO/Projects/PickupWinder/src/rpi')

from hal.esp32_controller import EspController, AxisId, EventType
from hal.spi_transport import SpiTransport

async def test_smooth_spindle():
    """Test spindle axis smooth motion with multiple speed/target changes."""
    
    transport = SpiTransport(port='/dev/spidev0.0', max_speed_hz=4_000_000)
    ctrl = EspController(transport)
    spindle = AxisId.BOBBIN
    
    print("[*] Starting ESP32 controller...")
    await ctrl.start()
    
    try:
        # Initial status
        status = await ctrl.get_status()
        print(f"[+] Spindle position: {status.axes[int(spindle)].position}, hz: {status.axes[int(spindle)].current_hz}")
        print(f"[+] Lateral position: {status.axes[int(AxisId.LATERAL)].position}, hz: {status.axes[int(AxisId.LATERAL)].current_hz}")
        print(f"[+] Tensioner position: {status.axes[int(AxisId.TENSIONER)].position}, hz: {status.axes[int(AxisId.TENSIONER)].current_hz}")
        
        # Enable spindle axis
        print("\n[*] Enabling spindle axis...")
        await ctrl.set_enabled(spindle, True)
        await asyncio.sleep(0.2)
        
        # Test 1: Simple speed change
        print("\n[TEST 1] Simple speed change (should see: idle->running, then speed updates)")
        print("[*] Setting spindle speed to 1000 Hz...")
        await ctrl.set_speed(spindle, hz=1000)
        await asyncio.sleep(0.1)
        
        status = await ctrl.get_status()
        print(f"[+] Spindle hz: {status.axes[int(spindle)].current_hz}")
        
        print("[*] Sending speed to 2000 Hz (motor already running)...")
        await ctrl.set_speed(spindle, hz=2000)
        await asyncio.sleep(0.1)
        
        status = await ctrl.get_status()
        print(f"[+] Spindle hz: {status.axes[int(spindle)].current_hz}")
        
        # Test 2: Speed + target
        print("\n[TEST 2] Speed change with target (should see: MOVE_ABS after SET_SPEED)")
        current_pos = status.axes[int(spindle)].position
        target = current_pos + 6400  # Move one full revolution
        
        print(f"[*] Current position: {current_pos}")
        print(f"[*] Setting speed to 5000 Hz...")
        await ctrl.set_speed(spindle, hz=5000)
        await asyncio.sleep(0.05)
        
        print(f"[*] Moving to target: {target}")
        await ctrl.move_to(spindle, target)
        await asyncio.sleep(0.5)
        
        status = await ctrl.get_status()
        print(f"[+] Spindle position: {status.axes[int(spindle)].position}, hz: {status.axes[int(spindle)].current_hz}")
        
        # Test 3: Multiple speed changes while moving
        print("\n[TEST 3] Speed changes while moving to target")
        current_pos = status.axes[int(spindle)].position
        target = current_pos + 12800  # Move two full revolutions
        
        print(f"[*] Current position: {current_pos}")
        print(f"[*] Setting speed to 8000 Hz and moving to {target}...")
        await ctrl.set_speed(spindle, hz=8000)
        await asyncio.sleep(0.05)
        await ctrl.move_to(spindle, target)
        
        # Send more speed changes while moving
        for i in range(3):
            await asyncio.sleep(0.1)
            new_hz = 8000 + (i+1) * 2000
            print(f"[*] Speed change #{i+1}: {new_hz} Hz (motor should continue smoothly)")
            await ctrl.set_speed(spindle, hz=new_hz)
        
        # Wait for move to complete (or timeout)
        print("[*] Waiting for move to complete...")
        try:
            await asyncio.wait_for(
                ctrl.wait_for_event(EventType.MOVE_COMPLETE, spindle),
                timeout=10.0
            )
            print("[+] MOVE_COMPLETE event received!")
        except asyncio.TimeoutError:
            print("[!] MOVE_COMPLETE timeout (motor may be stalled or not reporting)")
        
        await ctrl.ack_event(spindle)
        
        status = await ctrl.get_status()
        print(f"[+] Final spindle position: {status.axes[int(spindle)].position}, hz: {status.axes[int(spindle)].current_hz}")
        
        # Stop
        print("\n[*] Stopping spindle axis...")
        await ctrl.set_speed(spindle, hz=0)
        await asyncio.sleep(0.2)
        
        print("\n[SUCCESS] Test completed!")
        print("\nExpected log patterns (check 'platformio device monitor'):")
        print("  → SET_SPEED axis=0 hz=1000")
        print("  Axis 0 run: idle->running hz=1000")
        print("  → SET_SPEED axis=0 hz=2000")
        print("  Axis 0 run: speed update 1000->2000 Hz  (NO ramp restart)")
        print("  → MOVE_ABS axis=0 target=XXXX")
        print("  Axis 0 move_to: target=XXXX, running=1")
        
    finally:
        await ctrl.stop()

if __name__ == '__main__':
    asyncio.run(test_smooth_spindle())
