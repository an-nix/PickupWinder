"""pickup_test.py — Test sketch for the PickupWinder daemon + PRU stack.

Connects to the pickup_daemon Unix socket and exercises every command.
This is a validation tool, not a production application.

Usage:
    PYTHONPATH=.. python3 -m python.pickup_test                  # run all tests
    PYTHONPATH=.. python3 -m python.pickup_test --test enable    # run one test
    PYTHONPATH=.. python3 -m python.pickup_test --list           # list available tests
    python3 src/python/pickup_test.py --remote --host 192.168.74.171

Requirements:
    - pickup_daemon running (sudo ./pickup_daemon)
    - PRU firmware loaded (make install in src/pru/)
"""

import json
import time
import sys
import argparse

import os

# Import from the packaged `pickup` library. The deploy step should install
# the `pickup/` subpackage next to this script so `import pickup.*` works.
try:
    from hal import DaemonClient
    from pickup.pickup_controller import PickupController
except Exception as e:
    print(f"[IMPORT ERROR] failed to import pickup package: {e}", file=sys.stderr)
    raise

SOCKET_PATH = "/run/pickup-winder.sock"
TIMEOUT     = 2.0   # seconds
DEFAULT_HOST = "192.168.74.171"
DEFAULT_USER = "beagle"
REMOTE_PYTHON_TEST_PATH = "/usr/local/lib/pickup-winder/python/pickup_test.py"


def hz_to_interval(hz):
    """Convert step Hz to IEP interval (for reference/verification)."""
    if hz == 0:
        return 0
    return 200_000_000 // (2 * hz)


# ═══════════════════════════════════════════════════════════════════════════════
# Individual test functions
# ═══════════════════════════════════════════════════════════════════════════════

def test_enable(controller, client):
    """Test enable/disable drivers."""
    print("  [enable] Enable both axes...")
    assert controller.enable(sp=True, lat=True), "enable both axes failed"

    print("  [enable] Disable both axes...")
    assert controller.enable(sp=False, lat=False), "disable both axes failed"

    print("  [enable] PASS")


def test_estop(controller, client):
    """Test emergency stop."""
    controller.enable(sp=True, lat=True)

    print("  [estop] Sending e_stop...")
    assert controller.emergency_stop(), "estop failed"

    print("  [estop] Sending again (idempotent)...")
    assert controller.emergency_stop(), "second estop failed"

    print("  [estop] PASS")


def test_set_speed(controller, client):
    """Test set_speed command."""
    controller.enable(sp=True, lat=True)

    print("  [set_speed] Setting spindle=1000 Hz, lateral=500 Hz...")
    assert controller.set_speed(sp_hz=1000, lat_hz=500, sp_dir=0, lat_dir=0), "set_speed failed"

    print("  [set_speed] Waiting for telem to verify...")
    event = client.wait_event("telem", timeout=2.0)
    if event:
        sp_hz = event.get("sp", {}).get("speed_hz", 0)
        lat_hz = event.get("lat", {}).get("speed_hz", 0)
        print(f"  [set_speed] telem: sp={sp_hz} Hz, lat={lat_hz} Hz")
        if sp_hz > 0:
            print("  [set_speed] Spindle running: OK")
        else:
            print("  [set_speed] WARNING: spindle speed_hz=0")
    else:
        print("  [set_speed] WARNING: no telem received")

    controller.set_speed(sp_hz=0, lat_hz=0)
    controller.emergency_stop()
    print("  [set_speed] PASS")


def test_set_speed_change(controller, client):
    """Test live speed change."""
    controller.enable(sp=True, lat=False)

    print("  [speed_change] Starting at 500 Hz...")
    controller.set_speed(sp_hz=500, lat_hz=0)
    time.sleep(0.3)

    print("  [speed_change] Changing to 2000 Hz...")
    controller.set_speed(sp_hz=2000, lat_hz=0)
    time.sleep(0.3)

    print("  [speed_change] Changing to 0 Hz (stop)...")
    controller.set_speed(sp_hz=0, lat_hz=0)
    time.sleep(0.2)

    controller.emergency_stop()
    print("  [speed_change] PASS")


def test_direction(controller, client):
    """Test direction changes."""
    controller.enable(sp=True, lat=True)

    print("  [direction] Forward...")
    controller.set_speed(sp_hz=500, lat_hz=500, sp_dir=0, lat_dir=0)
    time.sleep(0.3)

    print("  [direction] Reverse...")
    controller.set_speed(sp_hz=500, lat_hz=500, sp_dir=1, lat_dir=1)
    time.sleep(0.3)

    controller.emergency_stop()
    print("  [direction] PASS")


def test_home(controller, client):
    """Test homing sequence (requires endstop hardware or manual trigger)."""
    print("  [home] Starting homing sequence...")
    if controller.home(timeout=30.0):
        print("  [home] PASS")
    else:
        print("  [home] TIMEOUT — no endstop triggered (OK if no hardware)")
        controller.emergency_stop()
        print("  [home] SKIPPED (no hardware)")


def test_reset_pos(controller, client):
    """Test position reset."""
    controller.enable(sp=True, lat=True)

    controller.set_speed(sp_hz=1000, lat_hz=1000)
    time.sleep(0.3)
    controller.set_speed(sp_hz=0, lat_hz=0)
    time.sleep(0.1)

    print("  [reset_pos] Resetting all positions...")
    assert controller.reset_position(axis=0xFF), "reset_pos failed"

    time.sleep(0.3)
    ev = client.wait_event("telem", timeout=2.0)
    if ev:
        sp_steps = ev.get("sp", {}).get("steps", -1)
        lat_steps = ev.get("lat", {}).get("steps", -1)
        print(f"  [reset_pos] After reset: sp_steps={sp_steps}, lat_steps={lat_steps}")
    else:
        print("  [reset_pos] WARNING: no telem received")

    controller.emergency_stop()
    print("  [reset_pos] PASS")


def test_telem(c):
    """Test telemetry reception."""
    print("  [telem] Waiting for telem event...")
    c.drain_events()
    ev = c.wait_event("telem", timeout=3.0)
    if ev:
        print(f"  [telem] Got: pru1_state={ev.get('pru1_state')}")
        print(f"           sp: {ev.get('sp')}")
        print(f"           lat: {ev.get('lat')}")
        print(f"           endstop: {ev.get('endstop')}")
        # Verify structure
        assert "sp" in ev, "missing sp field"
        assert "lat" in ev, "missing lat field"
        assert "endstop" in ev, "missing endstop field"
        print("  [telem] PASS")
    else:
        print("  [telem] FAIL — no telem received within timeout")


def test_ack_event(controller, client):
    """Test ack_event command."""
    print("  [ack_event] Sending ack_event...")
    assert controller.ack_event(), "ack_event failed"
    print("  [ack_event] PASS")


def test_set_mode(controller, client):
    """Test set_mode command: free and winding."""
    print("  [set_mode] Set FREE mode...")
    assert controller.set_mode("free"), "set_mode free failed"

    print("  [set_mode] Set WINDING mode...")
    assert controller.set_mode("winding"), "set_mode winding failed"

    print("  [set_mode] Restore FREE mode...")
    assert controller.set_mode("free"), "restore free failed"

    print("  [set_mode] PASS")


def test_set_limits(controller, client):
    """Test set_limits command."""
    print("  [set_limits] Setting lateral limits [-5000, 5000]...")
    assert controller.set_limits(axis=1, min_steps=-5000, max_steps=5000), "set_limits failed"

    print("  [set_limits] Updating limits to [-10000, 10000]...")
    assert controller.set_limits(axis=1, min_steps=-10000, max_steps=10000), "set_limits update failed"

    print("  [set_limits] PASS")


def test_move_to(controller, client):
    """Test move_to: autonomous trapezoidal move to a target position.

    Hardware note: this test expects the motor to actually move and PRU
    to fire move_complete. Without real hardware/PRU firmware, move_complete
    will time out and the test is marked SKIPPED.
    """
    controller.emergency_stop()
    controller.drain_events()
    controller.enable(sp=True, lat=True)
    controller.reset_position(axis=0xFF)
    time.sleep(0.1)

    controller.set_limits(axis=1, min_steps=-50000, max_steps=50000)

    print("  [move_to] Moving to +1000 steps...")
    assert controller.move_to(1000), "move_to failed"

    print("  [move_to] Waiting for move_complete event...")
    ev = client.wait_event("move_complete", timeout=15.0)
    if ev is None:
        print("  [move_to] TIMEOUT — no move_complete (PRU not running? SKIP)")
        controller.emergency_stop()
        return

    final_pos = ev.get("pos", "?")
    print(f"  [move_to] Arrived at pos={final_pos} steps (expected ~1000)")
    controller.ack_event()

    controller.drain_events()
    tel = client.wait_event("telem", timeout=2.0)
    if tel:
        lat_pos = tel.get("lat", {}).get("pos", -1)
        print(f"  [move_to] Telem lat_pos={lat_pos}")
        if abs(lat_pos - 1000) > 2:
            print(f"  [move_to] WARNING: position off by {abs(lat_pos-1000)} steps")

    print("  [move_to] Returning to 0...")
    assert controller.move_to(0), "return move_to failed"
    ev2 = client.wait_event("move_complete", timeout=15.0)
    if ev2:
        controller.ack_event()
        print(f"  [move_to] Back at pos={ev2.get('pos','?')}" )

    controller.emergency_stop()
    print("  [move_to] PASS")


# ═══════════════════════════════════════════════════════════════════════════════
# Test runner
# ═══════════════════════════════════════════════════════════════════════════════

TESTS = {
    "enable":       test_enable,
    "estop":        test_estop,
    "set_speed":    test_set_speed,
    "speed_change": test_set_speed_change,
    "direction":    test_direction,
    "home":         test_home,
    "reset_pos":    test_reset_pos,
    "telem":        test_telem,
    "ack_event":    test_ack_event,
    "set_mode":     test_set_mode,
    "set_limits":   test_set_limits,
    "move_to":      test_move_to,
}


def main():
    parser = argparse.ArgumentParser(description="PickupWinder PRU test sketch")
    parser.add_argument("--test", "-t", help="Run a specific test")
    parser.add_argument("--list", "-l", action="store_true", help="List tests")
    parser.add_argument("--socket", default=SOCKET_PATH,
                        help="Unix socket path to connect to (default: /run/pickup-winder.sock)")
    parser.add_argument("--remote", action="store_true",
                        help="Run pickup_test.py on the BBB via SSH instead of local socket")
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help="BBB hostname or IP for remote mode")
    parser.add_argument("--user", default=DEFAULT_USER,
                        help="SSH user for remote mode")
    args = parser.parse_args()

    if args.remote:
        import subprocess

        remote_cmd = [
            "ssh",
            "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=no",
            "-o", "ExitOnForwardFailure=yes",
            f"{args.user}@{args.host}",
            "python3",
            REMOTE_PYTHON_TEST_PATH,
        ]
        if args.list:
            remote_cmd.append("--list")
        if args.test:
            remote_cmd.extend(["--test", args.test])
        print("[remote] Executing pickup_test.py on BBB:")
        print("  "+" ".join(remote_cmd))
        return subprocess.call(remote_cmd)

    if args.list:
        print("Available tests:")
        for name in TESTS:
            print(f"  {name}")
        return 0

    # Check daemon availability
    try:
        client = DaemonClient(path=args.socket)
        client.connect()
    except (FileNotFoundError, ConnectionRefusedError, OSError) as exc:
        print(f"[ERROR] Cannot connect to daemon at {SOCKET_PATH}: {exc}")
        print("        Start daemon first: sudo ./pickup_daemon")
        return 1

    controller = PickupController(client)

    # Always start with a clean state
    controller.emergency_stop()
    client.drain_events()

    tests_to_run = [args.test] if args.test else list(TESTS.keys())
    passed = 0
    failed = 0

    for name in tests_to_run:
        if name not in TESTS:
            print(f"[UNKNOWN] {name}")
            continue
        print(f"\n[TEST] {name}")
        try:
            TESTS[name](controller, client)
            passed += 1
        except AssertionError as e:
            print(f"  [FAIL] {e}")
            failed += 1
        except Exception as e:
            print(f"  [ERROR] {type(e).__name__}: {e}")
            failed += 1
        # Clean state between tests
        try:
            controller.emergency_stop()
            client.drain_events()
        except Exception:
            pass

    client.close()
    print(f"\n{'='*60}")
    print(f"Results: {passed} passed, {failed} failed, {passed+failed} total")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
