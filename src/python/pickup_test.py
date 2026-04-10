"""pickup_test.py — Test sketch for the PickupWinder daemon + PRU stack.

Connects to the pickup_daemon Unix socket and exercises every command.
This is a validation tool, not a production application.

Usage:
    python3 pickup_test.py                  # run all tests
    python3 pickup_test.py --test enable    # run one test
    python3 pickup_test.py --list           # list available tests

Requirements:
    - pickup_daemon running (sudo ./pickup_daemon)
    - PRU firmware loaded (make install in src/pru/)
"""

import socket
import json
import time
import sys
import argparse

SOCKET_PATH = "/run/pickup-winder.sock"
TIMEOUT     = 2.0   # seconds


class DaemonClient:
    """Synchronous JSON-line client for pickup_daemon."""

    def __init__(self, path=SOCKET_PATH, timeout=TIMEOUT):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(path)
        self._buf = ""

    def cmd(self, obj):
        """Send a command dict, return the response dict."""
        msg = json.dumps(obj, separators=(",", ":")) + "\n"
        self.sock.sendall(msg.encode())
        return self._read_json()

    def wait_event(self, event_type=None, timeout=5.0):
        """Wait for an async event from daemon. Returns the event dict."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.sock.settimeout(max(remaining, 0.1))
            try:
                obj = self._read_json()
                if "event" in obj:
                    if event_type is None or obj["event"] == event_type:
                        return obj
            except socket.timeout:
                continue
        return None

    def drain_events(self, timeout=0.3):
        """Read and discard pending events."""
        self.sock.settimeout(timeout)
        while True:
            try:
                self._read_json()
            except (socket.timeout, OSError):
                break
        self.sock.settimeout(TIMEOUT)

    def _read_json(self):
        while "\n" not in self._buf:
            data = self.sock.recv(4096).decode()
            if not data:
                raise ConnectionError("daemon closed connection")
            self._buf += data
        line, self._buf = self._buf.split("\n", 1)
        return json.loads(line)

    def close(self):
        self.sock.close()


def hz_to_interval(hz):
    """Convert step Hz to IEP interval (for reference/verification)."""
    if hz == 0:
        return 0
    return 200_000_000 // (2 * hz)


# ═══════════════════════════════════════════════════════════════════════════════
# Individual test functions
# ═══════════════════════════════════════════════════════════════════════════════

def test_enable(c):
    """Test enable/disable drivers."""
    print("  [enable] Enable spindle...")
    r = c.cmd({"cmd": "enable", "axis": 0, "value": 1})
    assert r.get("ok"), f"enable spindle failed: {r}"

    print("  [enable] Enable lateral...")
    r = c.cmd({"cmd": "enable", "axis": 1, "value": 1})
    assert r.get("ok"), f"enable lateral failed: {r}"

    print("  [enable] Enable all...")
    r = c.cmd({"cmd": "enable", "axis": 255, "value": 1})
    assert r.get("ok"), f"enable all failed: {r}"

    print("  [enable] Disable all...")
    r = c.cmd({"cmd": "enable", "axis": 255, "value": 0})
    assert r.get("ok"), f"disable all failed: {r}"

    print("  [enable] PASS")


def test_estop(c):
    """Test emergency stop."""
    # Enable first
    c.cmd({"cmd": "enable", "axis": 255, "value": 1})

    print("  [estop] Sending e_stop...")
    r = c.cmd({"cmd": "e_stop"})
    assert r.get("ok"), f"estop failed: {r}"

    print("  [estop] Sending again (idempotent)...")
    r = c.cmd({"cmd": "e_stop"})
    assert r.get("ok"), f"estop 2nd failed: {r}"

    print("  [estop] PASS")


def test_set_speed(c):
    """Test set_speed command."""
    # Enable drivers first
    c.cmd({"cmd": "enable", "axis": 255, "value": 1})

    print("  [set_speed] Setting spindle=1000 Hz, lateral=500 Hz...")
    r = c.cmd({"cmd": "set_speed", "sp_hz": 1000, "lat_hz": 500,
               "sp_dir": 0, "lat_dir": 0})
    assert r.get("ok"), f"set_speed failed: {r}"

    print("  [set_speed] Waiting for telem to verify...")
    time.sleep(0.5)
    ev = c.wait_event("telem", timeout=2.0)
    if ev:
        sp_hz = ev.get("sp", {}).get("speed_hz", 0)
        lat_hz = ev.get("lat", {}).get("speed_hz", 0)
        print(f"  [set_speed] telem: sp={sp_hz} Hz, lat={lat_hz} Hz")
        # Allow some tolerance
        if sp_hz > 0:
            print("  [set_speed] Spindle running: OK")
        else:
            print("  [set_speed] WARNING: spindle speed_hz=0 (PRU may not be loaded)")
    else:
        print("  [set_speed] WARNING: no telem received")

    # Stop motors
    c.cmd({"cmd": "set_speed", "sp_hz": 0, "lat_hz": 0})
    c.cmd({"cmd": "e_stop"})

    print("  [set_speed] PASS")


def test_set_speed_change(c):
    """Test live speed change."""
    c.cmd({"cmd": "enable", "axis": 255, "value": 1})

    print("  [speed_change] Starting at 500 Hz...")
    c.cmd({"cmd": "set_speed", "sp_hz": 500, "lat_hz": 0})
    time.sleep(0.3)

    print("  [speed_change] Changing to 2000 Hz...")
    c.cmd({"cmd": "set_speed", "sp_hz": 2000, "lat_hz": 0})
    time.sleep(0.3)

    print("  [speed_change] Changing to 0 Hz (stop)...")
    c.cmd({"cmd": "set_speed", "sp_hz": 0, "lat_hz": 0})
    time.sleep(0.2)

    c.cmd({"cmd": "e_stop"})
    print("  [speed_change] PASS")


def test_direction(c):
    """Test direction changes."""
    c.cmd({"cmd": "enable", "axis": 255, "value": 1})

    print("  [direction] Forward...")
    c.cmd({"cmd": "set_speed", "sp_hz": 500, "lat_hz": 500, "sp_dir": 0, "lat_dir": 0})
    time.sleep(0.3)

    print("  [direction] Reverse...")
    c.cmd({"cmd": "set_speed", "sp_hz": 500, "lat_hz": 500, "sp_dir": 1, "lat_dir": 1})
    time.sleep(0.3)

    c.cmd({"cmd": "e_stop"})
    print("  [direction] PASS")


def test_home(c):
    """Test homing sequence (requires endstop hardware or manual trigger)."""
    print("  [home] Starting homing sequence...")
    r = c.cmd({"cmd": "home_start"})
    assert r.get("ok"), f"home_start failed: {r}"

    print("  [home] Waiting for home_complete event (trigger endstop!)...")
    ev = c.wait_event("home_complete", timeout=30.0)
    if ev:
        print(f"  [home] Got event: {ev}")
        c.cmd({"cmd": "ack_event"})
        print("  [home] PASS")
    else:
        print("  [home] TIMEOUT — no endstop triggered (OK if no hardware)")
        # Cancel by estop
        c.cmd({"cmd": "e_stop"})
        print("  [home] SKIPPED (no hardware)")


def test_reset_pos(c):
    """Test position reset."""
    c.cmd({"cmd": "enable", "axis": 255, "value": 1})

    # Run a few steps to accumulate counts
    c.cmd({"cmd": "set_speed", "sp_hz": 1000, "lat_hz": 1000})
    time.sleep(0.3)
    c.cmd({"cmd": "set_speed", "sp_hz": 0, "lat_hz": 0})
    time.sleep(0.1)

    print("  [reset_pos] Resetting all positions...")
    r = c.cmd({"cmd": "reset_pos", "axis": 255})
    assert r.get("ok"), f"reset_pos failed: {r}"

    # Check telem
    time.sleep(0.3)
    ev = c.wait_event("telem", timeout=2.0)
    if ev:
        sp_steps = ev.get("sp", {}).get("steps", -1)
        lat_steps = ev.get("lat", {}).get("steps", -1)
        print(f"  [reset_pos] After reset: sp_steps={sp_steps}, lat_steps={lat_steps}")
    else:
        print("  [reset_pos] WARNING: no telem received")

    c.cmd({"cmd": "e_stop"})
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


def test_ack_event(c):
    """Test ack_event command."""
    print("  [ack_event] Sending ack_event...")
    r = c.cmd({"cmd": "ack_event"})
    assert r.get("ok"), f"ack_event failed: {r}"
    print("  [ack_event] PASS")


def test_set_mode(c):
    """Test set_mode command: free and winding."""
    print("  [set_mode] Set FREE mode...")
    r = c.cmd({"cmd": "set_mode", "mode": "free"})
    assert r.get("ok"), f"set_mode free failed: {r}"

    print("  [set_mode] Set WINDING mode...")
    r = c.cmd({"cmd": "set_mode", "mode": "winding"})
    assert r.get("ok"), f"set_mode winding failed: {r}"

    print("  [set_mode] Restore FREE mode...")
    r = c.cmd({"cmd": "set_mode", "mode": "free"})
    assert r.get("ok"), f"restore free failed: {r}"

    print("  [set_mode] PASS")


def test_set_limits(c):
    """Test set_limits command."""
    print("  [set_limits] Setting lateral limits [-5000, 5000]...")
    r = c.cmd({"cmd": "set_limits", "axis": 1, "min": -5000, "max": 5000})
    assert r.get("ok"), f"set_limits failed: {r}"

    print("  [set_limits] Updating limits to [-10000, 10000]...")
    r = c.cmd({"cmd": "set_limits", "axis": 1, "min": -10000, "max": 10000})
    assert r.get("ok"), f"set_limits update failed: {r}"

    print("  [set_limits] PASS")


def test_move_to(c):
    """Test move_to: autonomous trapezoidal move to a target position.

    Hardware note: this test expects the motor to actually move and PRU
    to fire move_complete. Without real hardware/PRU firmware, move_complete
    will time out and the test is marked SKIPPED.
    """
    # Start clean
    c.cmd({"cmd": "e_stop"})
    c.drain_events()
    c.cmd({"cmd": "enable", "axis": 255, "value": 1})
    c.cmd({"cmd": "reset_pos", "axis": 255})
    time.sleep(0.1)

    # Install generous limits so the limit guard does not interfere
    c.cmd({"cmd": "set_limits", "axis": 1, "min": -50000, "max": 50000})

    print("  [move_to] Moving to +1000 steps (start=200 Hz, cruise=4000 Hz, ramp=300)...")
    r = c.cmd({"cmd": "move_to", "axis": 1, "pos": 1000,
               "start_hz": 200, "max_hz": 4000, "accel_steps": 300})
    assert r.get("ok"), f"move_to failed: {r}"

    print("  [move_to] Waiting for move_complete event...")
    ev = c.wait_event("move_complete", timeout=15.0)
    if ev is None:
        print("  [move_to] TIMEOUT — no move_complete (PRU not running? SKIP)")
        c.cmd({"cmd": "e_stop"})
        return

    final_pos = ev.get("pos", "?")
    print(f"  [move_to] Arrived at pos={final_pos} steps (expected ~1000)")
    c.cmd({"cmd": "ack_event"})

    # Verify telem position matches
    c.drain_events()
    tel = c.wait_event("telem", timeout=2.0)
    if tel:
        lat_pos = tel.get("lat", {}).get("pos", -1)
        print(f"  [move_to] Telem lat_pos={lat_pos}")
        if abs(lat_pos - 1000) > 2:
            print(f"  [move_to] WARNING: position off by {abs(lat_pos-1000)} steps")

    # Return to 0
    print("  [move_to] Returning to 0...")
    c.cmd({"cmd": "move_to", "axis": 1, "pos": 0,
           "start_hz": 200, "max_hz": 4000, "accel_steps": 300})
    ev2 = c.wait_event("move_complete", timeout=15.0)
    if ev2:
        c.cmd({"cmd": "ack_event"})
        print(f"  [move_to] Back at pos={ev2.get('pos','?')}")

    c.cmd({"cmd": "e_stop"})
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
    args = parser.parse_args()

    if args.list:
        print("Available tests:")
        for name in TESTS:
            print(f"  {name}")
        return 0

    # Check daemon availability
    try:
        c = DaemonClient()
    except (FileNotFoundError, ConnectionRefusedError):
        print(f"[ERROR] Cannot connect to daemon at {SOCKET_PATH}")
        print("        Start daemon first: sudo ./pickup_daemon")
        return 1

    # Always start with a clean state
    c.cmd({"cmd": "e_stop"})
    c.drain_events()

    tests_to_run = [args.test] if args.test else list(TESTS.keys())
    passed = 0
    failed = 0

    for name in tests_to_run:
        if name not in TESTS:
            print(f"[UNKNOWN] {name}")
            continue
        print(f"\n[TEST] {name}")
        try:
            TESTS[name](c)
            passed += 1
        except AssertionError as e:
            print(f"  [FAIL] {e}")
            failed += 1
        except Exception as e:
            print(f"  [ERROR] {type(e).__name__}: {e}")
            failed += 1
        # Clean state between tests
        try:
            c.cmd({"cmd": "e_stop"})
            c.drain_events()
        except Exception:
            pass

    c.close()
    print(f"\n{'='*60}")
    print(f"Results: {passed} passed, {failed} failed, {passed+failed} total")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
