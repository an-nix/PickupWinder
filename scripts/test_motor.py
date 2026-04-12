#!/usr/bin/env python3
"""
test_motor.py — PickupWinder motor rotation test script.

Tests spindle and lateral motor rotation at various speeds and directions
via the pickup_daemon Unix socket.

Usage:
    python3 scripts/test_motor.py [OPTIONS]

Options:
    --host HOST     BBB hostname or IP (default: 192.168.74.171)
    --port PORT     SSH port for remote execution (default: 22)
    --local         Connect to local socket (daemon running locally)
    --socket PATH   Unix socket path (default: /run/pickup-winder.sock)
    --spindle-only  Test spindle axis only
    --lateral-only  Test lateral axis only (requires homed lateral)
    --no-home       Skip lateral homing (lateral tests use absolute pos)
    -v, --verbose   Print all telemetry lines

Examples:
    python3 scripts/test_motor.py                  # run via SSH tunnel
    python3 scripts/test_motor.py --local          # daemon on localhost
    python3 scripts/test_motor.py --spindle-only   # spindle only

Architecture note:
    This script connects to the pickup_daemon Unix socket.
    The daemon is the ONLY entry point to the hardware (PRU, /dev/mem).
    Never access hardware directly from Python.
"""

import argparse
import asyncio
import json
import os
import socket
import sys
import time
from typing import Optional


# ── Configuration ─────────────────────────────────────────────────────────────

DEFAULT_HOST   = "192.168.74.171"
DEFAULT_USER   = "beagle"
SOCKET_PATH    = "/run/pickup-winder.sock"

# Spindle test profile  (constant-speed steps, no ramp)
# Hz = steps/s = RPM * steps_per_rev / 60  (steps_per_rev=6400)
# Cap at 300 RPM until higher-speed behaviour is validated.
SPINDLE_TESTS = [
    # (name,                      sp_hz,   direction, duration_s)
    ("Slow   CW  ~60 RPM",         6400,   0,          3.0),
    ("Mid    CW  ~120 RPM",       12800,   0,          3.0),
    ("Fast   CW  ~180 RPM",       19200,   0,          3.0),
    ("       CW  ~240 RPM",       25600,   0,          3.0),
    ("       CW  ~300 RPM",       32000,   0,          4.0),
    ("       CCW ~300 RPM",       32000,   1,          4.0),
    ("       CCW ~240 RPM",       25600,   1,          3.0),
    ("Fast   CCW ~180 RPM",       19200,   1,          3.0),
    ("Mid    CCW ~120 RPM",       12800,   1,          3.0),
    ("Slow   CCW ~60 RPM",         6400,   1,          3.0),
]

# Lateral test profile (requires homed axis)
LATERAL_TESTS = [
    # (name,            pos_steps)
    ("Move to  3072",       3072),   # 1 mm
    ("Move to  9216",       9216),   # 3 mm
    ("Move to     0",          0),   # back home
]

# Colours
if sys.stdout.isatty():
    CG = "\033[0;32m"; CR = "\033[0;31m"; CC = "\033[0;36m"
    CB = "\033[1m";    CY = "\033[0;33m"; CX = "\033[0m"
else:
    CG = CR = CC = CB = CY = CX = ""


# ── Daemon client ──────────────────────────────────────────────────────────────

class DaemonClient:
    """Synchronous client for the pickup_daemon Unix socket."""

    def __init__(self, socket_path: str = SOCKET_PATH, verbose: bool = False):
        self.socket_path = socket_path
        self.verbose = verbose
        self._sock: Optional[socket.socket] = None
        self._buf = ""

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.settimeout(5.0)
        try:
            self._sock.connect(self.socket_path)
        except FileNotFoundError:
            raise ConnectionError(
                f"Socket not found: {self.socket_path}\n"
                "  → Is pickup_daemon running? (sudo systemctl start pickup-winder)"
            )
        except PermissionError:
            raise ConnectionError(
                f"Permission denied: {self.socket_path}\n"
                "  → Run as root or check socket permissions (chmod 666)"
            )
        self._sock.settimeout(2.0)

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def send(self, cmd: dict) -> dict:
        """Send a command dict and return the JSON response."""
        msg = json.dumps(cmd) + "\n"
        if self.verbose:
            print(f"  {CY}→{CX} {msg.strip()}")
        self._sock.sendall(msg.encode())
        # Read response (may arrive with telem events mixed in)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(4096).decode(errors="replace")
                self._buf += chunk
            except socket.timeout:
                pass
            # Parse all complete lines
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                # Ignore telem events while waiting for a command response
                if "ok" in obj:
                    if self.verbose:
                        print(f"  {CG}←{CX} {line}")
                    return obj
                if self.verbose and obj.get("event") == "telem":
                    sp = obj.get("sp", {})
                    lat = obj.get("lat", {})
                    print(f"  {CC}telem{CX}  sp={sp.get('steps'):>8}  lat={lat.get('pos'):>7}  "
                          f"endstop={obj.get('endstop')}")
        raise TimeoutError(f"No response from daemon for cmd: {cmd}")

    def read_telem(self, duration: float) -> list:
        """Collect telem events for `duration` seconds."""
        results = []
        self._sock.settimeout(0.1)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(4096).decode(errors="replace")
                self._buf += chunk
            except socket.timeout:
                pass
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if obj.get("event") == "telem":
                    results.append(obj)
                    if self.verbose:
                        sp = obj.get("sp", {})
                        lat = obj.get("lat", {})
                        print(f"  {CC}telem{CX}  sp={sp.get('steps'):>8}  "
                              f"lat={lat.get('pos'):>7}  endstop={obj.get('endstop')}")
                elif obj.get("event") in ("endstop_hit", "home_complete",
                                           "fault", "limit_hit",
                                           "move_complete", "speed_reached"):
                    results.append(obj)
                    print(f"  {CY}event{CX}  {obj.get('event')}")
        self._sock.settimeout(2.0)
        return results


# ── Helpers ───────────────────────────────────────────────────────────────────

def section(title: str) -> None:
    print(f"\n{CB}{CC}{'═'*60}{CX}")
    print(f"{CB}{CC}  {title}{CX}")
    print(f"{CB}{CC}{'═'*60}{CX}")

def check(resp: dict, label: str) -> bool:
    if resp.get("ok"):
        print(f"  {CG}✓{CX}  {label}")
        return True
    else:
        print(f"  {CR}✗{CX}  {label}: {resp.get('error', '?')}")
        return False

def steps_to_rpm(delta_steps: int, duration_s: float, steps_per_rev: int = 6400) -> float:
    if duration_s <= 0:
        return 0.0
    return (delta_steps / steps_per_rev) / duration_s * 60.0


# ── Test suites ───────────────────────────────────────────────────────────────

def test_spindle(client: DaemonClient) -> bool:
    section("SPINDLE ROTATION TEST")
    print(f"  Motor: spindle  ({len(SPINDLE_TESTS)} speed/direction steps)")
    print(f"  Steps/rev: 6400  (200-step × 32µstep)")

    all_ok = True

    # Enable both axes
    r = client.send({"cmd": "enable", "value": 1})
    all_ok &= check(r, "enable motors")

    r = client.send({"cmd": "ack_event"})
    all_ok &= check(r, "ack_event (clear any pending)")

    # Conservative spindle acceleration: 30000 steps/s² ≈ 281 RPM/s
    # → 0 to 1500 RPM in ~5.3 s.  Increase once motor behaviour is validated.
    r = client.send({"cmd": "set_accel", "sp_accel": 30000})
    all_ok &= check(r, "set_accel (sp_accel=30000 steps/s²)")

    print()
    print(f"  {'Test':<28} {'Target Hz':>10} {'Dir':>5} {'Steps':>8} {'RPM meas':>10} {'Status':>8}")
    print(f"  {'-'*28} {'-'*10} {'-'*5} {'-'*8} {'-'*10} {'-'*8}")

    prev_dir = None
    for name, sp_hz, sp_dir, duration in SPINDLE_TESTS:

        # Direction change: must decelerate to 0 before reversing.
        # (set_speed with same Hz but different dir produces use_ramp=0
        #  → instant direction switch at full speed → guaranteed stall)
        if prev_dir is not None and sp_dir != prev_dir:
            r = client.send({"cmd": "e_stop"})
            check(r, "e_stop (direction change)")
            time.sleep(0.8)   # let motor coast to stop
            r = client.send({"cmd": "enable", "value": 1})
            check(r, "re-enable after direction change")
            client.send({"cmd": "ack_event"})
        prev_dir = sp_dir

        # Set speed and direction (daemon auto-ramps from current speed)
        r = client.send({
            "cmd": "set_speed",
            "sp_hz": sp_hz,
            "sp_dir": sp_dir,
        })
        if not r.get("ok"):
            print(f"  {CR}✗  set_speed failed: {r}{CX}")
            all_ok = False
            continue

        # Collect telemetry for the cruise phase
        t0 = time.monotonic()
        telem = client.read_telem(duration)
        elapsed = time.monotonic() - t0

        # Acknowledge ramp-complete event so PRU event queue stays clean
        client.send({"cmd": "ack_event"})

        # Compute delta steps from first to last telem
        if len(telem) >= 2:
            steps0 = telem[0].get("sp", {}).get("steps", 0)
            steps1 = telem[-1].get("sp", {}).get("steps", 0)
            delta  = abs(steps1 - steps0)
            rpm    = steps_to_rpm(delta, elapsed)
            status = f"{CG}OK{CX}" if delta > 0 else f"{CR}STALL{CX}"
        else:
            delta, rpm, status = 0, 0.0, f"{CR}NO TELEM{CX}"
            all_ok = False

        dir_str = "CW" if sp_dir == 0 else "CCW"
        print(f"  {name:<28} {sp_hz:>10}  {dir_str:>5} {delta:>8} {rpm:>9.1f}  {status}")

    # Stop
    r = client.send({"cmd": "e_stop"})
    check(r, "e_stop")

    return all_ok


def test_lateral(client: DaemonClient, do_home: bool = True) -> bool:
    section("LATERAL MOVE_TO TEST")
    print(f"  Motor: lateral  (3072 steps/mm, M6 1mm pitch × 32µstep)")
    print()

    all_ok = True

    # Enable
    r = client.send({"cmd": "enable", "value": 1})
    all_ok &= check(r, "enable motors")

    r = client.send({"cmd": "ack_event"})
    all_ok &= check(r, "ack_event")

    # Configure lateral accel/speed (daemon auto-plans all move_to profiles)
    r = client.send({"cmd": "set_accel",
                     "lat_max_speed": 4000, "lat_accel": 10000, "lat_decel": 10000})
    all_ok &= check(r, "set_accel (lat_max=4000 Hz, accel=decel=10000 steps/s²)")

    # Home
    if do_home:
        print(f"\n  {CB}Homing lateral axis...{CX}")
        r = client.send({"cmd": "home_start"})
        if not check(r, "home_start"):
            print(f"  {CR}Cannot home — skipping lateral tests{CX}")
            return False

        # Wait for home_complete event (up to 15s)
        print(f"  Waiting for home_complete (max 15s)...")
        home_done = False
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            telem = client.read_telem(0.5)
            for obj in telem:
                if obj.get("event") == "home_complete":
                    home_done = True
                    break
                if obj.get("event") == "endstop_hit":
                    home_done = True
                    break
            if home_done:
                break

        if not home_done:
            print(f"  {CR}✗  Homing timeout — skipping lateral tests{CX}")
            client.send({"cmd": "e_stop"})
            return False
        print(f"  {CG}✓  Homed{CX}")
        client.send({"cmd": "ack_event"})

    # Move_to tests
    print(f"\n  {'Move':<22} {'Target steps':>14} {'Pos reached':>14} {'Status':>8}")
    print(f"  {'-'*22} {'-'*14} {'-'*14} {'-'*8}")

    for name, target_pos in LATERAL_TESTS:
        r = client.send({"cmd": "move_to", "axis": 1, "pos": target_pos})
        if not r.get("ok"):
            print(f"  {CR}✗  move_to failed: {r}{CX}")
            all_ok = False
            continue

        # Wait for move_complete event (up to 10s)
        move_done = False
        final_pos = None
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            telem = client.read_telem(0.3)
            for obj in telem:
                if obj.get("event") == "move_complete":
                    move_done = True
                if obj.get("event") == "telem":
                    final_pos = obj.get("lat", {}).get("pos")
            if move_done:
                break

        if move_done and final_pos is not None:
            ok_str = f"{CG}OK{CX}" if abs(final_pos - target_pos) <= 5 else f"{CY}APPROX{CX}"
        else:
            ok_str = f"{CR}TIMEOUT{CX}"
            all_ok = False

        print(f"  {name:<22} {target_pos:>14} {str(final_pos if final_pos else '?'):>14}  {ok_str}")
        client.send({"cmd": "ack_event"})

    # Stop
    client.send({"cmd": "e_stop"})
    return all_ok




# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="PickupWinder motor test — spindle & lateral",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--host",         default=DEFAULT_HOST,  help="BBB IP/hostname")
    p.add_argument("--user",         default=DEFAULT_USER,  help="SSH user")
    p.add_argument("--socket",       default=SOCKET_PATH,   help="Unix socket path")
    p.add_argument("--local",        action="store_true",   help="Connect to local socket (no SSH)")
    p.add_argument("--spindle-only", action="store_true",   help="Spindle tests only")
    p.add_argument("--lateral-only", action="store_true",   help="Lateral tests only")
    p.add_argument("--no-home",      action="store_true",   help="Skip lateral homing")
    p.add_argument("-v", "--verbose", action="store_true",  help="Print all telemetry")
    return p.parse_args()


def main():
    args = parse_args()

    # Determine socket path — for remote BBB, forward the Unix socket via SSH
    if args.local:
        sock_path = args.socket
    else:
        # Forward the BBB Unix socket to a local temp file via SSH
        import tempfile
        import subprocess
        local_sock = tempfile.mktemp(prefix="pickup-winder-", suffix=".sock")
        ssh_cmd = [
            "ssh",
            "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=no",
            "-o", "ExitOnForwardFailure=yes",
            "-nNT",
            "-L", f"{local_sock}:{args.socket}",
            f"{args.user}@{args.host}",
        ]
        print(f"{CB}Forwarding Unix socket via SSH...{CX}")
        print(f"  {args.user}@{args.host}:{args.socket}  →  {local_sock}")
        tunnel = subprocess.Popen(ssh_cmd)
        time.sleep(1.5)  # let tunnel establish
        sock_path = local_sock

    client = DaemonClient(socket_path=sock_path, verbose=args.verbose)

    try:
        print(f"\n{CB}Connecting to daemon...{CX}")
        client.connect()
        print(f"  {CG}✓  Connected{CX}")

        results = []

        if args.lateral_only:
            results.append(("Lateral move_to", test_lateral(client, do_home=not args.no_home)))
        elif args.spindle_only:
            results.append(("Spindle rotation", test_spindle(client)))
        else:
            # Full test suite
            results.append(("Spindle rotation", test_spindle(client)))

        # Summary
        section("SUMMARY")
        all_passed = True
        for name, passed in results:
            icon = f"{CG}PASS{CX}" if passed else f"{CR}FAIL{CX}"
            print(f"  {icon}  {name}")
            all_passed &= passed
        print()
        if all_passed:
            print(f"  {CG}{CB}All tests passed.{CX}")
        else:
            print(f"  {CR}{CB}Some tests FAILED.{CX}")
        print()

    except (ConnectionError, TimeoutError) as e:
        print(f"\n{CR}ERROR: {e}{CX}")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n{CY}Interrupted — sending e_stop...{CX}")
        try:
            client.send({"cmd": "e_stop"})
        except Exception:
            pass
    finally:
        client.close()
        if not args.local:
            try:
                tunnel.terminate()
                tunnel.wait(timeout=2)
            except Exception:
                pass
            try:
                os.unlink(local_sock)
            except Exception:
                pass


if __name__ == "__main__":
    main()
