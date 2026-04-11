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
SPINDLE_TESTS = [
    # (name,                 sp_hz,  direction, duration_s)
    ("Slow  CW  ~6 RPM",      640,   0,          3.0),   #  640/6400*60 =  6 RPM
    ("Mid   CW  ~30 RPM",    3200,   0,          3.0),   # 3200/6400*60 = 30 RPM
    ("Fast  CW  ~90 RPM",    9600,   0,          3.0),   # 9600/6400*60 = 90 RPM
    ("Fast  CCW ~90 RPM",    9600,   1,          3.0),
    ("Mid   CCW ~30 RPM",    3200,   1,          3.0),
    ("Slow  CCW ~6 RPM",      640,   1,          3.0),
]

# Lateral test profile (requires homed axis)
LATERAL_TESTS = [
    # (name,            pos_steps,  start_hz, max_hz, accel_steps)
    ("Move to  3072",       3072,      200,   4000,      300),   # 1 mm
    ("Move to  9216",       9216,      200,   4000,      300),   # 3 mm
    ("Move to     0",          0,      200,   4000,      300),   # back home
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
                                           "move_complete", "ramp_complete"):
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

    print()
    print(f"  {'Test':<28} {'Target Hz':>10} {'Dir':>5} {'Steps':>8} {'RPM meas':>10} {'Status':>8}")
    print(f"  {'-'*28} {'-'*10} {'-'*5} {'-'*8} {'-'*10} {'-'*8}")

    for name, sp_hz, sp_dir, duration in SPINDLE_TESTS:
        # Set speed and direction
        r = client.send({
            "cmd": "set_speed",
            "sp_hz": sp_hz,
            "sp_dir": sp_dir,
        })
        if not r.get("ok"):
            print(f"  {CR}✗  set_speed failed: {r}{CX}")
            all_ok = False
            continue

        # Collect telemetry
        t0 = time.monotonic()
        telem = client.read_telem(duration)
        elapsed = time.monotonic() - t0

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
        target_rpm = sp_hz / 6400 * 60
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

    for name, target_pos, start_hz, max_hz, accel_steps in LATERAL_TESTS:
        r = client.send({
            "cmd":         "move_to",
            "axis":        1,
            "pos":         target_pos,
            "start_hz":    start_hz,
            "max_hz":      max_hz,
            "accel_steps": accel_steps,
        })
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


def test_ramp_spindle(client: DaemonClient) -> bool:
    """Test spindle Klipper multi-segment ramp via the daemon's ramp_to command.

    Architecture note:
        The daemon (C, ARM) pre-computes the full Klipper ramp (16 sub-segments,
        equal Δiv partitioning) and writes sp_ramp_seg_t[16] into PRU shared RAM
        before issuing HOST_CMD_RAMP_TO.  PRU1 executes the segments autonomously
        using "interval += add" per step, force-loading start_iv at each boundary
        (Klipper stepper_load_next equivalent).  No Python timing is involved in
        the ramp execution — Python only sends one command and waits for the event.

    Test sequence:
        1. enable + ack_event
        2. ramp_to 100 → 300 RPM  (CW)  — wait ramp_complete
        3. ramp_to 300 → 1500 RPM (CW)  — wait ramp_complete
        4. ramp_to 1500 → 100 RPM (CW)  — wait ramp_complete (decel ramp)
        5. ramp_to 100 → 1500 RPM (CCW) — wait ramp_complete
        6. e_stop
    """
    section("SPINDLE KLIPPER RAMP TEST  (daemon pre-computes, PRU1 executes)")
    print("  Ramp profiles computed by daemon (ARM), executed by PRU1 autonomously.")
    print("  Python sends one {'cmd':'ramp_to'} and waits for {'event':'ramp_complete'}.")
    print()

    all_ok = True
    steps_per_rev = 6400

    r = client.send({"cmd": "enable", "value": 1})
    all_ok &= check(r, "enable")
    r = client.send({"cmd": "ack_event"})
    all_ok &= check(r, "ack_event")

    # (name, start_hz, target_hz, accel_steps, sp_dir, timeout_s)
    RAMP_TESTS = [
        ("100 → 300 RPM  CW",   6400,  32000,  500, 0, 10.0),
        ("300 → 1500 RPM CW",  32000, 75000, 1000, 0, 15.0),
        ("300 → 1500 RPM CW",  75000, 100000, 1000, 0, 15.0),
        ("300 → 1500 RPM CW",  100000, 115000, 10, 0, 15.0),
        ("300 → 1500 RPM CW",  115000, 75000, 6400, 0, 15.0),
        ("1500 → 100 RPM CW", 75000,   6400,  500, 0, 10.0),  # decel ramp
        ("100 → 1500 RPM CCW",  6400, 16000, 1000, 1, 15.0),
    ]

    print(f"  {'Ramp':<28} {'start':>8} {'target':>8} {'accel':>7}  {'Event':>14}  {'Status':>8}")
    print(f"  {'-'*28} {'-'*8} {'-'*8} {'-'*7}  {'-'*14}  {'-'*8}")

    for name, start_hz, sp_hz, accel_steps, sp_dir, timeout_s in RAMP_TESTS:
        r = client.send({
            "cmd":         "ramp_to",
            "sp_hz":       sp_hz,
            "start_hz":    start_hz,
            "accel_steps": accel_steps,
            "sp_dir":      sp_dir,
        })
        if not r.get("ok"):
            print(f"  {CR}✗  ramp_to failed: {r}{CX}")
            all_ok = False
            continue

        # Wait for ramp_complete event (autonomous PRU1 execution, no Python jitter)
        ramp_done    = False
        reported_hz  = "?"
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            telem = client.read_telem(0.2)
            for obj in telem:
                if obj.get("event") == "ramp_complete":
                    ramp_done   = True
                    reported_hz = obj.get("sp_hz", "?")
                    break
            if ramp_done:
                break

        target_rpm = sp_hz / steps_per_rev * 60
        if ramp_done:
            status_str = f"{CG}OK{CX}"
        else:
            status_str = f"{CR}TIMEOUT{CX}"
            all_ok = False

        print(f"  {name:<28} {start_hz:>8} {sp_hz:>8} {accel_steps:>7}  "
              f"{str(reported_hz):>14}  {status_str}")
        # Brief cruise phase before next ramp
        client.read_telem(0.5)
        client.send({"cmd": "ack_event"})

    r = client.send({"cmd": "e_stop"})
    check(r, "e_stop")
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
    p.add_argument("--ramp",         action="store_true",   help="Run spindle ramp test")
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

        if args.ramp:
            results.append(("Spindle ramp", test_ramp_spindle(client)))
        elif args.lateral_only:
            results.append(("Lateral move_to", test_lateral(client, do_home=not args.no_home)))
        elif args.spindle_only:
            results.append(("Spindle rotation", test_spindle(client)))
        else:
            # Full test suite
            results.append(("Spindle rotation", test_spindle(client)))
            time.sleep(0.5)
            results.append(("Spindle ramp",     test_ramp_spindle(client)))

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
