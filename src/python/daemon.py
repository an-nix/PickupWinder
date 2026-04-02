"""PickupWinder daemon - BeagleBone Black Linux userspace entry point.

Asyncio-based main loop with:
  - 10 ms control task (stepper + lateral PRU, session arbitration)
  - WebSocket / HTTP server task (aiohttp)
  - Graceful shutdown on SIGINT / SIGTERM

Run as root on the target: python3 daemon.py
(requires /dev/mem access for PRU IPC, /sys/bus/iio for ADC)
"""
import asyncio
import signal
import sys
import time

from ipc import IpcChannel
from pot import Potentiometer
from encoder import Encoder
from stepper_pru import StepperPRU
from lateral_pru import LateralPRU
from winder_app import WinderApp
from session_controller import SessionController, TickInput
from command_controller import CommandController
from recipe_store import WindingRecipeStore
from web_interface import WebInterface
from config import SPEED_HZ_MIN, SPEED_HZ_MAX, WS_UPDATE_MS

# ── Hardware pin / channel config ─────────────────────────────────────────────
ADC_CHANNEL = 0            # AIN0  (P9_39)
ENCODER_PIN_A = 19         # adjust to actual GPIO numbers
ENCODER_PIN_B = 18
FOOTSWITCH_PIN = 26        # active-low footswitch GPIO

# ── Control cadence ────────────────────────────────────────────────────────────
CONTROL_INTERVAL_S = 0.010           # 10 ms
TELEM_LOG_INTERVAL_S = 5.0          # log worst loop every 5 s
WS_UPDATE_INTERVAL_S = WS_UPDATE_MS / 1000.0


class Daemon:
    def __init__(self):
        # Hardware IPC
        self._ipc = IpcChannel()
        self._stepper = StepperPRU(self._ipc)
        self._lateral = LateralPRU(self._ipc)

        # Input devices
        self._pot = Potentiometer(channel=ADC_CHANNEL)
        self._enc = Encoder(ENCODER_PIN_A, ENCODER_PIN_B)

        # Application layers
        self._recipe_store = WindingRecipeStore()
        self._app = WinderApp(self._stepper, self._lateral, self._recipe_store)
        self._session = SessionController(self._app)
        self._cmd_ctrl = CommandController(self._session)

        # Web interface
        self._web = WebInterface(self._cmd_ctrl, self._app)

        # Shutdown flag
        self._running = False

        # Diagnostics
        self._worst_loop_us = 0
        self._last_telem_log = 0.0
        self._last_enc_count = 0
        self._last_ws_update = 0.0
        self._footswitch_last = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def begin(self):
        self._ipc.open()
        self._stepper.begin()
        self._lateral.begin()
        self._pot.start()
        self._enc.start()
        self._app.begin()
        self._last_enc_count = self._enc.get_count()
        print("[Daemon] Initialized")

    def shutdown(self):
        self._running = False
        print("[Daemon] Shutdown: emergency stop")
        try:
            self._stepper.stop()
            self._stepper.disable()
            self._lateral.stop()
        except Exception as e:
            print(f"[Daemon] Shutdown error: {e}")
        finally:
            self._pot.stop()
            self._enc.stop()
            self._ipc.close()

    # ── Control loop task ─────────────────────────────────────────────────────
    async def _control_task(self):
        self._running = True
        next_tick = asyncio.get_event_loop().time()

        while self._running:
            tick_start = time.monotonic()

            # 1. Read hardware sensors
            pot_hz = self._pot.get_hz(SPEED_HZ_MIN, SPEED_HZ_MAX)
            cur_enc = self._enc.get_count()
            enc_delta = cur_enc - self._last_enc_count
            self._last_enc_count = cur_enc

            # 2. Read footswitch (active-low GPIO sysfs)
            fs_pressed = self._read_footswitch()
            fs_changed = fs_pressed != self._footswitch_last
            self._footswitch_last = fs_pressed

            # 3. Build tick input
            inp = TickInput()
            inp.pot_hz = pot_hz
            inp.footswitch_pressed = fs_pressed
            inp.footswitch_changed = fs_changed
            self._cmd_ctrl.drain(inp)

            # 5. Run session arbitration + windapp tick
            self._session.tick(inp)
            self._app.handle_encoder_delta(enc_delta)
            self._app.tick()

            # 6. Broadcast WebSocket update at reduced cadence
            now = asyncio.get_event_loop().time()
            if now - self._last_ws_update >= WS_UPDATE_INTERVAL_S:
                self._last_ws_update = now
                status = self._app.get_status()
                asyncio.ensure_future(self._web.send_update(status))

            # 7. Worst-loop diagnostics
            elapsed_us = int((time.monotonic() - tick_start) * 1e6)
            if elapsed_us > self._worst_loop_us:
                self._worst_loop_us = elapsed_us
            if tick_start - self._last_telem_log >= TELEM_LOG_INTERVAL_S:
                self._last_telem_log = tick_start
                print(f"[Diag] control_worst_loop_us={self._worst_loop_us}")
                self._worst_loop_us = 0

            # 8. Sleep to maintain cadence
            next_tick += CONTROL_INTERVAL_S
            sleep_s = next_tick - asyncio.get_event_loop().time()
            if sleep_s > 0:
                await asyncio.sleep(sleep_s)
            else:
                # We overran; resync
                next_tick = asyncio.get_event_loop().time()
                await asyncio.sleep(0)

    def _read_footswitch(self) -> bool:
        """Read footswitch GPIO value from sysfs (active-low)."""
        path = f"/sys/class/gpio/gpio{FOOTSWITCH_PIN}/value"
        try:
            with open(path) as f:
                return f.read(1) == '0'   # active-low
        except OSError:
            return False

    # ── Run ───────────────────────────────────────────────────────────────────
    async def run(self):
        loop = asyncio.get_event_loop()

        # Install signal handlers
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self._on_signal)

        # Start web server
        await self._web.start()

        # Run control loop
        await self._control_task()

        # Graceful web shutdown
        await self._web.stop()

    def _on_signal(self):
        print("\n[Daemon] Signal received - stopping")
        self._running = False


def main():
    daemon = Daemon()
    daemon.begin()
    try:
        asyncio.run(daemon.run())
    finally:
        daemon.shutdown()


if __name__ == "__main__":
    main()
