"""main.py — Pickup-winder application entry point.

Boot sequence:
  1. Load config.json  (hardware constants + app tunables).
  2. Load recipe.json  (persistent winding recipe).
  3. Connect to the C hardware daemon via Unix socket.
  4. Start WinderApp + SessionController.
  5. Run the 10 ms tick loop:
       - drain pending daemon events → app.on_event()
       - read stdin commands (non-blocking) → router.dispatch()
       - call app.tick()

Override config path at runtime:
    PICKUP_CONFIG=/path/to/config.json python main.py
"""

import json
import logging
import os
import select
import sys
import time

from core import (
    AppConfig,
    CommandRouter,
    RecipeStore,
    SessionController,
    TickInput,
    WinderApp,
    load_config,
    CONFIG_PATH,
)
from hal import DaemonClient
from pickup import PickupController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)s — %(message)s",
)
_log = logging.getLogger(__name__)


def _drain_events(client: DaemonClient, app: WinderApp) -> None:
    """Pull all pending events from the daemon and route them to the app."""
    for ev in client.poll_events():
        app.on_event(ev)


def _poll_stdin(router: CommandRouter) -> None:
    """Non-blocking stdin read; dispatches one line if available."""
    if not sys.stdin.isatty():
        return
    r, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not r:
        return
    line = sys.stdin.readline().strip()
    if not line:
        return
    result = router.dispatch(line)
    if result is not None:
        print(json.dumps(result, indent=2))


def main() -> int:
    # ── Config ─────────────────────────────────────────────────────────────────
    cfg_path = os.environ.get("PICKUP_CONFIG", CONFIG_PATH)
    cfg: AppConfig = load_config(cfg_path)

    # ── Recipe store ───────────────────────────────────────────────────────────
    store   = RecipeStore(cfg.recipe_path)
    recipe  = store.load()

    # ── Hardware connection ────────────────────────────────────────────────────
    client     = DaemonClient(path=cfg.socket_path)
    client.connect()
    controller = PickupController(client)

    # ── Application layer ──────────────────────────────────────────────────────
    app     = WinderApp(controller, cfg)
    session = SessionController(app, cfg)
    router  = CommandRouter(app)

    if recipe is not None:
        app.apply_recipe(recipe)
    app.set_recipe_changed_callback(store.save)
    app.begin()

    _log.info("Pickup Winder started (config: %s, recipe: %s)",
              cfg_path, "loaded" if recipe else "default")

    tick_period = 1.0 / max(1, cfg.tick_hz)

    # ── Main loop ──────────────────────────────────────────────────────────────
    try:
        while True:
            t0 = time.monotonic()

            _drain_events(client, app)
            _poll_stdin(router)

            # TODO: feed real pot / footswitch readings into session.tick()
            app.tick()

            elapsed = time.monotonic() - t0
            sleep   = tick_period - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        _log.info("Interrupted — stopping")
    finally:
        app.lateral.stop_winding()
        controller.emergency_stop()
        client.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
