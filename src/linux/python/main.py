import logging
import os

from core import (
    AppConfig,
    load_config
)
from hal import DaemonClient, PickupController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)s — %(message)s",
)
_log = logging.getLogger(__name__)


CONFIG_PATH = "C:\\temp\\pw\\data\\config.example.json"


def main() -> int:
    # ── Config ─────────────────────────────────────────────────────────────────
    cfg_path = os.environ.get("CONFIG_PATH", CONFIG_PATH)
    cfg: AppConfig = load_config(cfg_path)

    pass



    return 0


if __name__ == "__main__":
    raise SystemExit(main())
