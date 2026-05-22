from __future__ import annotations

import logging
import signal
import sys
import time

from app import WinderApplication

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
logger = logging.getLogger("main")


def main() -> None:
    app = WinderApplication()

    def _shutdown(sig, frame) -> None:
        logger.info("Shutdown requested (signal %s)", sig)
        app.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    app.start()
    logger.info("Winding controller started")

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
