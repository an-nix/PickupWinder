"""Basic quadrature encoder reader implemented with sysfs polling.

This is a simple, robust prototype. For production use prefer a PRU
decoder or `gpiod` edge events.
"""
import time
import threading
import os


class Encoder:
    """Poll two GPIO sysfs lines and decode quadrature transitions.

    pins are Linux GPIO numbers (the ones exposed under /sys/class/gpio).
    """
    _TRANSITION = {
        (0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
        (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1,
    }

    def __init__(self, pin_a, pin_b, poll_interval_s=0.002):
        self.pin_a = int(pin_a)
        self.pin_b = int(pin_b)
        self.poll_interval = max(0.001, float(poll_interval_s))
        self._count = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._la_path = f"/sys/class/gpio/gpio{self.pin_a}/value"
        self._lb_path = f"/sys/class/gpio/gpio{self.pin_b}/value"

    def _ensure_exported(self, pin):
        gpio_path = f"/sys/class/gpio/gpio{pin}"
        if not os.path.exists(gpio_path):
            try:
                with open('/sys/class/gpio/export', 'w') as f:
                    f.write(str(pin))
            except Exception:
                pass

    def _read_val(self, path):
        try:
            with open(path, 'r') as f:
                return int(f.read().strip())
        except Exception:
            return 0

    def start(self):
        self._ensure_exported(self.pin_a)
        self._ensure_exported(self.pin_b)
        # set directions to input if possible
        try:
            with open(f"/sys/class/gpio/gpio{self.pin_a}/direction", 'w') as f:
                f.write('in')
            with open(f"/sys/class/gpio/gpio{self.pin_b}/direction", 'w') as f:
                f.write('in')
        except Exception:
            pass

        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _worker(self):
        la = self._read_val(self._la_path)
        lb = self._read_val(self._lb_path)
        prev = (la << 1) | lb
        while self._running:
            a = self._read_val(self._la_path)
            b = self._read_val(self._lb_path)
            s = (a << 1) | b
            delta = self._TRANSITION.get((prev, s), 0)
            if delta:
                with self._lock:
                    self._count += delta
            prev = s
            time.sleep(self.poll_interval)

    def get_count(self):
        with self._lock:
            return int(self._count)

    def reset(self):
        with self._lock:
            self._count = 0
