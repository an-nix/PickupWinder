"""Potentiometer reader using the BeagleBone IIO sysfs ADC device.

This is a simple moving-average filter designed for prototyping.
"""
import time
import threading
import collections
import math

DEFAULT_ADC_DEVICE = '/sys/bus/iio/devices/iio:device0'
ADC_MAX = 4095


class Potentiometer:
    def __init__(self, channel=0, device=DEFAULT_ADC_DEVICE, read_interval_ms=20, filter_size=32, inverted=False, exp_k=4.5):
        self.channel = channel
        self.device = device
        self.path = f"{self.device}/in_voltage{self.channel}_raw"
        self.interval = max(0.001, read_interval_ms / 1000.0)
        self.filter_size = filter_size
        self.inverted = inverted
        self.exp_k = float(exp_k)
        self._buf = collections.deque(maxlen=self.filter_size)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _read_raw(self):
        try:
            with open(self.path, 'r') as f:
                return int(f.read().strip())
        except Exception:
            return None

    def _normalize(self, raw):
        if raw is None:
            return None
        v = max(0, min(ADC_MAX, raw)) / ADC_MAX
        if self.inverted:
            v = 1.0 - v
        return v

    def _filter_to_hz(self, t, hz_min=1067, hz_max=160000):
        """Apply exponential curve mapping t in [0,1] to Hz (matches C config)."""
        k = self.exp_k
        num = math.exp(k * t) - 1.0
        den = math.exp(k) - 1.0
        if den == 0:
            return int(hz_min)
        hz = hz_min + (hz_max - hz_min) * (num / den)
        return int(round(hz))

    def _worker(self):
        while self._running:
            raw = self._read_raw()
            norm = self._normalize(raw)
            if norm is not None:
                with self._lock:
                    self._buf.append(norm)
            time.sleep(self.interval)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def get_raw(self):
        with self._lock:
            if not self._buf:
                return None
            return int(round(sum(self._buf) / len(self._buf) * ADC_MAX))

    def get_normalized(self):
        with self._lock:
            if not self._buf:
                return None
            return float(sum(self._buf) / len(self._buf))

    def get_hz(self, hz_min=1067, hz_max=160000):
        t = self.get_normalized()
        if t is None:
            return None
        return self._filter_to_hz(t, hz_min, hz_max)
