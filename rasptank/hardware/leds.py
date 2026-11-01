"""Single-frame synchronous LED strip updates; no animations or threads."""

import time
from importlib import import_module

from ..config import POLICE_PIXELS
from ..exceptions import HardwareError
from ._lifecycle import ManagedController, ensure_open, release


class LEDStrip(ManagedController):
    def __init__(self, count=16, pin=12, *, use_gpio=True, strip=None, color=None):
        if type(count) is not int or count <= 0:
            raise ValueError("count must be a positive integer")
        self.count = count
        self.pin = pin
        self.use_gpio = use_gpio
        self.strip = strip
        self._color = color
        self._closed = False
        self.color = (0, 0, 0)
        if use_gpio:
            try:
                if self.strip is None or self._color is None:
                    module = import_module("rpi_ws281x")
                    if self.strip is None:
                        self.strip = module.Adafruit_NeoPixel(count, pin, 800000, 10, False, 255, 0)
                    if self._color is None:
                        self._color = module.Color
                self.strip.begin()
            except BaseException as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                if not isinstance(exc, Exception):
                    raise
                raise HardwareError("Unable to initialize LED strip") from exc

    def _write(self, rgb, ids=None):
        if self.use_gpio and self.strip is not None and self._color is not None:
            color = self._color(*rgb)
            for i in (range(self.strip.numPixels()) if ids is None else ids):
                self.strip.setPixelColor(i, color)
            self.strip.show()

    @staticmethod
    def _check_rgb(rgb):
        if any(type(value) is not int or not 0 <= value <= 255 for value in rgb):
            raise ValueError("RGB values must be integers from 0 to 255")

    def _show(self, rgb, ids=None):
        try:
            self._write(rgb, ids)
        except Exception as exc:
            try:
                self.close()
            except BaseException:
                pass
            raise HardwareError("LED output failed") from exc

    def color_wipe(self, red, green, blue):
        """Set all pixels, then show once (not the legacy per-pixel animation)."""
        ensure_open(self)
        rgb = (red, green, blue)
        self._check_rgb(rgb)
        self._show(rgb)
        self.color = rgb

    def set_pixels(self, ids, red, green, blue):
        """Set only the listed pixel indexes (legacy setSomeColor), one show."""
        ensure_open(self)
        rgb = (red, green, blue)
        self._check_rgb(rgb)
        ids = tuple(ids)
        if any(type(i) is not int or not 0 <= i < self.count for i in ids):
            raise ValueError(f"pixel ids must be integers in [0, {self.count - 1}]")
        self._show(rgb, ids)

    @staticmethod
    def police_frames():
        """One legacy police cycle as ((r, g, b), seconds) frames on pixels 0..12."""
        for colour in ((0, 0, 255), (255, 0, 0)):
            for _ in range(3):
                yield colour, 0.05
                yield (0, 0, 0), 0.05
            yield (0, 0, 0), 0.1

    @staticmethod
    def breath_frames(red, green, blue, steps=10):
        """One legacy breath cycle (fade up then down), 0.03 s per frame."""
        if type(steps) is not int or steps <= 0:
            raise ValueError("steps must be a positive integer")
        for i in range(steps):
            yield (int(red * i / steps), int(green * i / steps), int(blue * i / steps)), 0.03
        for i in range(steps):
            yield (int(red - red * i / steps), int(green - green * i / steps),
                   int(blue - blue * i / steps)), 0.03

    def play(self, frames, *, ids=None, sleep=time.sleep):
        """Show finite (rgb, seconds) frames; the strip is left dark afterwards."""
        try:
            for rgb, seconds in frames:
                ensure_open(self)
                self._check_rgb(rgb)
                self._show(rgb, ids)
                sleep(seconds)
        finally:
            if not self._closed:
                self._show((0, 0, 0))
                self.color = (0, 0, 0)

    def police(self, cycles=1, *, sleep=time.sleep):
        """Blue/red police flashes for a bounded number of cycles."""
        if type(cycles) is not int or cycles <= 0:
            raise ValueError("cycles must be a positive integer")
        ids = range(min(POLICE_PIXELS, self.count))
        for _ in range(cycles):
            self.play(self.police_frames(), ids=ids, sleep=sleep)

    def breath(self, red, green, blue, cycles=1, *, steps=10, sleep=time.sleep):
        """Breathing effect in one colour for a bounded number of cycles."""
        self._check_rgb((red, green, blue))
        if type(cycles) is not int or cycles <= 0:
            raise ValueError("cycles must be a positive integer")
        for _ in range(cycles):
            self.play(self.breath_frames(red, green, blue, steps), sleep=sleep)

    def off(self):
        self.color_wipe(0, 0, 0)

    def close(self):
        if self._closed:
            return
        self._closed = True
        actions = [lambda: self._write((0, 0, 0))]
        # rpi_ws281x versions exposing native release use _cleanup().
        cleanup = getattr(self.strip, "_cleanup", None) if self.use_gpio else None
        if callable(cleanup):
            actions.append(cleanup)
        release(actions)
        self.color = (0, 0, 0)
