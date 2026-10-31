"""Single-frame synchronous LED strip updates; no animations or threads."""

from importlib import import_module

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

    def _write(self, rgb):
        if self.use_gpio and self.strip is not None and self._color is not None:
            color = self._color(*rgb)
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, color)
            self.strip.show()

    def color_wipe(self, red, green, blue):
        """Set all pixels, then show once (not the legacy per-pixel animation)."""
        ensure_open(self)
        rgb = (red, green, blue)
        if any(type(value) is not int or not 0 <= value <= 255 for value in rgb):
            raise ValueError("RGB values must be integers from 0 to 255")
        try:
            self._write(rgb)
        except Exception as exc:
            try:
                self.close()
            except BaseException:
                pass
            raise HardwareError("LED output failed") from exc
        self.color = rgb

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
