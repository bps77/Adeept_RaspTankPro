"""LED wrapper for the Adeept RaspTank Pro.

This module preserves the original LED strip behavior from `server/LED.py` while
offering a simpler Python interface for novice learners and interactive experiments.
"""

from __future__ import annotations

try:
    from rpi_ws281x import Adafruit_NeoPixel, Color
except ImportError:  # pragma: no cover - used in mocked/test environments
    Adafruit_NeoPixel = None
    Color = None


class LEDStrip:
    """Wrap the original robot LED behavior.

    The original code uses a WS2812 LED strip connected to GPIO 12. This class
    keeps the same configuration and adds a friendlier, more explicit API.
    """

    def __init__(self, count: int = 16, pin: int = 12):
        self.count = count
        self.pin = pin
        self.strip = self._build_strip()

    def _build_strip(self):
        if Adafruit_NeoPixel is None:
            raise RuntimeError("rpi_ws281x is not installed.")

        strip = Adafruit_NeoPixel(
            self.count,
            self.pin,
            800000,
            10,
            False,
            255,
            0,
        )
        strip.begin()
        return strip

    def color_wipe(self, red: int, green: int, blue: int) -> None:
        """Set the whole LED strip to a single color."""
        if Color is None:
            raise RuntimeError("LED color support is not available in this environment.")

        color = Color(red, green, blue)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
        self.strip.show()

    def off(self) -> None:
        """Turn all LEDs off."""
        self.color_wipe(0, 0, 0)

    def close(self) -> None:
        """Shut the LED strip down cleanly."""
        self.off()


if __name__ == "__main__":
    strip = LEDStrip()
    strip.color_wipe(255, 0, 0)
