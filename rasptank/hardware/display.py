"""OLED wrapper for the Adeept RaspTank Pro.

This module preserves the original I2C display behavior from `server/OLED.py`
while exposing a friendlier class for Python-based control and debugging.
"""

from __future__ import annotations

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306
except ImportError:  # pragma: no cover - used in mocked/test environments
    i2c = None
    canvas = None
    ssd1306 = None


class OledDisplay:
    """Wrap the original robot OLED display logic."""

    def __init__(self, port: int = 1, address: int = 0x3C):
        self.port = port
        self.address = address
        self.device = self._build_device()

    def _build_device(self):
        if ssd1306 is None:
            raise RuntimeError("luma.oled is not installed.")

        serial = i2c(port=self.port, address=self.address)
        return ssd1306(serial, rotate=0)

    def show(self, line1: str = "GEWBOT.COM", line2: str = "IP:CONNECTING") -> None:
        """Show a simple two-line status message on the OLED screen."""
        if canvas is None:
            raise RuntimeError("OLED drawing support is not available in this environment.")

        with canvas(self.device) as draw:
            draw.text((0, 0), line1, fill="white")
            draw.text((0, 10), line2, fill="white")

    def close(self) -> None:
        """Clean up the screen resources."""
        pass


if __name__ == "__main__":
    display = OledDisplay()
    display.show("GEWBOT.COM", "READY")
