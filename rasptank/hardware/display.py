"""Simple synchronous two-line OLED messages, not the legacy six-line thread."""

from importlib import import_module

from ..exceptions import HardwareError
from ._lifecycle import ManagedController, ensure_open


class OledDisplay(ManagedController):
    """SSD1306 OLED on I2C (default port 1, address 0x3C) via luma.oled, loaded lazily."""

    def __init__(self, port=1, address=0x3C, *, use_gpio=True, device=None, canvas=None):
        """Open the display; device/canvas may be injected (e.g. mocks) for tests."""
        self.port = port
        self.address = address
        self.use_gpio = use_gpio
        self.device = device
        self._canvas = canvas
        self._serial = None
        self._closed = False
        self.lines = ("", "")
        if use_gpio:
            try:
                if self.device is None:
                    serial = import_module("luma.core.interface.serial")
                    oled = import_module("luma.oled.device")
                    self._serial = serial.i2c(port=port, address=address)
                    self.device = oled.ssd1306(self._serial, rotate=0)
                if self._canvas is None:
                    self._canvas = import_module("luma.core.render").canvas
            except BaseException as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                if not isinstance(exc, Exception):
                    raise
                raise HardwareError("Unable to initialize OLED") from exc

    def show(self, line1="GEWBOT.COM", line2="IP:CONNECTING"):
        """Draw two text lines (strings) and remember them in ``lines``."""
        ensure_open(self)
        if not isinstance(line1, str) or not isinstance(line2, str):
            raise ValueError("OLED lines must be strings")
        if self.use_gpio:
            try:
                with self._canvas(self.device) as draw:
                    draw.text((0, 0), line1, fill="white")
                    draw.text((0, 10), line2, fill="white")
            except Exception as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                raise HardwareError("OLED drawing failed") from exc
        self.lines = (line1, line2)

    def close(self):
        """Release the display device; idempotent."""
        if self._closed:
            return
        self._closed = True
        if self.use_gpio:
            if self.device is not None:
                self.device.cleanup()
            elif self._serial is not None:
                self._serial.cleanup()
