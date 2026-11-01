"""Three-channel infrared line tracker (server/findline.py, functions.py)."""

from importlib import import_module

from ..config import LINE_PIN_LEFT, LINE_PIN_MIDDLE, LINE_PIN_RIGHT
from ..exceptions import HardwareDisabledError, HardwareError
from ._lifecycle import ManagedController, ensure_open, release


class LineSensors(ManagedController):
    """read() returns (left, middle, right); 1 means the legacy 'line seen' level."""

    def __init__(self, use_gpio=True, *, gpio=None):
        self.use_gpio = use_gpio
        self.gpio = gpio
        self._pins = []
        self._closed = False
        if use_gpio:
            try:
                if self.gpio is None:
                    self.gpio = import_module("RPi.GPIO")
                self.gpio.setwarnings(False)
                self.gpio.setmode(self.gpio.BCM)
                for pin in (LINE_PIN_LEFT, LINE_PIN_MIDDLE, LINE_PIN_RIGHT):
                    self.gpio.setup(pin, self.gpio.IN)
                    self._pins.append(pin)
            except BaseException as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                if not isinstance(exc, Exception):
                    raise
                raise HardwareError("Unable to initialize line sensors") from exc

    def read(self):
        ensure_open(self)
        if not self.use_gpio:
            raise HardwareDisabledError("Line sensor hardware is disabled")
        try:
            return tuple(int(bool(self.gpio.input(pin)))
                         for pin in (LINE_PIN_LEFT, LINE_PIN_MIDDLE, LINE_PIN_RIGHT))
        except Exception as exc:
            try:
                self.close()
            except BaseException:
                pass
            raise HardwareError("Line sensor read failed") from exc

    def close(self):
        if self._closed:
            return
        self._closed = True
        release([lambda pin=pin: self.gpio.cleanup(pin) for pin in self._pins])
