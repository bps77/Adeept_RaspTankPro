"""Three GPIO switch ports (server/switch.py, robotLight.py); head light is port 1."""

from importlib import import_module

from ..config import SWITCH_PINS
from ..exceptions import HardwareError
from ._lifecycle import ManagedController, ensure_open, release


class SwitchPorts(ManagedController):
    """Ports 1..3 map to BCM 5, 6, 13 and all start off. Dry-run tracks state only."""

    def __init__(self, use_gpio=True, *, gpio=None):
        """Configure all three ports as outputs, initially low."""
        self.use_gpio = use_gpio
        self.gpio = gpio
        self.state = dict.fromkeys(SWITCH_PINS, False)
        self._pins = []
        self._closed = False
        if use_gpio:
            try:
                if self.gpio is None:
                    self.gpio = import_module("RPi.GPIO")
                self.gpio.setwarnings(False)
                self.gpio.setmode(self.gpio.BCM)
                for pin in SWITCH_PINS.values():
                    self.gpio.setup(pin, self.gpio.OUT, initial=self.gpio.LOW)
                    self._pins.append(pin)
            except BaseException as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                if not isinstance(exc, Exception):
                    raise
                raise HardwareError("Unable to initialize switch ports") from exc

    def set(self, port, on):
        """Switch port 1..3 on or off; ``on`` must be a bool."""
        ensure_open(self)
        if type(port) is not int or port not in SWITCH_PINS:
            raise ValueError("port must be 1, 2 or 3")
        if type(on) is not bool:
            raise ValueError("on must be a bool")
        if self.use_gpio:
            try:
                self.gpio.output(SWITCH_PINS[port], self.gpio.HIGH if on else self.gpio.LOW)
            except Exception as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                raise HardwareError("Switch output failed") from exc
        self.state[port] = on

    def head_light(self, on):
        """Switch port 1 (head light)."""
        self.set(1, on)

    def front_light(self, on):
        """Ports 2 and 3. Legacy 'off' cleared ports 1 and 3 (a typo); this clears 2 and 3."""
        self.set(2, on)
        self.set(3, on)

    def all_off(self):
        """Attempt to turn every port off, even if one fails."""
        ensure_open(self)
        release([lambda port=port: self.set(port, False) for port in SWITCH_PINS])

    def close(self):
        """Drive all ports low and clean up their pins; idempotent."""
        if self._closed:
            return
        self._closed = True
        actions = [lambda pin=pin: self.gpio.output(pin, self.gpio.LOW) for pin in self._pins]
        actions.extend(lambda pin=pin: self.gpio.cleanup(pin) for pin in self._pins)
        release(actions)
        self.state = dict.fromkeys(SWITCH_PINS, False)
