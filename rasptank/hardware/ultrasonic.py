"""Bounded synchronous ultrasonic measurements in metres."""

import time
from importlib import import_module

from ..config import ULTRASONIC_ECHO, ULTRASONIC_TIMEOUT, ULTRASONIC_TRIGGER
from ..exceptions import HardwareDisabledError, HardwareError, SensorTimeoutError
from ._lifecycle import ManagedController, ensure_open, number, release


class UltrasonicSensor(ManagedController):
    """HC-SR04 style sensor: trigger BCM 11, echo BCM 8 by default."""

    TRIGGER_PIN = ULTRASONIC_TRIGGER
    ECHO_PIN = ULTRASONIC_ECHO

    def __init__(self, trigger_pin=TRIGGER_PIN, echo_pin=ECHO_PIN, *,
                 use_gpio=True, gpio=None, timeout=ULTRASONIC_TIMEOUT,
                 clock=None, sleep=None):
        """Validate pins and timeout (seconds); gpio, clock and sleep are injectable."""
        number(timeout, 0, float("inf"), "timeout")
        if timeout == 0:
            raise ValueError("timeout must be positive")
        if (type(trigger_pin) is not int or type(echo_pin) is not int
                or not 0 <= trigger_pin <= 27 or not 0 <= echo_pin <= 27
                or trigger_pin == echo_pin):
            raise ValueError("trigger and echo must be distinct BCM pins in 0..27")
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.timeout = timeout
        self.use_gpio = use_gpio
        self.gpio = gpio
        self._clock = time.monotonic if clock is None else clock
        self._sleep = time.sleep if sleep is None else sleep
        self._pins = []
        self._closed = False
        self._ready = False
        if use_gpio:
            self.setup()

    def setup(self):
        """Configure the trigger and echo pins; idempotent, enables hardware."""
        ensure_open(self)
        if self._ready:
            return
        try:
            if self.gpio is None:
                self.gpio = import_module("RPi.GPIO")
            gpio = self.gpio
            gpio.setwarnings(False)
            gpio.setmode(gpio.BCM)
            gpio.setup(self.trigger_pin, gpio.OUT, initial=gpio.LOW)
            self._pins.append(self.trigger_pin)
            gpio.setup(self.echo_pin, gpio.IN)
            self._pins.append(self.echo_pin)
            self.use_gpio = True
            self._ready = True
        except BaseException as exc:
            try:
                self.close()
            except BaseException:
                pass
            if not isinstance(exc, Exception):
                raise
            raise HardwareError("Unable to initialize ultrasonic GPIO") from exc

    def _wait(self, level, deadline):
        """Block until echo reaches level; return the monotonic time or time out."""
        while True:
            value = self.gpio.input(self.echo_pin)
            now = self._clock()
            if now >= deadline:
                raise SensorTimeoutError(f"Timed out waiting for echo level {level}")
            if value == level:
                return now

    def distance(self):
        """Return metres rounded to 2 decimals, using legacy 340 m/s.

        Each echo edge has its own monotonic timeout. Dry-run readings raise
        HardwareDisabledError rather than fabricate sensor data.
        """
        ensure_open(self)
        if not self.use_gpio:
            raise HardwareDisabledError("Ultrasonic hardware is disabled")
        try:
            self.gpio.output(self.trigger_pin, self.gpio.HIGH)
            try:
                self._sleep(0.000015)
            finally:
                self.gpio.output(self.trigger_pin, self.gpio.LOW)
            start = self._wait(1, self._clock() + self.timeout)
            end = self._wait(0, start + self.timeout)
            return round((end - start) * 340 / 2, 2)
        except SensorTimeoutError:
            raise
        except Exception as exc:
            try:
                self.close()
            except BaseException:
                pass
            raise HardwareError("Ultrasonic measurement failed") from exc

    def close(self):
        """Drive trigger low and clean up configured pins; idempotent."""
        if self._closed:
            return
        self._closed = True
        actions = []
        if self.trigger_pin in self._pins:
            actions.append(lambda: self.gpio.output(self.trigger_pin, self.gpio.LOW))
        actions.extend(lambda pin=pin: self.gpio.cleanup(pin) for pin in self._pins)
        release(actions)
