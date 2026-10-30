"""RaspTank ultrasonic sensor wrapper.

This module wraps the original `server/ultra.py` sensor behavior into a clean,
readable object for interactive use and notebook experimentation.
"""

from __future__ import annotations

import time

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - used in mocked/test environments
    GPIO = None


class UltrasonicSensor:
    """Measure distance using the original signal timing logic.

    This keeps the same behavior as `server/ultra.py` while making the sensor easy
    to use in Python.
    """

    TRIGGER_PIN = 11
    ECHO_PIN = 8

    def __init__(self, trigger_pin: int = TRIGGER_PIN, echo_pin: int = ECHO_PIN):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin

        if GPIO is not None:
            self.setup()

    def setup(self) -> None:
        """Configure the ultrasonic sensor pins using the original BCM layout."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def distance(self) -> float:
        """Return the measured distance in centimeters or meters?

        The original implementation returns a floating-point value computed from the
        pulse duration. We keep the original numeric interpretation for compatibility.
        """
        if GPIO is None:
            raise RuntimeError("RPi.GPIO is not available in this environment.")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.echo_pin, GPIO.IN)

        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.trigger_pin, GPIO.LOW)

        while not GPIO.input(self.echo_pin):
            pass
        t1 = time.time()

        while GPIO.input(self.echo_pin):
            pass
        t2 = time.time()

        return round((t2 - t1) * 340 / 2, 2)

    def close(self) -> None:
        """Close the sensor resource if needed."""
        if GPIO is not None:
            GPIO.cleanup(self.trigger_pin)
            GPIO.cleanup(self.echo_pin)


if __name__ == "__main__":
    sensor = UltrasonicSensor()
    print(sensor.distance())
