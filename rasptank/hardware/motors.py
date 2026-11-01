"""Motor abstraction for the Adeept RaspTank Pro.

This module preserves the behavior of the original `server/move.py` logic while
making it easier to use in a structured Python API.
"""

from __future__ import annotations

from importlib import import_module

from ._lifecycle import ManagedController, ensure_open, number, release

from ..config import (
    DEFAULT_SPEED,
    DEFAULT_TURN_RADIUS,
    DIR_BACKWARD,
    DIR_FORWARD,
    LEFT_BACKWARD,
    LEFT_FORWARD,
    MOTOR_A_EN,
    MOTOR_A_PIN1,
    MOTOR_A_PIN2,
    MOTOR_B_EN,
    MOTOR_B_PIN1,
    MOTOR_B_PIN2,
    RIGHT_BACKWARD,
    RIGHT_FORWARD,
)
from ..exceptions import MotorHardwareError


class MotorController(ManagedController):
    """Wrap the original robot motor logic while keeping the same GPIO behavior.

    This class intentionally mirrors the original `move.py` implementation so the
    refactor remains transparent and safe.
    """

    def __init__(self, use_gpio: bool = True, *, gpio=None):
        """Set up stopped motors if use_gpio; gpio may be an injected RPi.GPIO stand-in."""
        self.use_gpio = use_gpio
        self.gpio = gpio
        self.pwm_a = None
        self.pwm_b = None
        self._pins = []
        self._closed = False
        self._ready = False

        if self.use_gpio:
            self.setup()

    def setup(self) -> None:
        """Initialize the Raspberry Pi GPIO pins used for the drivetrain."""
        ensure_open(self)
        if self._ready:
            return
        try:
            if self.gpio is None:
                self.gpio = import_module("RPi.GPIO")
            gpio = self.gpio
            gpio.setwarnings(False)
            gpio.setmode(gpio.BCM)
            for pin in (MOTOR_A_EN, MOTOR_B_EN, MOTOR_A_PIN1,
                        MOTOR_A_PIN2, MOTOR_B_PIN1, MOTOR_B_PIN2):
                gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
                self._pins.append(pin)
            self.pwm_a = gpio.PWM(MOTOR_A_EN, 1000)
            self.pwm_a.start(0)
            self.pwm_b = gpio.PWM(MOTOR_B_EN, 1000)
            self.pwm_b.start(0)
            self.use_gpio = True
            self._ready = True
        except BaseException as exc:
            try:
                self.close()
            except BaseException:
                pass
            if not isinstance(exc, Exception):
                raise
            raise MotorHardwareError("Unable to initialize PWM for motors") from exc

    def motorStop(self) -> None:
        """Immediately stop both motors and disable the enable pins."""
        ensure_open(self)
        self._stop_outputs()

    def _stop_outputs(self):
        """Zero both duty cycles and drive every owned pin low, attempting all."""
        actions = [lambda pwm=pwm: pwm.ChangeDutyCycle(0)
                   for pwm in (self.pwm_a, self.pwm_b) if pwm is not None]
        actions.extend(lambda pin=pin: self.gpio.output(pin, self.gpio.LOW)
                       for pin in self._pins)
        release(actions)

    def _motor(self, status, direction, speed, left):
        """Validate, then drive one track; failures close the controller and raise."""
        ensure_open(self)
        if (type(status) is not int or type(direction) is not int
                or status not in (0, 1) or direction not in (DIR_FORWARD, DIR_BACKWARD)):
            raise ValueError("status and direction must be 0 or 1")
        number(speed, 0, 100, "speed")
        if not self.use_gpio:
            return direction
        gpio = self.gpio
        pin1, pin2, enable, pwm = (
            (MOTOR_B_PIN1, MOTOR_B_PIN2, MOTOR_B_EN, self.pwm_b) if left else
            (MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_A_EN, self.pwm_a))
        try:
            pwm.ChangeDutyCycle(0)
            high_first = direction == (DIR_BACKWARD if left else DIR_FORWARD)
            gpio.output(pin1, gpio.HIGH if status and high_first else gpio.LOW)
            gpio.output(pin2, gpio.HIGH if status and not high_first else gpio.LOW)
            if status:
                pwm.ChangeDutyCycle(speed)
            else:
                gpio.output(enable, gpio.LOW)
        except Exception as exc:
            try:
                self.close()
            except BaseException:
                pass
            raise MotorHardwareError("Motor output failed") from exc
        return direction

    def motor_left(self, status: int, direction: int, speed: int) -> int:
        """Control the left motor using the original direction logic.

        Args:
            status: 0 stops the motor, 1 runs it.
            direction: raw legacy 0/1 polarity (left forward uses 1).
            speed: duty cycle from 0 to 100.
        """
        return self._motor(status, direction, speed, left=True)

    def motor_right(self, status: int, direction: int, speed: int) -> int:
        """Control the right motor using the original direction logic."""
        return self._motor(status, direction, speed, left=False)

    def move(self, speed: int, direction: str, turn: str, radius: float = DEFAULT_TURN_RADIUS) -> None:
        """Move the robot with the original direction/turn rules.

        This function intentionally matches the original implementation to keep the
        refactor behaviorally transparent.
        """
        ensure_open(self)
        number(speed, 0, 100, "speed")
        number(radius, 0, 1, "radius")
        if radius == 0:
            raise ValueError("radius must be greater than 0")
        if direction not in ("forward", "backward", "no") or turn not in ("left", "right", "no"):
            raise ValueError("Invalid movement direction or turn")
        if direction == "forward":
            if turn == "right":
                self.motor_left(0, LEFT_BACKWARD, int(speed * radius))
                self.motor_right(1, RIGHT_FORWARD, speed)
            elif turn == "left":
                self.motor_left(1, LEFT_FORWARD, speed)
                self.motor_right(0, RIGHT_BACKWARD, int(speed * radius))
            else:
                self.motor_left(1, LEFT_FORWARD, speed)
                self.motor_right(1, RIGHT_FORWARD, speed)
        elif direction == "backward":
            if turn == "right":
                self.motor_left(0, LEFT_FORWARD, int(speed * radius))
                self.motor_right(1, RIGHT_BACKWARD, speed)
            elif turn == "left":
                self.motor_left(1, LEFT_BACKWARD, speed)
                self.motor_right(0, RIGHT_FORWARD, int(speed * radius))
            else:
                self.motor_left(1, LEFT_BACKWARD, speed)
                self.motor_right(1, RIGHT_BACKWARD, speed)
        elif direction == "no":
            if turn == "right":
                self.motor_left(1, LEFT_BACKWARD, speed)
                self.motor_right(1, RIGHT_FORWARD, speed)
            elif turn == "left":
                self.motor_left(1, LEFT_FORWARD, speed)
                self.motor_right(1, RIGHT_BACKWARD, speed)
            else:
                self.motorStop()

    def forward(self, speed: int = DEFAULT_SPEED) -> None:
        """Drive forward at the requested speed."""
        self.move(speed, "forward", "no")

    def backward(self, speed: int = DEFAULT_SPEED) -> None:
        """Drive backward at the requested speed."""
        self.move(speed, "backward", "no")

    def turn_left(self, speed: int = DEFAULT_SPEED) -> None:
        """Spin left in place using the original turning behavior."""
        self.move(speed, "no", "left")

    def turn_right(self, speed: int = DEFAULT_SPEED) -> None:
        """Spin right in place using the original turning behavior."""
        self.move(speed, "no", "right")

    def stop(self) -> None:
        """Immediately stop both motors."""
        self.motorStop()

    def close(self) -> None:
        """Clean up the hardware state.

        This is safe to call when the robot is shutting down or the notebook session
        is interrupted.
        """
        if self._closed:
            return
        self._closed = True
        actions = [self._stop_outputs]
        actions.extend(lambda pwm=pwm: pwm.stop()
                       for pwm in (self.pwm_a, self.pwm_b) if pwm is not None)
        actions.extend(lambda pin=pin: self.gpio.cleanup(pin) for pin in self._pins)
        release(actions)
