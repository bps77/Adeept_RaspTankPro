"""Motor abstraction for the Adeept RaspTank Pro.

This module preserves the behavior of the original `server/move.py` logic while
making it easier to use in a structured Python API.
"""

from __future__ import annotations

from typing import Optional

import RPi.GPIO as GPIO

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


class MotorController:
    """Wrap the original robot motor logic while keeping the same GPIO behavior.

    This class intentionally mirrors the original `move.py` implementation so the
    refactor remains transparent and safe.
    """

    def __init__(self, use_gpio: bool = True):
        self.use_gpio = use_gpio
        self.pwm_a: Optional[GPIO.PWM] = None
        self.pwm_b: Optional[GPIO.PWM] = None

        if self.use_gpio:
            self.setup()

    def setup(self) -> None:
        """Initialize the Raspberry Pi GPIO pins used for the drivetrain."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_A_EN, GPIO.OUT)
        GPIO.setup(MOTOR_B_EN, GPIO.OUT)
        GPIO.setup(MOTOR_A_PIN1, GPIO.OUT)
        GPIO.setup(MOTOR_A_PIN2, GPIO.OUT)
        GPIO.setup(MOTOR_B_PIN1, GPIO.OUT)
        GPIO.setup(MOTOR_B_PIN2, GPIO.OUT)

        self.motorStop()

        try:
            self.pwm_a = GPIO.PWM(MOTOR_A_EN, 1000)
            self.pwm_b = GPIO.PWM(MOTOR_B_EN, 1000)
        except Exception as exc:  # pragma: no cover - hardware path only
            raise MotorHardwareError("Unable to initialize PWM for motors") from exc

    def motorStop(self) -> None:
        """Immediately stop both motors and disable the enable pins."""
        GPIO.output(MOTOR_A_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
        GPIO.output(MOTOR_B_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
        GPIO.output(MOTOR_A_EN, GPIO.LOW)
        GPIO.output(MOTOR_B_EN, GPIO.LOW)

    def motor_left(self, status: int, direction: int, speed: int) -> int:
        """Control the left motor using the original direction logic.

        Args:
            status: 0 stops the motor, 1 runs it.
            direction: 0 for forward, 1 for backward.
            speed: duty cycle from 0 to 100.
        """
        if status == 0:
            GPIO.output(MOTOR_B_PIN1, GPIO.LOW)
            GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
            GPIO.output(MOTOR_B_EN, GPIO.LOW)
            return direction

        if direction == DIR_BACKWARD:
            GPIO.output(MOTOR_B_PIN1, GPIO.HIGH)
            GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
            if self.pwm_b is not None:
                self.pwm_b.start(100)
                self.pwm_b.ChangeDutyCycle(speed)
        elif direction == DIR_FORWARD:
            GPIO.output(MOTOR_B_PIN1, GPIO.LOW)
            GPIO.output(MOTOR_B_PIN2, GPIO.HIGH)
            if self.pwm_b is not None:
                self.pwm_b.start(0)
                self.pwm_b.ChangeDutyCycle(speed)
        return direction

    def motor_right(self, status: int, direction: int, speed: int) -> int:
        """Control the right motor using the original direction logic."""
        if status == 0:
            GPIO.output(MOTOR_A_PIN1, GPIO.LOW)
            GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
            GPIO.output(MOTOR_A_EN, GPIO.LOW)
            return direction

        if direction == DIR_FORWARD:
            GPIO.output(MOTOR_A_PIN1, GPIO.HIGH)
            GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
            if self.pwm_a is not None:
                self.pwm_a.start(100)
                self.pwm_a.ChangeDutyCycle(speed)
        elif direction == DIR_BACKWARD:
            GPIO.output(MOTOR_A_PIN1, GPIO.LOW)
            GPIO.output(MOTOR_A_PIN2, GPIO.HIGH)
            if self.pwm_a is not None:
                self.pwm_a.start(0)
                self.pwm_a.ChangeDutyCycle(speed)
        return direction

    def move(self, speed: int, direction: str, turn: str, radius: float = DEFAULT_TURN_RADIUS) -> None:
        """Move the robot with the original direction/turn rules.

        This function intentionally matches the original implementation to keep the
        refactor behaviorally transparent.
        """
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
        self.stop()
        GPIO.cleanup()

    def __enter__(self) -> "MotorController":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


if __name__ == "__main__":
    controller = MotorController()
    controller.forward(60)
