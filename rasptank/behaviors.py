"""Bounded, synchronous single-step ports of server/functions.py and findline.py.

Each function performs exactly one control step and returns; callers own the loop,
timing and shutdown (no threads). Motor outputs use the raw legacy motor_left/right
polarity constants, and any failure stops the drivetrain before re-raising.
"""

import time

from .config import (LEFT_BACKWARD, LEFT_FORWARD, RADAR_SERVO, RIGHT_BACKWARD,
                     RIGHT_FORWARD, SERVO_RANGES, STEADY_SERVO)
from .exceptions import ControllerClosedError, SensorTimeoutError
from .filters import KalmanFilter

# Line states (left, middle, right) -> (mark, kick, left speed, right speed, extra sleep).
_LINE_TABLE = {
    (0, 1, 0): (1, False, 80, 80, 0.0),
    (1, 1, 0): (2, True, 70, 100, 0.0),
    (1, 0, 0): (3, True, 0, 100, 0.02),
    (0, 1, 1): (4, True, 100, 70, 0.0),
    (0, 0, 1): (5, True, 100, 0, 0.02),
}
# Behaviour for any other reading depends on the last mark.
_LINE_LOST = {0: (80, 80, 0.0), 1: (80, 80, 0.0), 2: (0, 100, 0.03), 3: (0, 100, 0.03),
              4: (100, 0, 0.03), 5: (100, 0, 0.03)}


def _guard(motors, action):
    try:
        return action()
    except BaseException:
        try:
            motors.stop()
        except BaseException:
            pass
        raise


def line_follow_step(motors, readings, mark=0, *, sleep=time.sleep):
    """One functions.py trackLine step; returns the new mark to pass back in.

    readings is (left, middle, right). When the line state changes the legacy code
    briefly reverses both tracks (0.03 s) as a brake before steering.
    """
    key = tuple(int(bool(value)) for value in readings)
    if len(key) != 3:
        raise ValueError("readings must be (left, middle, right)")

    def run():
        new_mark = mark
        if key in _LINE_TABLE:
            new_mark, kick_on_change, left, right, extra = _LINE_TABLE[key]
            if kick_on_change and mark != new_mark:
                motors.motor_left(1, LEFT_BACKWARD, 80)
                motors.motor_right(1, RIGHT_BACKWARD, 80)
                sleep(0.03)
        else:
            left, right, extra = _LINE_LOST.get(mark, (80, 80, 0.0))
        motors.motor_left(1, LEFT_FORWARD, left)
        motors.motor_right(1, RIGHT_FORWARD, right)
        if extra:
            sleep(extra)
        sleep(0.1)
        return new_mark

    return _guard(motors, run)


def simple_line_step(motors, readings):
    """One findline.py step at full speed (middle, then left, right, else reverse)."""
    left, middle, right = (int(bool(value)) for value in readings)
    if middle:
        _guard(motors, lambda: motors.move(100, "forward", "no", 1))
    elif left:
        _guard(motors, lambda: motors.move(100, "forward", "right", 0.6))
    elif right:
        _guard(motors, lambda: motors.move(100, "forward", "left", 0.6))
    else:
        _guard(motors, lambda: motors.move(100, "backward", "no", 1))


def obstacle_step(motors, ultrasonic, range_keep=0.7, *, sleep=time.sleep):
    """One functions.py 'Automatic' step using metres from the ultrasonic sensor.

    Closer than range_keep/3: reverse; closer than range_keep: turn left; else go
    forward. An echo timeout or sensor failure stops the tank and is re-raised.
    """
    def run():
        distance = ultrasonic.distance()
        if range_keep / 3 > distance:
            motors.move(100, "backward", "no", 0.5)
            action = "backward"
        elif range_keep > distance:
            motors.move(100, "no", "left", 0.5)
            action = "left"
        else:
            motors.move(100, "forward", "no", 0.5)
            action = "forward"
        sleep(0.1)
        return action

    return _guard(motors, run)


def radar_scan(servos, ultrasonic, *, channel=RADAR_SERVO, step=3, max_distance=20,
               settle=0.8, sleep=time.sleep):
    """Sweep one servo high-to-low in step ticks, returning [[metres, degrees], ...].

    Angle uses the legacy 100-tick/2.55-ticks-per-degree mapping. Points farther than
    max_distance and echo timeouts are skipped. The servo returns to its initial pulse.
    """
    low, high = SERVO_RANGES[channel]
    if type(step) is not int or step <= 0:
        raise ValueError("step must be a positive integer")
    result = []
    try:
        servos.set_pulse(channel, high)
        sleep(settle)
        for pulse in range(high - step, low - 1, -step):
            servos.set_pulse(channel, pulse)
            try:
                distance = ultrasonic.distance()
            except SensorTimeoutError:
                continue
            if distance <= max_distance:
                result.append([distance, 180 - (pulse - 100) / 2.55])
    finally:
        try:
            servos.center(channel)
        except ControllerClosedError:
            pass
    return result


class SteadyStabilizer:
    """functions.py 'Steady' mode: counter accelerometer x tilt on one servo.

    Output is goal + round(23/9 * 9 * filtered_x) ticks, clamped to the servo range
    (the legacy code did not clamp here). The Kalman state lives in this instance.
    """

    def __init__(self, servos, imu, goal=300, *, channel=STEADY_SERVO, kalman=None):
        self.servos = servos
        self.imu = imu
        self.goal = goal
        self.channel = channel
        self.kalman = kalman or KalmanFilter(0.01, 0.1)

    def step(self):
        filtered = self.kalman.update(self.imu.accel()["x"])
        low, high = SERVO_RANGES[self.channel]
        pulse = self.goal + int(round(23 / 9 * filtered * 9))
        return self.servos.set_pulse(self.channel, max(low, min(high, pulse)))
