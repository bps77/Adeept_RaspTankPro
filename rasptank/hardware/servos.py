"""Synchronous PCA9685 pulse control, without legacy threads or file calibration."""

from importlib import import_module

from ..config import (SERVO_ADDRESS, SERVO_BUS, SERVO_DIRECTIONS, SERVO_FREQUENCY,
                      SERVO_INITIAL, SERVO_RANGES)
from ..exceptions import ServoHardwareError
from ._lifecycle import ManagedController, ensure_open, release

_UNSET = object()


class ServoController(ManagedController):
    """Channels 0..4 use servo.py ranges; bus/address follow RPIservo.py.

    Positions begin as *assumed* 300-tick values, not physical measurements.
    Construction sets frequency only: no channel is centered or energized.
    Positive relative steps follow servo.py's direction flag (1 adds, 0 subtracts).
    """

    def __init__(self, use_gpio=True, *, pwm=None, address=SERVO_ADDRESS, bus=SERVO_BUS):
        self.use_gpio = use_gpio
        self.pwm = pwm
        self.positions = dict.fromkeys(range(5), SERVO_INITIAL)
        self.initial = self.positions.copy()
        self._channels = set()
        self._closed = False
        if use_gpio:
            try:
                if self.pwm is None:
                    self.pwm = import_module("Adafruit_PCA9685").PCA9685(address, busnum=bus)
                self.pwm.set_pwm_freq(SERVO_FREQUENCY)
            except BaseException as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                if not isinstance(exc, Exception):
                    raise
                raise ServoHardwareError("Unable to initialize PCA9685") from exc

    def _validate(self, channel, pulse=_UNSET):
        ensure_open(self)
        if type(channel) is not int or channel not in self.positions:
            raise ValueError("channel must be an integer from 0 to 4")
        if pulse is not _UNSET:
            low, high = SERVO_RANGES[channel]
            if type(pulse) is not int or not low <= pulse <= high:
                raise ValueError(f"pulse must be an integer in [{low}, {high}]")

    def set_pulse(self, channel, pulse):
        """Immediately command a pulse in PCA9685 ticks (not degrees)."""
        self._validate(channel, pulse)
        if self.use_gpio:
            self._channels.add(channel)
            try:
                self.pwm.set_pwm(channel, 0, pulse)
            except Exception as exc:
                try:
                    self.close()
                except BaseException:
                    pass
                raise ServoHardwareError("Servo output failed") from exc
        self.positions[channel] = pulse
        return pulse

    def move_relative(self, channel, steps):
        """Apply signed ticks in the configured direction; reject, never clamp."""
        self._validate(channel)
        if type(steps) is not int:
            raise ValueError("steps must be an integer")
        sign = 1 if SERVO_DIRECTIONS[channel] else -1
        return self.set_pulse(channel, self.positions[channel] + sign * steps)

    def calibrate(self, channel, pulse):
        """Change only this instance's initial pulse; no motion or persistence."""
        self._validate(channel, pulse)
        self.initial[channel] = pulse

    def center(self, channel):
        """Explicitly move one channel to its transient initial pulse."""
        self._validate(channel)
        return self.set_pulse(channel, self.initial[channel])

    def close(self):
        """Release only channels this controller has commanded, never all PWM."""
        if self._closed:
            return
        self._closed = True
        release([lambda channel=channel: self.pwm.set_pwm(channel, 0, 4096)
                 for channel in sorted(self._channels)])
