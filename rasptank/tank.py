"""Small synchronous façade with explicit optional-device ownership."""

from .config import DEFAULT_SPEED, DEFAULT_TURN_RADIUS
from .hardware._lifecycle import ManagedController, ensure_open, release
from .hardware.motors import MotorController
from .hardware.servos import ServoController
from .hardware.ultrasonic import UltrasonicSensor
from .hardware.leds import LEDStrip
from .hardware.display import OledDisplay


class RaspTank(ManagedController):
    """Initialize stopped motors; optional devices are opt-in, servos never center.

    Injected controllers are borrowed: their caller must close them. Controllers
    created here are owned and rolled back on partial initialization failure.
    use_gpio=False disables hardware on every controller created by this façade;
    injected controllers retain their caller-selected mode.
    """

    def __init__(self, use_gpio=True, *, enable_servos=False, enable_ultrasonic=False,
                 enable_leds=False, enable_display=False, motors=None, servos=None,
                 ultrasonic=None, leds=None, display=None):
        self._closed = False
        self._owned = []
        self.motors = self.servos = self.ultrasonic = self.leds = self.display = None
        devices = (
            ("motors", motors, True, MotorController),
            ("servos", servos, enable_servos, ServoController),
            ("ultrasonic", ultrasonic, enable_ultrasonic, UltrasonicSensor),
            ("leds", leds, enable_leds, LEDStrip),
            ("display", display, enable_display, OledDisplay),
        )
        try:
            for name, supplied, enabled, factory in devices:
                device = supplied
                if device is None and enabled:
                    device = factory(use_gpio=use_gpio)
                    self._owned.append(device)
                setattr(self, name, device)
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            raise

    def move(self, speed=DEFAULT_SPEED, direction="no", turn="no",
             radius=DEFAULT_TURN_RADIUS):
        ensure_open(self)
        return self.motors.move(speed, direction, turn, radius)

    def forward(self, speed=DEFAULT_SPEED):
        ensure_open(self)
        return self.motors.forward(speed)

    def backward(self, speed=DEFAULT_SPEED):
        ensure_open(self)
        return self.motors.backward(speed)

    def turn_left(self, speed=DEFAULT_SPEED):
        ensure_open(self)
        return self.motors.turn_left(speed)

    def turn_right(self, speed=DEFAULT_SPEED):
        ensure_open(self)
        return self.motors.turn_right(speed)

    def stop(self):
        ensure_open(self)
        return self.motors.stop()

    def close(self):
        if self._closed:
            return
        self._closed = True
        # Stop owned drivetrain first, then attempt every remaining resource.
        release([device.close for device in self._owned])
