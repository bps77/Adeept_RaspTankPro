"""Small synchronous façade with explicit optional-device ownership."""

from .config import DEFAULT_SPEED, DEFAULT_TURN_RADIUS
from .hardware._lifecycle import ManagedController, ensure_open, release
from .hardware.motors import MotorController
from .hardware.servos import ServoController
from .hardware.ultrasonic import UltrasonicSensor
from .hardware.leds import LEDStrip
from .hardware.display import OledDisplay
from .hardware.switches import SwitchPorts
from .hardware.line_sensors import LineSensors
from .hardware.imu import Mpu6050


class RaspTank(ManagedController):
    """Initialize stopped motors; optional devices are opt-in, servos never center.

    Injected controllers are borrowed: their caller must close them. Controllers
    created here are owned and rolled back on partial initialization failure.
    use_gpio=False disables hardware on every controller created by this façade;
    injected controllers retain their caller-selected mode.
    """

    def __init__(self, use_gpio=True, *, enable_servos=False, enable_ultrasonic=False,
                 enable_leds=False, enable_display=False, enable_switches=False,
                 enable_line_sensors=False, enable_imu=False, motors=None, servos=None,
                 ultrasonic=None, leds=None, display=None, switches=None,
                 line_sensors=None, imu=None):
        """Create stopped motors plus any enabled or injected optional devices.

        Args:
            use_gpio: False makes every created controller a hardware-free dry run.
            enable_*: create (and own) that optional device; default off.
            motors, servos, ultrasonic, leds, display, switches, line_sensors, imu:
                pre-built controllers to borrow instead of creating; the caller
                closes them.
        """
        self._closed = False
        self._owned = []
        self.motors = self.servos = self.ultrasonic = self.leds = self.display = None
        self.switches = self.line_sensors = self.imu = None
        devices = (
            ("motors", motors, True, MotorController),
            ("servos", servos, enable_servos, ServoController),
            ("ultrasonic", ultrasonic, enable_ultrasonic, UltrasonicSensor),
            ("leds", leds, enable_leds, LEDStrip),
            ("display", display, enable_display, OledDisplay),
            ("switches", switches, enable_switches, SwitchPorts),
            ("line_sensors", line_sensors, enable_line_sensors, LineSensors),
            ("imu", imu, enable_imu, Mpu6050),
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
        """Drive with speed 0-100, direction forward/backward/no, turn left/right/no."""
        ensure_open(self)
        return self.motors.move(speed, direction, turn, radius)

    def forward(self, speed=DEFAULT_SPEED):
        """Drive both tracks forward at speed (duty percent)."""
        ensure_open(self)
        return self.motors.forward(speed)

    def backward(self, speed=DEFAULT_SPEED):
        """Drive both tracks backward at speed (duty percent)."""
        ensure_open(self)
        return self.motors.backward(speed)

    def turn_left(self, speed=DEFAULT_SPEED):
        """Spin left in place."""
        ensure_open(self)
        return self.motors.turn_left(speed)

    def turn_right(self, speed=DEFAULT_SPEED):
        """Spin right in place."""
        ensure_open(self)
        return self.motors.turn_right(speed)

    def stop(self):
        """Stop both tracks immediately."""
        ensure_open(self)
        return self.motors.stop()

    def close(self):
        """Idempotently close owned devices only; borrowed ones are left open."""
        if self._closed:
            return
        self._closed = True
        # Stop owned drivetrain first, then attempt every remaining resource.
        release([device.close for device in self._owned])
