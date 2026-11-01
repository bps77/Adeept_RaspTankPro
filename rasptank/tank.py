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
from .hardware.camera import CameraController
from .autonomy import DriveSupervisor, RoamPolicy


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
                 line_sensors=None, imu=None, enable_camera=False, camera=None,
                 command_timeout=None):
        """Create stopped motors plus any enabled or injected optional devices.

        Args:
            use_gpio: False makes every created controller a hardware-free dry run.
            enable_*: create (and own) that optional device; default off.
            motors, servos, ultrasonic, leds, display, switches, line_sensors, imu:
                pre-built controllers to borrow instead of creating; the caller
                closes them.
            enable_camera, camera: optional owned or borrowed camera, not started.
            command_timeout: opt-in manual/autonomous motor lease in seconds
                (0.05..2); required for roaming. None preserves synchronous API.
        """
        self._closed = False
        self._owned = []
        self._supervisor = None
        self._views = []
        self.motors = self.servos = self.ultrasonic = self.leds = self.display = None
        self.switches = self.line_sensors = self.imu = None
        self.camera = None
        devices = (
            ("motors", motors, True, MotorController),
            ("servos", servos, enable_servos, ServoController),
            ("ultrasonic", ultrasonic, enable_ultrasonic, UltrasonicSensor),
            ("leds", leds, enable_leds, LEDStrip),
            ("display", display, enable_display, OledDisplay),
            ("switches", switches, enable_switches, SwitchPorts),
            ("line_sensors", line_sensors, enable_line_sensors, LineSensors),
            ("imu", imu, enable_imu, Mpu6050),
            ("camera", camera, enable_camera, CameraController),
        )
        try:
            for name, supplied, enabled, factory in devices:
                device = supplied
                if device is None and enabled:
                    device = factory(use_gpio=use_gpio)
                    self._owned.append(device)
                setattr(self, name, device)
            if command_timeout is not None:
                self._supervisor = DriveSupervisor(self.motors, command_timeout)
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
        return self._drive("move", speed, direction, turn, radius)

    def forward(self, speed=DEFAULT_SPEED):
        """Drive both tracks forward at speed (duty percent)."""
        ensure_open(self)
        return self._drive("forward", speed)

    def backward(self, speed=DEFAULT_SPEED):
        """Drive both tracks backward at speed (duty percent)."""
        ensure_open(self)
        return self._drive("backward", speed)

    def turn_left(self, speed=DEFAULT_SPEED):
        """Spin left in place."""
        ensure_open(self)
        return self._drive("turn_left", speed)

    def turn_right(self, speed=DEFAULT_SPEED):
        """Spin right in place."""
        ensure_open(self)
        return self._drive("turn_right", speed)

    def stop(self):
        """Stop both tracks immediately."""
        ensure_open(self)
        if self._supervisor is not None:
            return self._supervisor.stop()
        return self.motors.stop()

    def _drive(self, method, *args):
        """Route facade output through opt-in arbitration; low-level calls bypass it."""
        if self._supervisor is not None:
            return self._supervisor.manual(method, *args)
        return getattr(self.motors, method)(*args)

    def start_camera(self):
        """Start enabled/borrowed camera explicitly; raise if no camera was configured."""
        ensure_open(self)
        if self.camera is None:
            raise ValueError("Enable or inject a camera first")
        return self.camera.start()

    def show_camera(self, *, fps=10, transform=None):
        """Replace this tank's notebook view, leaving capture borrowed by presentation."""
        ensure_open(self)
        if self.camera is None:
            raise ValueError("Enable or inject a camera first")
        from .notebook import show_camera
        release([view.close for view in self._views])
        self._views = []
        view = show_camera(self.camera, fps=fps, transform=transform,
                           on_disconnect=self.stop if self._supervisor is not None else None)
        self._views.append(view)
        return view

    def start_roaming(self, *, observer, policy=None):
        """Start finite supervised roaming using fresh camera/range data; never autostart camera.

        Requires command_timeout, camera and ultrasonic. Observer must be
        explicitly calibrated; RoamPolicy defaults are unverified on hardware.
        """
        ensure_open(self)
        if self._supervisor is None:
            raise ValueError("Configure command_timeout when creating the tank")
        if self.camera is None or self.ultrasonic is None:
            raise ValueError("Roaming requires camera and ultrasonic controllers")
        return self._supervisor.start(self.camera, self.ultrasonic, observer,
                                      RoamPolicy() if policy is None else policy)

    def stop_roaming(self):
        """Cancel autonomous output and stop motors, including if no session is active."""
        return self.stop()

    def roaming_status(self):
        """Return driving telemetry; unsupervised tanks explicitly report supervision off."""
        ensure_open(self)
        if self._supervisor is None:
            return dict(state="supervision_disabled", error=None, decision=None)
        return self._supervisor.status()

    def reset_drive_fault(self):
        """Explicitly reset a latched fault after inspecting hardware; stops before reset."""
        ensure_open(self)
        if self._supervisor is None:
            raise ValueError("Driving supervision is not enabled")
        self._supervisor.reset_fault()

    def close(self):
        """Idempotently close owned devices only; borrowed ones are left open."""
        if self._closed:
            return
        self._closed = True
        # Stop owned drivetrain first, then attempt every remaining resource.
        actions = ([self._supervisor.close] if self._supervisor is not None else [])
        actions.extend(view.close for view in self._views)
        actions.extend(device.close for device in self._owned)
        release(actions)
