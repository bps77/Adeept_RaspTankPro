"""Adeept RaspTank Pro Python package.

This package is the current refactor stage: a clean object-oriented API around the
real Raspberry Pi hardware logic used by the original robot.
"""

from .tank import RaspTank
from .hardware.motors import MotorController
from .hardware.servos import ServoController
from .hardware.ultrasonic import UltrasonicSensor
from .hardware.leds import LEDStrip
from .hardware.display import OledDisplay
from .hardware.switches import SwitchPorts
from .hardware.line_sensors import LineSensors
from .hardware.imu import Mpu6050
from .hardware.camera import CameraController, Frame
from .autonomy import RoamPolicy, RoamingSession
from .filters import KalmanFilter, PID
from . import behaviors, system
from .config import DEFAULT_SPEED

__all__ = [
    "RaspTank",
    "MotorController",
    "ServoController",
    "UltrasonicSensor",
    "LEDStrip",
    "OledDisplay",
    "SwitchPorts",
    "LineSensors",
    "Mpu6050",
    "CameraController",
    "Frame",
    "RoamPolicy",
    "RoamingSession",
    "KalmanFilter",
    "PID",
    "behaviors",
    "system",
    "DEFAULT_SPEED",
]
