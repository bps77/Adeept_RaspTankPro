"""Explicit failures from the synchronous hardware API."""


class HardwareError(RuntimeError):
    """Hardware initialization, operation or cleanup failed."""


class MotorHardwareError(HardwareError):
    """Drivetrain hardware failed."""


class ServoHardwareError(HardwareError):
    """Servo hardware failed."""


class SensorTimeoutError(HardwareError, TimeoutError):
    """An ultrasonic echo edge did not arrive before its deadline."""


class HardwareDisabledError(HardwareError):
    """A reading was requested without hardware."""


class ControllerClosedError(HardwareError):
    """An operation was requested after close."""


class CameraError(HardwareError):
    """Camera startup, capture or worker shutdown failed."""


class CameraTimeoutError(CameraError, TimeoutError):
    """No fresh camera frame arrived within the requested deadline."""


class AutonomyError(HardwareError):
    """Supervised driving faulted; explicit restart is required."""
