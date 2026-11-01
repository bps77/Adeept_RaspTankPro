"""MPU6050 accelerometer/gyro at I2C 0x68 (server/functions.py, mpu6050Test.py)."""

import time
from importlib import import_module

from ..config import MPU_ADDRESS
from ..exceptions import HardwareDisabledError, HardwareError
from ._lifecycle import ManagedController, ensure_open


class Mpu6050(ManagedController):
    """Uses the third-party `mpu6050` driver, imported only when enabled."""

    def __init__(self, address=MPU_ADDRESS, *, use_gpio=True, sensor=None):
        self.address = address
        self.use_gpio = use_gpio
        self.sensor = sensor
        self._closed = False
        if use_gpio and self.sensor is None:
            try:
                self.sensor = import_module("mpu6050").mpu6050(address)
            except Exception as exc:
                raise HardwareError("Unable to initialize MPU6050") from exc

    def _get(self, name):
        ensure_open(self)
        if not self.use_gpio:
            raise HardwareDisabledError("IMU hardware is disabled")
        try:
            return getattr(self.sensor, name)()
        except Exception as exc:
            raise HardwareError("MPU6050 read failed") from exc

    def accel(self):
        """Return {'x','y','z'} in m/s^2."""
        return self._get("get_accel_data")

    def gyro(self):
        return self._get("get_gyro_data")

    def temperature(self):
        return self._get("get_temp")

    def average_accel(self, samples=10, *, delay=0.0, sleep=time.sleep):
        """Mean of samples accelerometer readings (mpu6050Test.py uses 10)."""
        if type(samples) is not int or samples <= 0:
            raise ValueError("samples must be a positive integer")
        total = dict.fromkeys("xyz", 0.0)
        for _ in range(samples):
            reading = self.accel()
            for axis in total:
                total[axis] += reading[axis]
            if delay:
                sleep(delay)
        return {axis: value / samples for axis, value in total.items()}

    def close(self):
        self._closed = True
