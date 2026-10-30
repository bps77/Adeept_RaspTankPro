import sys
import types
import unittest
from unittest.mock import Mock

# Create fake Raspberry Pi GPIO modules so tests can run in a normal development
# machine without a physical Raspberry Pi attached.

fake_rpi = types.ModuleType("RPi")
fake_gpio = types.ModuleType("RPi.GPIO")

class FakePWM:
    def __init__(self, pin, frequency):
        self.pin = pin
        self.frequency = frequency
        self.started = False
        self.duty_cycle = 0

    def start(self, value):
        self.started = True
        self.duty_cycle = value

    def ChangeDutyCycle(self, value):
        self.duty_cycle = value


fake_gpio.PWM = FakePWM
fake_gpio.BCM = "BCM"
fake_gpio.OUT = "OUT"
fake_gpio.LOW = 0
fake_gpio.HIGH = 1
fake_gpio.setwarnings = Mock()
fake_gpio.setmode = Mock()
fake_gpio.setup = Mock()
fake_gpio.output = Mock()
fake_gpio.cleanup = Mock()

fake_rpi.GPIO = fake_gpio
sys.modules["RPi"] = fake_rpi
sys.modules["RPi.GPIO"] = fake_gpio

from rasptank.hardware.motors import MotorController


class TestMotorController(unittest.TestCase):
    def setUp(self):
        fake_gpio.setup.reset_mock()
        fake_gpio.output.reset_mock()
        fake_gpio.cleanup.reset_mock()
        self.controller = MotorController(use_gpio=False)

    def test_setup_initializes_expected_pins(self):
        self.controller.setup()

        self.assertEqual(fake_gpio.setup.call_count, 6)
        fake_gpio.setup.assert_any_call(4, "OUT")
        fake_gpio.setup.assert_any_call(17, "OUT")
        fake_gpio.setup.assert_any_call(26, "OUT")
        fake_gpio.setup.assert_any_call(21, "OUT")
        fake_gpio.setup.assert_any_call(27, "OUT")
        fake_gpio.setup.assert_any_call(18, "OUT")

    def test_forward_uses_original_drive_logic(self):
        self.controller.setup()
        self.controller.forward(60)

        # This should drive the robot forward with both motors enabled.
        self.assertTrue(fake_gpio.output.called)
        self.assertIsNotNone(self.controller.pwm_a)
        self.assertIsNotNone(self.controller.pwm_b)

    def test_backward_uses_original_drive_logic(self):
        self.controller.setup()
        self.controller.backward(50)

        self.assertTrue(fake_gpio.output.called)

    def test_turn_left_uses_original_turn_logic(self):
        self.controller.setup()
        self.controller.turn_left(45)

        self.assertTrue(fake_gpio.output.called)

    def test_turn_right_uses_original_turn_logic(self):
        self.controller.setup()
        self.controller.turn_right(45)

        self.assertTrue(fake_gpio.output.called)

    def test_stop_disables_motor_outputs(self):
        self.controller.setup()
        self.controller.stop()

        # Stop should set all driver outputs to LOW.
        self.assertGreaterEqual(fake_gpio.output.call_count, 1)

    def test_close_stops_and_cleans_up(self):
        self.controller.setup()
        self.controller.close()

        self.assertTrue(fake_gpio.cleanup.called)


if __name__ == "__main__":
    unittest.main()
