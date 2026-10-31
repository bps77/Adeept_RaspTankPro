import unittest
from unittest.mock import Mock, call, patch

from rasptank.config import (
    DIR_BACKWARD, DIR_FORWARD, LEFT_BACKWARD, LEFT_FORWARD, MOTOR_A_EN,
    MOTOR_A_PIN1, MOTOR_A_PIN2, MOTOR_B_EN, MOTOR_B_PIN1, MOTOR_B_PIN2,
    RIGHT_BACKWARD, RIGHT_FORWARD,
)
from rasptank.exceptions import ControllerClosedError, MotorHardwareError
from rasptank.hardware.motors import MotorController


def make_gpio():
    gpio = Mock(BCM="BCM", OUT="OUT", IN="IN", LOW=0, HIGH=1)
    gpio.PWM.side_effect = [Mock(), Mock()]
    return gpio


class TestMotorController(unittest.TestCase):
    def setUp(self):
        self.gpio = make_gpio()
        self.controller = MotorController(use_gpio=False, gpio=self.gpio)
        self.addCleanup(self.controller.close)

    def setup(self):
        self.controller.setup()
        self.gpio.output.reset_mock()
        return self.controller

    def test_exact_legacy_config(self):
        self.assertEqual((MOTOR_A_EN, MOTOR_B_EN, MOTOR_A_PIN1, MOTOR_A_PIN2,
                          MOTOR_B_PIN1, MOTOR_B_PIN2), (4, 17, 26, 21, 27, 18))
        self.assertEqual((DIR_FORWARD, DIR_BACKWARD, LEFT_FORWARD, LEFT_BACKWARD,
                          RIGHT_FORWARD, RIGHT_BACKWARD), (0, 1, 1, 0, 0, 1))

    def test_setup_initializes_expected_pins(self):
        self.controller.setup()
        self.assertEqual(self.gpio.setup.call_args_list, [
            call(pin, "OUT", initial=0) for pin in (4, 17, 26, 21, 27, 18)])
        self.gpio.setmode.assert_called_once_with("BCM")
        self.assertEqual(self.gpio.PWM.call_args_list, [call(4, 1000), call(17, 1000)])
        self.controller.pwm_a.start.assert_called_once_with(0)
        self.controller.pwm_b.start.assert_called_once_with(0)
        self.assertTrue(self.controller.use_gpio)

    def test_setup_explicitly_enables_hardware_after_disabled_constructor(self):
        with patch("rasptank.hardware.motors.import_module", return_value=self.gpio) as load:
            controller = MotorController(use_gpio=False)
            load.assert_not_called()
            controller.setup()
            load.assert_called_once_with("RPi.GPIO")
            controller.forward()
            controller.close()

    def test_setup_is_idempotent(self):
        self.controller.setup()
        self.controller.setup()
        self.assertEqual(self.gpio.setup.call_count, 6)

    def assert_drive(self, method, speed, values):
        controller = self.setup()
        getattr(controller, method)(speed)
        self.assertEqual(self.gpio.output.call_args_list,
                         [call(pin, level) for pin, level in zip((27, 18, 26, 21), values)])
        self.assertEqual(controller.pwm_a.ChangeDutyCycle.call_args_list, [call(0), call(speed)])
        self.assertEqual(controller.pwm_b.ChangeDutyCycle.call_args_list, [call(0), call(speed)])

    def test_forward_uses_original_drive_logic(self):
        self.assert_drive("forward", 60, (1, 0, 1, 0))

    def test_backward_uses_original_drive_logic(self):
        self.assert_drive("backward", 50, (0, 1, 0, 1))

    def test_turn_left_uses_original_turn_logic(self):
        self.assert_drive("turn_left", 45, (1, 0, 0, 1))

    def test_turn_right_uses_original_turn_logic(self):
        self.assert_drive("turn_right", 45, (0, 1, 1, 0))

    def test_all_move_combinations_keep_legacy_stopped_inner_track(self):
        expected = {
            ("forward", "no"): [(1, 1, 50), (1, 0, 50)],
            ("forward", "right"): [(0, 0, 30), (1, 0, 50)],
            ("forward", "left"): [(1, 1, 50), (0, 1, 30)],
            ("backward", "no"): [(1, 0, 50), (1, 1, 50)],
            ("backward", "right"): [(0, 1, 30), (1, 1, 50)],
            ("backward", "left"): [(1, 0, 50), (0, 0, 30)],
            ("no", "right"): [(1, 0, 50), (1, 0, 50)],
            ("no", "left"): [(1, 1, 50), (1, 1, 50)],
        }
        with patch.object(self.controller, "motor_left") as left, \
                patch.object(self.controller, "motor_right") as right:
            for (direction, turn), values in expected.items():
                with self.subTest(direction=direction, turn=turn):
                    self.controller.move(50, direction, turn)
                    left.assert_called_with(*values[0])
                    right.assert_called_with(*values[1])

    def test_stop_disables_motor_outputs(self):
        controller = self.setup()
        controller.stop()
        self.assertEqual(self.gpio.output.call_args_list,
                         [call(pin, 0) for pin in (4, 17, 26, 21, 27, 18)])
        controller.pwm_a.ChangeDutyCycle.assert_called_once_with(0)
        controller.pwm_b.ChangeDutyCycle.assert_called_once_with(0)

    def test_stopped_track_has_zero_duty(self):
        controller = self.setup()
        controller.move(50, "forward", "right")
        self.assertEqual(controller.pwm_b.ChangeDutyCycle.call_args_list, [call(0)])
        self.assertEqual(self.gpio.output.call_args_list,
                         [call(27, 0), call(18, 0), call(17, 0), call(26, 1), call(21, 0)])

    def test_pwm_is_started_once_at_zero_not_on_each_command(self):
        controller = self.setup()
        controller.forward()
        controller.backward()
        controller.pwm_a.start.assert_called_once_with(0)
        controller.pwm_b.start.assert_called_once_with(0)

    def test_invalid_move_never_outputs(self):
        controller = self.setup()
        for args in ((101, "forward", "no", .6), (-1, "forward", "no", .6),
                     (float("nan"), "forward", "no", .6), (True, "forward", "no", .6),
                     ("50", "forward", "no", .6), (50, "bad", "no", .6),
                     (50, "forward", "bad", .6), (50, "forward", "no", 0),
                     (50, "forward", "no", 1.1), (50, "forward", "no", float("inf"))):
            with self.subTest(args=args), self.assertRaises(ValueError):
                controller.move(*args)
        self.gpio.output.assert_not_called()
        controller.pwm_a.ChangeDutyCycle.assert_not_called()
        controller.pwm_b.ChangeDutyCycle.assert_not_called()

    def test_invalid_raw_motor_never_outputs(self):
        controller = self.setup()
        for args in ((2, 0, 50), (1, 2, 50), (1, 0, 101), (True, 0, 50)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                controller.motor_left(*args)
        self.gpio.output.assert_not_called()

    def test_disabled_operation_does_not_touch_injected_gpio(self):
        self.controller.forward()
        self.controller.backward()
        self.controller.turn_left()
        self.controller.turn_right()
        self.controller.stop()
        self.controller.close()
        self.assertEqual(self.gpio.mock_calls, [])

    def test_close_stops_and_cleans_up_only_acquired_pins(self):
        controller = self.setup()
        controller.close()
        controller.close()
        controller.pwm_a.stop.assert_called_once_with()
        controller.pwm_b.stop.assert_called_once_with()
        self.assertEqual(self.gpio.cleanup.call_args_list,
                         [call(pin) for pin in (4, 17, 26, 21, 27, 18)])

    def test_closed_controller_rejects_all_commands(self):
        self.controller.close()
        for method in ("setup", "forward", "backward", "turn_left", "turn_right",
                       "stop", "motorStop"):
            with self.subTest(method=method), self.assertRaises(ControllerClosedError):
                getattr(self.controller, method)()
        with self.assertRaises(ControllerClosedError):
            self.controller.motor_right(1, 0, 50)

    def test_partial_pin_initialization_cleans_only_successful_pins(self):
        self.gpio.setup.side_effect = [None, None, OSError("pin")]
        with self.assertRaises(MotorHardwareError):
            self.controller.setup()
        self.assertEqual(self.gpio.cleanup.call_args_list, [call(4), call(17)])
        self.gpio.PWM.assert_not_called()

    def test_partial_pwm_initialization_stops_first_pwm(self):
        pwm = Mock()
        self.gpio.PWM.side_effect = [pwm, OSError("PWM")]
        with self.assertRaises(MotorHardwareError):
            self.controller.setup()
        pwm.start.assert_called_once_with(0)
        pwm.stop.assert_called_once_with()
        self.assertEqual(self.gpio.cleanup.call_count, 6)

    def test_failed_pwm_start_is_released(self):
        pwm = Mock()
        pwm.start.side_effect = OSError("start")
        self.gpio.PWM.side_effect = [pwm]
        with self.assertRaises(MotorHardwareError):
            self.controller.setup()
        pwm.stop.assert_called_once_with()

    def test_cleanup_attempts_every_resource_after_failures(self):
        controller = self.setup()
        self.gpio.output.side_effect = OSError("output")
        controller.pwm_a.stop.side_effect = OSError("stop")
        self.gpio.cleanup.side_effect = OSError("cleanup")
        with self.assertRaises(OSError):
            controller.close()
        controller.pwm_b.stop.assert_called_once_with()
        self.assertEqual(self.gpio.cleanup.call_count, 6)
        self.assertEqual(self.gpio.output.call_count, 6)
        controller.close()

    def test_output_failure_closes_controller_and_preserves_cause(self):
        controller = self.setup()
        self.gpio.output.side_effect = OSError("output")
        with self.assertRaises(MotorHardwareError) as failure:
            controller.forward()
        self.assertIsInstance(failure.exception.__cause__, OSError)
        self.assertEqual(self.gpio.cleanup.call_count, 6)
        with self.assertRaises(ControllerClosedError):
            controller.forward()

    def test_context_closes_even_on_exception(self):
        self.setup()
        with self.assertRaisesRegex(ValueError, "body"):
            with self.controller:
                raise ValueError("body")
        self.assertEqual(self.gpio.cleanup.call_count, 6)


if __name__ == "__main__":
    unittest.main()
