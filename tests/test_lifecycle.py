import unittest
from unittest.mock import Mock, call, patch

from rasptank import MotorController, RaspTank, ServoController, UltrasonicSensor
from rasptank.exceptions import HardwareError
from rasptank.hardware._lifecycle import release
from rasptank.hardware.display import OledDisplay
from rasptank.hardware.leds import LEDStrip


class TestInterruptedLifecycle(unittest.TestCase):
    def test_release_attempts_every_action_and_preserves_first_failure(self):
        for first in (KeyboardInterrupt("first"), SystemExit("first"), OSError("first")):
            with self.subTest(first=type(first)):
                actions = [Mock(side_effect=first),
                           Mock(side_effect=KeyboardInterrupt("later")),
                           Mock(side_effect=SystemExit("last")), Mock()]
                with self.assertRaises(type(first)) as failure:
                    release(actions)
                self.assertIs(failure.exception, first)
                for action in actions:
                    action.assert_called_once_with()

    def test_motor_close_finishes_nested_shutdown_after_interrupt(self):
        for interrupt in (KeyboardInterrupt("duty"), SystemExit("duty")):
            with self.subTest(interrupt=type(interrupt)):
                gpio = Mock(BCM="BCM", OUT="OUT", LOW=0)
                pwm_a, pwm_b = Mock(), Mock()
                gpio.PWM.side_effect = [pwm_a, pwm_b]
                controller = MotorController(gpio=gpio)
                pwm_a.ChangeDutyCycle.side_effect = interrupt
                pwm_a.stop.side_effect = OSError("stop")
                gpio.cleanup.side_effect = KeyboardInterrupt("cleanup")
                with self.assertRaises(type(interrupt)) as failure:
                    controller.close()
                self.assertIs(failure.exception, interrupt)
                pwm_b.ChangeDutyCycle.assert_called_once_with(0)
                pwm_a.stop.assert_called_once_with()
                pwm_b.stop.assert_called_once_with()
                self.assertEqual(gpio.output.call_count, 6)
                self.assertEqual(gpio.cleanup.call_args_list,
                                 [call(pin) for pin in (4, 17, 26, 21, 27, 18)])
                controller.close()
                self.assertEqual(gpio.cleanup.call_count, 6)

    def test_optional_controller_closes_finish_after_interrupt(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            with self.subTest(controller="servo", interrupt=error_type):
                pwm = Mock()
                servo = ServoController(pwm=pwm)
                servo.set_pulse(0, 300)
                servo.set_pulse(2, 300)
                interrupt = error_type("channel")
                pwm.set_pwm.reset_mock()
                pwm.set_pwm.side_effect = [interrupt, None]
                with self.assertRaises(error_type) as failure:
                    servo.close()
                self.assertIs(failure.exception, interrupt)
                self.assertEqual(pwm.set_pwm.call_args_list,
                                 [call(0, 0, 4096), call(2, 0, 4096)])
                servo.close()
                self.assertEqual(pwm.set_pwm.call_count, 2)
            with self.subTest(controller="sensor", interrupt=error_type):
                gpio = Mock(BCM="BCM", OUT="OUT", IN="IN", LOW=0)
                sensor = UltrasonicSensor(gpio=gpio)
                interrupt = error_type("trigger")
                gpio.output.side_effect = interrupt
                with self.assertRaises(error_type) as failure:
                    sensor.close()
                self.assertIs(failure.exception, interrupt)
                self.assertEqual(gpio.cleanup.call_args_list, [call(11), call(8)])
                sensor.close()
                self.assertEqual(gpio.cleanup.call_count, 2)
            with self.subTest(controller="led", interrupt=error_type):
                strip = Mock()
                strip.numPixels.return_value = 16
                leds = LEDStrip(strip=strip, color=Mock())
                interrupt = error_type("off")
                strip.show.side_effect = interrupt
                with self.assertRaises(error_type) as failure:
                    leds.close()
                self.assertIs(failure.exception, interrupt)
                leds.close()
                strip._cleanup.assert_called_once_with()

    def test_facade_close_finishes_after_interrupt(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            with self.subTest(interrupt=error_type):
                motors, servos, sensor = Mock(), Mock(), Mock()
                with patch("rasptank.tank.MotorController", return_value=motors), \
                        patch("rasptank.tank.ServoController", return_value=servos), \
                        patch("rasptank.tank.UltrasonicSensor", return_value=sensor):
                    tank = RaspTank(enable_servos=True, enable_ultrasonic=True)
                interrupt = error_type("motors")
                motors.close.side_effect = interrupt
                servos.close.side_effect = OSError("servos")
                with self.assertRaises(error_type) as failure:
                    tank.close()
                self.assertIs(failure.exception, interrupt)
                tank.close()
                for device in (motors, servos, sensor):
                    device.close.assert_called_once_with()

    def test_every_controller_initialization_rolls_back_interrupts(self):
        for module, factory in (
                ("motors", MotorController), ("servos", ServoController),
                ("ultrasonic", UltrasonicSensor), ("leds", LEDStrip),
                ("display", OledDisplay)):
            for error_type in (KeyboardInterrupt, SystemExit):
                with self.subTest(module=module, interrupt=error_type):
                    interrupt = error_type("initialize")
                    original_close = factory.close
                    with patch("rasptank.hardware." + module + ".import_module",
                               side_effect=interrupt), \
                            patch.object(factory, "close", autospec=True,
                                         side_effect=original_close) as close:
                        with self.assertRaises(error_type) as failure:
                            factory()
                    self.assertIs(failure.exception, interrupt)
                    close.assert_called_once()
                    self.assertTrue(close.call_args.args[0]._closed)

    def test_partial_gpio_constructor_rollback_preserves_original_failure(self):
        for factory, pins in ((MotorController, (4, 17)), (UltrasonicSensor, (11,))):
            for first in (KeyboardInterrupt("setup"), SystemExit("setup"), OSError("setup")):
                with self.subTest(factory=factory, failure=type(first)):
                    gpio = Mock(BCM="BCM", OUT="OUT", IN="IN", LOW=0)
                    gpio.setup.side_effect = [None] * len(pins) + [first]
                    gpio.output.side_effect = KeyboardInterrupt("output")
                    gpio.cleanup.side_effect = SystemExit("cleanup")
                    expected = type(first) if not isinstance(first, Exception) else HardwareError
                    with self.assertRaises(expected) as failure:
                        factory(gpio=gpio)
                    if isinstance(first, Exception):
                        self.assertIs(failure.exception.__cause__, first)
                    else:
                        self.assertIs(failure.exception, first)
                    self.assertEqual(gpio.cleanup.call_args_list, [call(pin) for pin in pins])

    def test_partial_led_constructor_rolls_back_despite_second_interrupt(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            with self.subTest(interrupt=error_type):
                strip = Mock()
                strip.numPixels.return_value = 16
                interrupt = error_type("begin")
                strip.begin.side_effect = interrupt
                strip.show.side_effect = SystemExit("off")
                strip._cleanup.side_effect = KeyboardInterrupt("cleanup")
                with self.assertRaises(error_type) as failure:
                    LEDStrip(strip=strip, color=Mock())
                self.assertIs(failure.exception, interrupt)
                strip._cleanup.assert_called_once_with()

    def test_motor_constructor_releases_pwm_after_interrupted_start(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            with self.subTest(interrupt=error_type):
                gpio = Mock(BCM="BCM", OUT="OUT", LOW=0)
                pwm = gpio.PWM.return_value
                interrupt = error_type("start")
                pwm.start.side_effect = interrupt
                pwm.ChangeDutyCycle.side_effect = SystemExit("duty")
                pwm.stop.side_effect = KeyboardInterrupt("stop")
                with self.assertRaises(error_type) as failure:
                    MotorController(gpio=gpio)
                self.assertIs(failure.exception, interrupt)
                pwm.stop.assert_called_once_with()
                self.assertEqual(gpio.cleanup.call_count, 6)

    def test_servo_frequency_interrupt_closes_without_commanding_channels(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            with self.subTest(interrupt=error_type):
                pwm = Mock()
                interrupt = error_type("frequency")
                pwm.set_pwm_freq.side_effect = interrupt
                with patch.object(ServoController, "close", autospec=True,
                                  side_effect=ServoController.close) as close:
                    with self.assertRaises(error_type) as failure:
                        ServoController(pwm=pwm)
                self.assertIs(failure.exception, interrupt)
                close.assert_called_once()
                self.assertTrue(close.call_args.args[0]._closed)
                pwm.set_pwm.assert_not_called()

    def test_output_failure_is_not_replaced_by_cleanup_interrupt(self):
        gpio = Mock(BCM="BCM", OUT="OUT", LOW=0, HIGH=1)
        controller = MotorController(gpio=gpio)
        first = OSError("output")
        gpio.output.side_effect = first
        gpio.cleanup.side_effect = KeyboardInterrupt("cleanup")
        with self.assertRaises(HardwareError) as failure:
            controller.forward()
        self.assertIs(failure.exception.__cause__, first)
        self.assertEqual(gpio.cleanup.call_count, 6)

    def test_oled_constructor_releases_acquired_serial_or_device_on_interrupt(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            for failed_stage in ("device", "render"):
                with self.subTest(interrupt=error_type, stage=failed_stage):
                    serial, oled = Mock(), Mock()
                    interrupt = error_type(failed_stage)
                    if failed_stage == "device":
                        oled.ssd1306.side_effect = interrupt
                        imports = [serial, oled]
                        resource = serial.i2c.return_value
                    else:
                        imports = [serial, oled, interrupt]
                        resource = oled.ssd1306.return_value
                    resource.cleanup.side_effect = KeyboardInterrupt("cleanup")
                    with patch("rasptank.hardware.display.import_module", side_effect=imports):
                        with self.assertRaises(error_type) as failure:
                            OledDisplay()
                    self.assertIs(failure.exception, interrupt)
                    resource.cleanup.assert_called_once_with()

    def test_facade_constructor_rollback_preserves_original_failure(self):
        for first in (KeyboardInterrupt("sensor"), SystemExit("sensor"), OSError("sensor")):
            with self.subTest(failure=type(first)):
                motors, servos, borrowed = Mock(), Mock(), Mock()
                motors.close.side_effect = KeyboardInterrupt("close")
                servos.close.side_effect = SystemExit("close")
                with patch("rasptank.tank.MotorController", return_value=motors), \
                        patch("rasptank.tank.ServoController", return_value=servos), \
                        patch("rasptank.tank.UltrasonicSensor", side_effect=first):
                    with self.assertRaises(type(first)) as failure:
                        RaspTank(enable_servos=True, enable_ultrasonic=True, leds=borrowed)
                self.assertIs(failure.exception, first)
                motors.close.assert_called_once_with()
                servos.close.assert_called_once_with()
                borrowed.close.assert_not_called()

    def test_context_preserves_original_body_failure_after_cleanup_interrupt(self):
        for first in (KeyboardInterrupt("body"), SystemExit("body"), ValueError("body")):
            with self.subTest(failure=type(first)):
                with patch("rasptank.tank.MotorController") as factory:
                    factory.return_value.close.side_effect = KeyboardInterrupt("cleanup")
                    with self.assertRaises(type(first)) as failure:
                        with RaspTank():
                            raise first
                self.assertIs(failure.exception, first)
                factory.return_value.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
