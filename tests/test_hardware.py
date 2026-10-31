import ast
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock, call, patch

from rasptank import MotorController, RaspTank, ServoController, UltrasonicSensor
from rasptank.config import SERVO_DIRECTIONS, SERVO_RANGES
from rasptank.exceptions import (
    ControllerClosedError, HardwareDisabledError, HardwareError,
    SensorTimeoutError, ServoHardwareError,
)
from rasptank.hardware.display import OledDisplay
from rasptank.hardware.leds import LEDStrip


class TestServoController(unittest.TestCase):
    def setUp(self):
        self.pwm = Mock()
        self.servo = ServoController(pwm=self.pwm)
        self.addCleanup(self.servo.close)

    def test_constructor_only_sets_legacy_frequency(self):
        self.assertEqual(self.pwm.mock_calls, [call.set_pwm_freq(50)])
        self.assertEqual(self.servo.positions, dict.fromkeys(range(5), 300))
        self.assertEqual(SERVO_RANGES, ((150, 450), (160, 480), (100, 500),
                                      (100, 500), (100, 500)))
        self.assertEqual(SERVO_DIRECTIONS, (1, 0, 1, 1, 0))

    def test_lazy_default_address_and_bus(self):
        module = Mock()
        with patch("rasptank.hardware.servos.import_module", return_value=module) as load:
            controller = ServoController()
            load.assert_called_once_with("Adafruit_PCA9685")
            module.PCA9685.assert_called_once_with(0x40, busnum=1)
            controller.close()
            module.PCA9685.return_value.set_pwm.assert_not_called()

    def test_all_channel_ranges_include_boundaries(self):
        for channel, (low, high) in enumerate(SERVO_RANGES):
            self.assertEqual(self.servo.set_pulse(channel, low), low)
            self.pwm.set_pwm.assert_called_with(channel, 0, low)
            self.assertEqual(self.servo.set_pulse(channel, high), high)
            for invalid in (low - 1, high + 1, 300.0, True, "300", None):
                self.pwm.set_pwm.reset_mock()
                with self.subTest(channel=channel, invalid=invalid), self.assertRaises(ValueError):
                    self.servo.set_pulse(channel, invalid)
                self.pwm.set_pwm.assert_not_called()

    def test_channel_validation(self):
        for channel in (-1, 5, 1.0, True, "0"):
            with self.subTest(channel=channel), self.assertRaises(ValueError):
                self.servo.set_pulse(channel, 300)
        self.pwm.set_pwm.assert_not_called()

    def test_relative_directions_and_signed_steps(self):
        for channel, direction in enumerate(SERVO_DIRECTIONS):
            self.assertEqual(self.servo.move_relative(channel, 10), 310 if direction else 290)
            self.assertEqual(self.servo.move_relative(channel, -10), 300)

    def test_relative_rejects_overrun_without_state_change(self):
        with self.assertRaises(ValueError):
            self.servo.move_relative(0, 151)
        with self.assertRaises(ValueError):
            self.servo.move_relative(0, 1.5)
        self.assertEqual(self.servo.positions[0], 300)
        self.pwm.set_pwm.assert_not_called()

    def test_calibration_is_transient_and_no_implicit_motion(self):
        self.servo.calibrate(0, 320)
        self.assertEqual(self.servo.initial[0], 320)
        self.assertEqual(self.servo.positions[0], 300)
        self.pwm.set_pwm.assert_not_called()
        self.servo.center(0)
        self.pwm.set_pwm.assert_called_once_with(0, 0, 320)
        other = ServoController(use_gpio=False)
        self.assertEqual(other.initial[0], 300)
        other.close()

    def test_disabled_mode_updates_only_symbolic_state(self):
        pwm = Mock()
        with ServoController(use_gpio=False, pwm=pwm) as servo:
            servo.set_pulse(1, 310)
            servo.move_relative(1, 10)
            servo.calibrate(1, 330)
            servo.center(1)
            self.assertEqual(servo.positions[1], 330)
        self.assertEqual(pwm.mock_calls, [])

    def test_close_releases_only_used_channels_idempotently(self):
        self.servo.set_pulse(1, 310)
        self.servo.set_pulse(4, 320)
        self.pwm.reset_mock()
        self.servo.close()
        self.servo.close()
        self.assertEqual(self.pwm.mock_calls, [call.set_pwm(1, 0, 4096), call.set_pwm(4, 0, 4096)])
        for operation in (lambda: self.servo.set_pulse(0, 300),
                          lambda: self.servo.move_relative(0, 5),
                          lambda: self.servo.calibrate(0, 300), lambda: self.servo.center(0)):
            with self.assertRaises(ControllerClosedError):
                operation()

    def test_close_attempts_all_channels_on_failure(self):
        self.servo.set_pulse(0, 300)
        self.servo.set_pulse(2, 300)
        self.pwm.set_pwm.reset_mock()
        self.pwm.set_pwm.side_effect = OSError("release")
        with self.assertRaises(OSError):
            self.servo.close()
        self.assertEqual(self.pwm.set_pwm.call_count, 2)

    def test_write_failure_releases_attempted_channel_and_closes(self):
        self.pwm.set_pwm.side_effect = OSError("write")
        with self.assertRaises(ServoHardwareError):
            self.servo.set_pulse(0, 310)
        self.assertEqual(self.pwm.set_pwm.call_args_list, [call(0, 0, 310), call(0, 0, 4096)])
        self.assertEqual(self.servo.positions[0], 300)

    def test_frequency_failure_does_not_release_unowned_channels(self):
        pwm = Mock()
        pwm.set_pwm_freq.side_effect = OSError("frequency")
        with self.assertRaises(ServoHardwareError):
            ServoController(pwm=pwm)
        pwm.set_pwm.assert_not_called()


class TestUltrasonic(unittest.TestCase):
    def sensor(self, inputs=(), times=()):
        gpio = Mock(BCM="BCM", OUT="OUT", IN="IN", LOW=0, HIGH=1)
        gpio.input.side_effect = inputs
        clock = Mock(side_effect=times)
        sleep = Mock()
        sensor = UltrasonicSensor(gpio=gpio, clock=clock, sleep=sleep, timeout=.03)
        self.addCleanup(sensor.close)
        return sensor, gpio, sleep

    def test_setup_exact_pins(self):
        sensor, gpio, _ = self.sensor()
        self.assertEqual((sensor.trigger_pin, sensor.echo_pin), (11, 8))
        self.assertEqual(gpio.setup.call_args_list, [call(11, "OUT", initial=0), call(8, "IN")])
        sensor.setup()
        self.assertEqual(gpio.setup.call_count, 2)

    def test_distance_metres_and_round_precision(self):
        sensor, gpio, sleep = self.sensor([0, 1, 1, 0], [0, .001, .002, .003, .005])
        self.assertEqual(sensor.distance(), .51)
        sleep.assert_called_once_with(.000015)
        self.assertEqual(gpio.output.call_args_list, [call(11, 1), call(11, 0)])

    def test_rising_echo_timeout_at_deadline(self):
        sensor, gpio, _ = self.sensor([0, 0], [0, .01, .03])
        with self.assertRaises(SensorTimeoutError):
            sensor.distance()
        gpio.output.assert_called_with(11, 0)

    def test_falling_echo_timeout_at_deadline(self):
        sensor, _, _ = self.sensor([1, 1, 1], [0, .001, .02, .031])
        with self.assertRaises(SensorTimeoutError):
            sensor.distance()

    def test_edge_arriving_exactly_at_deadline_is_timeout(self):
        sensor, _, _ = self.sensor([1], [0, .03])
        with self.assertRaises(SensorTimeoutError):
            sensor.distance()

    def test_falling_edge_exactly_at_deadline_is_timeout(self):
        sensor, _, _ = self.sensor([1, 0], [0, .001, .031])
        with self.assertRaises(SensorTimeoutError):
            sensor.distance()

    def test_immediate_edges_return_zero(self):
        sensor, _, _ = self.sensor([1, 0], [0, .001, .001])
        self.assertEqual(sensor.distance(), 0)

    def test_timeout_just_before_deadline_succeeds(self):
        sensor, _, _ = self.sensor([1, 0], [0, .001, .030999])
        self.assertEqual(sensor.distance(), 5.1)

    def test_disabled_sensor_never_fabricates_reading(self):
        gpio = Mock()
        with UltrasonicSensor(use_gpio=False, gpio=gpio) as sensor:
            with self.assertRaises(HardwareDisabledError):
                sensor.distance()
        self.assertEqual(gpio.mock_calls, [])

    def test_invalid_timeout_or_pins_never_touches_gpio(self):
        gpio = Mock()
        for timeout in (0, -1, float("inf"), float("nan"), True, "1"):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                UltrasonicSensor(gpio=gpio, timeout=timeout)
        for pins in ((8, 8), (-1, 8), (11, 28), (True, 8)):
            with self.assertRaises(ValueError):
                UltrasonicSensor(*pins, gpio=gpio)
        self.assertEqual(gpio.mock_calls, [])

    def test_cleanup_scoped_idempotent_and_closed_guard(self):
        sensor, gpio, _ = self.sensor()
        sensor.close()
        sensor.close()
        self.assertEqual(gpio.cleanup.call_args_list, [call(11), call(8)])
        with self.assertRaises(ControllerClosedError):
            sensor.distance()
        with self.assertRaises(ControllerClosedError):
            sensor.setup()

    def test_partial_initialization_rolls_back_trigger_only(self):
        gpio = Mock(BCM="BCM", OUT="OUT", IN="IN", LOW=0)
        gpio.setup.side_effect = [None, OSError("echo")]
        with self.assertRaises(HardwareError):
            UltrasonicSensor(gpio=gpio)
        gpio.cleanup.assert_called_once_with(11)

    def test_cleanup_continues_when_output_and_cleanup_fail(self):
        sensor, gpio, _ = self.sensor()
        gpio.output.side_effect = OSError("output")
        gpio.cleanup.side_effect = OSError("cleanup")
        with self.assertRaises(OSError):
            sensor.close()
        self.assertEqual(gpio.cleanup.call_args_list, [call(11), call(8)])

    def test_pulse_failure_closes_and_releases(self):
        sensor, gpio, sleep = self.sensor()
        sleep.side_effect = OSError("sleep")
        with self.assertRaises(HardwareError):
            sensor.distance()
        self.assertEqual(gpio.cleanup.call_args_list, [call(11), call(8)])
        with self.assertRaises(ControllerClosedError):
            sensor.distance()

    def test_trigger_reset_failure_closes_and_releases(self):
        sensor, gpio, _ = self.sensor()
        gpio.output.side_effect = [None, OSError("reset"), None]
        with self.assertRaises(HardwareError):
            sensor.distance()
        self.assertEqual(gpio.cleanup.call_args_list, [call(11), call(8)])

    def test_input_failure_closes_and_releases(self):
        sensor, gpio, _ = self.sensor(times=[0])
        gpio.input.side_effect = OSError("input")
        with self.assertRaises(HardwareError):
            sensor.distance()
        self.assertEqual(gpio.cleanup.call_args_list, [call(11), call(8)])


class TestOptionalDevices(unittest.TestCase):
    def test_led_one_synchronous_frame_and_release(self):
        strip = Mock()
        strip.numPixels.return_value = 16
        color = Mock(return_value=123)
        led = LEDStrip(strip=strip, color=color)
        self.addCleanup(led.close)
        led.color_wipe(1, 2, 3)
        color.assert_called_once_with(1, 2, 3)
        self.assertEqual(strip.setPixelColor.call_args_list, [call(i, 123) for i in range(16)])
        strip.show.assert_called_once_with()
        led.close()
        led.close()
        strip._cleanup.assert_called_once_with()
        with self.assertRaises(ControllerClosedError):
            led.off()

    def test_led_default_legacy_constructor(self):
        module = Mock()
        module.Adafruit_NeoPixel.return_value.numPixels.return_value = 16
        with patch("rasptank.hardware.leds.import_module", return_value=module):
            led = LEDStrip()
            module.Adafruit_NeoPixel.assert_called_once_with(16, 12, 800000, 10, False, 255, 0)
            led.close()

    def test_invalid_led_color_does_not_output(self):
        strip = Mock()
        led = LEDStrip(strip=strip, color=Mock())
        strip.numPixels.return_value = 16
        for rgb in ((-1, 0, 0), (0, 256, 0), (True, 0, 0), (1.0, 0, 0)):
            with self.assertRaises(ValueError):
                led.color_wipe(*rgb)
        strip.show.assert_not_called()
        led.close()

    def test_led_failed_begin_attempts_native_release(self):
        strip = Mock()
        strip.begin.side_effect = OSError("begin")
        strip.numPixels.return_value = 16
        with self.assertRaises(HardwareError):
            LEDStrip(strip=strip, color=Mock())
        strip._cleanup.assert_called_once_with()

    def test_led_release_attempted_when_off_fails(self):
        strip = Mock()
        strip.numPixels.return_value = 16
        led = LEDStrip(strip=strip, color=Mock())
        strip.show.side_effect = OSError("off")
        with self.assertRaises(OSError):
            led.close()
        strip._cleanup.assert_called_once_with()

    def test_oled_two_line_synchronous_render_and_native_cleanup(self):
        from unittest.mock import MagicMock
        device = Mock()
        canvas = MagicMock()
        draw = canvas.return_value.__enter__.return_value
        display = OledDisplay(device=device, canvas=canvas)
        display.show("one", "two")
        canvas.assert_called_once_with(device)
        self.assertEqual(draw.text.call_args_list,
                         [call((0, 0), "one", fill="white"), call((0, 10), "two", fill="white")])
        display.close()
        display.close()
        device.cleanup.assert_called_once_with()
        with self.assertRaises(ControllerClosedError):
            display.show()

    def test_oled_failed_device_initialization_releases_serial(self):
        serial, oled = Mock(), Mock()
        oled.ssd1306.side_effect = OSError("device")
        with patch("rasptank.hardware.display.import_module", side_effect=[serial, oled]):
            with self.assertRaises(HardwareError):
                OledDisplay()
        serial.i2c.assert_called_once_with(port=1, address=0x3C)
        serial.i2c.return_value.cleanup.assert_called_once_with()

    def test_oled_missing_render_dependency_releases_device(self):
        serial, oled = Mock(), Mock()
        with patch("rasptank.hardware.display.import_module",
                   side_effect=[serial, oled, ImportError("render")]):
            with self.assertRaises(HardwareError):
                OledDisplay()
        oled.ssd1306.return_value.cleanup.assert_called_once_with()

    def test_disabled_optional_devices_do_not_touch_injections(self):
        strip, device = Mock(), Mock()
        with LEDStrip(use_gpio=False, strip=strip) as led:
            led.color_wipe(10, 20, 30)
            self.assertEqual(led.color, (10, 20, 30))
        with OledDisplay(use_gpio=False, device=device) as display:
            display.show("dry", "run")
            self.assertEqual(display.lines, ("dry", "run"))
        self.assertEqual(strip.mock_calls, [])
        self.assertEqual(device.mock_calls, [])


class TestRaspTank(unittest.TestCase):
    def test_default_wiring_initializes_only_motors(self):
        with patch("rasptank.tank.MotorController") as motors, \
                patch("rasptank.tank.ServoController") as servos, \
                patch("rasptank.tank.UltrasonicSensor") as sensor, \
                patch("rasptank.tank.LEDStrip") as leds, \
                patch("rasptank.tank.OledDisplay") as display:
            with RaspTank() as tank:
                motors.assert_called_once_with(use_gpio=True)
                self.assertIs(tank.motors, motors.return_value)
                self.assertIsNone(tank.servos)
                for factory in (servos, sensor, leds, display):
                    factory.assert_not_called()
            motors.return_value.close.assert_called_once_with()

    def test_borrowed_devices_are_not_closed_and_motor_delegates(self):
        motors, servos = Mock(), Mock()
        tank = RaspTank(motors=motors, servos=servos)
        tank.forward(10)
        tank.backward(20)
        tank.turn_left(30)
        tank.turn_right(40)
        tank.move(50, "backward", "left", .5)
        tank.stop()
        self.assertEqual(motors.mock_calls, [call.forward(10), call.backward(20),
                         call.turn_left(30), call.turn_right(40),
                         call.move(50, "backward", "left", .5), call.stop()])
        tank.close()
        motors.close.assert_not_called()
        servos.close.assert_not_called()

    def test_all_owned_optional_devices_disabled_and_no_center(self):
        with RaspTank(use_gpio=False, enable_servos=True, enable_ultrasonic=True,
                      enable_leds=True, enable_display=True) as tank:
            tank.forward()
            tank.servos.set_pulse(0, 310)
            tank.leds.off()
            tank.display.show("dry", "run")
            self.assertEqual(tank.servos.positions[1], 300)
            self.assertIsNone(tank.servos.pwm)
            with self.assertRaises(HardwareDisabledError):
                tank.ultrasonic.distance()
        for device in (tank.motors, tank.servos, tank.ultrasonic, tank.leds, tank.display):
            self.assertTrue(device._closed)

    def test_partial_initialization_rolls_back_owned_resources(self):
        motors, servos = Mock(), Mock()
        with patch("rasptank.tank.MotorController", return_value=motors), \
                patch("rasptank.tank.ServoController", return_value=servos), \
                patch("rasptank.tank.UltrasonicSensor", side_effect=OSError("sensor")):
            motors.close.side_effect = OSError("close")
            with self.assertRaisesRegex(OSError, "sensor"):
                RaspTank(enable_servos=True, enable_ultrasonic=True)
        motors.close.assert_called_once_with()
        servos.close.assert_called_once_with()

    def test_rollback_does_not_close_borrowed_motor(self):
        motors = Mock()
        with patch("rasptank.tank.ServoController", side_effect=OSError("servo")):
            with self.assertRaises(OSError):
                RaspTank(motors=motors, enable_servos=True)
        motors.close.assert_not_called()

    def test_close_attempts_every_owned_resource_and_is_idempotent(self):
        motors, servos = Mock(), Mock()
        with patch("rasptank.tank.MotorController", return_value=motors), \
                patch("rasptank.tank.ServoController", return_value=servos):
            tank = RaspTank(enable_servos=True)
        motors.close.side_effect = OSError("close")
        with self.assertRaises(OSError):
            tank.close()
        tank.close()
        servos.close.assert_called_once_with()
        motors.close.assert_called_once_with()
        for method in ("forward", "backward", "turn_left", "turn_right", "move", "stop"):
            with self.subTest(method=method), self.assertRaises(ControllerClosedError):
                getattr(tank, method)()

    def test_context_preserves_body_error_after_failed_cleanup(self):
        with patch("rasptank.tank.MotorController") as factory:
            factory.return_value.close.side_effect = OSError("cleanup")
            with self.assertRaisesRegex(ValueError, "body"):
                with RaspTank():
                    raise ValueError("body")
        factory.return_value.close.assert_called_once_with()


class TestHardwareIndependentValidation(unittest.TestCase):
    def test_missing_hardware_dependencies_report_initialization_errors(self):
        for module, factory in (
                ("rasptank.hardware.motors", MotorController),
                ("rasptank.hardware.servos", ServoController),
                ("rasptank.hardware.ultrasonic", UltrasonicSensor),
                ("rasptank.hardware.leds", LEDStrip),
                ("rasptank.hardware.display", OledDisplay)):
            with self.subTest(module=module), \
                    patch(module + ".import_module", side_effect=ImportError("missing")), \
                    self.assertRaises(HardwareError):
                factory()

    def test_fresh_process_import_and_disabled_operation_without_hardware_modules(self):
        script = """
import sys
class BlockHardware:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'RPi', 'Adafruit_PCA9685', 'rpi_ws281x', 'luma'}:
            raise AssertionError('Hardware import attempted: ' + fullname)
sys.meta_path.insert(0, BlockHardware())
from rasptank import RaspTank
from rasptank.hardware import motors, servos, ultrasonic
from rasptank.hardware import leds, display
with RaspTank(use_gpio=False, enable_servos=True, enable_ultrasonic=True,
              enable_leds=True, enable_display=True) as tank:
    tank.forward()
    tank.backward()
    tank.turn_left()
    tank.turn_right()
    tank.stop()
    tank.servos.set_pulse(0, 300)
    tank.leds.off()
    tank.display.show()
"""
        result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_notebooks_are_valid_json_and_python_without_executing_hardware_cells(self):
        root = Path(__file__).resolve().parents[1]
        for path in sorted((root / "notebooks").glob("*.ipynb")):
            notebook = json.loads(path.read_text())
            self.assertEqual(notebook["nbformat"], 4)
            for index, cell in enumerate(notebook["cells"]):
                if cell["cell_type"] == "code":
                    with self.subTest(path=path.name, cell=index):
                        ast.parse("".join(cell["source"]), filename=f"{path.name}:{index}")


if __name__ == "__main__":
    unittest.main()
