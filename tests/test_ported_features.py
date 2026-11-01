import unittest
from unittest.mock import Mock, call

from rasptank import (KalmanFilter, LEDStrip, LineSensors, Mpu6050, PID, RaspTank,
                      SwitchPorts, behaviors, system)
from rasptank.exceptions import (ControllerClosedError, HardwareDisabledError,
                                 SensorTimeoutError)
from rasptank.hardware.servos import ServoController


class TestSwitches(unittest.TestCase):
    def test_ports_pins_and_cleanup(self):
        gpio = Mock(HIGH=1, LOW=0, OUT="out")
        sw = SwitchPorts(gpio=gpio)
        self.assertEqual([c.args[0] for c in gpio.setup.call_args_list], [5, 6, 13])
        sw.set(2, True)
        gpio.output.assert_called_with(6, 1)
        sw.head_light(True)
        gpio.output.assert_called_with(5, 1)
        sw.front_light(False)
        self.assertEqual(gpio.output.call_args_list[-2:], [call(6, 0), call(13, 0)])
        sw.close()
        self.assertEqual(sw.state, {1: False, 2: False, 3: False})
        gpio.cleanup.assert_any_call(13)
        with self.assertRaises(ControllerClosedError):
            sw.set(1, True)

    def test_validation_and_dry_run(self):
        sw = SwitchPorts(use_gpio=False)
        for port in (0, 4, True, "1"):
            with self.assertRaises(ValueError):
                sw.set(port, True)
        with self.assertRaises(ValueError):
            sw.set(1, 1)
        sw.set(3, True)
        self.assertTrue(sw.state[3])
        sw.all_off()
        self.assertFalse(sw.state[3])


class TestLineSensors(unittest.TestCase):
    def test_read_order_left_middle_right(self):
        gpio = Mock(IN="in")
        gpio.input.side_effect = lambda pin: {20: 1, 16: 0, 19: 1}[pin]
        sensors = LineSensors(gpio=gpio)
        self.assertEqual(sensors.read(), (1, 0, 1))
        sensors.close()
        self.assertEqual(gpio.cleanup.call_count, 3)

    def test_dry_run_raises(self):
        with self.assertRaises(HardwareDisabledError):
            LineSensors(use_gpio=False).read()


class TestImu(unittest.TestCase):
    def test_average_and_dry_run(self):
        sensor = Mock()
        sensor.get_accel_data.side_effect = [{"x": 1, "y": 2, "z": 3}, {"x": 3, "y": 4, "z": 5}]
        imu = Mpu6050(sensor=sensor)
        self.assertEqual(imu.average_accel(2), {"x": 2.0, "y": 3.0, "z": 4.0})
        with self.assertRaises(ValueError):
            imu.average_accel(0)
        with self.assertRaises(HardwareDisabledError):
            Mpu6050(use_gpio=False).accel()


class TestLedEffects(unittest.TestCase):
    def setUp(self):
        self.strip = Mock()
        self.strip.numPixels.return_value = 16
        self.led = LEDStrip(strip=self.strip, color=lambda r, g, b: (r, g, b))

    def test_set_pixels_validates(self):
        self.led.set_pixels([0, 3], 1, 2, 3)
        self.strip.setPixelColor.assert_any_call(3, (1, 2, 3))
        self.assertEqual(self.strip.show.call_count, 1)
        for ids in ([16], [-1], [True]):
            with self.assertRaises(ValueError):
                self.led.set_pixels(ids, 0, 0, 0)

    def test_police_uses_13_pixels_and_ends_dark(self):
        sleeps = []
        self.led.police(1, sleep=sleeps.append)
        lit = {c.args[0] for c in self.strip.setPixelColor.call_args_list
               if c.args[1] != (0, 0, 0)}
        self.assertEqual(lit, set(range(13)))
        self.assertEqual(len(sleeps), 14)
        self.assertEqual(self.strip.setPixelColor.call_args_list[-1].args[1], (0, 0, 0))

    def test_breath_and_bad_args(self):
        self.led.breath(100, 0, 50, 1, steps=2, sleep=lambda s: None)
        frames = list(LEDStrip.breath_frames(100, 0, 50, 2))
        self.assertEqual([f[0] for f in frames],
                         [(0, 0, 0), (50, 0, 25), (100, 0, 50), (50, 0, 25)])
        with self.assertRaises(ValueError):
            self.led.breath(256, 0, 0)
        with self.assertRaises(ValueError):
            self.led.police(0)


class TestFilters(unittest.TestCase):
    def test_kalman_converges_and_matches_first_step(self):
        k = KalmanFilter(0.01, 0.1)
        first = k.update(10)
        self.assertAlmostEqual(first, 1.01 / 1.11 * 10)
        for _ in range(200):
            value = k.update(10)
        self.assertAlmostEqual(value, 10, places=3)

    def test_pid(self):
        times = iter([0, 1, 2])
        pid = PID(1, 0.5, 2, clock=lambda: next(times))
        self.assertEqual(pid.update(2), 2 + 0.5 * 2 + 2 * 2)
        self.assertEqual(pid.update(2), 2 + 0.5 * 4)


class TestSystem(unittest.TestCase):
    def test_cpu_temperature(self):
        import tempfile
        with tempfile.NamedTemporaryFile("w") as f:
            f.write("45678\n")
            f.flush()
            self.assertEqual(system.cpu_temperature(f.name), 45.7)


class TestBehaviors(unittest.TestCase):
    def test_line_follow_table(self):
        motors = Mock()
        mark = behaviors.line_follow_step(motors, (0, 1, 0), 0, sleep=lambda s: None)
        self.assertEqual(mark, 1)
        self.assertEqual(motors.mock_calls, [call.motor_left(1, 1, 80), call.motor_right(1, 0, 80)])
        motors.reset_mock()
        sleeps = []
        mark = behaviors.line_follow_step(motors, (1, 0, 0), 1, sleep=sleeps.append)
        self.assertEqual(mark, 3)
        self.assertEqual(motors.mock_calls, [
            call.motor_left(1, 0, 80), call.motor_right(1, 1, 80),
            call.motor_left(1, 1, 0), call.motor_right(1, 0, 100)])
        self.assertEqual(sleeps, [0.03, 0.02, 0.1])

    def test_line_no_kick_when_mark_unchanged_and_lost_line(self):
        motors = Mock()
        behaviors.line_follow_step(motors, (0, 0, 1), 5, sleep=lambda s: None)
        self.assertEqual(motors.mock_calls, [call.motor_left(1, 1, 100), call.motor_right(1, 0, 0)])
        motors.reset_mock()
        self.assertEqual(behaviors.line_follow_step(motors, (0, 0, 0), 2, sleep=lambda s: None), 2)
        self.assertEqual(motors.mock_calls, [call.motor_left(1, 1, 0), call.motor_right(1, 0, 100)])
        with self.assertRaises(ValueError):
            behaviors.line_follow_step(motors, (1, 1), 0)

    def test_simple_line(self):
        motors = Mock()
        behaviors.simple_line_step(motors, (1, 0, 0))
        motors.move.assert_called_once_with(100, "forward", "right", 0.6)

    def test_obstacle_thresholds_and_failure_stops(self):
        motors, sonar = Mock(), Mock()
        for dist, expected in ((0.2, "backward"), (0.5, "left"), (0.9, "forward")):
            sonar.distance.return_value = dist
            self.assertEqual(behaviors.obstacle_step(motors, sonar, sleep=lambda s: None), expected)
        sonar.distance.side_effect = SensorTimeoutError("x")
        motors.reset_mock()
        with self.assertRaises(SensorTimeoutError):
            behaviors.obstacle_step(motors, sonar, sleep=lambda s: None)
        motors.stop.assert_called_once_with()

    def test_radar_scan_sweeps_and_recentres(self):
        pwm = Mock()
        servos = ServoController(pwm=pwm)
        sonar = Mock()
        readings = iter([0.5, SensorTimeoutError("none"), 0.4])
        def distance():
            value = next(readings, 0.4)
            if isinstance(value, Exception):
                raise value
            return value
        sonar.distance.side_effect = distance
        result = behaviors.radar_scan(servos, sonar, step=100, sleep=lambda s: None)
        self.assertEqual(pwm.set_pwm.call_args_list[0], call(1, 0, 480))
        self.assertEqual(pwm.set_pwm.call_args_list[-1], call(1, 0, 300))
        self.assertEqual(result[0], [0.5, 180 - (380 - 100) / 2.55])
        self.assertEqual(len(result), 2)
        self.assertEqual(sonar.distance.call_count, 3)
        with self.assertRaises(ValueError):
            behaviors.radar_scan(servos, sonar, step=0)

    def test_steady_stabilizer_clamps(self):
        servos, imu = Mock(), Mock()
        imu.accel.return_value = {"x": 1000.0}
        stab = behaviors.SteadyStabilizer(servos, imu, 300, kalman=Mock(update=lambda v: v))
        stab.step()
        servos.set_pulse.assert_called_once_with(4, 500)
        imu.accel.return_value = {"x": 1.0}
        servos.reset_mock()
        behaviors.SteadyStabilizer(servos, imu, 300, kalman=Mock(update=lambda v: v)).step()
        servos.set_pulse.assert_called_once_with(4, 323)


class TestFacadeNewDevices(unittest.TestCase):
    def test_opt_in_owned_and_injected(self):
        with RaspTank(use_gpio=False, enable_switches=True, enable_line_sensors=True,
                      enable_imu=True) as tank:
            self.assertIsInstance(tank.switches, SwitchPorts)
            self.assertIsNotNone(tank.line_sensors)
            self.assertIsNotNone(tank.imu)
        self.assertTrue(tank.switches._closed)
        injected = Mock()
        tank = RaspTank(use_gpio=False, switches=injected)
        tank.close()
        injected.close.assert_not_called()
        self.assertIsNone(RaspTank(use_gpio=False).imu)


if __name__ == "__main__":
    unittest.main()
