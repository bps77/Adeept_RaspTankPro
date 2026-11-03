"""Named joints and real widget callbacks, with no Pi or browser hardware access."""

import importlib.util
import json
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, call, patch

from rasptank import RaspTank, RobotArm, ServoController
from rasptank.arm import ARM_CHANNELS
from rasptank.config import SERVO_RANGES
from rasptank.exceptions import ControllerClosedError
from rasptank.notebook import TankControls


class TestRobotArm(unittest.TestCase):
    """Check the webServer.py joint layout without changing low-level direction rules."""

    def test_creation_and_facade_share_existing_controller_without_motion(self):
        pwm = Mock()
        with ServoController(pwm=pwm) as servos:
            with RaspTank(use_gpio=False, servos=servos) as tank:
                self.assertIs(tank.arm.servos, servos)
                self.assertEqual(tank.arm.positions, dict.fromkeys(ARM_CHANNELS, 300))
                pwm.set_pwm.assert_not_called()
            self.assertFalse(servos._closed)
            tank.arm.set_pulse("gripper", 310)
            pwm.set_pwm.assert_called_once_with(3, 0, 310)
        with self.assertRaises(ControllerClosedError):
            tank.arm.step("grab")
        with RaspTank(use_gpio=False) as tank:
            self.assertIsNone(tank.arm)

    def test_web_action_channels_and_raw_signs(self):
        expected = {
            "look_left": (0, 310), "look_right": (0, 290),
            "arm_up": (1, 310), "arm_down": (1, 290),
            "hand_up": (2, 310), "hand_down": (2, 290),
            "grab": (3, 290), "loosen": (3, 310),
            "camera_up": (4, 310), "camera_down": (4, 290),
        }
        for action, (channel, pulse) in expected.items():
            with self.subTest(action=action), ServoController(pwm=Mock()) as servos:
                arm = RobotArm(servos)
                self.assertEqual(arm.step(action), pulse)
                servos.pwm.set_pwm.assert_called_once_with(channel, 0, pulse)

    def test_limits_validation_center_and_position_snapshot(self):
        with ServoController(use_gpio=False) as servos:
            arm = RobotArm(servos)
            for joint, channel in ARM_CHANNELS.items():
                self.assertEqual(arm.limits(joint), SERVO_RANGES[channel])
                low, high = arm.limits(joint)
                for pulse in (low, high):
                    self.assertEqual(arm.set_pulse(joint, pulse), pulse)
                for pulse in (low - 1, high + 1, True, 300.0):
                    with self.assertRaises(ValueError):
                        arm.set_pulse(joint, pulse)
            snapshot = arm.positions
            snapshot["arm"] = 1
            self.assertEqual(servos.positions[1], 480)
            servos.calibrate(1, 320)
            self.assertEqual(arm.center("arm"), 320)
            self.assertEqual(arm.move_relative("arm", 10), 330)
            self.assertEqual(servos.move_relative(1, 10), 320)
            for joint in ("unknown", None, [], 1):
                with self.assertRaises(ValueError):
                    arm.limits(joint)
            for action in ("unknown", [], None):
                with self.assertRaises(ValueError):
                    arm.step(action)
            for ticks in (0, -1, True, 1.0):
                with self.assertRaises(ValueError):
                    arm.step("grab", ticks)
            arm.set_pulse("gripper", 100)
            with self.assertRaises(ValueError):
                arm.step("grab", 1)
            self.assertEqual(arm.positions["gripper"], 100)


class TestControlValidation(unittest.TestCase):
    """Validation and dependency errors work even without optional widgets."""

    def test_rejects_unsupervised_closed_invalid_speed_and_missing_widgets(self):
        with RaspTank(use_gpio=False) as tank:
            with self.assertRaisesRegex(ValueError, "command_timeout"):
                tank.show_controls()
        with self.assertRaises(ControllerClosedError):
            tank.show_controls()
        with RaspTank(use_gpio=False, command_timeout=0.1) as tank:
            for speed in (-1, 101, True, 15.5, float("nan")):
                with self.subTest(speed=speed), self.assertRaises(ValueError):
                    tank.show_controls(speed=speed)
            with patch.dict("sys.modules", {"ipywidgets": None}):
                with self.assertRaisesRegex(ImportError, "pip install.*ipywidgets"):
                    tank.show_controls()


@unittest.skipUnless(importlib.util.find_spec("ipywidgets") is not None,
                     "optional ipywidgets not installed")
class TestTankControls(unittest.TestCase):
    """Exercise actual Button.click, traits, watchdog expiry, error and ownership paths."""

    def setUp(self):
        self.display = patch("IPython.display.display").start()
        self.addCleanup(patch.stopall)
        self.motors = Mock()
        self.servos = ServoController(pwm=Mock())
        self.addCleanup(self.servos.close)
        self.tank = RaspTank(use_gpio=False, motors=self.motors, servos=self.servos,
                             command_timeout=0.05)
        self.addCleanup(self.tank.close)
        self.controls = self.tank.show_controls()

    def test_display_and_slider_drag_have_no_output(self):
        self.display.assert_called_once_with(self.controls.widget)
        self.motors.forward.assert_not_called()
        self.motors.stop.assert_not_called()
        self.servos.pwm.set_pwm.assert_not_called()
        self.assertFalse(self.controls.drive_enabled.value)
        self.assertFalse(self.controls.servo_enabled.value)
        self.assertEqual(tuple(self.controls.sliders), tuple(ARM_CHANNELS))
        for joint, slider in self.controls.sliders.items():
            self.assertEqual((slider.min, slider.max), self.tank.arm.limits(joint))
            slider.value = 320
        self.servos.pwm.set_pwm.assert_not_called()
        self.controls.drive_enabled.value = True
        self.controls.servo_enabled.value = True
        self.motors.forward.assert_not_called()
        self.servos.pwm.set_pwm.assert_not_called()

    def test_drive_disarmed_pulses_expire_stop_and_disarm(self):
        self.controls.buttons["forward"].click()
        self.motors.forward.assert_not_called()
        self.assertIn("Enable driving", self.controls.message.value)
        self.controls.drive_enabled.value = True
        self.controls.speed.value = 20
        for button, method in (("forward", "forward"), ("backward", "backward"),
                               ("left", "turn_left"), ("right", "turn_right")):
            self.controls.buttons[button].click()
            getattr(self.motors, method).assert_called_once_with(20)
            self.assertEqual(self.tank.roaming_status()["state"], "manual")
        deadline = time.monotonic() + 1
        while self.tank.roaming_status()["state"] == "manual" and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertNotEqual(self.tank.roaming_status()["state"], "manual")
        self.controls.buttons["stop"].click()
        self.assertEqual(self.tank.roaming_status()["state"], "stopped")
        self.controls.buttons["forward"].click()
        self.controls.drive_enabled.value = False
        self.assertEqual(self.tank.roaming_status()["state"], "stopped")
        self.servos.pwm.set_pwm.assert_not_called()

    def test_all_joint_apply_and_steps_stop_before_servo_output(self):
        self.controls.servo_enabled.value = True
        log = Mock()
        log.attach_mock(self.motors.stop, "stop")
        log.attach_mock(self.servos.pwm.set_pwm, "set_pwm")
        for joint, channel in ARM_CHANNELS.items():
            log.reset_mock()
            self.controls.sliders[joint].value = 320
            self.controls.buttons[f"set_{joint}"].click()
            self.assertEqual(log.mock_calls, [call.stop(), call.set_pwm(channel, 0, 320)])
        self.controls.step_size.value = 5
        self.controls.buttons["grab"].click()
        self.assertEqual(self.tank.arm.positions["gripper"], 315)
        self.assertEqual(self.controls.sliders["gripper"].value, 315)
        # Relative actions use current controller state, not a stale slider selection.
        self.tank.arm.set_pulse("arm", 350)
        self.controls.buttons["arm_up"].click()
        self.assertEqual(self.controls.sliders["arm"].value, 355)

    def test_servo_disarmed_and_range_errors_remain_visible_and_disarm(self):
        self.controls.buttons["grab"].click()
        self.servos.pwm.set_pwm.assert_not_called()
        self.assertIn("Enable servos", self.controls.message.value)
        self.controls.servo_enabled.value = True
        self.tank.arm.set_pulse("gripper", 100)
        self.controls.drive_enabled.value = True
        self.controls.buttons["forward"].click()
        self.controls.buttons["grab"].click()
        self.assertIn("ValueError", self.controls.message.value)
        self.assertIn("[100, 500]", self.controls.message.value)
        self.assertFalse(self.controls.drive_enabled.value)
        self.assertFalse(self.controls.servo_enabled.value)
        self.assertEqual(self.tank.roaming_status()["state"], "stopped")

    def test_hardware_failure_and_failed_stop_are_visible(self):
        self.controls.drive_enabled.value = True
        self.motors.forward.side_effect = OSError("motor output lost")
        self.controls.buttons["forward"].click()
        self.assertIn("motor output lost", self.controls.message.value)
        self.assertFalse(self.controls.drive_enabled.value)
        self.motors.stop.side_effect = OSError("stop output lost")
        try:
            self.controls.buttons["stop"].click()
            self.assertIn("motor stop failed", self.controls.message.value)
            self.assertIn("stop output lost", self.controls.message.value)
        finally:
            self.motors.stop.side_effect = None

    def test_replacement_and_tank_close_invalidate_old_callbacks_without_borrowed_close(self):
        old = self.controls
        self.controls = self.tank.show_controls(speed=10)
        self.assertTrue(old._closed)
        old.drive_enabled.value = True
        old.buttons["forward"].click()
        self.motors.forward.assert_not_called()
        self.assertTrue(all(widget.comm is None for widget in old._widgets))
        self.controls.close()
        self.controls.close()
        self.assertFalse(self.tank._closed)
        self.controls = self.tank.show_controls()
        self.tank.close()
        self.assertTrue(self.controls._closed)
        self.assertTrue(all(widget.comm is None for widget in self.controls._widgets))
        self.motors.close.assert_not_called()
        self.assertFalse(self.servos._closed)

    def test_close_attempts_widget_cleanup_even_when_motor_stop_fails(self):
        self.motors.stop.side_effect = OSError("cannot stop")
        try:
            with self.assertRaisesRegex(OSError, "cannot stop"):
                self.controls.close()
            self.assertTrue(all(widget.comm is None for widget in self.controls._widgets))
            self.controls.close()
        finally:
            self.motors.stop.side_effect = None

    def test_motor_only_controls_stop_without_servo_widgets(self):
        with RaspTank(use_gpio=False, command_timeout=0.1) as tank:
            controls = tank.show_controls()
            self.assertEqual(controls.sliders, {})
            self.assertTrue(controls.servo_enabled.disabled)
            controls.buttons["stop"].click()
            self.assertEqual(tank.roaming_status()["state"], "stopped")

    def test_display_failure_closes_widgets_but_not_borrowed_tank(self):
        closed = []
        original = TankControls.close

        def close(controls):
            """Keep the failed handle for explicit resource assertions."""
            closed.append(controls)
            return original(controls)

        self.display.side_effect = RuntimeError("frontend unavailable")
        with patch.object(TankControls, "close", close):
            with self.assertRaisesRegex(RuntimeError, "frontend unavailable"):
                self.tank.show_controls()
        self.assertEqual(len(closed), 2)  # previous GUI, then failed replacement
        self.assertTrue(all(widget.comm is None for widget in closed[-1]._widgets))
        self.assertFalse(self.tank._closed)
        self.assertEqual(self.tank._controls, [])


@unittest.skipUnless(importlib.util.find_spec("ipywidgets") is not None,
                     "optional ipywidgets not installed")
class TestDemoNotebook(unittest.TestCase):
    """Execute the configuration-first notebook as separate cells, including reruns."""

    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "notebooks/rasptank_camera_roaming.ipynb"
        self.notebook = json.loads(path.read_text())
        self.code = ["".join(cell["source"]) for cell in self.notebook["cells"]
                     if cell["cell_type"] == "code"]

    def test_config_first_video_then_arm_then_gui_with_dry_run(self):
        self.assertEqual(self.notebook["cells"][0]["cell_type"], "code")
        self.assertIn("RUN_CAMERA = False", self.code[0])
        self.assertIn("camera.start()", self.code[1])
        self.assertIn("arm = tank.arm", self.code[2])
        self.assertIn("tank.show_controls", self.code[3])
        namespace = {}
        exec(self.code[0], namespace)
        namespace.update(RUN_CONTROLS=True, ENABLE_SERVOS=True, RUN_ARM_EXAMPLE=True,
                         ARM_SAFE_POSE={"arm": 310, "gripper": 320})
        with patch("IPython.display.display"):
            for source in self.code[1:4]:
                exec(source, namespace)
            tank, old = namespace["tank"], namespace["controls"]
            try:
                self.assertFalse(tank.motors.use_gpio)
                self.assertEqual(tank.arm.positions["arm"], 310)
                self.assertEqual(tank.arm.positions["gripper"], 320)
                exec(self.code[3], namespace)
                self.assertTrue(old._closed)
                exec(self.code[2], namespace)
                self.assertTrue(tank._closed)
                exec(self.code[3], namespace)
            finally:
                exec(self.code[-1], namespace)
            self.assertIsNone(namespace["tank"])

    def test_real_setup_safety_gates_before_construction(self):
        namespace = {}
        exec(self.code[0], namespace)
        exec(self.code[1], namespace)
        namespace.update(USE_HARDWARE=True, RUN_CONTROLS=True)
        with patch.dict(namespace, {"RaspTank": Mock()}) as state:
            with self.assertRaisesRegex(RuntimeError, "stand"):
                exec(self.code[2], state)
            state.update(STAND_AND_POWER_CUTOFF_READY=True, ENABLE_SERVOS=True)
            with self.assertRaisesRegex(RuntimeError, "servo"):
                exec(self.code[2], state)
            state["RaspTank"].assert_not_called()

    def test_invalid_full_pose_does_not_partially_move(self):
        namespace = {}
        exec(self.code[0], namespace)
        exec(self.code[1], namespace)
        namespace.update(RUN_ARM_EXAMPLE=True, ENABLE_SERVOS=True,
                         ARM_SAFE_POSE={"arm": 320, "gripper": 501})
        with patch.object(RobotArm, "set_pulse") as output:
            with self.assertRaises(ValueError):
                exec(self.code[2], namespace)
            output.assert_not_called()
            self.assertTrue(namespace["tank"]._closed)

    def test_camera_reused_and_cleanup_owned_by_caller_for_both_video_modes(self):
        for url in (False, True):
            with self.subTest(url=url), patch("IPython.display.display"), \
                    patch("rasptank.notebook.show_camera") as inline, \
                    patch("rasptank.stream.CameraStream") as attached_stream:
                namespace = {}
                exec(self.code[0], namespace)
                camera, preview = Mock(), Mock()
                namespace.update(USE_HARDWARE=True, RUN_CAMERA=True, USE_STREAM_URL=url,
                                 RUN_CONTROLS=True, ENABLE_SERVOS=True,
                                 STAND_AND_POWER_CUTOFF_READY=True,
                                 SERVO_CLEARANCE_VERIFIED=True,
                                 CameraController=Mock(return_value=camera),
                                 CameraStream=Mock(return_value=preview))
                real_tank = RaspTank

                def dry_tank(**kwargs):
                    """Keep real ownership/supervision and replace only hardware mode."""
                    kwargs["use_gpio"] = False
                    return real_tank(**kwargs)

                namespace["RaspTank"] = dry_tank
                inline.return_value = preview
                exec(self.code[1], namespace)
                namespace["CameraController"].assert_called_once()
                camera.start.assert_called_once()
                exec(self.code[2], namespace)
                exec(self.code[3], namespace)
                tank = namespace["tank"]
                try:
                    self.assertIs(tank.camera, camera)
                    camera.close.assert_not_called()
                    if url:
                        attached_stream.assert_called_once()
                        self.assertIs(attached_stream.call_args.args[0], camera)
                        self.assertEqual(attached_stream.call_args.kwargs["on_disconnect"],
                                         tank.stop)
                    else:
                        self.assertEqual(inline.call_count, 2)
                        self.assertEqual(inline.call_args.kwargs["on_disconnect"], tank.stop)
                    exec(self.code[2], namespace)
                    self.assertTrue(tank._closed)
                    namespace["CameraController"].assert_called_once()
                    camera.close.assert_not_called()
                finally:
                    exec(self.code[-1], namespace)
                camera.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
