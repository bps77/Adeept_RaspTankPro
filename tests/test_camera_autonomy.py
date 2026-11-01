"""Software-only capture process, fused policy, watchdog and notebook integration checks."""

from functools import partial
import json
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from rasptank import CameraController, Frame, RaspTank, RoamPolicy
from rasptank.autonomy import DriveSupervisor
from rasptank.exceptions import (AutonomyError, CameraError, CameraTimeoutError,
                                ControllerClosedError, HardwareDisabledError)
from rasptank.vision import VisualObservation


class FakeSource:
    """Picklable child-side source; optionally record graceful close or stall after one frame."""

    def __init__(self, settings, close_path=None, stall=False, fail=False, large=False,
                 fail_close=False, startup_hang=False):
        """Retain explicit test-only settings without hardware access."""
        self.close_path, self.stall, self.fail, self.count = close_path, stall, fail, 0
        self.large, self.fail_close = large, fail_close
        self.startup_hang = startup_hang

    def read(self):
        """Return tiny valid-shaped frame or simulate driver failure/hang."""
        if self.fail:
            raise OSError("test capture failure")
        if self.startup_hang or (self.stall and self.count):
            time.sleep(30)
        self.count += 1
        if self.large:
            return 640, 480, bytes([255, 0, 0] * 640 * 480), b"test-jpeg"
        return 2, 2, bytes([255, 0, 0] * 4), b"test-jpeg"

    def close(self):
        """Record that normal process shutdown reaches source cleanup."""
        if self.close_path:
            Path(self.close_path).write_text("closed")
        if self.fail_close:
            raise OSError("test cleanup failure")


class FakeCamera:
    """In-process camera double, always yielding genuinely new stopped-state frames."""

    def __init__(self):
        """Initialize a frame sequence; no hardware."""
        self.sequence = 0

    def capture(self, timeout=1, *, after_sequence=None, min_timestamp=0):
        """Generate immutable RGB frame with current monotonic timestamp."""
        self.sequence += 1
        return Frame(self.sequence, max(time.monotonic(), min_timestamp),
                     20, 20, bytes([120] * 1200), b"test-jpeg")


def clear_observer(frame):
    """Provide explicit clear test verdict for a matching frame."""
    return VisualObservation("clear", frame.sequence, frame.timestamp, 1)


def wait_until(predicate, timeout=2):
    """Wait a bounded interval for test thread state and fail explicitly if absent."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Timed out waiting for test condition")


class TestCamera(unittest.TestCase):
    """Validate real spawned workers without a real camera."""

    def test_imports_inert_without_optional_dependencies(self):
        script = (
            "import sys; import rasptank; import rasptank.vision; import rasptank.notebook; "
            "assert not any(n in sys.modules for n in "
            "('cv2','numpy','picamera2','libcamera','ipywidgets','RPi.GPIO'))")
        subprocess.run([sys.executable, "-c", script], check=True)

    def test_disabled_and_validation(self):
        with CameraController(use_gpio=False, source_factory=FakeSource) as camera:
            for operation in (camera.start, camera.capture, camera.snapshot):
                with self.assertRaises(HardwareDisabledError):
                    operation()
        with self.assertRaises(ControllerClosedError):
            camera.start()
        for kwargs in (dict(size=(0, 2)), dict(fps=True), dict(quality=100),
                       dict(hflip=1), dict(max_age=float("nan"))):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                CameraController(**kwargs)
        with self.assertRaises(ValueError):
            Frame(1, time.monotonic(), 2, 2, b"short", b"jpeg")

    def test_start_latest_freshness_restart_and_graceful_close(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "source-close.txt")
            camera = CameraController(source_factory=partial(FakeSource, close_path=path),
                                      fps=30, max_age=0.5)
            with camera:
                camera.start()
                process = camera._process
                camera.start()
                self.assertIs(camera._process, process)
                first = camera.capture()
                time.sleep(0.12)
                latest = camera.capture()
                self.assertGreater(latest.sequence, first.sequence)
                self.assertEqual(latest.rgb, bytes([255, 0, 0] * 4))
                future = camera.capture(after_sequence=latest.sequence,
                                        min_timestamp=time.monotonic())
                self.assertGreater(future.sequence, latest.sequence)
                self.assertEqual(camera.snapshot(), b"test-jpeg")
                camera.stop()
                self.assertEqual(Path(path).read_text(), "closed")
                self.assertIsNone(camera._reader)
                with self.assertRaises(CameraError):
                    camera.capture()
                camera.start()
                self.assertEqual(camera.capture().width, 2)
            camera.close()

    def test_capture_failure_rolls_back(self):
        for fail_close in (False, True):
            with self.subTest(fail_close=fail_close):
                camera = CameraController(source_factory=partial(
                    FakeSource, fail=True, fail_close=fail_close))
                with self.assertRaisesRegex(CameraError, "test capture failure"):
                    camera.start()
                self.assertIsNone(camera._process)
                if fail_close:
                    with self.assertRaisesRegex(CameraError, "test cleanup failure"):
                        camera.close()
                else:
                    camera.close()

    def test_large_ipc_frames_drain_before_graceful_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "closed")
            with CameraController(source_factory=partial(FakeSource, close_path=path,
                                                         large=True), fps=30) as camera:
                camera.start()
                self.assertEqual(len(camera.capture().rgb), 640 * 480 * 3)
            self.assertEqual(Path(path).read_text(), "closed")

    def test_cleanup_failure_surfaces_after_resources_release(self):
        camera = CameraController(source_factory=partial(FakeSource, fail_close=True))
        camera.start()
        with self.assertRaisesRegex(CameraError, "test cleanup failure"):
            camera.close()
        self.assertIsNone(camera._process)

    def test_picamera2_backend_flushes_requests_and_defines_rgb_bytes(self):
        from rasptank.hardware.camera import _PiSource

        native = Mock()
        request = Mock()
        native.captured_request.return_value.__enter__ = Mock(return_value=request)
        native.captured_request.return_value.__exit__ = Mock(return_value=False)
        image = Mock(width=2, height=2)
        image.tobytes.return_value = bytes([255, 0, 0] * 4)
        image.save.side_effect = lambda stream, **kwargs: stream.write(b"jpeg")
        request.make_image.return_value.convert.return_value = image
        with patch.dict(sys.modules, {"picamera2": Mock(Picamera2=Mock(return_value=native)),
                                     "libcamera": Mock()}):
            source = _PiSource(dict(size=(2, 2), fps=15, quality=80, hflip=False, vflip=False))
            self.assertEqual(source.read(), (2, 2, image.tobytes.return_value, b"jpeg"))
            native.captured_request.assert_called_once_with(flush=True)
            request.make_image.return_value.convert.assert_called_once_with("RGB")
            native.create_video_configuration.assert_called_once_with(
                main={"size": (2, 2), "format": "RGB888"},
                transform=native.create_video_configuration.call_args.kwargs["transform"],
                controls={"FrameRate": 15}, buffer_count=4, queue=False)
            source.close()
            native.stop.assert_called_once()
            native.close.assert_called_once()

    def test_stuck_driver_frame_age_and_shutdown_are_bounded(self):
        with CameraController(source_factory=partial(FakeSource, stall=True),
                              shutdown_timeout=0.05, max_age=0.05) as camera:
            camera.start()
            time.sleep(0.07)
            with self.assertRaises(CameraTimeoutError):
                camera.capture(timeout=0.02)
            started = time.monotonic()
            camera.stop()
            self.assertLess(time.monotonic() - started, 0.5)

    def test_startup_hang_and_unpicklable_factory_release_resources(self):
        camera = CameraController(source_factory=partial(FakeSource, startup_hang=True),
                                  startup_timeout=0.1, shutdown_timeout=0.05)
        started = time.monotonic()
        with self.assertRaises(CameraTimeoutError):
            camera.start()
        self.assertLess(time.monotonic() - started, 0.5)
        self.assertIsNone(camera._process)
        camera.close()
        camera = CameraController(source_factory=lambda settings: FakeSource(settings))
        with self.assertRaises((AttributeError, TypeError)):
            camera.start()
        self.assertIsNone(camera._process)
        camera.close()

    def test_facade_camera_owned_borrowed_and_rollback(self):
        with patch("rasptank.tank.CameraController") as factory:
            with RaspTank(use_gpio=False, enable_camera=True) as tank:
                self.assertIs(tank.camera, factory.return_value)
                tank.start_camera()
            factory.assert_called_once_with(use_gpio=False)
            factory.return_value.close.assert_called_once_with()
        borrowed = Mock()
        with RaspTank(use_gpio=False, camera=borrowed):
            pass
        borrowed.close.assert_not_called()
        with patch("rasptank.tank.MotorController") as motors, \
                patch("rasptank.tank.CameraController", side_effect=OSError("open")):
            with self.assertRaises(OSError):
                RaspTank(enable_camera=True)
            motors.return_value.close.assert_called_once()


class TestRoamPolicy(unittest.TestCase):
    """Exercise exact fused decisions/thresholds without motor output."""

    def setUp(self):
        """Create timestamp-controlled fixtures."""
        self.frame = Frame(1, 10, 20, 20, bytes(1200), b"jpeg")
        self.policy = RoamPolicy(stop_distance=0.5, resume_distance=0.6, max_turns=2)

    def step(self, distance=1, state="clear", now=10):
        """Execute policy against a matching timestamped observation."""
        observation = VisualObservation(state, 1, 10, 1)
        return self.policy.step(self.frame, distance, 10, observation, now=now)

    def test_both_sensors_veto_hysteresis_and_recovery(self):
        self.assertEqual(self.step().action, "forward")
        self.assertEqual(self.step(distance=0.5).action, "left")
        self.assertEqual(self.step(distance=0.6).action, "left")
        self.assertEqual(self.step(distance=0.2).action, "stop")
        self.assertEqual(self.step(distance=0.61).action, "forward")
        self.assertEqual(self.step(state="blocked").reason, "visual corridor blocked")
        self.assertEqual(self.step(state="unknown").action, "stop")
        self.policy.reset()
        self.assertEqual(self.step(distance=0.5).action, "left")
        self.assertEqual(self.step(distance=0.2).reason, "insufficient clearance even for turning")

    def test_stale_invalid_future_and_mismatched_data(self):
        self.assertEqual(self.step(now=10.3).action, "forward")
        for now in (10.30001, 9.99):
            with self.assertRaises(AutonomyError):
                self.step(now=now)
        for distance in (float("nan"), float("inf"), -1, True):
            with self.assertRaises(ValueError):
                self.step(distance=distance)
        with self.assertRaises(AutonomyError):
            self.policy.step(self.frame, 1, 10, VisualObservation("clear", 2, 10, 1), now=10)
        for kwargs in (dict(speed=50), dict(pulse=0), dict(stop_distance=1, resume_distance=0.5),
                       dict(max_turns=True), dict(max_age=float("inf"))):
            with self.assertRaises(ValueError):
                RoamPolicy(**kwargs)


class TestDriving(unittest.TestCase):
    """Validate watchdog independently of blocked perception and token races."""

    def test_exact_manual_lease_and_borrowed_motor_stop(self):
        clock = [10]
        motors = Mock()
        supervisor = DriveSupervisor(motors, 0.2, clock=lambda: clock[0])
        try:
            supervisor.manual("forward", 20)
            motors.reset_mock()
            clock[0] = 10.1999
            supervisor.check_expiry()
            motors.stop.assert_not_called()
            clock[0] = 10.2
            supervisor.check_expiry()
            motors.stop.assert_called_once()
            self.assertEqual(supervisor.status()["state"], "stopped")
        finally:
            supervisor.close()
        motors.close.assert_not_called()

    def test_roam_drives_and_finishes_finite_duration(self):
        motors, camera, sensor = Mock(), FakeCamera(), Mock()
        sensor.distance.return_value = 1
        with RaspTank(motors=motors, camera=camera, ultrasonic=sensor,
                      command_timeout=0.3) as tank:
            session = tank.start_roaming(observer=clear_observer,
                                        policy=RoamPolicy(pulse=0.02, max_duration=0.15))
            wait_until(lambda: not tank.roaming_status()["worker_alive"])
            self.assertGreater(motors.move.call_count, 1)
            self.assertTrue(all(c.args == (20, "forward", "no", 0.6)
                                for c in motors.move.call_args_list))
            self.assertIsNone(session.status()["error"])
        motors.close.assert_not_called()

    def test_stop_and_manual_override_cannot_be_undone_by_queued_observation(self):
        for override in ("stop", "forward"):
            with self.subTest(override=override):
                entered, unblock = threading.Event(), threading.Event()
                motors, camera, sensor = Mock(), FakeCamera(), Mock()
                sensor.distance.return_value = 1

                def observer(frame):
                    """Block until test supplies manual command, then return old result."""
                    entered.set()
                    unblock.wait(2)
                    return clear_observer(frame)

                with RaspTank(motors=motors, camera=camera, ultrasonic=sensor,
                              command_timeout=0.5) as tank:
                    session = tank.start_roaming(observer=observer)
                    self.assertTrue(entered.wait(1))
                    getattr(tank, override)(*(() if override == "stop" else (15,)))
                    calls = motors.move.call_count
                    unblock.set()
                    wait_until(lambda: not tank.roaming_status()["worker_alive"])
                    self.assertEqual(motors.move.call_count, calls)
                    self.assertEqual(session.status()["state"], "cancelled")
                    if override == "forward":
                        motors.forward.assert_called_once_with(15)
                        session.stop()
                        self.assertEqual(tank.roaming_status()["state"], "manual")

    def test_hung_perception_watchdog_stops_latches_and_requires_reset(self):
        entered, unblock = threading.Event(), threading.Event()
        motors, camera, sensor = Mock(), FakeCamera(), Mock()
        sensor.distance.return_value = 1

        def observer(frame):
            """Simulate a vision hang bounded by the test event, not the supervisor."""
            entered.set()
            unblock.wait(3)
            return clear_observer(frame)

        tank = RaspTank(motors=motors, camera=camera, ultrasonic=sensor, command_timeout=0.15)
        try:
            tank.start_roaming(observer=observer, policy=RoamPolicy(pulse=0.02))
            self.assertTrue(entered.wait(1))
            wait_until(lambda: tank.roaming_status()["error"] is not None)
            self.assertIn("deadline", tank.roaming_status()["error"])
            motors.move.assert_not_called()
            self.assertGreater(motors.stop.call_count, 1)
            with self.assertRaises(AutonomyError):
                tank.forward(15)
            with self.assertRaises(AutonomyError):
                tank.reset_drive_fault()
            unblock.set()
            wait_until(lambda: not tank.roaming_status()["worker_alive"])
            tank.reset_drive_fault()
            tank.forward(15)
        finally:
            unblock.set()
            tank.close()

    def test_range_failure_visual_unknown_stale_and_repeat_start(self):
        sensor, motors, camera = Mock(), Mock(), FakeCamera()
        sensor.distance.side_effect = OSError("range failed")
        with RaspTank(motors=motors, camera=camera, ultrasonic=sensor,
                      command_timeout=0.3) as tank:
            tank.start_roaming(observer=clear_observer)
            wait_until(lambda: not tank.roaming_status()["worker_alive"])
            self.assertIn("range failed", tank.roaming_status()["error"])
            motors.move.assert_not_called()
            tank.reset_drive_fault()
            sensor.distance.side_effect = None
            sensor.distance.return_value = 1
            tank.start_roaming(observer=lambda f: VisualObservation(
                "unknown", f.sequence, f.timestamp, 0))
            wait_until(lambda: not tank.roaming_status()["worker_alive"])
            self.assertEqual(tank.roaming_status()["state"], "blocked")
            motors.move.assert_not_called()

    def test_camera_range_and_delayed_observer_faults_never_drive(self):
        for failed_stage in ("camera", "range", "stale"):
            with self.subTest(stage=failed_stage):
                motors, sensor, camera = Mock(), Mock(), FakeCamera()
                sensor.distance.return_value = 1
                observer = clear_observer
                if failed_stage == "camera":
                    camera = Mock(capture=Mock(side_effect=CameraError("camera failed")))
                elif failed_stage == "range":
                    sensor.distance.side_effect = OSError("range failed")
                else:
                    def observer(frame):
                        """Return only after the input has exceeded the configured age."""
                        time.sleep(0.05)
                        return clear_observer(frame)
                with RaspTank(motors=motors, camera=camera, ultrasonic=sensor,
                              command_timeout=0.3) as tank:
                    tank.start_roaming(observer=observer,
                                       policy=RoamPolicy(pulse=0.01, max_age=0.02))
                    wait_until(lambda: not tank.roaming_status()["worker_alive"])
                    motors.move.assert_not_called()
                    self.assertIsNotNone(tank.roaming_status()["error"])

    def test_repeat_start_and_shutdown_of_stuck_observer_are_explicit(self):
        entered, unblock = threading.Event(), threading.Event()
        motors, sensor, camera = Mock(), Mock(), FakeCamera()
        sensor.distance.return_value = 1

        def observer(frame):
            """Remain stuck until test cleanup permits return."""
            entered.set()
            unblock.wait(3)
            return clear_observer(frame)

        tank = RaspTank(motors=motors, camera=camera, ultrasonic=sensor, command_timeout=0.15)
        tank.start_roaming(observer=observer, policy=RoamPolicy(pulse=0.02))
        self.assertTrue(entered.wait(1))
        try:
            with self.assertRaisesRegex(AutonomyError, "already running"):
                tank.start_roaming(observer=clear_observer, policy=RoamPolicy(pulse=0.02))
            with self.assertRaisesRegex(AutonomyError, "still running"):
                tank.close()
            self.assertGreater(motors.stop.call_count, 1)
            self.assertFalse(tank._supervisor._watchdog.is_alive())
            motors.close.assert_not_called()
        finally:
            unblock.set()
            tank._supervisor._worker.join(1)

    def test_configuration_gates_and_close_fault_attempts_cleanup(self):
        with RaspTank(use_gpio=False) as tank:
            with self.assertRaises(ValueError):
                tank.start_roaming(observer=clear_observer)
        with RaspTank(use_gpio=False, command_timeout=0.2) as tank:
            with self.assertRaises(ValueError):
                tank.start_roaming(observer=clear_observer)
        with patch("rasptank.tank.MotorController") as motor_factory, \
                patch("rasptank.tank.CameraController") as camera_factory:
            tank = RaspTank(enable_camera=True, command_timeout=0.2)
            motor_factory.return_value.stop.side_effect = OSError("stop failed")
            with self.assertRaises(OSError):
                tank.close()
            camera_factory.return_value.close.assert_called_once()
            motor_factory.return_value.close.assert_called_once()
            self.assertFalse(tank._supervisor._watchdog.is_alive())


class TestNotebook(unittest.TestCase):
    """Check notebook defaults and actual widget worker without a browser or Pi."""

    @unittest.skipUnless(importlib.util.find_spec("ipywidgets") is not None
                         and importlib.util.find_spec("IPython") is not None,
                         "optional notebook dependencies not installed")
    def test_widget_stream_nonblocking_rerun_error_and_borrowed_cleanup(self):
        from rasptank.notebook import show_camera

        camera = FakeCamera()
        with patch("IPython.display.display"):
            view = show_camera(camera, fps=30)
            wait_until(lambda: bool(view.image.value))
            replacement = show_camera(camera, previous=view)
            self.assertFalse(view._worker.is_alive())
            replacement.close()
            self.assertFalse(replacement._worker.is_alive())
            stop = Mock()
            view = show_camera(Mock(capture=Mock(side_effect=CameraError("lost"))),
                               on_disconnect=stop)
            wait_until(lambda: view.status()["error"] is not None)
            wait_until(lambda: not view._worker.is_alive())
            self.assertIn("lost", view.status()["error"])
            stop.assert_called_once_with()
            view.close()

    def test_all_notebooks_parse_and_default_execution_is_hardware_free(self):
        root = Path(__file__).resolve().parents[1]
        for path in (root / "notebooks").glob("*.ipynb"):
            notebook = json.loads(path.read_text())
            source = "\n\n".join("".join(c["source"]) for c in notebook["cells"]
                                if c["cell_type"] == "code")
            compile(source, str(path), "exec")
            subprocess.run([sys.executable, "-c", source], cwd=root,
                           check=True, stdout=subprocess.DEVNULL, timeout=20)


if __name__ == "__main__":
    unittest.main()
