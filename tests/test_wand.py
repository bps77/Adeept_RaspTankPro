"""Synthetic-frame tests for the wand tracker; no camera or hardware.

Frames are built with numpy (black images with white squares), the camera is a
FakeCamera, and servos use a mock PWM driver, so the suite runs anywhere with
numpy/OpenCV and never touches the Pi. Tests skip when those are missing.
"""

import time
import unittest
from unittest.mock import Mock, patch

from rasptank import Frame, RobotArm, ServoController
from rasptank import wand_diagnostics as diagnostics
from rasptank.exceptions import CameraTimeoutError, ServoHardwareError
from rasptank.wand_arm import WandArmFollower
from rasptank.wand import (WandTracker, bounding_box, direction_changes, normalize,
                           path_length, resample)

try:
    import numpy as np
except ImportError:
    np = None


def frame(*points, size=(120, 160)):
    """Black RGB test image (size is (height, width)) with a 5x5 white square at each (x, y)."""
    image = np.zeros((*size, 3), dtype=np.uint8)
    for x, y in points:
        image[y - 2:y + 3, x - 2:x + 3] = 255
    return image


@unittest.skipIf(np is None, "numpy/opencv not installed")
class WandTrackerTests(unittest.TestCase):
    """WandTracker detection, lock handling, threading, rendering and trajectory helpers."""
    def test_tracks_trajectory_and_ignores_far_distractor(self):
        """A locked target follows the tip; a far second blob must not steal the lock."""
        tracker = WandTracker(min_area=5, max_jump=30, max_trajectory_points=3)
        for x in (20, 30, 40, 50):
            tracker.process(frame((x, 60)))
        self.assertTrue(tracker.is_tracking)
        self.assertEqual(len(tracker.trajectory), 3)
        tracker.process(frame((52, 60), (140, 20)))
        self.assertAlmostEqual(tracker.position[0], 52, delta=1)

    def test_loss_tolerates_gaps_then_restarts_stroke(self):
        """Short dropouts keep the lock; a long one ends the stroke and the next starts fresh."""
        tracker = WandTracker(min_area=5, max_missed_frames=2)
        tracker.process(frame((20, 20)))
        for _ in range(2):
            tracker.process(frame())
        self.assertTrue(tracker.is_tracking)
        tracker.process(frame())
        self.assertFalse(tracker.is_tracking)
        self.assertEqual(len(tracker.trajectory), 1)
        tracker.process(frame((100, 80)))
        self.assertEqual(len(tracker.trajectory), 1)

    def test_area_filter_and_render(self):
        """Blobs outside min/max area are ignored; render() returns an overlay image."""
        tracker = WandTracker(min_area=50)
        self.assertIsNone(tracker.process(frame((20, 20))))
        tracker.min_area = 5
        tracker.process(frame((20, 20)))
        self.assertEqual(tracker.render().shape, (120, 160, 3))
        self.assertTrue(tracker.jpeg().startswith(b"\xff\xd8"))

    def test_thread_start_stop_reports_source_error(self):
        """The thread stops cleanly and records a fatal source error without closing the source."""
        calls = []

        def source():
            calls.append(1)
            if len(calls) > 3:
                raise RuntimeError("camera gone")
            time.sleep(0.01)
            return frame((30, 30))

        tracker = WandTracker(source, min_area=5).start()
        deadline = time.monotonic() + 2
        while tracker.running and time.monotonic() < deadline:
            time.sleep(0.01)
        tracker.stop()
        self.assertIsInstance(tracker.error, RuntimeError)
        self.assertFalse(tracker.running)

    def test_transient_timeouts_are_tolerated_then_fail_closed(self):
        """A few timeouts are survived; too many stop tracking and clear the position."""
        results = iter([frame((30, 30)), CameraTimeoutError("t"), CameraTimeoutError("t"),
                        frame((32, 30))])

        def flaky():
            item = next(results, None)
            if isinstance(item, Exception):
                raise item
            if item is None:
                raise CameraTimeoutError("t")
            return item

        tracker = WandTracker(flaky, min_area=5, max_source_failures=4).start()
        deadline = time.monotonic() + 2
        while tracker.running and time.monotonic() < deadline:
            time.sleep(0.01)
        tracker.stop()
        self.assertIn("consecutive", str(tracker.error))
        self.assertFalse(tracker.is_tracking)
        self.assertGreaterEqual(len(tracker.trajectory), 2)

    def test_clear_resets_overlay_and_motion_filter_ignores_static_blob(self):
        """clear() blanks the cached overlay; the motion filter drops static bright blobs."""
        tracker = WandTracker(min_area=5, motion_threshold=25)
        for _ in range(3):
            self.assertIsNone(tracker.process(frame((40, 40))))
        self.assertIsNotNone(tracker.process(frame((50, 40))))
        tracker.clear()
        self.assertFalse(tracker.is_tracking)
        self.assertEqual(len(tracker.trajectory), 0)
        self.assertEqual(tracker.render().shape, (120, 160, 3))
        self.assertIsNone(tracker._latest[1])

    def test_restart_resets_fps(self):
        """Restarting the thread must not show stale FPS from the previous run."""
        tracker = WandTracker(lambda: None, min_area=5)
        tracker.fps, tracker._last_time = 99.0, 1.0
        tracker.start()
        tracker.stop()
        self.assertEqual(tracker.fps, 0.0)

    def test_trajectory_utilities(self):
        """path_length, bounding_box, normalize and direction_changes on known shapes."""
        square = [(0, 0), (10, 0), (10, 10), (0, 10)]
        self.assertEqual(path_length(square), 30)
        self.assertEqual(bounding_box(square), (0, 0, 10, 10))
        self.assertEqual(direction_changes(square), 2)
        self.assertEqual(normalize([(5, 5), (15, 5)]).max(), 1)
        self.assertEqual(len(WandTracker().trajectory), 0)

    def test_trace_is_white_on_black_and_resample_is_even(self):
        """render_trace draws only white on black; resample spaces points evenly by arc length."""
        tracker = WandTracker(min_area=5)
        self.assertIsNone(tracker.render_trace())
        for x in (20, 40, 60):
            tracker.process(frame((x, 60)))
        trace = tracker.render_trace()
        self.assertEqual(trace.shape, (120, 160, 3))
        self.assertTrue((trace[60, 60] == 255).all())
        self.assertTrue((trace[100:, 100:] == 0).all())
        points = resample([(0, 0), (10, 0), (10, 30)], 5)
        self.assertEqual(points.shape, (5, 2))
        self.assertTrue(np.allclose(points[2], (10, 10)))
        self.assertEqual(tuple(points[-1]), (10, 30))

    def test_synthetic_wand_yields_strokes_and_rejects_lamp(self):
        """The simulated wand produces finished strokes and the large lamp is never tracked."""
        strokes = []
        tracker = WandTracker(min_area=5, on_stroke=strokes.append, min_stroke_points=5)
        source = diagnostics.SyntheticWand(fps=240, dropout=(40, 10))
        for _ in range(100):
            tracker.process(source())
        self.assertGreaterEqual(len(strokes), 1)
        self.assertGreaterEqual(len(strokes[0]), 5)
        self.assertTrue((strokes[0][:, 0] < 520).all())  # never jumped to the lamp at x>520
        self.assertIsNotNone(tracker.last_stroke)

    def test_stroke_callback_error_does_not_stop_tracking(self):
        """An exception in on_stroke is stored in callback_error and tracking continues."""
        def boom(_):
            raise ValueError("bad callback")
        tracker = WandTracker(min_area=5, on_stroke=boom, min_stroke_points=2,
                              max_missed_frames=0)
        tracker.process(frame((20, 20)))
        tracker.process(frame((24, 20)))
        tracker.process(frame())
        self.assertIsInstance(tracker.callback_error, ValueError)
        tracker.process(frame((90, 90)))
        self.assertTrue(tracker.is_tracking)

    def test_scene_report_hints(self):
        """scene_report gives the right tuning hint for dark, washed-out and good scenes."""
        tracker = WandTracker(min_area=5)
        self.assertIn("lower THRESHOLD", diagnostics.scene_report(frame(), tracker)["hint"])
        self.assertEqual(diagnostics.scene_report(frame((50, 50)), tracker)["hint"], "Looks good.")
        self.assertIn("raise THRESHOLD",
                      diagnostics.scene_report(np.full((120, 160, 3), 255, np.uint8), tracker)["hint"])


class FakeCamera:
    """Delivers a fixed bright frame; skips sequence 3 to model a dropped frame."""

    def __init__(self):
        self.n = 0

    def capture(self, timeout=1, after_sequence=None):
        self.n += 1
        sequence = self.n + (1 if self.n >= 3 else 0)
        time.sleep(0.005)
        image = frame((50, 50))
        return Frame(sequence, time.monotonic() - 0.01, 160, 120, image.tobytes(), b"jpeg")


@unittest.skipIf(np is None, "numpy/opencv not installed")
class DiagnosticsTests(unittest.TestCase):
    """benchmark_camera, sweep_modes and LiveView against fakes."""
    def test_benchmark_counts_frames_drops_latency_and_detection(self):
        """benchmark_camera reports frames, a skipped sequence as dropped, latency and detection rate."""
        result = diagnostics.benchmark_camera(FakeCamera(), WandTracker(min_area=5), seconds=0.5)
        self.assertGreater(result["frames"], 5)
        self.assertEqual(result["dropped"], 1)
        self.assertGreaterEqual(result["latency_ms_mean"], 10)
        self.assertEqual(result["detection_rate"], 1.0)
        self.assertIn("FPS", diagnostics.format_results([{"requested": ((160, 120), 30), **result}]))

    def test_sweep_reports_failures_and_always_closes_cameras(self):
        """sweep_modes records per-mode errors and closes every camera it opened."""
        cameras = []

        def factory(size, fps, **_):
            camera = Mock()
            camera.start.side_effect = RuntimeError("no mode") if size[0] > 200 else None
            camera.capture.side_effect = FakeCamera().capture
            cameras.append(camera)
            return camera

        with patch.object(diagnostics, "CameraController", factory):
            rows = diagnostics.sweep_modes([((160, 120), 30), ((320, 240), 60)],
                                           lambda: WandTracker(min_area=5), seconds=0.5)
        self.assertNotIn("error", rows[0])
        self.assertIn("no mode", rows[1]["error"])
        for camera in cameras:
            camera.close.assert_called_once_with()

    @unittest.skipIf(__import__("importlib").util.find_spec("ipywidgets") is None, "no ipywidgets")
    def test_live_view_updates_stops_and_replays(self):
        """LiveView refreshes the widget, stops on request and replays a stroke."""
        tracker = WandTracker(min_area=5)
        for x in (20, 40, 60):
            tracker.process(frame((x, 60)))
        with patch("IPython.display.display"):
            view = diagnostics.LiveView(tracker, mode="trace", delay=0.01)
            deadline = time.monotonic() + 2
            while not view.image.value and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(bytes(view.image.value).startswith(b"\xff\xd8"))
            view.stop()
            self.assertFalse(view._thread.is_alive())
            view.replay(tracker.trajectory, (160, 120), delay=0.001)
            self.assertIn("Replayed 3", view.label.value)


@unittest.skipIf(np is None, "numpy/opencv not installed")
class WandArmFollowerTests(unittest.TestCase):
    """WandArmFollower on a real RobotArm/ServoController whose PWM driver is a Mock."""
    def setUp(self):
        self.pwm = Mock()
        self.servos = ServoController(pwm=self.pwm)
        self.arm = RobotArm(self.servos)
        self.tracker = WandTracker(min_area=5)
        self.addCleanup(self.servos.close)

    def follower(self, **kwargs):
        return WandArmFollower(self.tracker, self.arm, **kwargs)

    def test_disarmed_and_untracked_never_move(self):
        """No servo command is sent while disarmed or while the wand is not tracked."""
        follower = self.follower()
        self.tracker.process(frame((150, 60)))
        self.assertEqual(follower.update(), [])
        follower.armed = True
        self.tracker.clear()
        self.assertEqual(follower.update(), [])
        self.pwm.set_pwm.assert_not_called()

    def test_off_center_wand_steps_joints_and_center_is_deadband(self):
        """Off-center tips step the matching joints; a centered tip stays inside the dead band."""
        follower = self.follower()
        follower.armed = True
        self.tracker.process(frame((80, 60)))
        self.assertEqual(follower.update(), [])
        self.tracker.process(frame((150, 10)))
        moves = follower.update()
        self.assertEqual([a for a, _ in moves], ["look_right", "camera_up"])
        self.assertTrue(all(1 <= t <= follower.max_ticks for _, t in moves))
        self.assertEqual(self.pwm.set_pwm.call_count, 2)
        flipped = self.follower(invert_x=True, invert_y=True)
        flipped.armed = True
        self.assertEqual([a for a, _ in flipped.update()], ["look_left", "camera_down"])

    def test_joint_limit_is_counted_not_forced(self):
        """A joint at its limit is counted in limit_hits and never forced."""
        follower = self.follower(x_joint="pan", y_joint=None, max_ticks=20, gain=200)
        follower.armed = True
        self.tracker.process(frame((155, 60)))
        for _ in range(40):
            follower.update()
        low, high = self.arm.limits("pan")
        self.assertTrue(low <= self.arm.positions["pan"] <= high)
        self.assertGreater(follower.limit_hits, 0)

    def test_hardware_error_disarms_thread_and_stop_is_clean(self):
        """A hardware error disarms the follower thread; stop() still returns cleanly."""
        self.pwm.set_pwm.side_effect = RuntimeError("i2c")
        self.tracker.process(frame((150, 60)))
        follower = self.follower(rate_hz=20)
        follower.armed = True
        follower.start()
        deadline = time.monotonic() + 2
        while follower.error is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIsInstance(follower.error, ServoHardwareError)
        self.assertFalse(follower.armed)
        follower.stop()


if __name__ == "__main__":
    unittest.main()
