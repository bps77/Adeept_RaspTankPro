"""Loopback HTTP integration checks using borrowed, hardware-free capture."""

import unittest
from types import SimpleNamespace
from urllib.error import HTTPError, URLError
from urllib.request import urlopen
from unittest.mock import Mock

from rasptank import CameraStream, RaspTank


class TestCameraStream(unittest.TestCase):
    """Verify actual HTTP responses, cleanup, ownership and explicit failures."""

    def setUp(self):
        """Create a borrowed fake camera and loopback listener on an unused port."""
        self.camera = Mock()
        self.camera.capture.return_value = SimpleNamespace(jpeg=b"\xff\xd8fake\xff\xd9")
        self.stream = CameraStream(self.camera, port=0)
        self.addCleanup(self.stream.close)

    def test_page_and_frame(self):
        """Serve the player and uncached fresh JPEG without opening another camera."""
        self.assertEqual(self.stream._server.server_address[0], "127.0.0.1")
        with urlopen(self.stream.url, timeout=2) as response:
            page = response.read()
            self.assertIn(b'fetch("frame.jpg"', page)
            self.assertIn(b"does not stop the tank", page)
        with urlopen(self.stream.url + "frame.jpg", timeout=2) as response:
            self.assertEqual(response.headers["Content-Type"], "image/jpeg")
            self.assertEqual(response.headers["Cache-Control"], "no-store")
            self.assertEqual(response.read(), b"\xff\xd8fake\xff\xd9")
        self.camera.capture.assert_called_once_with(timeout=1)
        self.camera.start.assert_not_called()
        self.camera.close.assert_not_called()

    def test_no_unscoped_routes(self):
        """Reject requests without the per-instance token, including control routes."""
        base = self.stream.url.rsplit("/", 2)[0]
        for path in ("/", "/frame.jpg", "/stop", "/../frame.jpg"):
            with self.subTest(path=path), self.assertRaises(HTTPError) as error:
                urlopen(base + path, timeout=2)
            self.assertEqual(error.exception.code, 404)

    def test_error_and_recovery(self):
        """HTTP errors show capture failures; successful requests clear status."""
        self.camera.capture.side_effect = RuntimeError("Camera busy")
        with self.assertRaises(HTTPError) as error:
            urlopen(self.stream.url + "frame.jpg", timeout=2)
        self.assertEqual(error.exception.code, 503)
        self.assertIn(b"Camera busy", error.exception.read())
        self.assertIn("Camera busy", self.stream.status()["error"])
        self.camera.capture.side_effect = None
        with urlopen(self.stream.url + "frame.jpg", timeout=2) as response:
            response.read()
        self.assertIsNone(self.stream.status()["error"])

    def test_transform_and_motor_failure(self):
        """Report invalid transform output and failures of the motor-stop callback."""
        stop = Mock(side_effect=RuntimeError("stop failed"))
        with CameraStream(self.camera, port=0, transform=lambda frame: None,
                          on_disconnect=stop) as stream:
            with self.assertRaises(HTTPError) as error:
                urlopen(stream.url + "frame.jpg", timeout=2)
            message = error.exception.read()
            self.assertIn(b"nonempty JPEG", message)
            self.assertIn(b"motor stop failed", message)
            stop.assert_called_once_with()

    def test_close_releases_listener_not_camera(self):
        """Idempotent close ends serving and keeps the caller's camera open."""
        self.stream.close()
        self.stream.close()
        self.assertFalse(self.stream.status()["running"])
        self.assertFalse(self.stream._worker.is_alive())
        self.camera.close.assert_not_called()
        with self.assertRaises(URLError):
            urlopen(self.stream.url, timeout=1)

    def test_validation(self):
        """Reject invalid configuration before starting a listener."""
        for options in ({"port": True}, {"port": -1}, {"fps": 0},
                        {"transform": 1}, {"on_disconnect": 1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                CameraStream(self.camera, **options)

    def test_tank_owns_server_and_stops_supervised_motion_on_error(self):
        """The facade owns HTTP presentation, borrows capture, and wires error stop."""
        with RaspTank(use_gpio=False, camera=self.camera, command_timeout=0.5) as tank:
            stream = tank.start_camera_stream(port=0)
            tank.stop = Mock()
            with urlopen(stream.url, timeout=2) as response:
                self.assertEqual(response.status, 200)
            second = tank.start_camera_stream(port=0)
            self.assertFalse(stream.status()["running"])
            self.camera.capture.side_effect = RuntimeError("capture failed")
            with self.assertRaises(HTTPError):
                urlopen(second.url + "frame.jpg", timeout=2)
            tank.stop.assert_called_once_with()
        self.assertFalse(second.status()["running"])
        self.camera.close.assert_not_called()
