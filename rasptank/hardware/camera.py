"""Import-inert, process-isolated Picamera2 capture with one latest RGB frame."""

from dataclasses import dataclass
import multiprocessing
import threading
import time

from ._lifecycle import ManagedController, ensure_open, number, release
from ..exceptions import CameraError, CameraTimeoutError, HardwareDisabledError

NO_CAMERA_HELP = (
    "libcamera detected no camera. This is Pi configuration/connection, not "
    "tank code: run 'libcamera-hello --list-cameras' (or 'rpicam-hello "
    "--list-cameras'); disable Legacy Camera in raspi-config (remove start_x=1); "
    "set camera_auto_detect=1, or camera_auto_detect=0 plus the sensor overlay "
    "(e.g. dtoverlay=ov5647) in config.txt; reseat the ribbon cable powered "
    "off; reboot. See docs/CAMERA_USAGE.md 'Troubleshooting'.")


@dataclass(frozen=True)
class Frame:
    """Immutable packed RGB bytes and JPEG, dimensions, sequence and monotonic seconds."""

    sequence: int
    timestamp: float
    width: int
    height: int
    rgb: bytes
    jpeg: bytes

    def __post_init__(self):
        """Reject malformed frames instead of allowing downstream shape confusion."""
        if type(self.sequence) is not int or self.sequence < 1:
            raise ValueError("sequence must be a positive integer")
        number(self.timestamp, 0, float("inf"), "timestamp")
        if any(type(v) is not int or not 1 <= v <= 4096
               for v in (self.width, self.height)):
            raise ValueError("frame dimensions must be integers in 1..4096")
        if not isinstance(self.rgb, bytes) or len(self.rgb) != self.width * self.height * 3:
            raise ValueError("rgb must be packed RGB bytes matching the dimensions")
        if not isinstance(self.jpeg, bytes) or not self.jpeg:
            raise ValueError("jpeg must be nonempty bytes")


class _PiSource:
    """Child-only Picamera2 owner; Pillow conversion defines actual RGB byte order."""

    def __init__(self, settings):
        """Configure explicitly; caller closes even if start fails."""
        from picamera2 import Picamera2
        from libcamera import Transform

        if not Picamera2.global_camera_info():
            raise CameraError(NO_CAMERA_HELP)
        self.camera = Picamera2()
        self.quality = settings["quality"]
        try:
            config = self.camera.create_video_configuration(
                main={"size": settings["size"], "format": "RGB888"},
                transform=Transform(hflip=settings["hflip"], vflip=settings["vflip"]),
                controls={"FrameRate": settings["fps"]}, buffer_count=4, queue=False)
            self.camera.configure(config)
            self.camera.start()
        except BaseException:
            try:
                self.camera.close()
            except BaseException:
                pass
            raise

    def read(self):
        """Wait for a post-read exposure; reject missing timing and release every request."""
        from io import BytesIO

        started_ns = time.monotonic_ns()
        while True:
            request = self.camera.capture_request()
            try:
                metadata = request.get_metadata()
                sensor_ns = metadata.get("SensorTimestamp")
                exposure_us = metadata.get("ExposureTime")
                if (type(sensor_ns) is not int or sensor_ns <= 0
                        or type(exposure_us) is not int or exposure_us < 0):
                    raise CameraError(
                        "Camera frame lacks valid SensorTimestamp/ExposureTime; "
                        "cannot verify freshness. Update the Raspberry Pi OS "
                        "libcamera and python3-picamera2 packages.")
                # Match Picamera2's flush rule without requiring its newer API.
                if sensor_ns - 1000 * exposure_us < started_ns:
                    continue
                image = request.make_image("main").convert("RGB")
                output = BytesIO()
                image.save(output, format="JPEG", quality=self.quality)
                return image.width, image.height, image.tobytes(), output.getvalue()
            finally:
                request.release()

    def close(self):
        """Stop and release the child-owned camera, attempting both operations."""
        release([self.camera.stop, self.camera.close])


def _capture_process(connection, factory, settings, halt):
    """Produce frames in a killable process; relay errors, including cleanup failures."""
    source = None
    try:
        source = factory(settings)
        sequence = 0
        while not halt.is_set():
            started = time.monotonic()
            width, height, rgb, jpeg = source.read()
            sequence += 1
            connection.send(("frame", Frame(sequence, started, width, height, rgb, jpeg)))
            halt.wait(max(0, 1 / settings["fps"] - (time.monotonic() - started)))
    except BaseException as exc:
        try:
            connection.send(("error", f"{type(exc).__name__}: {exc}"))
        except (BrokenPipeError, EOFError, OSError):
            pass  # Parent has already stopped consuming.
    finally:
        if source is not None:
            try:
                source.close()
            except BaseException as exc:
                try:
                    connection.send(("cleanup_error", f"camera cleanup: {type(exc).__name__}: {exc}"))
                except (BrokenPipeError, EOFError, OSError):
                    pass
        connection.close()


class CameraController(ManagedController):
    """Explicit camera lifecycle; injected source factories run in a spawned process.

    A source factory must be picklable and return an object with read()/close().
    read returns (width, height, packed RGB bytes, JPEG bytes). Disabled mode
    never calls the factory. Construction opens no camera and starts no worker.
    """

    def __init__(self, use_gpio=True, *, size=(640, 480), fps=15, quality=80,
                 hflip=False, vflip=False, source_factory=None, startup_timeout=5,
                 shutdown_timeout=2, max_age=0.5):
        """Validate settings (seconds, Hz, JPEG 1..95); defer hardware to start()."""
        if len(size) != 2 or any(type(v) is not int or not 1 <= v <= 4096 for v in size):
            raise ValueError("size must contain two integers in 1..4096")
        if type(quality) is not int or not 1 <= quality <= 95:
            raise ValueError("quality must be an integer in 1..95")
        if type(hflip) is not bool or type(vflip) is not bool:
            raise ValueError("flips must be bool")
        self.settings = dict(size=tuple(size), fps=number(fps, 1, 60, "fps"),
                             quality=quality, hflip=hflip, vflip=vflip)
        self.startup_timeout = number(startup_timeout, 0.01, 60, "startup_timeout")
        self.shutdown_timeout = number(shutdown_timeout, 0.01, 10, "shutdown_timeout")
        self.max_age = number(max_age, 0.01, 10, "max_age")
        self.use_gpio = use_gpio
        self._factory = source_factory or _PiSource
        self._closed = False
        self._condition = threading.Condition()
        self._lifecycle_lock = threading.RLock()
        self._halt = threading.Event()
        self._process = self._reader = self._connection = None
        self._process_halt = None
        self._frame = self._error = None
        self._cleanup_error = None

    def start(self):
        """Start one capture process; wait for a fresh frame or raise and roll back."""
        with self._lifecycle_lock:
            ensure_open(self)
            if not self.use_gpio:
                raise HardwareDisabledError("Camera hardware is disabled")
            if self._process is not None:
                self.capture(timeout=self.startup_timeout)
                return self
            context = multiprocessing.get_context("spawn")
            receiver, sender = context.Pipe(duplex=False)
            self._connection = receiver
            self._halt.clear()
            self._process_halt = context.Event()
            self._frame = self._error = None
            self._cleanup_error = None
            self._process = context.Process(
                target=_capture_process, args=(sender, self._factory, self.settings,
                                              self._process_halt),
                daemon=True)
            try:
                self._process.start()
                sender.close()
                self._reader = threading.Thread(target=self._receive, daemon=True,
                                                name="rasptank-camera-reader")
                self._reader.start()
                self.capture(timeout=self.startup_timeout)
            except BaseException:
                sender.close()
                try:
                    self.stop()
                except BaseException:
                    pass
                raise
            return self

    def _receive(self):
        """Drain IPC continuously into one latest slot and wake consumers on errors."""
        try:
            while not self._halt.is_set():
                if not self._connection.poll(0.05):
                    continue
                kind, payload = self._connection.recv()
                with self._condition:
                    if kind == "frame":
                        self._frame = payload
                    else:
                        error = CameraError(payload)
                        if self._error is None:
                            self._error = error
                        if kind == "cleanup_error":
                            self._cleanup_error = error
                    self._condition.notify_all()
        except (EOFError, OSError) as exc:
            with self._condition:
                if (not self._halt.is_set() and not self._process_halt.is_set()
                        and self._error is None):
                    self._error = CameraError(f"Capture process disconnected: {exc}")
                self._condition.notify_all()

    def capture(self, timeout=1, *, after_sequence=None, min_timestamp=0):
        """Return fresh latest Frame, optionally newer than sequence; never autostart.

        Wait at most timeout seconds. Stale images are not returned; stopped,
        closed, disabled or failed capture raises an explicit hardware error.
        """
        number(timeout, 0.001, 60, "timeout")
        number(min_timestamp, 0, float("inf"), "min_timestamp")
        if after_sequence is not None and (type(after_sequence) is not int or after_sequence < 0):
            raise ValueError("after_sequence must be a nonnegative integer")
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                ensure_open(self)
                if not self.use_gpio:
                    raise HardwareDisabledError("Camera hardware is disabled")
                if self._error is not None:
                    raise self._error
                if (self._process is None or self._halt.is_set()
                        or self._process_halt.is_set()):
                    raise CameraError("Camera is not started")
                frame = self._frame
                if (frame is not None and frame.timestamp <= time.monotonic() <= frame.timestamp + self.max_age
                        and frame.timestamp >= min_timestamp
                        and (after_sequence is None or frame.sequence > after_sequence)):
                    return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise CameraTimeoutError("No fresh camera frame before deadline")
                self._condition.wait(remaining)

    def snapshot(self, timeout=1):
        """Return JPEG bytes from a fresh frame; same failures as capture()."""
        return self.capture(timeout).jpeg

    def stop(self):
        """Bound shutdown of stuck capture; terminate/kill process, then join reader."""
        with self._lifecycle_lock:
            if self._process_halt is not None:
                self._process_halt.set()
            with self._condition:
                self._condition.notify_all()
            process = self._process
            if process is not None and process.pid is not None:
                process.join(self.shutdown_timeout)
                if process.is_alive():
                    process.terminate()
                    process.join(self.shutdown_timeout)
                if process.is_alive():
                    process.kill()
                    process.join(self.shutdown_timeout)
                if process.is_alive():
                    raise CameraError("Capture process did not exit after kill")
            if self._reader is not None:
                self._reader.join(self.shutdown_timeout)
                if self._reader.is_alive():
                    self._halt.set()
                    raise CameraError("Camera reader did not exit")
            self._halt.set()
            if self._connection is not None:
                self._connection.close()
            if process is not None:
                process.close()
            self._process = self._reader = self._connection = self._frame = None
            self._process_halt = None
            if self._cleanup_error is not None:
                raise self._cleanup_error

    def close(self):
        """Idempotently release capture, wake waiters and reject subsequent commands."""
        with self._lifecycle_lock:
            if self._closed:
                return
            self._closed = True
            self.stop()
