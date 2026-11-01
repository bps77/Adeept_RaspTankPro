"""Lazy optional notebook JPEG presentation; cells return while video updates."""

import threading

from .hardware._lifecycle import number


class CameraView:
    """Rate-limited widget stream with visible errors; the camera is always borrowed."""

    def __init__(self, camera, *, fps=10, transform=None, on_disconnect=None):
        """Show JPEG widget and status; transform(Frame) optionally returns annotated JPEG."""
        number(fps, 1, 30, "fps")
        if transform is not None and not callable(transform):
            raise ValueError("transform must be callable")
        if on_disconnect is not None and not callable(on_disconnect):
            raise ValueError("on_disconnect must be callable")
        try:
            import ipywidgets as widgets
            from IPython.display import display
        except ImportError as exc:
            raise ImportError("Notebook video requires ipywidgets; see requirements-camera.txt") from exc
        self.camera, self.fps, self.transform = camera, fps, transform
        self._on_disconnect = on_disconnect
        self._cancel = threading.Event()
        self._error = None
        self._closed = False
        self.image = widgets.Image(format="jpeg")
        self.message = widgets.Label(value="Starting camera view")
        self.stop_button = widgets.Button(description="Stop video")
        self.stop_button.on_click(self._stop_clicked)
        self.widget = widgets.VBox([self.image, self.message, self.stop_button])
        display(self.widget)
        self._worker = threading.Thread(target=self._run, daemon=True,
                                         name="rasptank-notebook-video")
        self._worker.start()

    def _stop_clicked(self, button):
        """Stop presentation on user request, without closing the borrowed camera."""
        self.stop()

    def _run(self):
        """Publish fresh frames without a FIFO; faults stay visible and stop supervised motion."""
        sequence = None
        try:
            while not self._cancel.is_set():
                frame = self.camera.capture(timeout=1, after_sequence=sequence)
                sequence = frame.sequence
                jpeg = self.transform(frame) if self.transform else frame.jpeg
                if not isinstance(jpeg, bytes) or not jpeg:
                    raise ValueError("view transform must return nonempty JPEG bytes")
                if self._cancel.is_set():
                    break
                self.image.value = jpeg
                self.message.value = f"Streaming frame {sequence}"
                self._cancel.wait(1 / self.fps)
        except BaseException as exc:
            if self._cancel.is_set():
                return  # Expected cancellation while the owner shuts down capture.
            self._error = f"{type(exc).__name__}: {exc}"
            self.message.value = self._error
            self._cancel.set()
            if self._on_disconnect is not None:
                try:
                    self._on_disconnect()
                except BaseException as stop_error:
                    self._error += f"; motor stop failed: {stop_error}"
                    self.message.value = self._error

    def status(self):
        """Return running flag and explicit error text, not a success-shaped fallback."""
        return dict(running=self._worker.is_alive() and not self._cancel.is_set(),
                    error=self._error)

    def stop(self):
        """Bounded-join display worker; raise if a supplied transform/camera hangs."""
        self._cancel.set()
        if threading.current_thread() is not self._worker:
            self._worker.join(1.5)
            if self._worker.is_alive():
                self._error = "Camera view worker did not stop"
                self.message.value = self._error
                raise RuntimeError(self._error)
        if self._error is None:
            self.message.value = "Video stopped"

    def close(self):
        """Stop presentation and close widgets, leaving borrowed capture running."""
        if self._closed:
            return
        try:
            self.stop()
        finally:
            self._closed = True
            self.widget.close()
            self.image.close()
            self.message.close()
            self.stop_button.close()

    def __enter__(self):
        """Return this explicit streaming context handle."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close the view on notebook context exit."""
        self.close()


def show_camera(camera, *, fps=10, transform=None, previous=None, on_disconnect=None):
    """Display nonblocking video; close previous handle on cell reruns, never autostart camera."""
    if previous is not None:
        previous.close()
    return CameraView(camera, fps=fps, transform=transform, on_disconnect=on_disconnect)
