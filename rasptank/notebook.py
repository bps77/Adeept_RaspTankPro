"""Lazy optional notebook video and supervised drive/arm widgets."""

import threading

from .hardware._lifecycle import number, release


def _popout_image():
    """Create a lazy browser widget; optional anywidget is needed only for pop-out."""
    from pathlib import Path

    try:
        import anywidget
        from traitlets import Bytes, Unicode
    except ImportError as exc:
        raise ImportError(
            "Pop-out video requires anywidget in the notebook kernel: "
            "python -m pip install 'anywidget>=0.9,<1'") from exc

    class PopoutImage(anywidget.AnyWidget):
        """Share one JPEG payload between inline and browser pop-out presentation."""

        _esm = Path(__file__).with_name("camera_popout.js")
        value = Bytes().tag(sync=True)
        message = Unicode("Starting camera view").tag(sync=True)

    return PopoutImage()


class CameraView:
    """Rate-limited widget stream with visible errors; the camera is always borrowed."""

    def __init__(self, camera, *, fps=10, transform=None, on_disconnect=None,
                 popout=False):
        """Show JPEG widget and status; transform(Frame) optionally returns annotated JPEG."""
        number(fps, 1, 30, "fps")
        if type(popout) is not bool:
            raise ValueError("popout must be bool")
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
        self.popout = popout
        self._on_disconnect = on_disconnect
        self._cancel = threading.Event()
        self._error = None
        self._closed = False
        self.image = _popout_image() if popout else widgets.Image(format="jpeg")
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

    def _message(self, value):
        """Keep inline and optional pop-out status consistent, including errors/stops."""
        self.message.value = value
        if self.popout:
            self.image.message = value

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
                self._message(f"Streaming frame {sequence}")
                self._cancel.wait(1 / self.fps)
        except BaseException as exc:
            if self._cancel.is_set():
                return  # Expected cancellation while the owner shuts down capture.
            self._error = f"{type(exc).__name__}: {exc}"
            self._message(self._error)
            self._cancel.set()
            if self._on_disconnect is not None:
                try:
                    self._on_disconnect()
                except BaseException as stop_error:
                    self._error += f"; motor stop failed: {stop_error}"
                    self._message(self._error)

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
                self._message(self._error)
                raise RuntimeError(self._error)
        if self._error is None:
            self._message("Video stopped")

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


def show_camera(camera, *, fps=10, transform=None, previous=None, on_disconnect=None,
                popout=False):
    """Display video; optional browser pop-out needs a click, never autostart camera."""
    if previous is not None:
        previous.close()
    return CameraView(camera, fps=fps, transform=transform, on_disconnect=on_disconnect,
                      popout=popout)


class TankControls:
    """Borrow a supervised tank; drive clicks are leased, servo actions are single steps.

    No held-key worker or automatic lease renewal: losing the frontend cannot
    leave a continuous drive command. Closing controls stops motors, but does
    not close the tank or release servo holding torque.
    """

    def __init__(self, tank, *, speed=15):
        """Display disarmed controls, importing widgets only when explicitly requested."""
        number(speed, 0, 100, "speed")
        if type(speed) is not int:
            raise ValueError("speed must be an integer")
        state = tank.roaming_status()
        if state["state"] == "supervision_disabled":
            raise ValueError("Notebook controls require RaspTank(command_timeout=...)")
        try:
            import ipywidgets as widgets
            from IPython.display import display
        except ImportError as exc:
            raise ImportError(
                "Notebook controls require ipywidgets in the kernel: "
                "python -m pip install 'ipywidgets>=8.0'") from exc
        self.tank = tank
        self._closed = False
        self._widgets = []
        self.buttons = {}

        def create(factory, **kwargs):
            """Track every widget for complete, idempotent cleanup."""
            widget = factory(**kwargs)
            self._widgets.append(widget)
            return widget

        def button(key, label, action, style=""):
            """Bind a guarded callback; retained callbacks cannot act after close."""
            widget = create(widgets.Button, description=label, button_style=style)
            widget.on_click(lambda _: self._execute(action))
            self.buttons[key] = widget
            return widget

        self.message = create(widgets.Label, value="Disarmed; drive clicks expire automatically")
        self.drive_enabled = create(widgets.Checkbox, description="Enable driving", value=False)
        self.servo_enabled = create(widgets.Checkbox, description="Enable servos", value=False,
                                    disabled=tank.arm is None)
        self.speed = create(widgets.IntSlider, description="Speed (%)", min=0, max=100,
                            value=speed, continuous_update=False)
        self.step_size = create(widgets.IntSlider, description="Step ticks", min=1, max=25,
                                value=10, continuous_update=False)
        self.drive_enabled.observe(self._drive_armed, names="value")
        drive = create(widgets.VBox, children=[
            self.drive_enabled, self.speed,
            create(widgets.HBox, children=[
                button("forward", "Forward", lambda: self._drive("forward")),
                button("backward", "Backward", lambda: self._drive("backward"))]),
            create(widgets.HBox, children=[
                button("left", "Left", lambda: self._drive("turn_left")),
                button("stop", "STOP motors", self._stop, "danger"),
                button("right", "Right", lambda: self._drive("turn_right"))]),
        ])
        rows = [self.servo_enabled, self.step_size]
        self.sliders = {}
        if tank.arm is not None:
            from .arm import ARM_CHANNELS
            actions = (("look_left", "look_right"), ("arm_up", "arm_down"),
                       ("hand_up", "hand_down"), ("grab", "loosen"),
                       ("camera_up", "camera_down"))
            for joint, pair in zip(ARM_CHANNELS, actions):
                low, high = tank.arm.limits(joint)
                slider = create(widgets.IntSlider, description=f"{joint} ticks",
                                min=low, max=high, value=tank.arm.positions[joint],
                                continuous_update=False)
                self.sliders[joint] = slider
                rows.append(create(widgets.HBox, children=[
                    slider,
                    button(f"set_{joint}", "Apply", lambda joint=joint: self._set_joint(joint)),
                    *[button(action, action.replace("_", " ").title(),
                             lambda action=action: self._step_joint(action))
                      for action in pair],
                ]))
        else:
            rows.append(create(widgets.Label, value="Servos not configured on this tank"))
        self.widget = create(widgets.VBox, children=[
            drive, create(widgets.VBox, children=rows), self.message])
        try:
            display(self.widget)
        except BaseException as exc:
            try:
                self.close()
            except BaseException as cleanup_error:
                raise exc from cleanup_error
            raise

    def _execute(self, action):
        """Show callback failures and attempt a motor stop; never silently swallow faults."""
        if self._closed:
            self.message.value = "Controls closed; rerun the GUI cell"
            return
        try:
            action()
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            try:
                self.tank.stop()
            except Exception as stop_error:
                error += f"; motor stop failed: {stop_error}"
            self.drive_enabled.value = False
            self.servo_enabled.value = False
            self.message.value = error

    def _drive_armed(self, change):
        """Disarming stops immediately; arming alone never outputs a drive command."""
        if not change["new"] and not self._closed:
            self._execute(self._stop)

    def _drive(self, method):
        """Issue exactly one supervised command; do not renew it from a kernel loop."""
        if not self.drive_enabled.value:
            raise ValueError("Enable driving before issuing a drive command")
        getattr(self.tank, method)(self.speed.value)
        self.message.value = f"{method}: one watchdog-limited command (not continuous drive)"

    def _stop(self):
        """Cancel autonomy/manual drive; leave servo holding positions unchanged."""
        self.tank.stop()
        self.message.value = "Motors stopped; servos hold their last commanded pulses"

    def _servo_ready(self):
        """Require deliberate servo opt-in and stop drive/autonomy before changing joints."""
        if not self.servo_enabled.value:
            raise ValueError("Enable servos after checking linkage clearance and safe pulses")
        self.tank.stop()

    def _set_joint(self, joint):
        """Apply one slider value only on click; dragging/creating sliders never moves."""
        self._servo_ready()
        pulse = self.tank.arm.set_pulse(joint, self.sliders[joint].value)
        self.message.value = f"{joint}: commanded {pulse} ticks (not measured feedback)"

    def _step_joint(self, action):
        """Read current controller state for a bounded named step, then refresh sliders."""
        self._servo_ready()
        pulse = self.tank.arm.step(action, self.step_size.value)
        for joint, position in self.tank.arm.positions.items():
            self.sliders[joint].value = position
        self.message.value = f"{action}: commanded {pulse} ticks (not measured feedback)"

    def close(self):
        """Stop borrowed motors and disable/close all widgets, attempting every cleanup."""
        if self._closed:
            return
        self._closed = True
        self.drive_enabled.value = self.servo_enabled.value = False
        for button in self.buttons.values():
            button.disabled = True
        # The owning tank stops its supervisor before closing attached controls.
        actions = [] if self.tank._closed else [self.tank.stop]
        release([*actions, *(widget.close for widget in reversed(self._widgets))])
