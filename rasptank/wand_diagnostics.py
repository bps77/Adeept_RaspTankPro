"""Diagnostics for the wand POC: benchmarks, scene checks, live views, synthetic wand.

Nothing here commands motors or servos. Camera use is limited to the caller's
CameraController (benchmark_camera) or to controllers sweep_modes() creates and
always closes.

Contents, in the order a notebook uses them:
  SyntheticWand    fake frame source, so everything works without a camera.
  scene_report     "what does the tracker see in this frame, and what to tune".
  benchmark_camera measured FPS, drops, latency and tracking cost for one camera.
  sweep_modes      the same benchmark across several size/FPS modes.
  format_results   readable text for benchmark rows.
  LiveView         ipywidgets view (camera overlay or white trace on black)
                   with a deliberate display delay and a Stop button.

Reading benchmark numbers: if measured FPS is far below the requested FPS the
camera pipeline is the limit; if process_ms is close to the frame time the
tracker is; a low detection_rate points at lighting/threshold, not speed.
"""

import math
import threading
import time

from .hardware._lifecycle import number
from .hardware.camera import CameraController
from .vision import _modules


class SyntheticWand:
    """Frame source drawing a bright tip on a figure-8, with optional lamp and dropouts.

    Lets the tracker, trace view and strokes be exercised with no camera.

    Instances are callable: `SyntheticWand()` returns one RGB uint8 frame per
    call, paced to `fps` by sleeping, so it can be passed to WandTracker as
    `source`. The tip is a 7x7 white square following a figure-8 whose period is
    period_frames; the optional lamp is a large static 100x100 white square
    that the area limit should reject; noise is low-level random background.
    """

    def __init__(self, size=(640, 480), fps=30, *, period_frames=90, lamp=True,
                 dropout=(60, 8), noise=6):
        """size is (width, height); noise is the maximum background gray level.

        dropout=(every, length) hides the tip for length frames every N frames; None disables.
        """
        self.size, self.fps, self.period = size, number(fps, 1, 240, "fps"), period_frames
        self.lamp, self.dropout, self.noise = lamp, dropout, noise
        self.n = 0
        self._next = None

    def __call__(self):
        """Return the next synthetic RGB frame, sleeping first to hold the frame rate."""
        _, np = _modules()
        now = time.monotonic()
        if self._next is not None and self._next > now:
            time.sleep(self._next - now)
        self._next = max(now, self._next or now) + 1 / self.fps
        self.n += 1
        width, height = self.size
        image = np.random.randint(0, self.noise + 1, (height, width, 3), dtype=np.uint8)
        if self.lamp:  # Large static light that must be rejected by max_area / jump gating.
            image[20:120, width - 120:width - 20] = 255
        hidden = self.dropout is not None and self.n % self.dropout[0] < self.dropout[1]
        if not hidden:
            phase = 2 * math.pi * self.n / self.period
            x = int(width / 2 + 0.3 * width * math.sin(phase))
            y = int(height / 2 + 0.3 * height * math.sin(2 * phase))
            image[max(0, y - 3):y + 4, max(0, x - 3):x + 4] = 255
        return image


def _percentile(values, q):
    """Nearest-rank percentile (q in 0..1) without numpy; NaN for no data."""
    values = sorted(values)
    return values[min(len(values) - 1, int(len(values) * q))] if values else float("nan")


def benchmark_camera(camera, tracker=None, seconds=3, timeout=1):
    """Measure a started CameraController (and optionally tracker.process cost) for seconds.

    latency_ms is frame age on arrival (capture start to receipt, including JPEG
    encode and IPC); dropped counts skipped sequence numbers.

    camera: started CameraController; borrowed, never closed here.
    tracker: optional WandTracker; when given, every frame is also processed so
        the numbers include real tracking cost and detection rate. Note that
        this feeds the tracker, changing its position/trajectory.
    seconds: measuring window (0.5-600). timeout: per-frame wait in seconds.

    Returns a dict: size, fps (frames received per second, which is the real
    capture rate), frames, dropped, timeouts, latency_ms_mean/p95 and, with a
    tracker, process_ms_mean/p95 and detection_rate (fraction of frames with a
    position). Timeouts are counted rather than raised so a stalling camera
    still yields a report.
    """
    _, np = _modules()
    number(seconds, 0.5, 600, "seconds")
    sequence, frames, dropped, timeouts, detections = None, 0, 0, 0, 0
    ages, costs = [], []
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        try:
            frame = camera.capture(timeout=timeout, after_sequence=sequence)
        except TimeoutError:
            timeouts += 1
            continue
        arrived = time.monotonic()
        if sequence is not None:
            dropped += frame.sequence - sequence - 1
        sequence, frames = frame.sequence, frames + 1
        ages.append(1000 * (arrived - frame.timestamp))
        if tracker is not None:
            rgb = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)
            begin = time.perf_counter()
            detections += tracker.process(rgb) is not None
            costs.append(1000 * (time.perf_counter() - begin))
    elapsed = time.monotonic() - started
    result = {"size": (frame.width, frame.height) if frames else None,
              "fps": frames / elapsed, "frames": frames, "dropped": dropped,
              "timeouts": timeouts,
              "latency_ms_mean": sum(ages) / len(ages) if ages else float("nan"),
              "latency_ms_p95": _percentile(ages, 0.95)}
    if tracker is not None:
        result.update(process_ms_mean=sum(costs) / len(costs) if costs else float("nan"),
                      process_ms_p95=_percentile(costs, 0.95),
                      detection_rate=detections / frames if frames else 0.0)
    return result


def sweep_modes(modes, tracker_factory, seconds=3, **camera_options):
    """Benchmark (size, fps) modes one camera at a time; the caller's camera must be closed first.

    Each mode gets its own CameraController, always closed. Failures are
    reported per mode instead of raised, so one bad mode does not hide the rest.

    modes: iterable of ((width, height), fps). tracker_factory: zero-argument
    callable returning a fresh WandTracker per mode (so runs do not share
    state). camera_options: extra CameraController keyword arguments.
    The camera is exclusive, so the notebook's own camera must be closed first.
    Returns a list of benchmark_camera dicts, each with "requested": (size, fps)
    and, on failure, "error" (and "cleanup_error" if closing also failed).
    The size the camera actually delivers is in "size"; Picamera2 may pick the
    nearest sensor mode, so compare it with the requested one.
    """
    results = []
    for size, fps in modes:
        row = {"requested": (size, fps)}
        camera = None
        try:
            camera = CameraController(size=size, fps=fps, **camera_options)
            camera.start()
            row.update(benchmark_camera(camera, tracker_factory(), seconds))
        except Exception as exc:
            row["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            if camera is not None:
                try:
                    camera.close()
                except Exception as exc:
                    row["cleanup_error"] = str(exc)
        results.append(row)
    return results


def format_results(results):
    """One line per benchmark row for printing.

    Accepts the list returned by sweep_modes() (rows need "requested"). A row
    with "error" prints as FAILED; latency is shown as mean/p95 in ms and the
    process/detected columns appear only when a tracker was benchmarked.
    """
    lines = []
    for row in results:
        size, fps = row["requested"]
        head = f"{size[0]}x{size[1]}@{fps}"
        if "error" in row:
            lines.append(f"{head:>12}  FAILED  {row['error']}")
            continue
        text = (f"{head:>12}  {row['fps']:5.1f} FPS  dropped {row['dropped']:>3}  "
                f"timeouts {row['timeouts']}  latency {row['latency_ms_mean']:.0f}/"
                f"{row['latency_ms_p95']:.0f} ms (mean/p95)")
        if "process_ms_mean" in row:
            text += (f"  process {row['process_ms_mean']:.1f}/{row['process_ms_p95']:.1f} ms"
                     f"  detected {100 * row['detection_rate']:.0f}%")
        lines.append(text)
    return "\n".join(lines)


def scene_report(rgb, tracker):
    """Explain what the current threshold/area settings see in one frame, with a hint.

    rgb: one RGB frame. tracker: supplies threshold, area limits and the mask
    pipeline; its motion baseline and trajectory are not touched.
    Returns a dict: brightest (max gray 0-255), percent_over (share of pixels
    above threshold), candidates ([(x, y, area)] as the tracker would see
    them) and hint (plain-language next tuning step). The checks run in order:
    nothing reaches the threshold, too much of the image is bright (over 2%),
    bright pixels but no blob fits the area limits, too many candidates (over
    3), otherwise "Looks good".
    """
    cv2, np = _modules()
    gray, mask = tracker.mask(rgb, update_motion=False)
    blobs = tracker.candidates(mask)
    over = float(np.count_nonzero(mask)) / mask.size
    brightest = int(gray.max())
    if brightest < tracker.threshold:
        hint = (f"No pixel reaches threshold {tracker.threshold} (brightest {brightest}): "
                "lower THRESHOLD or use a brighter tip / dimmer room.")
    elif over > 0.02:
        hint = (f"{100 * over:.1f}% of the image is above threshold: raise THRESHOLD, dim the "
                "room, or shrink exposure.")
    elif not blobs:
        hint = "Bright pixels exist but no blob fits MIN_AREA..MAX_AREA: adjust the area limits."
    elif len(blobs) > 3:
        hint = f"{len(blobs)} candidates: expect hopping; raise THRESHOLD or enable MOTION_THRESHOLD."
    else:
        hint = "Looks good."
    return {"brightest": brightest, "percent_over": 100 * over, "candidates": blobs, "hint": hint}


class LiveView:
    """Notebook widget refreshing a tracker view every `delay` seconds, with a Stop button.

    mode "overlay" shows the camera image with trail; "trace" shows the white
    trail on black. delay slows the display so motion stays visible; it never
    slows tracking. seconds auto-stops the view. The tracker is borrowed.

    Creating a LiveView immediately displays the widget and starts a daemon
    refresh thread; stop() (or the Stop button) ends it. Stopping the view never
    stops the tracker, and stopping the tracker never closes the view: if the
    tracker dies, the label shows its error. Requires ipywidgets/IPython,
    imported lazily so the module imports without them.
    """

    def __init__(self, tracker, *, mode="trace", delay=0.05, seconds=None):
        """mode: "trace" or "overlay"; delay: seconds between refreshes (0.01-5).

        seconds: optional auto-stop time. Raises ValueError for bad arguments
        and ImportError if ipywidgets is missing.
        """
        if mode not in ("overlay", "trace"):
            raise ValueError("mode must be 'overlay' or 'trace'")
        number(delay, 0.01, 5, "delay")
        if seconds is not None:
            number(seconds, 0.5, 3600, "seconds")
        try:
            import ipywidgets as widgets
            from IPython.display import display
        except ImportError as exc:
            raise ImportError("LiveView requires ipywidgets; see requirements-camera.txt") from exc
        self.tracker, self.mode, self.delay, self.seconds = tracker, mode, delay, seconds
        self._cancel = threading.Event()
        self.image = widgets.Image(format="jpeg")
        self.label = widgets.Label("Waiting for frames")
        button = widgets.Button(description="Stop view")
        button.on_click(lambda _: self.stop())
        display(widgets.VBox([self.image, self.label, button]))
        self._thread = threading.Thread(target=self._run, daemon=True, name="wand-view")
        self._thread.start()

    def _run(self):
        """Refresh loop: wait delay, then push the latest JPEG to the widget until stopped."""
        deadline = None if self.seconds is None else time.monotonic() + self.seconds
        while not self._cancel.wait(self.delay):
            if deadline is not None and time.monotonic() > deadline:
                break
            if self.tracker.error is not None:
                self.label.value = f"Tracker stopped: {self.tracker.error}"
                return
            data = self.tracker.jpeg(trace=self.mode == "trace")
            if data:
                self.image.value = data
                self.label.value = (f"{'tracking' if self.tracker.is_tracking else 'lost'} "
                                    f"position={self.tracker.position} fps={self.tracker.fps:.0f}")
        self.label.value = "View stopped"

    def stop(self, timeout=2):
        """Stop refreshing; idempotent.

        Safe to call from the Stop button (which runs on another thread) and
        from the refresh thread itself; it never joins the current thread.
        """
        self._cancel.set()
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout)

    def replay(self, stroke, size, delay=0.03):
        """Redraw a finished stroke point by point in white on black, slowly, until done or stopped.

        stroke: (N, 2) points, e.g. tracker.last_stroke. size: (width, height)
        of the canvas. delay: seconds between added points (slow motion).
        Blocks the calling cell until finished; stop() from another thread
        (the Stop button) interrupts it. Stops the live refresh first so the two
        do not fight over the image, and it is not restarted afterward.
        """
        cv2, np = _modules()
        self.stop()
        self._cancel.clear()
        points = np.asarray(stroke, dtype=np.int32).reshape(-1, 2)
        for end in range(1, len(points) + 1):
            if self._cancel.wait(delay):
                break
            canvas = np.zeros((size[1], size[0], 3), dtype=np.uint8)
            if end > 1:
                cv2.polylines(canvas, [points[:end].reshape(-1, 1, 2)], False, (255, 255, 255), 2)
            cv2.circle(canvas, tuple(int(v) for v in points[end - 1]), 6, (255, 255, 255), -1)
            self.image.value = cv2.imencode(".jpg", canvas)[1].tobytes()
        self.label.value = f"Replayed {len(points)} points"
