"""Camera-independent bright-blob wand-tip tracker with trajectory utilities.

The tracker only needs RGB ndarrays, so frames may come from the RaspTank
camera, a USB camera, a NoIR camera or a recorded video.

Pipeline, one frame at a time (see WandTracker.process):

    RGB frame -> grayscale -> Gaussian blur -> threshold -> optional motion
    filter -> connected components -> area filter -> pick one candidate ->
    (x, y) -> bounded trajectory history -> optional finished-stroke callback

Layers, from hardware-specific to hardware-free:

    CameraController (existing wrapper, unchanged)
        -> camera_source()   adapter: Frame -> RGB ndarray
        -> WandTracker       frames in, position/trajectory out
        -> trajectory helpers (path_length, bounding_box, normalize,
           resample, direction_changes) that work on plain (N, 2) arrays

Coordinates are pixels of the frame given to the tracker: x grows to the right,
y grows downward, origin is the top-left corner. Nothing in this module moves
motors or servos, and nothing here opens or closes a camera.

OpenCV and numpy are imported lazily through vision._modules(), so importing
this module never requires them; calling a function that needs them does.
"""

from collections import deque
import threading
import time

from .hardware._lifecycle import number
from .vision import _modules


def camera_source(camera, timeout=1):
    """Adapt a started CameraController to a read() -> RGB ndarray callable.

    This is the only place the tracker touches the camera API. Swap it for any
    other zero-argument callable to feed USB, NoIR or video frames instead.

    camera: a CameraController on which start() was already called. It is
        borrowed: this adapter never starts or closes it.
    timeout: seconds to wait for a new frame; on expiry the camera raises
        TimeoutError, which WandTracker treats as a tolerated stall.

    The returned read() asks only for frames newer than the last one it
    returned (after_sequence), so a slow consumer skips frames instead of
    processing stale ones, which keeps latency low. The array is a read-only
    view over the frame bytes with shape (height, width, 3), dtype uint8, RGB.
    """
    _, np = _modules()
    state = {"sequence": None}

    def read():
        """Return the next RGB frame; raises TimeoutError if none arrives in time."""
        frame = camera.capture(timeout=timeout, after_sequence=state["sequence"])
        state["sequence"] = frame.sequence
        return np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)

    return read


def path_length(trajectory):
    """Total polyline length of a trajectory, in the units of its coordinates.

    trajectory: anything numpy can view as (N, 2) points. Returns a float;
    0.0 for fewer than two points. Useful to reject accidental tiny strokes.
    """
    _, np = _modules()
    points = np.asarray(trajectory, dtype=float).reshape(-1, 2)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points) > 1 else 0.0


def bounding_box(trajectory):
    """Return the axis-aligned bounding box as (x, y, width, height).

    (x, y) is the top-left corner (smallest x and y). Returns None for an
    empty trajectory. A single point gives a zero-size box.
    """
    _, np = _modules()
    points = np.asarray(trajectory, dtype=float).reshape(-1, 2)
    if not len(points):
        return None
    low, high = points.min(axis=0), points.max(axis=0)
    return (float(low[0]), float(low[1]), float(high[0] - low[0]), float(high[1] - low[1]))


def normalize(trajectory):
    """Translate to the origin and scale the longest bounding-box side to 1.

    Position and size independent, ready for a future gesture matcher.

    Both axes are divided by the same factor, so the aspect ratio is kept and
    the result lies in [0, 1] on the longer axis. An empty trajectory is
    returned as an empty (0, 2) array; a stroke that never moved (all points
    identical) is returned as all zeros rather than dividing by zero.
    """
    _, np = _modules()
    points = np.asarray(trajectory, dtype=float).reshape(-1, 2)
    if not len(points):
        return points
    points = points - points.min(axis=0)
    scale = points.max()
    return points / scale if scale > 0 else points


def resample(trajectory, count=32):
    """Resample a stroke to count points evenly spaced along its path (for gesture matching).

    Raw strokes have a variable number of points that depend on frame rate and
    wand speed; template matchers need a fixed-length, speed-independent form.
    Points are interpolated linearly by arc length (cumulative distance), so
    the first and last points are preserved.

    Edge cases: raises ValueError if count < 2; an empty input is returned
    unchanged; one point, or a stroke with zero length, is repeated count times.
    """
    _, np = _modules()
    points = np.asarray(trajectory, dtype=float).reshape(-1, 2)
    if count < 2:
        raise ValueError("count must be at least 2")
    if len(points) < 2:
        return np.repeat(points[:1], count, axis=0) if len(points) else points
    distance = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    if distance[-1] == 0:
        return np.repeat(points[:1], count, axis=0)
    targets = np.linspace(0, distance[-1], count)
    return np.column_stack([np.interp(targets, distance, points[:, 0]),
                            np.interp(targets, distance, points[:, 1])])


def direction_changes(trajectory, angle_degrees=45, min_step=3):
    """Count turns sharper than angle_degrees, ignoring steps shorter than min_step.

    A "turn" is a change of heading between two consecutive significant steps.
    min_step filters out centroid jitter (sub-pixel wobble of a stationary tip)
    that would otherwise register as random turns. Returns an int; 0 when
    fewer than two significant steps exist.
    """
    _, np = _modules()
    points = np.asarray(trajectory, dtype=float).reshape(-1, 2)
    steps = np.diff(points, axis=0)
    steps = steps[np.linalg.norm(steps, axis=1) >= min_step]
    if len(steps) < 2:
        return 0
    headings = np.arctan2(steps[:, 1], steps[:, 0])
    # Wrap the heading difference into [-pi, pi) so 350 -> 10 degrees is a 20 degree turn.
    turns = np.abs((np.diff(headings) + np.pi) % (2 * np.pi) - np.pi)
    return int((turns > np.radians(angle_degrees)).sum())


class WandTracker:
    """Track one bright blob (the wand tip) across frames and keep its trajectory.

    Two ways to drive it:
      * process(rgb): synchronous, one frame per call (tests, video files).
      * start()/stop(): a daemon thread that pulls frames from `source`.

    Public state (safe to read from any thread):
      position     (x, y) of the current tip, or None when not tracking.
      is_tracking  True while a target is locked.
      trajectory   copy of recent points as an (N, 2) float array.
      fps          smoothed processed-frames-per-second.
      frame_size   (width, height) of the last processed frame, or None.
      last_stroke  most recent finished stroke, kept after tracking is lost.
      error        exception that stopped the thread, else None.
      callback_error  last exception raised by on_stroke, else None.

    Candidate selection: while acquiring, the largest blob in the area limits
    wins; once locked, the blob nearest the previous position within max_jump
    wins, so a second bright object elsewhere does not steal the lock. A
    mid-size lamp may still be locked onto while acquiring (known limitation).

    Lock loss: the target is dropped after more than max_missed_frames
    consecutive frames without a candidate. The finished trajectory stays
    available (and is rendered) until the next acquisition clears it.

    Threading: _lock guards published state, _process_lock serializes frame
    processing (thread vs. direct process()/clear() calls) and _lifecycle_lock
    serializes start()/stop(). The tracker never closes its frame source.
    """

    def __init__(self, source=None, *, threshold=200, min_area=20, max_area=5000,
                 max_trajectory_points=100, max_jump=120, max_missed_frames=5,
                 blur=5, motion_threshold=None, max_source_failures=10,
                 on_stroke=None, min_stroke_points=8):
        """Validate and store tuning parameters; no frame is read here.

        source: zero-argument callable returning an RGB ndarray (or None for
            "no frame"); may be None if you only call process() yourself.
        threshold: 1-255 gray level above which a pixel counts as bright.
        min_area / max_area: accepted blob size in pixels. max_area rejects
            large lamps and windows; min_area rejects noise specks.
        max_trajectory_points: history length (int >= 2); old points drop off.
        max_jump: farthest a locked target may move between frames, in pixels.
        max_missed_frames: tolerated consecutive frames with no candidate
            before the lock is dropped (bridges flicker and motion blur).
        blur: Gaussian kernel size, 0 (off) or odd. Merges speckle into a blob.
        motion_threshold: None disables the motion filter. A value (e.g. 25)
            keeps only bright pixels that also changed since the last frame,
            ignoring static lights. The first frame yields no candidates.
        max_source_failures: consecutive timeouts/empty reads after which the
            thread stops and `error` is set.
        on_stroke: callable(trajectory) invoked from the tracking thread with
            the finished (N, 2) stroke when the wand is lost after at least
            min_stroke_points points. Exceptions are stored in callback_error
            and never stop tracking.
        min_stroke_points: shortest trajectory that counts as a stroke.

        Raises ValueError for any out-of-range or wrongly typed argument.
        """
        number(threshold, 1, 255, "threshold")
        number(min_area, 1, 10 ** 8, "min_area")
        number(max_area, min_area, 10 ** 8, "max_area")
        number(max_jump, 1, 10 ** 5, "max_jump")
        if source is not None and not callable(source):
            raise ValueError("source must be a callable returning an RGB ndarray or None")
        if type(max_trajectory_points) is not int or max_trajectory_points < 2:
            raise ValueError("max_trajectory_points must be an integer >= 2")
        if type(max_missed_frames) is not int or max_missed_frames < 0:
            raise ValueError("max_missed_frames must be a nonnegative integer")
        if type(blur) is not int or blur < 0 or (blur and blur % 2 == 0):
            raise ValueError("blur must be 0 or an odd integer")
        if motion_threshold is not None:
            number(motion_threshold, 1, 255, "motion_threshold")
        if type(max_source_failures) is not int or max_source_failures < 1:
            raise ValueError("max_source_failures must be a positive integer")
        if on_stroke is not None and not callable(on_stroke):
            raise ValueError("on_stroke must be callable")
        if type(min_stroke_points) is not int or min_stroke_points < 2:
            raise ValueError("min_stroke_points must be an integer >= 2")
        self.on_stroke, self.min_stroke_points = on_stroke, min_stroke_points
        self.last_stroke = None
        self.callback_error = None
        self._pending = []
        self.max_source_failures = max_source_failures
        self.source = source
        self.threshold, self.min_area, self.max_area = threshold, min_area, max_area
        self.max_jump, self.max_missed_frames = max_jump, max_missed_frames
        self.blur, self.motion_threshold = blur, motion_threshold
        self._history = deque(maxlen=max_trajectory_points)
        self.position = None
        self.is_tracking = False
        self.fps = 0.0
        self.error = None
        self._missed = 0
        self._previous_gray = None
        self._last_time = None
        self.frame_size = None
        self._lock = threading.RLock()
        self._process_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._thread = None
        self._cancel = threading.Event()
        self._latest = None

    @property
    def trajectory(self):
        """Copy of recent (x, y) points as an (N, 2) float array."""
        _, np = _modules()
        with self._lock:
            return np.array(self._history, dtype=float).reshape(-1, 2)

    def clear(self):
        """Forget trajectory and current lock; the next bright blob starts a new stroke.

        Also blanks the cached overlay so render()/render_trace() do not keep
        drawing the old trail. last_stroke is intentionally kept.
        """
        with self._process_lock, self._lock:
            self._history.clear()
            self.position, self.is_tracking, self._missed = None, False, 0
            if self._latest is not None:
                self._latest = (self._latest[0], None, False, self.fps, self.trajectory)

    def mask(self, rgb, update_motion=True):
        """Return (gray, binary mask) after blur, threshold and optional motion filter.

        update_motion=False leaves the motion baseline alone, for diagnostics,
        so inspecting a frame in a notebook does not disturb live tracking.

        gray is the blurred grayscale image (uint8, H x W); mask is uint8 with
        255 where a pixel is bright (and, if enabled, also moving). With the
        motion filter on, the very first frame (or a size change) has no
        baseline to compare against and returns an all-zero mask.
        """
        cv2, _ = _modules()
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        if self.blur:
            gray = cv2.GaussianBlur(gray, (self.blur, self.blur), 0)
        mask = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)[1]
        if self.motion_threshold is not None:
            previous = self._previous_gray
            if update_motion:
                self._previous_gray = gray
            if previous is not None and previous.shape == gray.shape:
                moving = cv2.dilate(cv2.threshold(cv2.absdiff(gray, previous),
                                                  self.motion_threshold, 255,
                                                  cv2.THRESH_BINARY)[1], None, iterations=2)
                mask = cv2.bitwise_and(mask, moving)
            else:
                mask[:] = 0
        return gray, mask

    def candidates(self, mask):
        """Return [(x, y, area)] centroids of blobs within the area limits.

        Uses 8-connected components of the binary mask; label 0 is the
        background and is skipped. x/y are the blob centroid in pixels (float,
        sub-pixel) and area is its pixel count. Order is not meaningful.
        """
        cv2, _ = _modules()
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        return [(float(centroids[i][0]), float(centroids[i][1]), int(stats[i][cv2.CC_STAT_AREA]))
                for i in range(1, count)
                if self.min_area <= stats[i][cv2.CC_STAT_AREA] <= self.max_area]

    def _select(self, blobs):
        """Nearest blob within max_jump while tracking; otherwise the largest blob.

        Returns None when there is no acceptable blob. Distances are compared
        squared to avoid a square root per candidate.
        """
        if self.position is not None:
            blobs = [b for b in blobs
                     if ((b[0] - self.position[0]) ** 2 + (b[1] - self.position[1]) ** 2)
                     <= self.max_jump ** 2]
            return min(blobs, key=lambda b: (b[0] - self.position[0]) ** 2
                       + (b[1] - self.position[1]) ** 2, default=None)
        return max(blobs, key=lambda b: b[2], default=None)

    def process(self, rgb):
        """Update state from one RGB frame and return the (x, y) position or None.

        rgb: uint8 ndarray shaped (height, width, 3) in RGB order (not BGR).
        Steps: detect candidates, pick one, update position/trajectory/FPS,
        then deliver any finished stroke to on_stroke (outside the state lock).
        A miss does not immediately clear the position: it stays at the last
        value until max_missed_frames is exceeded. Returns the current position,
        so None means "not tracking".
        """
        with self._process_lock:
            now = time.monotonic()
            _, mask = self.mask(rgb)
            blob = self._select(self.candidates(mask))
            with self._lock:
                self.frame_size = (rgb.shape[1], rgb.shape[0])
                if self._last_time is not None and now > self._last_time:
                    # Exponential moving average so the displayed FPS is stable.
                    instant = 1 / (now - self._last_time)
                    self.fps = 0.9 * self.fps + 0.1 * instant if self.fps else instant
                self._last_time = now
                if blob is not None:
                    if not self.is_tracking and self.position is None:
                        self._history.clear()  # New acquisition starts a fresh stroke.
                    self.position = (blob[0], blob[1])
                    self._history.append(self.position)
                    self.is_tracking, self._missed = True, 0
                else:
                    self._register_miss()
                self._latest = (rgb, self.position, self.is_tracking, self.fps, self.trajectory)
            self._fire_strokes()
            return self.position

    def _register_miss(self):
        """Count a frame without a target; caller holds _lock.

        Once misses exceed max_missed_frames the lock is dropped and, if the
        stroke was long enough, it is saved as last_stroke and queued for
        on_stroke. A stroke still in progress when stop() is called is not
        emitted, because the wand was never observed to end it.
        """
        self._missed += 1
        if self._missed > self.max_missed_frames:
            # Keep the finished stroke visible; the next acquisition clears it.
            if self.is_tracking and len(self._history) >= self.min_stroke_points:
                self.last_stroke = self.trajectory
                self._pending.append(self.last_stroke)
            self.position, self.is_tracking = None, False

    def _fire_strokes(self):
        """Deliver finished strokes outside the state lock; callback errors never kill tracking."""
        with self._lock:
            pending, self._pending = self._pending, []
        for stroke in pending:
            if self.on_stroke is not None:
                try:
                    self.on_stroke(stroke)
                except Exception as exc:
                    self.callback_error = exc

    def render(self, rgb=None):
        """Return an RGB overlay (trajectory, point, status, FPS) of the latest or given frame.

        rgb: frame to draw on; defaults to the last frame the tracker saw.
        Returns a new array (the input is never modified), or None if no frame
        has been seen yet. Green polyline = trajectory, blue ring = current
        tip, yellow text = tracking status, FPS and point count. Colors are in
        RGB order.
        """
        cv2, np = _modules()
        with self._lock:
            latest = self._latest
        if rgb is None:
            if latest is None:
                return None
            rgb = latest[0]
        _, position, tracking, fps, points = latest if latest else (None, None, False, 0.0, np.empty((0, 2)))
        image = np.ascontiguousarray(rgb).copy()
        if len(points) > 1:
            cv2.polylines(image, [points.astype(np.int32).reshape(-1, 1, 2)], False, (0, 255, 0), 2)
        if position is not None:
            cv2.circle(image, (int(position[0]), int(position[1])), 8, (255, 0, 0), 2)
        lines = [f"Tracking: {'YES' if tracking else 'NO'}", f"FPS: {fps:.0f}",
                 f"Points: {len(points)}"]
        for row, text in enumerate(lines):
            cv2.putText(image, text, (8, 20 + 20 * row), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 0), 2)
        return image

    def render_trace(self, size=None):
        """White trail and tip on a black canvas (no camera image); None until a frame is seen.

        size: (width, height) of the canvas; defaults to the frame size. This
        view shows only what the tracker believes, which makes tracking
        problems (jumps, dropouts, jitter) easier to see than on a busy image.
        """
        cv2, np = _modules()
        with self._lock:
            latest, size = self._latest, size or self.frame_size
        if latest is None or size is None:
            return None
        _, position, tracking, fps, points = latest
        canvas = np.zeros((size[1], size[0], 3), dtype=np.uint8)
        if len(points) > 1:
            cv2.polylines(canvas, [points.astype(np.int32).reshape(-1, 1, 2)], False,
                          (255, 255, 255), 2)
        if position is not None:
            cv2.circle(canvas, (int(position[0]), int(position[1])), 6, (255, 255, 255), -1)
        cv2.putText(canvas, f"{'TRACKING' if tracking else 'lost'}  {fps:.0f} FPS  {len(points)} pts",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        return canvas

    def jpeg(self, trace=False):
        """Latest frame overlay (or black-canvas trace) as JPEG bytes; None before any frame.

        Convenient for ipywidgets.Image(format="jpeg"). The image is RGB, so it
        is converted to BGR before OpenCV encodes it.
        """
        cv2, _ = _modules()
        image = self.render_trace() if trace else self.render()
        if image is None:
            return None
        return cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))[1].tobytes()

    def start(self):
        """Run process() on source frames in a daemon thread until stop() or repeated source errors.

        Idempotent: calling it while already running returns immediately.
        Resets FPS and the last error so a restart reports fresh numbers.
        Raises ValueError if no source was provided. Returns self for chaining.
        """
        if self.source is None:
            raise ValueError("Provide a frame source before start()")
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return self
            self._cancel.clear()
            self.error = None
            with self._lock:
                self.fps, self._last_time = 0.0, None
            self._thread = threading.Thread(target=self._run, daemon=True, name="wand-tracker")
            self._thread.start()
        return self

    def _run(self):
        """Thread body: read frames, tolerate stalls, record a fatal error in self.error.

        A TimeoutError or None frame counts as a miss (so the lock decays
        normally) and as a source failure; max_source_failures in a row is
        fatal. Any other exception is fatal immediately. On exit the position
        is cleared so nothing downstream (e.g. the arm follower) keeps acting
        on a stale target.
        """
        failures = 0
        try:
            while not self._cancel.is_set():
                try:
                    rgb = self.source()
                except TimeoutError:
                    rgb = None  # Transient stall: tolerated up to max_source_failures.
                if rgb is None:
                    failures += 1
                    with self._process_lock, self._lock:
                        self._register_miss()
                        self._latest = None if self._latest is None else (
                            self._latest[0], self.position, self.is_tracking, self.fps,
                            self.trajectory)
                    self._fire_strokes()
                    if failures >= self.max_source_failures:
                        raise RuntimeError(f"Frame source failed {failures} consecutive times")
                    continue
                failures = 0
                self.process(rgb)
        except BaseException as exc:
            self.error = exc  # Camera ownership stays with the caller.
        finally:
            with self._process_lock, self._lock:
                self.position, self.is_tracking = None, False

    def stop(self, timeout=2):
        """Stop the thread; never closes the (borrowed) frame source.

        Safe to call when not running. Raises RuntimeError if the thread is
        still alive after timeout seconds (for example a source blocked
        longer than that); the caller then still owns the source and should
        close it, which unblocks the thread.
        """
        with self._lifecycle_lock:
            self._cancel.set()
            if self._thread is not None:
                self._thread.join(timeout)
                if self._thread.is_alive():
                    raise RuntimeError("Wand tracker thread did not stop")
                self._thread = None

    @property
    def running(self):
        """True while the tracking thread is alive."""
        return self._thread is not None and self._thread.is_alive()

    def __enter__(self):
        """Context-manager entry; does not start the thread (call start())."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Stop the thread on leaving the with-block, even after an exception."""
        self.stop()
