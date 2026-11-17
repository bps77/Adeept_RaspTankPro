"""Opt-in demo: nudge one RobotArm joint pair toward the tracked wand tip.

Proportional, rate-limited and disarmed by default. It only borrows the arm and
never moves it when the wand is not tracked.

Why pan and camera: on the RaspTank the camera rides on the arm, so the arm's
left/right joint ("pan", servo channel 0) turns the view horizontally and the
"camera" joint (channel 4) tilts it. Keeping the tip near the frame center
therefore needs one correction per image axis.

Safety properties (all enforced in code, covered by tests with a mock PWM):
  * disarmed at construction; start() alone never moves anything;
  * moves only while the tracker reports a live target;
  * small steps (max_ticks) at a limited rate (rate_hz) with a center dead band;
  * joint limits raise ValueError in RobotArm; they are counted, not forced;
  * any other (hardware) error disarms the follower and is kept in `error`.
The caller still owns the arm, its power and its cleanup.
"""

import threading

from .hardware._lifecycle import number


class WandArmFollower:
    """Poll a WandTracker and step arm joints so the target drifts toward frame center.

    x_joint/y_joint name RobotArm joints (default pan/camera). Image-right and
    image-down map to look_right/camera_down unless invert_* is set. This
    matches the legacy color tracker (server/FPV.py: target left -> lookleft
    on channel 0, target above -> up on channel 4, with server/servo.py
    directions equal to config.SERVO_DIRECTIONS). Not yet checked on hardware.

    Control law per axis, each tick: offset = position / frame_extent - 0.5
    (range -0.5..0.5, 0 at the center). Inside the dead band nothing happens;
    outside it the step is round(|offset| * gain) ticks, clamped to
    1..max_ticks. Larger offsets therefore move faster, but never more than
    max_ticks per tick.

    Attributes:
      armed       must be set True by the caller before anything can move.
      limit_hits  how many steps were refused because a joint hit its limit.
      error       exception that stopped the thread (also disarms), else None.
    """

    # Per joint: (action when the target is right of / below center, otherwise).
    ACTIONS = {"pan": ("look_right", "look_left"), "camera": ("camera_down", "camera_up")}

    def __init__(self, tracker, arm, *, x_joint="pan", y_joint="camera", deadband=0.1,
                 gain=20, max_ticks=5, rate_hz=5, invert_x=False, invert_y=False):
        """Validate settings; nothing moves and no thread starts here.

        tracker: a WandTracker (only position, is_tracking, frame_size are read).
        arm: a RobotArm-like object with step(action, ticks); borrowed.
        x_joint / y_joint: joint driven by horizontal / vertical image offset,
            "pan", "camera" or None to leave that axis alone.
        deadband: fraction of the frame (0-0.5) around center with no motion.
        gain: ticks per unit of offset (offset is at most 0.5).
        max_ticks: integer 1-20, the largest single step.
        rate_hz: control ticks per second (0.5-20).
        invert_x / invert_y: flip the direction if the camera turns away from
            the wand on your robot.
        Raises ValueError on an unknown joint or out-of-range value.
        """
        for joint in (x_joint, y_joint):
            if joint is not None and joint not in self.ACTIONS:
                raise ValueError(f"joint must be None or one of {tuple(self.ACTIONS)}")
        self.tracker, self.arm = tracker, arm
        self.x_joint, self.y_joint = x_joint, y_joint
        self.deadband = number(deadband, 0, 0.5, "deadband")
        self.gain = number(gain, 1, 200, "gain")
        if type(max_ticks) is not int or not 1 <= max_ticks <= 20:
            raise ValueError("max_ticks must be an integer in 1..20")
        self.max_ticks = max_ticks
        self.rate_hz = number(rate_hz, 0.5, 20, "rate_hz")
        self.invert = {"pan": bool(invert_x), "camera": bool(invert_y)}
        self.armed = False
        self.limit_hits = 0
        self.error = None
        self._cancel = threading.Event()
        self._thread = None

    def update(self):
        """Do one control tick; return the list of (action, ticks) performed.

        Returns [] (and moves nothing) unless armed, tracking, and a frame
        size is known. A refused limit step is counted in limit_hits and
        skipped; other exceptions propagate to the caller (the thread loop
        turns them into a disarm). Callable directly for deterministic tests.
        """
        size = self.tracker.frame_size
        position = self.tracker.position
        if not self.armed or not self.tracker.is_tracking or position is None or size is None:
            return []
        performed = []
        for joint, value, extent in ((self.x_joint, position[0], size[0]),
                                     (self.y_joint, position[1], size[1])):
            if joint is None:
                continue
            offset = value / extent - 0.5
            if abs(offset) <= self.deadband:
                continue
            ticks = min(self.max_ticks, max(1, round(abs(offset) * self.gain)))
            positive = (offset > 0) != self.invert[joint]
            action = self.ACTIONS[joint][0 if positive else 1]
            try:
                self.arm.step(action, ticks)
            except ValueError:
                self.limit_hits += 1  # Joint limit reached: never clipped or forced.
                continue
            performed.append((action, ticks))
        return performed

    def start(self):
        """Run update() at rate_hz in a daemon thread; hardware errors stop it and set error."""
        if self._thread is not None and self._thread.is_alive():
            return self
        self._cancel.clear()
        self.error = None
        self._thread = threading.Thread(target=self._run, daemon=True, name="wand-arm")
        self._thread.start()
        return self

    def _run(self):
        """Thread body: call update() at rate_hz; any error disarms and is recorded."""
        try:
            while not self._cancel.wait(1 / self.rate_hz):
                self.update()
        except BaseException as exc:
            self.error = exc
            self.armed = False

    def stop(self, timeout=2):
        """Disarm and stop the thread; the arm remains borrowed and unpowered by us.

        Raises RuntimeError if the thread does not exit within timeout seconds.
        Does not recenter the arm or cut servo power; that is the caller's job.
        """
        self.armed = False
        self._cancel.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise RuntimeError("Wand arm follower did not stop")
            self._thread = None

    def __enter__(self):
        """Context-manager entry; neither starts nor arms the follower."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Disarm and stop on leaving the with-block, even after an exception."""
        self.stop()
