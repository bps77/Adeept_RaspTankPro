"""Opt-in motor arbitration and watchdog independent of camera/inference workers."""

from dataclasses import dataclass
import threading
import time

from .exceptions import AutonomyError, ControllerClosedError
from .hardware._lifecycle import number


@dataclass(frozen=True)
class RoamDecision:
    """Bounded action and reason plus metre reading and sensor monotonic timestamps."""

    action: str
    reason: str
    distance: float
    range_timestamp: float
    frame_timestamp: float
    sequence: int


class RoamPolicy:
    """Reactive fused policy: range veto, camera veto, bounded turning, no reverse."""

    def __init__(self, *, speed=20, stop_distance=0.5, resume_distance=0.6,
                 emergency_distance=0.2, max_age=0.3, pulse=0.1, max_turns=12, max_duration=60):
        """Validate percent duty, metres and seconds; defaults require physical testing."""
        self.speed = number(speed, 1, 40, "speed")
        self.stop_distance = number(stop_distance, 0.05, 5, "stop_distance")
        self.resume_distance = number(resume_distance, self.stop_distance + 0.01, 6,
                                      "resume_distance")
        self.emergency_distance = number(emergency_distance, 0.01,
                                         self.stop_distance - 0.01, "emergency_distance")
        self.max_age = number(max_age, 0.01, 2, "max_age")
        self.pulse = number(pulse, 0.01, 0.5, "pulse")
        self.max_duration = number(max_duration, 0.1, 600, "max_duration")
        if type(max_turns) is not int or not 1 <= max_turns <= 100:
            raise ValueError("max_turns must be an integer in 1..100")
        self.max_turns = max_turns
        self._blocked = False
        self._turns = 0

    def reset(self):
        """Reset recovery/hysteresis state for a new explicitly started session."""
        self._blocked = False
        self._turns = 0

    def step(self, frame, distance, range_timestamp, observation, *, now=None):
        """Return one decision; invalid/stale observations raise before any motor output."""
        now = time.monotonic() if now is None else number(now, 0, float("inf"), "now")
        number(distance, 0, 20, "distance")
        number(range_timestamp, 0, float("inf"), "range_timestamp")
        if (observation.sequence != frame.sequence or observation.timestamp != frame.timestamp
                or observation.state not in ("clear", "blocked", "unknown")):
            raise AutonomyError("Visual observation must describe the current frame")
        number(observation.floor_fraction, 0, 1, "floor_fraction")
        if any(not stamp <= now <= stamp + self.max_age
               for stamp in (frame.timestamp, range_timestamp)):
            raise AutonomyError("Camera or range reading is stale or from the future")
        if distance <= self.emergency_distance:
            action, reason = "stop", "insufficient clearance even for turning"
        elif observation.state == "unknown":
            action, reason = "stop", "visual visibility unknown"
        else:
            threshold = self.resume_distance if self._blocked else self.stop_distance
            self._blocked = distance <= threshold or observation.state == "blocked"
            if not self._blocked:
                self._turns = 0
                action, reason = "forward", "fresh range and calibrated corridor clear"
            elif self._turns >= self.max_turns:
                action, reason = "stop", "turn recovery budget exhausted"
            else:
                self._turns += 1
                action = "left"
                reason = "range blocked" if distance <= threshold else "visual corridor blocked"
        return RoamDecision(action, reason, distance, range_timestamp,
                            frame.timestamp, frame.sequence)


class RoamingSession:
    """Cancellation/status handle; borrowed devices remain caller-owned."""

    def __init__(self, supervisor, token, cancel):
        """Retain only this session's arbitration token and cancellation event."""
        self._supervisor, self._token, self._cancel = supervisor, token, cancel

    def stop(self):
        """Cancel this session without stopping a later session or manual command."""
        self._supervisor.cancel(self._token)

    def close(self):
        """Alias stop for notebook cleanup."""
        self.stop()

    def status(self):
        """Get supervisor telemetry; superseded handles are marked cancelled."""
        return self._supervisor.status(self._token)

    def __enter__(self):
        """Return the session for explicit context-managed use."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Cancel motion on all context exits."""
        self.stop()


class DriveSupervisor:
    """Single motor command owner with independent expiry thread and latched faults.

    Hardware output calls must themselves be bounded; GPIO/OS failure still
    requires a physical cutoff. A hung perception worker is abandoned as a
    daemon, reported as a fault, and prevents another autonomy start until exit.
    """

    def __init__(self, motors, command_timeout, *, clock=time.monotonic):
        """Start an opt-in watchdog; timeout seconds bounds manual commands and sensing."""
        self.timeout = number(command_timeout, 0.05, 2, "command_timeout")
        self.motors, self.clock = motors, clock
        self._lock = threading.RLock()
        self._wake, self._closed = threading.Event(), False
        self._cancel = None
        self._worker = None
        self._token = 0
        self._deadline = None
        self._session_end = None
        self._fault = None
        self._state = "stopped"
        self._decision = None
        self._watchdog = threading.Thread(target=self._watch, daemon=True,
                                          name="rasptank-motor-watchdog")
        self._watchdog.start()

    def _ensure_open(self):
        """Reject commands after supervisor shutdown."""
        if self._closed:
            raise ControllerClosedError("Drive supervisor is closed")

    def _cancel_locked(self):
        """Invalidate queued autonomous commands while holding the output lock."""
        if self._cancel is not None:
            self._cancel.set()
        self._token += 1
        self._deadline = None
        self._session_end = None

    def _stop_locked(self):
        """Stop outputs; latch and expose any hardware failure."""
        self._deadline = None
        try:
            self.motors.stop()
        except BaseException as exc:
            self._fault = f"Motor stop failed: {type(exc).__name__}: {exc}"
            self._state = "fault"
            raise

    def manual(self, method, *args):
        """Cancel autonomy then perform a leased manual command; failure stops and raises."""
        with self._lock:
            self._ensure_open()
            self._cancel_locked()
            self._stop_locked()
            if self._fault is not None:
                raise AutonomyError(f"{self._fault}; call reset_drive_fault() after inspection")
            try:
                result = getattr(self.motors, method)(*args)
            except BaseException:
                self._state = "fault"
                self._fault = "Manual command failed"
                try:
                    self._stop_locked()
                except BaseException:
                    pass
                raise
            self._state = "manual"
            self._deadline = self.clock() + self.timeout
            return result

    def stop(self):
        """Cancel all queued autonomous output and stop now, preserving any latched fault."""
        with self._lock:
            self._ensure_open()
            self._cancel_locked()
            self._stop_locked()
            self._state = "fault" if self._fault else "stopped"

    def cancel(self, token):
        """Stop only if token still owns the active session."""
        with self._lock:
            if not self._closed and token == self._token:
                self.stop()

    def reset_fault(self):
        """Explicitly clear a fault while stopped; reject a still-running abandoned worker."""
        with self._lock:
            self._ensure_open()
            self.stop()
            if self._worker is not None and self._worker.is_alive():
                raise AutonomyError("Previous perception worker is still running")
            self._fault = None
            self._state = "stopped"

    def start(self, camera, ultrasonic, observer, policy):
        """Begin finite roaming; camera must already run, observer must be explicitly supplied."""
        if not callable(observer):
            raise ValueError("observer must be an explicitly calibrated callable")
        if not isinstance(policy, RoamPolicy):
            raise ValueError("policy must be a RoamPolicy")
        if policy.pulse >= self.timeout:
            raise ValueError("policy pulse must be shorter than command_timeout")
        with self._lock:
            self._ensure_open()
            if self._worker is not None and self._worker.is_alive():
                raise AutonomyError("A roaming/perception worker is already running")
            if self._fault:
                raise AutonomyError(f"{self._fault}; explicit fault reset required")
            self._cancel_locked()
            self._stop_locked()
            self._cancel = threading.Event()
            policy.reset()
            token = self._token
            self._state, self._decision = "assessing", None
            started = self.clock()
            self._session_end = started + policy.max_duration
            self._deadline = min(started + self.timeout, self._session_end)
            self._worker = threading.Thread(target=self._run, daemon=True,
                                             args=(token, self._cancel, camera, ultrasonic,
                                                   observer, policy, started),
                                             name="rasptank-roaming")
            self._worker.start()
            return RoamingSession(self, token, self._cancel)

    def _active(self, token, cancel):
        """Check ownership under the output lock, including cancellation and closed state."""
        return not self._closed and token == self._token and not cancel.is_set()

    def _run(self, token, cancel, camera, ultrasonic, observer, policy, started):
        """Assess while stopped, pulse one action, stop, then acquire entirely fresh input."""
        sequence = None
        try:
            while not cancel.is_set():
                if self.clock() - started >= policy.max_duration:
                    break
                frame = camera.capture(timeout=min(self.timeout, policy.max_age),
                                       after_sequence=sequence, min_timestamp=self.clock())
                sequence = frame.sequence
                stamp = self.clock()
                distance = ultrasonic.distance()
                observation = observer(frame)
                decision = policy.step(frame, distance, stamp, observation, now=self.clock())
                with self._lock:
                    self.check_expiry()
                    if not self._active(token, cancel):
                        return
                    if self.clock() >= started + policy.max_duration:
                        break
                    self._decision = decision
                    self._state = "blocked" if decision.action == "stop" else decision.action
                    if decision.action == "stop":
                        break
                    self.motors.move(policy.speed, "forward" if decision.action == "forward" else "no",
                                     "left" if decision.action == "left" else "no", 0.6)
                    self._deadline = min(self.clock() + self.timeout,
                                         started + policy.max_duration)
                cancel.wait(min(policy.pulse,
                                max(0, policy.max_duration - (self.clock() - started))))
                with self._lock:
                    if not self._active(token, cancel):
                        return
                    self._stop_locked()
                    self._state = "assessing"
                    self._deadline = min(self.clock() + self.timeout,
                                         started + policy.max_duration)
            with self._lock:
                if self._active(token, cancel):
                    self._stop_locked()
                    if self._state != "blocked":
                        self._state = "stopped"
                    cancel.set()
        except BaseException as exc:
            with self._lock:
                if self._active(token, cancel):
                    self._fault = f"{type(exc).__name__}: {exc}"
                    self._state = "fault"
                    self._cancel_locked()
                    try:
                        self._stop_locked()
                    except BaseException:
                        pass  # _stop_locked has recorded the hardware fault in status.

    def check_expiry(self):
        """Evaluate exact monotonic deadline; public to allow deterministic clock tests."""
        with self._lock:
            if self._closed or self._deadline is None or self.clock() < self._deadline:
                return
            autonomous = self._cancel is not None and not self._cancel.is_set()
            finished = self._session_end is not None and self.clock() >= self._session_end
            self._cancel_locked()
            if autonomous and not finished:
                self._fault = "Autonomous command/sensor deadline expired"
                self._state = "fault"
            else:
                self._state = "stopped"
            try:
                self._stop_locked()
            except BaseException:
                pass  # Stop failure is latched and visible in status.

    def _watch(self):
        """Expire commands without waiting for capture/inference; never silently lose faults."""
        while not self._wake.wait(min(0.01, self.timeout / 5)):
            self.check_expiry()

    def status(self, token=None):
        """Return telemetry snapshot; superseded handles cannot masquerade as active ones."""
        with self._lock:
            return dict(state=self._state if token is None or token == self._token else "cancelled",
                        error=self._fault, decision=self._decision,
                        worker_alive=self._worker is not None and self._worker.is_alive(),
                        command_timeout=self.timeout)

    def close(self):
        """Stop first, terminate watchdog, bounded-join perception; report a stuck worker."""
        error = None
        with self._lock:
            if self._closed:
                return
            try:
                self.stop()
            except BaseException as exc:
                error = exc
            finally:
                self._closed = True
                self._wake.set()
        self._watchdog.join(self.timeout + 0.1)
        if self._worker is not None:
            self._worker.join(self.timeout + 0.1)
            if self._worker.is_alive():
                if error is None:
                    error = AutonomyError(
                        "Perception worker still running; stop requested, devices must not be reused")
        if error is not None:
            raise error
