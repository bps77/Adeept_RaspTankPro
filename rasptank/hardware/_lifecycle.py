"""Small shared validation and best-effort resource release helpers."""

import math
from numbers import Real

from ..exceptions import ControllerClosedError


def ensure_open(controller):
    """Raise ControllerClosedError if the controller has been closed."""
    if controller._closed:
        raise ControllerClosedError("Controller is closed")


def number(value, low, high, name):
    """Return value if it is a finite real (not bool) in [low, high], else ValueError."""
    if (isinstance(value, bool) or not isinstance(value, Real)
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return value


def release(actions):
    """Attempt every action, deferring even interrupts until the first failure is raised."""
    error = None
    for action in actions:
        try:
            action()
        except BaseException as exc:
            if error is None:
                error = exc
    if error is not None:
        raise error


class ManagedController:
    """Context-manager base: subclasses set ``_closed`` and implement ``close()``."""

    def __enter__(self):
        """Return the open controller; raise if it is already closed."""
        ensure_open(self)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close on exit; cleanup errors are suppressed if the body already failed."""
        if exc_type is None:
            self.close()
        else:
            try:
                self.close()
            except BaseException:
                pass
