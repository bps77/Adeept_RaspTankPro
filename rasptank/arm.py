"""Named, synchronous joints using the legacy webServer.py control layout."""

from .config import SERVO_RANGES


ARM_CHANNELS = {"pan": 0, "arm": 1, "hand": 2, "gripper": 3, "camera": 4}
ARM_ACTIONS = {
    "look_left": ("pan", 1), "look_right": ("pan", -1),
    "arm_up": ("arm", 1), "arm_down": ("arm", -1),
    "hand_up": ("hand", 1), "hand_down": ("hand", -1),
    "grab": ("gripper", -1), "loosen": ("gripper", 1),
    "camera_up": ("camera", 1), "camera_down": ("camera", -1),
}


class RobotArm:
    """Borrow one ServoController; construction neither moves nor owns hardware.

    Channels and physical action signs follow webServer.py/RPIservo.py, not
    servo.py's different joint labels/direction flags. Pulses are PCA9685 ticks,
    not angles or measured positions. Validate linkage clearance on your robot.
    """

    def __init__(self, servos):
        """Wrap an existing controller without opening another PCA9685 driver."""
        self.servos = servos

    def _channel(self, joint):
        """Resolve a web-control joint name or raise before any output."""
        if not isinstance(joint, str) or joint not in ARM_CHANNELS:
            raise ValueError(f"joint must be one of {tuple(ARM_CHANNELS)}")
        return ARM_CHANNELS[joint]

    @property
    def positions(self):
        """Return a snapshot of commanded/assumed ticks, never physical feedback."""
        return {joint: self.servos.positions[channel]
                for joint, channel in ARM_CHANNELS.items()}

    def limits(self, joint):
        """Return this joint's inclusive legacy pulse limits in ticks."""
        return SERVO_RANGES[self._channel(joint)]

    def set_pulse(self, joint, pulse):
        """Command an absolute integer pulse; controller validates range/open state."""
        return self.servos.set_pulse(self._channel(joint), pulse)

    def move_relative(self, joint, ticks):
        """Add raw signed pulse ticks, deliberately bypassing servo.py direction flags."""
        channel = self._channel(joint)
        if type(ticks) is not int:
            raise ValueError("ticks must be an integer")
        return self.servos.set_pulse(channel, self.servos.positions[channel] + ticks)

    def step(self, action, ticks=10):
        """Perform one named web-control action; reject limits rather than clipping."""
        if not isinstance(action, str) or action not in ARM_ACTIONS:
            raise ValueError(f"action must be one of {tuple(ARM_ACTIONS)}")
        if type(ticks) is not int or ticks <= 0:
            raise ValueError("ticks must be a positive integer")
        joint, sign = ARM_ACTIONS[action]
        return self.move_relative(joint, sign * ticks)

    def center(self, joint):
        """Explicitly move one joint to its controller's transient initial pulse."""
        return self.servos.center(self._channel(joint))
