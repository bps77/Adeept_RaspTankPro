"""Pure-Python Kalman filter and PID controller ported from server/."""

import time


class KalmanFilter:
    """server/Kalman_filter.py, including its jump-smoothing at |delta| >= 60."""

    def __init__(self, q=0.01, r=0.1):
        self.q = q
        self.r = r
        self.p = 1.0
        self.old = 0.0

    def update(self, value):
        if abs(self.old - value) >= 60:
            prior = value * 0.382 + self.old * 0.618
        else:
            prior = self.old
        p_pred = self.p + self.q
        gain = p_pred / (p_pred + self.r)
        estimate = prior + gain * (value - self.old)
        self.p = (1 - gain) * p_pred
        self.old = estimate
        return estimate


class PID:
    """server/PID.py with an injectable clock."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, *, clock=time.time):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._clock = clock
        self.reset()

    def reset(self):
        self.prev_time = self._clock()
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error):
        now = self._clock()
        dt = now - self.prev_time
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.integral += error * dt
        self.prev_time = now
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
