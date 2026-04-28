"""Mecanum wheel forward kinematics for odometry integration.

The YB-ERF01-V3.0 board's `get_motion_data()` returns body velocities computed
from a hardcoded mecanum FK whose chassis dimensions are baked into the
firmware (and don't match chassis v2). We bypass that by reading raw encoder
counts via `get_motor_encoder()` and computing the FK ourselves with the real
half-wheelbase / half-track / wheel-radius.

Coordinate frame (REP 103):
  x = forward, y = left, z = up
  Positive yaw = counter-clockwise

Wheel labels (Yahboom YB-ERF01-V3.0 motor port mapping):
  M1 = front-left,  M2 = rear-left,  M3 = front-right,  M4 = rear-right
"""

import math


def _short_diff(curr: int, prev: int, modulus: int = 65536) -> int:
    """Signed delta between two counter readings, handling 16-bit wrap.

    Treats inputs as if they came from a 16-bit hardware counter that wraps
    at `modulus`. Returns a signed integer in [-modulus/2, +modulus/2). Works
    for both signed-int16 and unsigned-uint16 raw values, since we only care
    about the difference.
    """
    d = (curr - prev) % modulus
    if d > modulus // 2:
        d -= modulus
    return d


class MecanumFK:
    """Forward kinematics: raw encoder counts → body velocities (vx, vy, wz).

    Standard Yahboom ABBA mecanum convention with M1=FL, M2=RL, M3=FR, M4=RR.
    All wheel angular velocities are positive when the wheel rolls forward.
    """

    def __init__(self, wheel_radius: float, half_wheelbase: float,
                 half_track: float, encoder_cpr: float = 1320.0):
        self.R = wheel_radius
        self.Lx = half_wheelbase
        self.Ly = half_track
        self.cpr = encoder_cpr
        self._prev_counts = None
        self._prev_time = None

    def update(self, m1: int, m2: int, m3: int, m4: int, t: float):
        """Compute body velocities from latest encoder counts.

        Args:
            m1..m4: raw encoder counts (M1=FL, M2=RL, M3=FR, M4=RR)
            t: timestamp in seconds (monotonic)

        Returns:
            (vx, vy, wz) body velocities in m/s, m/s, rad/s. Returns
            (0, 0, 0) on the first call (no prior reading to diff against).
        """
        if self._prev_counts is None or self._prev_time is None:
            self._prev_counts = (m1, m2, m3, m4)
            self._prev_time = t
            return 0.0, 0.0, 0.0

        dt = t - self._prev_time
        if dt <= 0.0 or dt > 1.0:
            self._prev_counts = (m1, m2, m3, m4)
            self._prev_time = t
            return 0.0, 0.0, 0.0

        d1 = _short_diff(m1, self._prev_counts[0])
        d2 = _short_diff(m2, self._prev_counts[1])
        d3 = _short_diff(m3, self._prev_counts[2])
        d4 = _short_diff(m4, self._prev_counts[3])
        self._prev_counts = (m1, m2, m3, m4)
        self._prev_time = t

        # Wheel angular velocities (rad/s, +ve = wheel rolls forward)
        k = (2.0 * math.pi) / (self.cpr * dt)
        w_FL = d1 * k
        w_RL = d2 * k
        w_FR = d3 * k
        w_RR = d4 * k

        # Mecanum forward kinematics (ABBA pattern, REP-103 frame)
        vx = (self.R / 4.0) * ( w_FL + w_FR + w_RL + w_RR)
        vy = (self.R / 4.0) * (-w_FL + w_FR + w_RL - w_RR)
        wz = (self.R / (4.0 * (self.Lx + self.Ly))) * (-w_FL + w_FR - w_RL + w_RR)
        return vx, vy, wz


class MecanumOdometry:
    """Integrates body velocities into 2D pose (x, y, theta)."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._last_time = None

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._last_time = None

    def update(self, vx: float, vy: float, vz: float, current_time: float):
        """Integrate body velocities into world-frame pose.

        Args:
            vx: Forward velocity (m/s) in body frame.
            vy: Lateral velocity (m/s) in body frame (positive = left).
            vz: Yaw rate (rad/s, positive = CCW).
            current_time: Current timestamp in seconds.

        Returns:
            Tuple of (x, y, theta) in world frame.
        """
        if self._last_time is None:
            self._last_time = current_time
            return self.x, self.y, self.theta

        dt = current_time - self._last_time
        self._last_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return self.x, self.y, self.theta

        # Transform body velocities to world frame and integrate
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        self.x += (vx * cos_theta - vy * sin_theta) * dt
        self.y += (vx * sin_theta + vy * cos_theta) * dt
        self.theta += vz * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta
