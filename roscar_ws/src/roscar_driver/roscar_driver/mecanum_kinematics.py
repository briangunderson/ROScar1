"""Mecanum wheel forward kinematics for odometry integration.

The YB-ERF01-V3.0 board handles inverse kinematics (cmd_vel -> wheel speeds)
internally. We only need forward kinematics to compute odometry from the
board's reported body velocities.

Coordinate frame (REP 103):
  x = forward, y = left, z = up
  Positive yaw = counter-clockwise
"""

import math


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
