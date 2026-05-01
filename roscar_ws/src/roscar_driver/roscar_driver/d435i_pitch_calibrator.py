"""One-shot D435i pitch calibration helper.

Fits a plane to the floor pixels in a depth frame and reports the
camera's actual pitch angle relative to horizontal. The recommended
value can then be baked into the URDF as the `d435i_pitch` xacro
property so the cliff_detector_node's "expected floor depth" math is
accurate.

WHY THIS EXISTS
The cliff_detector_node assumes the camera is mounted level (pitch=0)
and uses `expected_z(v) = h * fy / (v - cy)` to predict floor depth at
each pixel row. Even a small mounting tilt (a few degrees) makes that
prediction wrong by tens of cm at 1 m forward, causing the detector to
fire on phantom cliffs all over a normal floor.

USAGE
1. Park the robot facing flat empty floor (a hallway, a clear room).
   The lower 1/3 of the camera FOV must be unobstructed floor — no
   feet, no boxes, no rugs with weird patterns. Best on smooth solid
   surfaces.
2. With the depth camera running (slam_nav or any mode with
   `use_depth=true`):
       ros2 run roscar_driver d435i_pitch_calibrator
3. The tool collects a small batch of frames (default 10), fits a
   plane, and prints recommended URDF values.
4. Update `d435i_pitch` in `roscar_description/urdf/roscar.urdf.xacro`
   and rebuild `roscar_description`.

ALGORITHM
For each accepted frame:
  - Sample the bottom HALF of the depth image (rows below image center)
  - Back-project pixels to 3D in optical frame:
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy   (Y is DOWN in optical convention)
        Z = z (depth)
  - Keep only points with z in [0.4, 2.5] m and finite
  - Aggregate across frames

Then fit a plane via SVD: subtract centroid, compute SVD of the centered
point cloud — the singular vector corresponding to the smallest
singular value is the plane normal. (Equivalent to least-squares
plane fit.)

The expected floor-plane normal in the OPTICAL frame, for a perfectly
level camera, is (0, -1, 0): the floor is below the camera, normal
points UP, but optical Y is DOWN, so the upward normal is -Y.

Pitch is the rotation about the optical X axis (horizontal-right) that
takes a level camera to the actual mounting. Recovered from the normal
by projecting onto the YZ plane:
    pitch = atan2(n_z, -n_y)
where (n_x, n_y, n_z) is the plane normal (with n_y < 0). Sign
convention: positive pitch tilts the camera DOWN (looking at the floor
in front, which is the typical mounting offset on this robot).

URDF MAPPING
The xacro `d435i_pitch` property is the rotation about the BASE_LINK Y
axis applied to the camera link. Optical-frame pitch and base_link
pitch are aligned (the URDF's _d435i.urdf.xacro macro handles the
optical convention chain). So the recovered angle plugs directly into
`d435i_pitch` with the same sign.
"""

from __future__ import annotations

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image


def _decode_depth(msg: Image) -> np.ndarray | None:
    h, w = msg.height, msg.width
    if msg.encoding in ('16UC1', 'mono16'):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        return arr.astype(np.float32) * 1e-3
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
    return None


class PitchCalibrator(Node):

    def __init__(self) -> None:
        super().__init__('d435i_pitch_calibrator')

        # Parameters
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('frames_required', 10)
        self.declare_parameter('min_valid_depth_m', 0.4)
        self.declare_parameter('max_valid_depth_m', 2.5)
        self.declare_parameter('min_points_per_frame', 1000)
        self.declare_parameter('row_stride', 4)
        self.declare_parameter('col_stride', 4)

        self._n_frames_target = int(self.get_parameter('frames_required').value)
        self._min_z = float(self.get_parameter('min_valid_depth_m').value)
        self._max_z = float(self.get_parameter('max_valid_depth_m').value)
        self._min_points = int(self.get_parameter('min_points_per_frame').value)
        self._row_stride = max(1, int(self.get_parameter('row_stride').value))
        self._col_stride = max(1, int(self.get_parameter('col_stride').value))

        # Latched intrinsics
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None

        # Accumulated 3D points across frames (in optical-frame coords, meters)
        self._accumulated: list[np.ndarray] = []
        self._n_accepted = 0

        sensor_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            info_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            sensor_qos,
        )

        self.get_logger().info(
            f'd435i_pitch_calibrator: collecting {self._n_frames_target} frames...'
        )
        self.get_logger().info(
            f'  Park the robot facing flat empty floor; the bottom half of '
            f'the depth FOV must be unobstructed floor.'
        )

    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._fx is not None or msg.k[0] == 0.0:
            return
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self._fx, _, self._cx, _, self._fy, self._cy, *_ = msg.k
        self.get_logger().info(
            f'  intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    def _on_depth(self, msg: Image) -> None:
        if self._fy is None:
            return
        if self._n_accepted >= self._n_frames_target:
            return
        depth = _decode_depth(msg)
        if depth is None:
            return

        h, w = depth.shape
        cy_int = int(round(self._cy))   # type: ignore[arg-type]
        if cy_int >= h - 1:
            return

        # Sample bottom half (rows below image center) — that's where
        # the floor projects.
        rows = np.arange(cy_int + 1, h, self._row_stride, dtype=np.int32)
        cols = np.arange(0, w, self._col_stride, dtype=np.int32)
        sub = depth[np.ix_(rows, cols)]                            # (R, C)
        u_grid, v_grid = np.meshgrid(
            cols.astype(np.float32),
            rows.astype(np.float32),
        )

        # Filter valid depth in reasonable range
        valid = (sub > self._min_z) & (sub < self._max_z) & np.isfinite(sub)
        if int(valid.sum()) < self._min_points:
            self.get_logger().info(
                f'  frame skipped: only {int(valid.sum())} valid floor pixels'
                f' (min {self._min_points})',
                throttle_duration_sec=2.0,
            )
            return

        u = u_grid[valid]
        v = v_grid[valid]
        z = sub[valid]
        x = (u - float(self._cx)) * z / float(self._fx)            # type: ignore[arg-type]
        y = (v - float(self._cy)) * z / float(self._fy)            # type: ignore[arg-type]
        pts = np.column_stack((x, y, z)).astype(np.float64)
        self._accumulated.append(pts)
        self._n_accepted += 1
        self.get_logger().info(
            f'  frame {self._n_accepted}/{self._n_frames_target} accepted '
            f'({pts.shape[0]} floor pixels)'
        )
        if self._n_accepted >= self._n_frames_target:
            self._finalize()

    # ------------------------------------------------------------------
    def _finalize(self) -> None:
        all_pts = np.concatenate(self._accumulated, axis=0)
        n = all_pts.shape[0]
        self.get_logger().info(f'fitting plane to {n:,} pooled points...')

        # SVD plane fit. The smallest singular vector of the centered
        # cloud is the plane normal.
        centroid = all_pts.mean(axis=0)
        centered = all_pts - centroid
        # Use SVD on the (n, 3) matrix; full_matrices=False gives a 3x3 V^T.
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        normal = vt[-1]
        # We expect the floor's "up" direction to be -Y in optical
        # (Y points DOWN). If the SVD picked the opposite sign, flip.
        if normal[1] > 0:
            normal = -normal
        nx, ny, nz = (float(v) for v in normal)

        # Pitch about optical X axis: angle in the YZ plane.
        # For a level camera, normal = (0, -1, 0) → atan2(0, 1) = 0.
        # For a camera tilted DOWN (looking at floor in front), the
        # plane normal acquires a +Z component → atan2(+, +) > 0.
        pitch_rad = math.atan2(nz, -ny)
        pitch_deg = math.degrees(pitch_rad)

        # Camera height = -ny * (distance from origin along normal).
        # Plane equation through centroid: normal · (P - centroid) = 0
        # → normal · centroid = signed distance from origin to plane.
        dist_to_plane = abs(float(np.dot(normal, centroid)))

        # A perfectly level camera with normal (0,-1,0) gives:
        #   dist_to_plane = |(-1) * centroid_y| = camera height above floor.
        # With pitch != 0, the math still gives the perpendicular
        # distance from the camera optical center to the floor plane,
        # which IS what cliff_detector wants.

        # Residual fit quality
        residuals = centered @ normal
        rms = float(np.sqrt(np.mean(residuals ** 2)))

        self.get_logger().info('=' * 60)
        self.get_logger().info('PITCH CALIBRATION RESULT')
        self.get_logger().info(f'  Plane normal (optical):   ({nx:+.4f}, {ny:+.4f}, {nz:+.4f})')
        self.get_logger().info(f'  RMS fit residual:         {rms*1000:.2f} mm')
        self.get_logger().info(f'  Perpendicular cam height: {dist_to_plane:.4f} m')
        self.get_logger().info(f'  Recommended d435i_pitch:  {pitch_rad:+.4f} rad ({pitch_deg:+.2f}°)')
        self.get_logger().info('')
        self.get_logger().info('To apply, edit roscar_description/urdf/roscar.urdf.xacro:')
        self.get_logger().info(f'  <xacro:property name="d435i_pitch" value="{pitch_rad:.4f}"/>')
        self.get_logger().info('Then rebuild + redeploy roscar_description and restart the stack.')
        self.get_logger().info('=' * 60)

        # Quick sanity check: warn if the fit looks bad.
        if rms > 0.05:
            self.get_logger().warn(
                f'  WARNING: residual {rms*1000:.0f} mm is high — the bottom '
                f'half of the FOV likely has obstructions or is not a single '
                f'flat floor plane. Re-park and re-run.'
            )
        elif abs(pitch_deg) > 30:
            self.get_logger().warn(
                f'  WARNING: pitch {pitch_deg:+.1f}° is large — the camera '
                f'may not be on the floor plane, or the depth FOV is showing '
                f'mostly walls/ceilings. Re-check what the bottom half sees.'
            )

        # Done. Caller will shut down via spin loop watching this attr.
        self._done = True

    # spin-loop check
    @property
    def done(self) -> bool:
        return getattr(self, '_done', False)


def main() -> None:
    rclpy.init()
    node = PitchCalibrator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        print('\nInterrupted before fit completed.', file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
