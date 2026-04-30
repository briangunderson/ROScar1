"""Cliff / dropoff detector for ROScar1.

Watches the D435i depth image for negative-space hazards — anywhere the
floor should be visible but isn't. Publishes virtual obstacle points so
the Nav2 local costmap will refuse to drive into the area approaching
a cliff edge.

ALGORITHM
For a horizontally-mounted depth camera at height h above the floor with
focal length fy (vertical) and image principal-point cy, every pixel row
v BELOW image center sees the floor at a known expected depth:

    expected_z(v) = h * fy / (v - cy)         (only valid for v > cy)

When the camera looks at a normal floor at depth z_actual, the relation
holds within a tolerance band. When there's a cliff:

    z_actual >> expected_z   (apparent floor is much further away than it
                              should be) — OR
    z_actual is NaN / 0      (no return at all — could be cliff or just
                              dark/specular floor)

For every pixel that fails the floor test, we emit a virtual obstacle
point at the EXPECTED FLOOR LOCATION (the place the robot would drive
TOWARD if it kept moving forward). The costmap inflation layer then
treats that area as unsafe — robot stops before it ever reaches the
cliff edge.

PARAMETERS
- camera_height_m       (auto from TF if not provided)
- floor_tolerance_m     (z_actual within ±this of expected → "ok floor")
- min_cliff_drop_m      (z_actual must exceed expected by at least this
                          much to fire — guards against measurement noise)
- max_detect_distance_m (skip rows where expected_z exceeds this — far-
                          away cliffs aren't actionable)
- min_valid_depth_m     (skip rows where expected_z is below this — too
                          close, depth gets unreliable)
- output_frame          (target frame for the published obstacle points)
- col_stride            (sample every Nth column — CPU saver)
- row_stride            (sample every Nth row — CPU saver)
- treat_invalid_as_cliff(NaN/0 depth → cliff, default True)
- min_consecutive_cliff (require N adjacent cliff pixels in a column
                          before emitting — denoises against single-
                          pixel measurement glitches)

OUTPUT
sensor_msgs/PointCloud2 on /cliff_obstacles, points in `output_frame`
(default: base_link). Add as a 3rd source on the local_costmap
obstacle_layer (data_type: PointCloud2, marking: true, clearing: false).
"""

from __future__ import annotations

import numpy as np
import rclpy
import struct
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _depth_image_to_meters(img_msg: Image) -> np.ndarray | None:
    """Decode a sensor_msgs/Image (depth) into a 2D float32 array in meters.

    realsense2_camera publishes depth as 16UC1 with values in millimeters;
    32FC1 means values already in meters. We support both because that's
    the only invariant.
    """
    h, w = img_msg.height, img_msg.width
    if img_msg.encoding == '16UC1' or img_msg.encoding == 'mono16':
        arr = np.frombuffer(img_msg.data, dtype=np.uint16).reshape(h, w)
        depth = arr.astype(np.float32) * 1e-3
    elif img_msg.encoding == '32FC1':
        depth = np.frombuffer(img_msg.data, dtype=np.float32).reshape(h, w).copy()
    else:
        return None
    # Sentinels for "no return"
    depth[depth == 0.0] = np.nan
    return depth


class CliffDetector(Node):

    def __init__(self) -> None:
        super().__init__('cliff_detector')

        # Parameters
        self.declare_parameter('camera_height_m', -1.0)         # -1 ⇒ derive from TF
        self.declare_parameter('floor_tolerance_m', 0.05)
        self.declare_parameter('min_cliff_drop_m', 0.10)
        self.declare_parameter('max_detect_distance_m', 1.5)
        self.declare_parameter('min_valid_depth_m', 0.30)
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('camera_link_frame', 'camera_link')
        self.declare_parameter('col_stride', 4)
        self.declare_parameter('row_stride', 4)
        # Invalid depth (NaN / 0) is a noisy cliff signal. Far walls
        # beyond range_max, glossy floors, dark matte surfaces, and
        # glass all read as invalid. Default off: rely on the
        # "actual > expected" predicate which is robust. Flip on only
        # for environments where a real cliff needs to be conservatively
        # avoided AND the floor is reliably visible to depth.
        self.declare_parameter('treat_invalid_as_cliff', False)
        self.declare_parameter('min_consecutive_cliff', 3)
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/depth/camera_info')

        self._tol = float(self.get_parameter('floor_tolerance_m').value)
        self._min_drop = float(self.get_parameter('min_cliff_drop_m').value)
        self._max_dist = float(self.get_parameter('max_detect_distance_m').value)
        self._min_dist = float(self.get_parameter('min_valid_depth_m').value)
        self._out_frame = str(self.get_parameter('output_frame').value)
        self._cam_frame = str(self.get_parameter('camera_link_frame').value)
        self._col_stride = max(1, int(self.get_parameter('col_stride').value))
        self._row_stride = max(1, int(self.get_parameter('row_stride').value))
        self._treat_invalid = bool(self.get_parameter('treat_invalid_as_cliff').value)
        self._min_consec = max(1, int(self.get_parameter('min_consecutive_cliff').value))

        # Camera intrinsics (latched from camera_info)
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None
        self._cam_optical_frame: str | None = None

        # TF for camera-height lookup if not supplied
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cam_height: float | None = None
        cfg_h = float(self.get_parameter('camera_height_m').value)
        if cfg_h > 0:
            self._cam_height = cfg_h

        # I/O
        sensor_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            sensor_qos,
        )
        # Costmap obstacle_layer reads PointCloud2 with sensor_data QoS
        # (BEST_EFFORT, VOLATILE) — match it on the publisher side.
        self._pub = self.create_publisher(PointCloud2, '/cliff_obstacles', sensor_qos)

        # Throttle log spam
        self._frames_seen = 0
        self._frames_emitted = 0

        self.get_logger().info(
            f'cliff_detector ready: out_frame={self._out_frame} '
            f'tol={self._tol:.2f} min_drop={self._min_drop:.2f} '
            f'max_dist={self._max_dist:.1f} stride=({self._row_stride},{self._col_stride}) '
            f'min_consec={self._min_consec}'
        )

    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        # Intrinsics: K = [fx 0 cx; 0 fy cy; 0 0 1] (row-major flatten)
        if msg.k[0] == 0.0:
            return  # uncalibrated header
        self._fx, _, self._cx, _, self._fy, self._cy, *_ = msg.k
        self._cam_optical_frame = msg.header.frame_id

    def _ensure_cam_height(self) -> bool:
        if self._cam_height is not None:
            return True
        # Derive from TF: base_footprint → camera_link, take z component.
        # base_footprint is at floor level (z=0), camera_link's translation
        # in that frame IS the camera's height above the floor.
        try:
            t = self._tf_buffer.lookup_transform(
                'base_footprint',
                self._cam_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        self._cam_height = float(t.transform.translation.z)
        self.get_logger().info(
            f'camera height resolved from TF: {self._cam_height:.3f} m'
        )
        return True

    # ------------------------------------------------------------------
    def _on_depth(self, msg: Image) -> None:
        self._frames_seen += 1
        if self._fy is None or self._cy is None:
            return  # camera_info not received yet
        if not self._ensure_cam_height():
            return  # TF not ready yet
        depth = _depth_image_to_meters(msg)
        if depth is None:
            self.get_logger().warn(
                f'cliff_detector: unsupported depth encoding {msg.encoding!r}',
                throttle_duration_sec=10.0,
            )
            return

        cliff_pts_in_optical = self._detect(depth)
        if cliff_pts_in_optical.size == 0:
            return  # nothing to publish

        # Transform to output_frame using TF (chain optical → base_link
        # via the URDF static transforms — already done by the realsense2
        # description xacro).
        pc_msg = self._build_pointcloud2(
            msg.header.stamp,
            self._cam_optical_frame or msg.header.frame_id,
            cliff_pts_in_optical,
        )
        if pc_msg is None:
            return
        self._pub.publish(pc_msg)
        self._frames_emitted += 1
        if self._frames_emitted == 1 or self._frames_emitted % 50 == 0:
            self.get_logger().info(
                f'emitted cliff cloud: {cliff_pts_in_optical.shape[0]} pts '
                f'(frame {self._frames_emitted}/{self._frames_seen})'
            )

    # ------------------------------------------------------------------
    def _detect(self, depth: np.ndarray) -> np.ndarray:
        """Return Nx3 array of cliff points in OPTICAL frame coords (meters)."""
        h, w = depth.shape
        cy_int = int(round(self._cy))   # type: ignore[arg-type]
        if cy_int >= h - 1:
            return np.empty((0, 3), dtype=np.float32)

        # Pixel rows below image center (these look at the floor).
        # Vectorize: for each row v, build expected_z(v) once.
        rows = np.arange(cy_int + 1, h, self._row_stride, dtype=np.int32)
        if rows.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Expected floor depth per row (broadcast to a column vector).
        v_offsets = rows.astype(np.float32) - float(self._cy)  # > 0
        expected_z = float(self._cam_height) * float(self._fy) / v_offsets  # type: ignore[arg-type]

        # Drop rows whose expected_z is outside our actionable window.
        keep_rows = (expected_z >= self._min_dist) & (expected_z <= self._max_dist)
        if not np.any(keep_rows):
            return np.empty((0, 3), dtype=np.float32)
        rows = rows[keep_rows]
        expected_z = expected_z[keep_rows]

        # Slice the depth grid for the rows + column stride we care about.
        cols = np.arange(0, w, self._col_stride, dtype=np.int32)
        sub = depth[np.ix_(rows, cols)]                # shape (R, C)
        exp_grid = expected_z[:, None]                  # shape (R, 1) → broadcasts

        # Cliff predicate per pixel:
        #   (actual is NaN AND treat_invalid)  OR
        #   (actual > expected + max(tol, min_drop))
        threshold = exp_grid + max(self._tol, self._min_drop)
        cliff_mask = sub > threshold
        if self._treat_invalid:
            cliff_mask |= np.isnan(sub)

        # Denoise: require min_consecutive_cliff cliff pixels in the same
        # column. We OR the mask with itself shifted up by k for k in
        # [1..min_consec-1] then AND across — equivalent to a 1D dilation
        # then a per-column "run length ≥ N" check. Simpler: convolve with
        # ones and threshold.
        if self._min_consec > 1:
            # Use a vertical 1D moving-sum equivalent.
            # cliff_mask shape (R, C). Sum across rows in a sliding window.
            # For small R this is cheap.
            # We approximate by: for each cell, count #True in next
            # min_consec rows of the same column.
            R = cliff_mask.shape[0]
            kernel = self._min_consec
            kept = np.zeros_like(cliff_mask)
            running = np.zeros((cliff_mask.shape[1],), dtype=np.int32)
            for r in range(R):
                running += cliff_mask[r].astype(np.int32)
                if r >= kernel:
                    running -= cliff_mask[r - kernel].astype(np.int32)
                if r >= kernel - 1:
                    kept[r] = running >= kernel
            cliff_mask = kept

        if not np.any(cliff_mask):
            return np.empty((0, 3), dtype=np.float32)

        # For each cliff pixel, emit a point at the EXPECTED FLOOR
        # location (not the actual raw depth value — we want the costmap
        # to mark the floor in front of the cliff, not the back wall of
        # the dropoff which might be tens of meters away).
        # In optical convention:
        #   X = (u - cx) * z / fx
        #   Y = (v - cy) * z / fy   (Y is DOWN; expected_z makes Y = camera_height)
        #   Z = z = expected_z
        rr, cc = np.where(cliff_mask)
        u = cols[cc].astype(np.float32)
        v = rows[rr].astype(np.float32)
        z = expected_z[rr]
        x = (u - float(self._cx)) * z / float(self._fx)        # type: ignore[arg-type]
        y = (v - float(self._cy)) * z / float(self._fy)        # type: ignore[arg-type]
        return np.column_stack((x, y, z)).astype(np.float32)

    # ------------------------------------------------------------------
    def _build_pointcloud2(
        self,
        stamp,
        source_frame: str,
        pts_optical: np.ndarray,
    ) -> PointCloud2 | None:
        """Wrap Nx3 float32 points into a PointCloud2 in source_frame.

        We deliberately publish in the OPTICAL frame (the frame the depth
        image arrives in). The costmap obstacle_layer's ObservationBuffer
        will use tf2 to chain optical → output_frame → costmap frame — and
        the URDF already publishes that chain as static transforms, so it
        works without any work here.

        We don't pre-transform to output_frame ourselves because doing so
        in Python at 15 Hz adds latency, and tf2_sensor_msgs would require
        a service call per frame; the costmap layer already handles it.
        """
        n = pts_optical.shape[0]
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = source_frame
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = pts_optical.tobytes()
        return msg


def main() -> None:
    rclpy.init()
    node = CliffDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
