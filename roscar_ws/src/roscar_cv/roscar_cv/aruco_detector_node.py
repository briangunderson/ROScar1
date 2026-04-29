"""ArUco marker detector node for ROScar1."""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

# ArUco dictionary name → OpenCV constant mapping
ARUCO_DICTS = {
    'DICT_4X4_50':   cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100':  cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250':  cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50':   cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100':  cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250':  cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50':   cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100':  cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250':  cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_prefix', 'aruco_')
        self.declare_parameter('detection_rate', 15.0)

        dict_name = self.get_parameter('dictionary').value
        self.marker_size = self.get_parameter('marker_size').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.detection_rate = self.get_parameter('detection_rate').value

        # ArUco setup
        dict_id = ARUCO_DICTS.get(dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Tuned for longer range and wider viewing angles:
        # - Lower min marker perimeter to detect smaller (distant) markers
        self.aruco_params.minMarkerPerimeterRate = 0.01  # default 0.03
        # - Increase adaptive threshold window for uneven lighting
        self.aruco_params.adaptiveThreshWinSizeMin = 3   # default 3
        self.aruco_params.adaptiveThreshWinSizeMax = 53  # default 23
        self.aruco_params.adaptiveThreshWinSizeStep = 5  # default 10
        # - Relax corner refinement for skewed views
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5    # default 5
        self.aruco_params.cornerRefinementMinAccuracy = 0.05  # default 0.1
        self.aruco_params.cornerRefinementMaxIterations = 50  # default 30
        # - Lower perspective removal threshold for more skewed markers
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.2  # default 0.13
        # - More tolerant bit extraction for partially occluded/blurry markers
        self.aruco_params.maxErroneousBitsInBorderRate = 0.5  # default 0.35
        self.aruco_params.errorCorrectionRate = 0.8  # default 0.6

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self._intrinsics_dims = None  # (width, height) of cached intrinsics

        # Latest-frame slot (single-slot pattern to prevent backlog under load).
        # image_cb stores the most recent frame; the detection timer pulls and
        # processes it, silently overwriting older frames if inference is slow.
        self._latest_frame = None       # the sensor_msgs/Image message itself
        self._last_processed_frame = None  # identity check to skip duplicates

        # Publishers
        self.image_pub = self.create_publisher(Image, '/aruco/image', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco/markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Image, '/image_raw', self.image_cb, 1)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, 1)

        # Detection timer — pulls latest frame at the configured rate.
        # This decouples inference from the subscriber callback so the executor
        # never blocks on slow detection work.
        if self.detection_rate > 0:
            period = 1.0 / self.detection_rate
            self.create_timer(period, self._detect_timer_cb)

        self.get_logger().info(
            f'ArUco detector started: dict={dict_name}, '
            f'marker_size={self.marker_size}m, rate={self.detection_rate}Hz'
        )

    def camera_info_cb(self, msg):
        """Cache camera intrinsics from /camera_info; refresh on resolution change."""
        dims = (msg.width, msg.height)
        if self._intrinsics_dims == dims:
            return  # Same resolution as cached — no-op
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        if self._intrinsics_dims is None:
            self.get_logger().info(
                f'Camera intrinsics received ({dims[0]}x{dims[1]})'
            )
        else:
            self.get_logger().info(
                f'Camera intrinsics refreshed: '
                f'{self._intrinsics_dims[0]}x{self._intrinsics_dims[1]} '
                f'-> {dims[0]}x{dims[1]}'
            )
        self._intrinsics_dims = dims

    def image_cb(self, msg):
        """Store latest frame; detection runs on a separate timer."""
        self._latest_frame = msg

    def _detect_timer_cb(self):
        """Pull latest frame and run detection. Skip if no new frame available."""
        msg = self._latest_frame
        if msg is None or msg is self._last_processed_frame:
            return
        self._last_processed_frame = msg
        self._process_frame(msg)

    def _process_frame(self, msg):
        """Process a single camera frame for ArUco markers."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        stamp = msg.header.stamp
        marker_array = MarkerArray()

        if ids is not None and len(ids) > 0:
            # Draw detected markers on debug image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Pose estimation (requires camera intrinsics)
            if self.camera_matrix is not None:
                # estimatePoseSingleMarkers removed in OpenCV 4.8+;
                # use solvePnP per marker with the known square corners.
                half = self.marker_size / 2.0
                obj_points = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0],
                ], dtype=np.float32)

                for i, marker_id in enumerate(ids.flatten()):
                    # IPPE_SQUARE: deterministic, designed for square coplanar
                    # markers. Avoids the orientation-flip ambiguity that plain
                    # SOLVEPNP_ITERATIVE has on coplanar 4-point inputs.
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0],
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if not ok:
                        continue
                    rvec = rvec.flatten()
                    tvec = tvec.flatten()

                    # Draw axis on debug image
                    cv2.drawFrameAxes(
                        cv_image, self.camera_matrix, self.dist_coeffs,
                        rvec, tvec, self.marker_size * 0.5
                    )

                    # Publish TF
                    if self.publish_tf:
                        self._publish_tf(stamp, marker_id, rvec, tvec)

                    # Add to MarkerArray for rviz
                    marker_array.markers.append(
                        self._make_rviz_marker(stamp, marker_id, tvec, i)
                    )
            else:
                # No intrinsics — just report marker IDs without pose
                for i, marker_id in enumerate(ids.flatten()):
                    self.get_logger().debug(f'Marker {marker_id} detected (no pose — missing camera_info)')

        # Publish annotated debug image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        except Exception as e:
            self.get_logger().warn(f'Failed to publish aruco image: {e}')

        # Publish marker array (even if empty, to clear old markers in rviz)
        self.marker_pub.publish(marker_array)

    def _publish_tf(self, stamp, marker_id, rvec, tvec):
        """Broadcast TF: camera_optical_frame → aruco_{id}."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'camera_optical_frame'
        t.child_frame_id = f'{self.tf_prefix}{marker_id}'

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        # Convert rotation vector to quaternion via scipy (robust for all angles)
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rot_mat).as_quat()  # returns [x, y, z, w]

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def _make_rviz_marker(self, stamp, marker_id, tvec, index):
        """Create an rviz Marker (cube) for a detected ArUco marker."""
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'camera_optical_frame'
        m.ns = 'aruco'
        m.id = int(marker_id)
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(tvec[0])
        m.pose.position.y = float(tvec[1])
        m.pose.position.z = float(tvec[2])
        m.pose.orientation.w = 1.0
        m.scale.x = self.marker_size
        m.scale.y = self.marker_size
        m.scale.z = 0.005
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.8
        m.lifetime.sec = 0
        m.lifetime.nanosec = 500_000_000  # 0.5s
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
