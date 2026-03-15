"""Landmark localizer: ArUco marker-based pose correction for slam_toolbox/AMCL.

When the robot sees an ArUco marker at a known map-frame position, this node
computes the expected robot pose and compares it to the current TF-based pose.
If drift exceeds a threshold, it publishes a corrected pose to /initialpose
to relocalize slam_toolbox or AMCL.

Supports two modes:
  - auto_learn (default): First sighting records marker position in map frame.
    Subsequent sightings use the saved position for drift correction.
    Learned positions are saved to disk for persistence across sessions.
  - fixed: Marker positions loaded from config YAML only. No auto-learning.

Runs on the Pi alongside the localization stack. Requires ArUco detector
(roscar_cv) to be running and publishing TFs via CycloneDDS.
"""

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros


def _yaw_from_quaternion(x, y, z, w):
    """Extract yaw from quaternion (only needs z-axis rotation)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    """Create quaternion from yaw angle (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class LandmarkLocalizerNode(Node):
    def __init__(self):
        super().__init__('landmark_localizer')

        # Parameters
        self.declare_parameter('check_rate', 2.0)           # Hz
        self.declare_parameter('correction_threshold', 0.3) # meters
        self.declare_parameter('correction_interval', 10.0) # seconds between corrections
        self.declare_parameter('max_marker_distance', 3.0)  # ignore markers further than this
        self.declare_parameter('auto_learn', True)           # auto-learn marker positions
        self.declare_parameter('learned_markers_file',       # persistence file
                               os.path.expanduser('~/roscar_ws/learned_markers.yaml'))
        self.declare_parameter('tf_prefix', 'aruco_')
        self.declare_parameter('marker_ids', [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        self.declare_parameter('velocity_threshold', 0.05)  # m/s — only correct when ~stopped

        self.check_rate = self.get_parameter('check_rate').value
        self.correction_threshold = self.get_parameter('correction_threshold').value
        self.correction_interval = self.get_parameter('correction_interval').value
        self.max_marker_distance = self.get_parameter('max_marker_distance').value
        self.auto_learn = self.get_parameter('auto_learn').value
        self.learned_file = self.get_parameter('learned_markers_file').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.marker_ids = self.get_parameter('marker_ids').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Known marker positions in map frame: {marker_id: (x, y, z, yaw)}
        self.known_markers = {}
        self._load_learned_markers()

        # State
        self.last_correction_time = 0.0
        self.current_velocity = 0.0  # magnitude

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Subscriber for velocity (to check if robot is stopped)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 1)

        # Timer
        self.create_timer(1.0 / self.check_rate, self._check_markers)

        self.get_logger().info(
            f'Landmark localizer started: auto_learn={self.auto_learn}, '
            f'threshold={self.correction_threshold}m, '
            f'watching marker IDs {self.marker_ids}, '
            f'{len(self.known_markers)} known markers loaded'
        )

    def _odom_cb(self, msg):
        """Track robot velocity to only correct when stationary."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_velocity = math.sqrt(vx * vx + vy * vy)

    def _load_learned_markers(self):
        """Load auto-learned marker positions from disk."""
        if os.path.exists(self.learned_file):
            try:
                with open(self.learned_file, 'r') as f:
                    data = yaml.safe_load(f) or {}
                for mid_str, pos in data.items():
                    mid = int(mid_str)
                    self.known_markers[mid] = (
                        pos['x'], pos['y'], pos.get('z', 0.0), pos.get('yaw', 0.0))
                self.get_logger().info(
                    f'Loaded {len(self.known_markers)} learned markers from {self.learned_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load learned markers: {e}')

    def _save_learned_markers(self):
        """Save auto-learned marker positions to disk."""
        try:
            data = {}
            for mid, (x, y, z, yaw) in self.known_markers.items():
                data[str(mid)] = {'x': float(x), 'y': float(y),
                                  'z': float(z), 'yaw': float(yaw)}
            os.makedirs(os.path.dirname(self.learned_file), exist_ok=True)
            with open(self.learned_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().warn(f'Failed to save learned markers: {e}')

    def _check_markers(self):
        """Check for visible ArUco markers and compute drift correction."""
        now_sec = self.get_clock().now().nanoseconds / 1e9

        for marker_id in self.marker_ids:
            frame_id = f'{self.tf_prefix}{marker_id}'

            # Try to look up marker pose in map frame via current TF chain
            # map -> odom -> base_footprint -> base_link -> camera_link -> aruco_N
            try:
                map_to_marker = self.tf_buffer.lookup_transform(
                    'map', frame_id, Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            # Check staleness — ArUco TFs have short lifetime
            tf_age = now_sec - (map_to_marker.header.stamp.sec +
                                map_to_marker.header.stamp.nanosec / 1e9)
            if tf_age > 1.0:
                continue  # Stale TF, marker not currently visible

            # Extract observed marker position in map frame
            obs_x = map_to_marker.transform.translation.x
            obs_y = map_to_marker.transform.translation.y
            obs_z = map_to_marker.transform.translation.z

            # Check marker distance from camera (use the camera->marker TF)
            try:
                cam_to_marker = self.tf_buffer.lookup_transform(
                    'camera_link', frame_id, Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
                dx = cam_to_marker.transform.translation.x
                dy = cam_to_marker.transform.translation.y
                dz = cam_to_marker.transform.translation.z
                marker_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if marker_dist > self.max_marker_distance:
                    continue  # Too far for reliable pose estimation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            obs_q = map_to_marker.transform.rotation
            obs_yaw = _yaw_from_quaternion(obs_q.x, obs_q.y, obs_q.z, obs_q.w)

            # Auto-learn: first sighting records the position
            if marker_id not in self.known_markers:
                if self.auto_learn:
                    self.known_markers[marker_id] = (obs_x, obs_y, obs_z, obs_yaw)
                    self._save_learned_markers()
                    self.get_logger().info(
                        f'Learned marker {marker_id} at map position '
                        f'({obs_x:.2f}, {obs_y:.2f}, yaw={obs_yaw:.2f})')
                continue

            # Compare observed position to known position
            known_x, known_y, known_z, known_yaw = self.known_markers[marker_id]
            drift_x = known_x - obs_x
            drift_y = known_y - obs_y
            drift_dist = math.sqrt(drift_x * drift_x + drift_y * drift_y)

            if drift_dist < self.correction_threshold:
                continue  # Within tolerance

            # Rate limit corrections
            if (now_sec - self.last_correction_time) < self.correction_interval:
                continue

            # Only correct when robot is nearly stationary
            if self.current_velocity > self.velocity_threshold:
                self.get_logger().debug(
                    f'Marker {marker_id} drift={drift_dist:.2f}m but robot moving '
                    f'({self.current_velocity:.2f} m/s), skipping correction')
                continue

            # Compute corrected robot pose
            # The drift vector (known - observed) in map frame tells us how
            # much the map->odom transform is off. Apply this to the current
            # robot pose to get the corrected pose.
            try:
                map_to_base = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            corrected_x = map_to_base.transform.translation.x + drift_x
            corrected_y = map_to_base.transform.translation.y + drift_y

            # For yaw correction, compute the angular drift
            drift_yaw = known_yaw - obs_yaw
            # Normalize to [-pi, pi]
            drift_yaw = math.atan2(math.sin(drift_yaw), math.cos(drift_yaw))

            base_q = map_to_base.transform.rotation
            base_yaw = _yaw_from_quaternion(base_q.x, base_q.y, base_q.z, base_q.w)
            corrected_yaw = base_yaw + drift_yaw

            # Publish corrected pose to /initialpose
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = corrected_x
            pose_msg.pose.pose.position.y = corrected_y
            pose_msg.pose.pose.position.z = 0.0

            qx, qy, qz, qw = _quaternion_from_yaw(corrected_yaw)
            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw

            # Covariance: moderate confidence (diagonal)
            cov = [0.0] * 36
            cov[0] = 0.1   # x
            cov[7] = 0.1   # y
            cov[35] = 0.05  # yaw
            pose_msg.pose.covariance = cov

            self.pose_pub.publish(pose_msg)
            self.last_correction_time = now_sec

            self.get_logger().warn(
                f'LANDMARK CORRECTION: marker {marker_id} drift={drift_dist:.2f}m '
                f'(dx={drift_x:.2f}, dy={drift_y:.2f}, dyaw={math.degrees(drift_yaw):.1f}deg) '
                f'-> corrected pose ({corrected_x:.2f}, {corrected_y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
