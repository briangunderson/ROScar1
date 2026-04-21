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

Architecture: Subscribes directly to /aruco/markers (visualization_msgs) from
the ArUco detector on WSL2. Uses the raw marker pose (in optical frame convention
from OpenCV's solvePnP) plus local TF chain (map->webcam_optical_frame) to compute
marker position in map frame. The URDF's webcam_optical_frame joint handles the
optical-to-ROS coordinate conversion via its rpy=(-π/2, 0, -π/2) rotation.
This avoids relying on cross-machine /tf for ArUco frames, which has DDS
discovery issues with CycloneDDS unicast when many participants are active.
"""

import json
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
from robot_localization.srv import SetPose
import tf2_ros

import numpy as np


def _yaw_from_quaternion(x, y, z, w):
    """Extract yaw from quaternion (only needs z-axis rotation)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    """Create quaternion from yaw angle (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_to_matrix(x, y, z, w):
    """Convert quaternion to 4x4 homogeneous transform matrix (rotation only)."""
    m = np.eye(4)
    m[0, 0] = 1 - 2*(y*y + z*z)
    m[0, 1] = 2*(x*y - w*z)
    m[0, 2] = 2*(x*z + w*y)
    m[1, 0] = 2*(x*y + w*z)
    m[1, 1] = 1 - 2*(x*x + z*z)
    m[1, 2] = 2*(y*z - w*x)
    m[2, 0] = 2*(x*z - w*y)
    m[2, 1] = 2*(y*z + w*x)
    m[2, 2] = 1 - 2*(x*x + y*y)
    return m


def _transform_to_matrix(transform):
    """Convert a geometry_msgs Transform to a 4x4 homogeneous matrix."""
    t = transform.translation
    r = transform.rotation
    m = np.eye(4)
    # Rotation from quaternion
    x, y, z, w = r.x, r.y, r.z, r.w
    m[0, 0] = 1 - 2*(y*y + z*z)
    m[0, 1] = 2*(x*y - w*z)
    m[0, 2] = 2*(x*z + w*y)
    m[1, 0] = 2*(x*y + w*z)
    m[1, 1] = 1 - 2*(x*x + z*z)
    m[1, 2] = 2*(y*z - w*x)
    m[2, 0] = 2*(x*z - w*y)
    m[2, 1] = 2*(y*z + w*x)
    m[2, 2] = 1 - 2*(x*x + y*y)
    # Translation
    m[0, 3] = t.x
    m[1, 3] = t.y
    m[2, 3] = t.z
    return m


def _pose_to_matrix(pose):
    """Convert a geometry_msgs Pose to a 4x4 homogeneous matrix."""
    p = pose.position
    r = pose.orientation
    m = np.eye(4)
    x, y, z, w = r.x, r.y, r.z, r.w
    m[0, 0] = 1 - 2*(y*y + z*z)
    m[0, 1] = 2*(x*y - w*z)
    m[0, 2] = 2*(x*z + w*y)
    m[1, 0] = 2*(x*y + w*z)
    m[1, 1] = 1 - 2*(x*x + z*z)
    m[1, 2] = 2*(y*z - w*x)
    m[2, 0] = 2*(x*z - w*y)
    m[2, 1] = 2*(y*z + w*x)
    m[2, 2] = 1 - 2*(x*x + y*y)
    m[0, 3] = p.x
    m[1, 3] = p.y
    m[2, 3] = p.z
    return m


def _yaw_from_matrix(m):
    """Extract yaw from a rotation matrix."""
    return math.atan2(m[1, 0], m[0, 0])


class LandmarkLocalizerNode(Node):
    def __init__(self):
        super().__init__('landmark_localizer')

        # Parameters
        self.declare_parameter('check_rate', 2.0)           # Hz (for correction check)
        self.declare_parameter('correction_threshold', 0.3) # meters
        self.declare_parameter('correction_interval', 10.0) # seconds between corrections
        self.declare_parameter('max_marker_distance', 3.0)  # ignore markers further than this
        self.declare_parameter('auto_learn', True)           # auto-learn marker positions
        self.declare_parameter('learned_markers_file',       # persistence file
                               os.path.expanduser('~/roscar_ws/learned_markers.yaml'))
        self.declare_parameter('tf_prefix', 'aruco_')
        self.declare_parameter('marker_ids', [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        self.declare_parameter('velocity_threshold', 0.05)  # m/s — only correct when ~stopped
        self.declare_parameter('load_learned', False)        # load markers from file on startup
            # False for SLAM (new map frame each session)
            # True for navigation (fixed map, markers persist)

        self.correction_threshold = self.get_parameter('correction_threshold').value
        self.correction_interval = self.get_parameter('correction_interval').value
        self.max_marker_distance = self.get_parameter('max_marker_distance').value
        self.auto_learn = self.get_parameter('auto_learn').value
        self.learned_file = self.get_parameter('learned_markers_file').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.marker_ids = self.get_parameter('marker_ids').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value

        # TF — only used for LOCAL transforms (map->webcam_optical_frame)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.load_learned = self.get_parameter('load_learned').value

        # Known marker positions in map frame: {marker_id: (x, y, z, yaw)}
        self.known_markers = {}
        if self.load_learned:
            self._load_learned_markers()
        else:
            self.get_logger().info(
                'load_learned=False: starting with empty marker set (SLAM mode)')

        # State
        self.last_correction_time = 0.0
        self.current_velocity = 0.0  # magnitude

        # Publishers
        self.markers_pub = self.create_publisher(
            String, '/landmark/known_markers', 10)

        # Service client for EKF pose reset (slam_toolbox doesn't subscribe
        # to /initialpose in online_async mode — use EKF's /set_pose instead)
        self.set_pose_client = self.create_client(SetPose, '/set_pose')

        # Service to clear learned markers (called from dashboard reset button)
        self.create_service(Trigger, '/landmark/clear_markers', self._clear_markers_cb)

        # Subscriber for velocity (to check if robot is stopped)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 1)

        # Subscribe DIRECTLY to /aruco/markers — bypasses /tf for cross-machine data
        self.create_subscription(
            MarkerArray, '/aruco/markers', self._aruco_markers_cb, 10)

        # Track which markers are currently visible (for dashboard display)
        self._visible_markers = {}  # {marker_id: last_seen_time}

        # Heartbeat: log available TF frames every 5 seconds for debugging
        self._heartbeat_count = 0
        self.create_timer(5.0, self._heartbeat)

        # Publish known markers for dashboard display every 2 seconds
        self.create_timer(2.0, self._publish_known_markers)

        self.get_logger().info(
            f'Landmark localizer started (direct /aruco/markers mode): '
            f'auto_learn={self.auto_learn}, '
            f'threshold={self.correction_threshold}m, '
            f'watching marker IDs {self.marker_ids}, '
            f'{len(self.known_markers)} known markers loaded'
        )

    def _heartbeat(self):
        """Periodic debug heartbeat."""
        self._heartbeat_count += 1
        if self._heartbeat_count > 24:
            return  # Stop after 2 minutes
        self.get_logger().info(
            f'HEARTBEAT #{self._heartbeat_count}: '
            f'known_markers={list(self.known_markers.keys())}, '
            f'velocity={self.current_velocity:.3f} m/s')

    def _clear_markers_cb(self, request, response):
        """Clear all learned markers (service callback)."""
        count = len(self.known_markers)
        self.known_markers.clear()
        self._visible_markers.clear()
        # Delete the persistence file
        try:
            if os.path.exists(self.learned_file):
                os.remove(self.learned_file)
        except Exception as e:
            self.get_logger().warn(f'Failed to delete {self.learned_file}: {e}')
        self.get_logger().warn(f'Cleared {count} learned markers')
        response.success = True
        response.message = f'Cleared {count} markers'
        return response

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

    def _publish_known_markers(self):
        """Publish known marker positions as JSON for the web dashboard."""
        if not self.known_markers:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        markers = []
        for mid, (x, y, z, yaw) in self.known_markers.items():
            visible = (mid in self._visible_markers and
                       (now_sec - self._visible_markers[mid]) < 2.0)
            markers.append({
                'id': mid,
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw': round(yaw, 3),
                'visible': visible,
            })
        msg = String()
        msg.data = json.dumps(markers)
        self.markers_pub.publish(msg)

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

    def _aruco_markers_cb(self, msg):
        """Process ArUco marker detections directly from /aruco/markers topic.

        Each marker in the array has:
        - header.frame_id = 'webcam_optical_frame'
        - id = ArUco marker ID
        - pose = marker pose in OPTICAL frame convention (z=forward, x=right, y=down)
                 (OpenCV's solvePnP always returns in camera/optical convention)

        Instead of manually converting optical→webcam_link, we use the URDF's
        webcam_optical_frame (which already has the correct rpy=-π/2,0,-π/2
        rotation relative to webcam_link). We look up map→webcam_optical_frame
        and use the raw tvec directly — the TF tree handles the frame rotation.
        """
        if not msg.markers:
            return  # No markers detected

        now_sec = self.get_clock().now().nanoseconds / 1e9

        for marker in msg.markers:
            marker_id = marker.id

            # Only process markers we're watching
            if marker_id not in self.marker_ids:
                continue

            # Use raw optical-frame coordinates directly (x=right, y=down, z=forward)
            raw_x = marker.pose.position.x
            raw_y = marker.pose.position.y
            raw_z = marker.pose.position.z

            # Compute distance from camera (same regardless of frame)
            marker_dist = math.sqrt(raw_x*raw_x + raw_y*raw_y + raw_z*raw_z)

            if marker_dist > self.max_marker_distance:
                self.get_logger().debug(
                    f'Marker {marker_id}: too far ({marker_dist:.2f}m)')
                continue

            # Build pose in optical frame (use raw tvec, identity orientation)
            optical_pose = marker.pose
            optical_pose.position.x = raw_x
            optical_pose.position.y = raw_y
            optical_pose.position.z = raw_z
            optical_pose.orientation.x = 0.0
            optical_pose.orientation.y = 0.0
            optical_pose.orientation.z = 0.0
            optical_pose.orientation.w = 1.0

            # Look up LOCAL TF: map -> webcam_optical_frame
            # The URDF defines webcam_optical_frame with rpy=(-π/2, 0, -π/2)
            # relative to camera_link, so the TF tree handles the optical→ROS
            # coordinate conversion automatically.
            try:
                map_to_optical = self.tf_buffer.lookup_transform(
                    'map', 'webcam_optical_frame', Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug(
                    f'Marker {marker_id}: map->webcam_optical_frame TF failed: {e}')
                continue

            # Compute map -> marker by chaining: map->optical * optical->marker
            map_opt_mat = _transform_to_matrix(map_to_optical.transform)
            opt_marker_mat = _pose_to_matrix(optical_pose)
            map_marker_mat = map_opt_mat @ opt_marker_mat

            obs_x = map_marker_mat[0, 3]
            obs_y = map_marker_mat[1, 3]
            obs_z = map_marker_mat[2, 3]
            obs_yaw = _yaw_from_matrix(map_marker_mat)

            # Track visibility for dashboard
            self._visible_markers[marker_id] = now_sec

            self.get_logger().debug(
                f'Marker {marker_id}: detected at {marker_dist:.2f}m, '
                f'map pos=({obs_x:.2f}, {obs_y:.2f})')

            # Auto-learn: first sighting records the position
            if marker_id not in self.known_markers:
                if self.auto_learn:
                    self.known_markers[marker_id] = (obs_x, obs_y, obs_z, obs_yaw)
                    self._save_learned_markers()
                    self.get_logger().warn(
                        f'LEARNED marker {marker_id} at map position '
                        f'({obs_x:.2f}, {obs_y:.2f}, yaw={math.degrees(obs_yaw):.1f}deg)')
                continue

            # Compare observed position to known position
            known_x, known_y, known_z, known_yaw = self.known_markers[marker_id]
            drift_x = known_x - obs_x
            drift_y = known_y - obs_y
            drift_dist = math.sqrt(drift_x * drift_x + drift_y * drift_y)

            if drift_dist < self.correction_threshold:
                self.get_logger().debug(
                    f'Marker {marker_id}: drift={drift_dist:.2f}m < threshold, OK')
                continue  # Within tolerance

            # Rate limit corrections
            if (now_sec - self.last_correction_time) < self.correction_interval:
                continue

            # Only correct when robot is nearly stationary
            if self.current_velocity > self.velocity_threshold:
                self.get_logger().info(
                    f'Marker {marker_id}: drift={drift_dist:.2f}m but robot moving '
                    f'({self.current_velocity:.2f} m/s), skipping correction')
                continue

            # Compute corrected robot pose
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

            # Call EKF /set_pose service to reset the odom->base_footprint transform.
            # slam_toolbox will re-match scans on next cycle, adjusting map->odom.
            if not self.set_pose_client.service_is_ready():
                self.get_logger().warn(
                    f'Marker {marker_id}: drift={drift_dist:.2f}m but /set_pose not ready')
                continue

            request = SetPose.Request()
            request.pose = pose_msg
            future = self.set_pose_client.call_async(request)
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
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down


if __name__ == '__main__':
    main()
