"""Republish YOLO detections in map-frame coordinates.

PR #30 added 3D pose to each Detection2DArray entry — the bbox center
back-projected to the camera_color_optical_frame using D435i depth.
This node consumes that, transforms each pose into the map frame via
tf2, and republishes a compact JSON summary on /detections_world.

The dashboard subscribes to /detections_world and plots the detected
objects on the map alongside the robot. Per-class icon, distance label.

WHY A SEPARATE TOPIC
- The Pi has the full TF tree available (map → odom → base_footprint
  → camera_link → camera_color_optical_frame). Doing the chain in JS
  would require stitching multiple TF transforms with quaternion math
  per detection — fragile.
- Publishing as JSON String keeps the dashboard subscription dirt-simple
  (same pattern as /landmark/known_markers which the dashboard already
  consumes for ArUco markers).
- Throttling to 2 Hz avoids spamming rosbridge with 11 Hz detections
  the user can't visually parse anyway.

WHAT'S PUBLISHED
JSON object:
  {
    "stamp": <unix epoch seconds>,
    "frame_id": "map",
    "objects": [
      {
        "class": "person",
        "score": 0.91,
        "x": 2.34,         // map frame X (m)
        "y": -0.78,        // map frame Y (m)
        "z": 0.30,         // map frame Z above ground (m, info only)
        "dist_m": 1.42     // straight-line distance from camera to object
      },
      ...
    ]
  }

z=0 detections (depth was unknown for that bbox) are SKIPPED — we have
no map position to plot.

Detections older than `max_age_sec` are dropped. The dashboard sees
the live set on every publish.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    m = np.eye(4)
    m[0, 0] = 1 - 2 * (y * y + z * z)
    m[0, 1] = 2 * (x * y - w * z)
    m[0, 2] = 2 * (x * z + w * y)
    m[1, 0] = 2 * (x * y + w * z)
    m[1, 1] = 1 - 2 * (x * x + z * z)
    m[1, 2] = 2 * (y * z - w * x)
    m[2, 0] = 2 * (x * z - w * y)
    m[2, 1] = 2 * (y * z + w * x)
    m[2, 2] = 1 - 2 * (x * x + y * y)
    return m


def _transform_to_matrix(transform) -> np.ndarray:
    t = transform.translation
    r = transform.rotation
    m = _quat_to_matrix(r.x, r.y, r.z, r.w)
    m[0, 3] = t.x
    m[1, 3] = t.y
    m[2, 3] = t.z
    return m


class DetectionWorldNode(Node):

    def __init__(self) -> None:
        super().__init__('detection_world')

        self.declare_parameter('input_topic', '/detections')
        self.declare_parameter('output_topic', '/detections_world')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('max_age_sec', 3.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('tf_timeout_sec', 0.1)

        self._cam_frame = str(self.get_parameter('camera_frame').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._max_age = float(self.get_parameter('max_age_sec').value)
        self._min_conf = float(self.get_parameter('min_confidence').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_sec').value)

        # TF buffer for camera→map lookup
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Recent-detection buffer: deque of dicts with 'expires_at' field
        self._recent: deque[dict] = deque()

        # I/O
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('input_topic').value),
            self._on_detections,
            10,
        )
        # Latched/transient_local so the dashboard sees the latest summary
        # even when subscribing late.
        out_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(
            String,
            str(self.get_parameter('output_topic').value),
            out_qos,
        )

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if rate_hz > 0:
            self.create_timer(1.0 / rate_hz, self._publish_summary)

        self.get_logger().info(
            f'detection_world ready: {self._cam_frame} → {self._map_frame}, '
            f'{rate_hz:.1f} Hz, max_age={self._max_age:.1f}s, '
            f'min_conf={self._min_conf:.2f}'
        )

    # ------------------------------------------------------------------
    def _on_detections(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return
        # Look up camera→map ONCE per message (all detections share the
        # same timestamp). Use the LATEST available transform — the
        # detection timestamp may be slightly behind the latest /tf,
        # which causes "extrapolation into the past" errors.
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._cam_frame,
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(
                f'TF lookup {self._cam_frame}→{self._map_frame} failed: {e}',
                throttle_duration_sec=5.0,
            )
            return

        T = _transform_to_matrix(transform.transform)

        now = time.time()
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]
            if hyp.hypothesis.score < self._min_conf:
                continue
            p = hyp.pose.pose.position
            # PR #30 convention: z=0 means depth unknown — skip
            if p.z <= 0.0:
                continue

            # Transform optical-frame point → map frame.
            cam_pt = np.array([p.x, p.y, p.z, 1.0])
            map_pt = T @ cam_pt
            dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

            self._recent.append({
                'expires_at': now + self._max_age,
                'class': hyp.hypothesis.class_id,
                'score': float(hyp.hypothesis.score),
                'x': float(map_pt[0]),
                'y': float(map_pt[1]),
                'z': float(map_pt[2]),
                'dist_m': float(dist),
            })

    # ------------------------------------------------------------------
    def _publish_summary(self) -> None:
        # Drop expired
        now = time.time()
        while self._recent and self._recent[0]['expires_at'] < now:
            self._recent.popleft()

        # Deduplicate per (class, ~15cm grid cell): the same person seen
        # for 3 seconds at 11 Hz would otherwise be 33 entries.
        deduped: dict[tuple, dict] = {}
        for d in self._recent:
            key = (d['class'], round(d['x'] / 0.15), round(d['y'] / 0.15))
            existing = deduped.get(key)
            if existing is None or d['score'] > existing['score']:
                deduped[key] = d

        out_objects = [
            {k: v for k, v in d.items() if k != 'expires_at'}
            for d in deduped.values()
        ]

        msg = String()
        msg.data = json.dumps({
            'stamp': now,
            'frame_id': self._map_frame,
            'objects': out_objects,
        })
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DetectionWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
