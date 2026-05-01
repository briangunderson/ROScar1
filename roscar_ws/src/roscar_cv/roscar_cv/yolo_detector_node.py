"""YOLO object detection node for ROScar1.

Distance-keyed detections: when the D435i is running on the Pi, each
detection's pose.position is back-projected from the bbox center pixel
using the aligned-depth image and camera_info intrinsics. The dashboard
displays the result as e.g. "person 1.4 m" and the value is also
available as a real 3D point in the camera frame for downstream nodes.

Depth comes from /camera/camera/aligned_depth_to_color/image_raw — the
D435i driver publishes this when align_depth.enable=true (default in
realsense_params.yaml). The aligned topic has the same dimensions and
pixel correspondence as the color image, so bbox pixels map directly.

If aligned depth or camera_info isn't available (camera off, network
issue), detections still publish with zero distance — the .pose stays
at its default (0,0,0). Consumers should treat z=0 as "unknown
distance" rather than "object at the camera origin".
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', '0')
        self.declare_parameter('detection_rate', 15.0)
        self.declare_parameter('classes', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('annotate_image', True)

        model_path = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        self.detection_rate = self.get_parameter('detection_rate').value
        try:
            self.filter_classes = self.get_parameter('classes').value or []
        except rclpy.exceptions.ParameterUninitializedException:
            self.filter_classes = []
        self.annotate = self.get_parameter('annotate_image').value

        # Latest-frame slot (single-slot pattern to prevent backlog under load).
        # image_cb stores the most recent frame; the detection timer pulls and
        # processes it, silently overwriting older frames if inference is slow.
        self._latest_frame = None         # the sensor_msgs/Image message itself
        self._last_processed_frame = None # identity check to skip duplicates

        # Latest-frame slot for depth + camera intrinsics (used to back-project
        # bbox centers into 3D). Both can be missing if the D435i is off.
        self._latest_depth = None         # sensor_msgs/Image (16UC1 mm or 32FC1 m)
        self._depth_intrinsics = None     # (fx, fy, cx, cy, frame_id) tuple

        self.bridge = CvBridge()

        # Lazy-load YOLO model (import ultralytics here to fail fast if missing)
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            # Warm up the model on the specified device
            self.model.predict(
                np.zeros((480, 640, 3), dtype=np.uint8),
                device=device, verbose=False
            )
            self.device = device
            self.get_logger().info(
                f'YOLO model loaded: {model_path} on device={device}, '
                f'conf={self.conf_threshold}, rate={self.detection_rate}Hz'
            )
        except ImportError:
            self.get_logger().error(
                'ultralytics not installed! Run: pip3 install ultralytics'
            )
            self.model = None
            return
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
            return

        # Publishers
        self.image_pub = self.create_publisher(Image, '/image_annotated', 1)
        self.detect_pub = self.create_publisher(Detection2DArray, '/detections', 10)

        # Subscriber — use QoS depth 5 to avoid drops during GPU inference spikes
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, '/image_raw', self.image_cb, img_qos)
        self._frame_count = 0

        # Aligned depth + camera_info subscribers. Both are optional —
        # missing them just means detections publish without a 3D pose.
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_cb,
            depth_qos,
        )
        # camera_info is published with TRANSIENT_LOCAL durability by the
        # realsense node, so a late subscriber still gets the latest value.
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # NOTE: subscribe to /camera_info (the remapped canonical topic),
        # NOT /camera/camera/color/camera_info — depth_camera.launch.py
        # remaps the latter to the former so the dashboard MJPEG stream
        # picks up calibration. The remapped topic has the same K matrix.
        self.create_subscription(
            CameraInfo,
            '/camera_info',
            self._camera_info_cb,
            info_qos,
        )

        # Detection timer — pulls latest frame at the configured rate.
        # This decouples GPU inference from the subscriber callback so the
        # executor never blocks on slow inference work.
        if self.detection_rate > 0:
            period = 1.0 / self.detection_rate
            self.create_timer(period, self._detect_timer_cb)

        self.get_logger().info('Subscribed to /image_raw (BEST_EFFORT, depth=5)')

    def image_cb(self, msg):
        """Store latest frame; inference runs on a separate timer."""
        if self.model is None:
            return
        self._frame_count += 1
        if self._frame_count <= 3 or self._frame_count % 100 == 0:
            self.get_logger().info(f'image_cb frame #{self._frame_count}')
        self._latest_frame = msg

    def _depth_cb(self, msg):
        """Latch the latest aligned-to-color depth image."""
        self._latest_depth = msg

    def _camera_info_cb(self, msg):
        """Latch the color camera intrinsics for back-projection."""
        if msg.k[0] == 0.0:
            return  # uncalibrated
        # CameraInfo.k is the row-major 3×3 K matrix:
        #   [fx  0 cx
        #     0 fy cy
        #     0  0  1]
        fx, _, cx, _, fy, cy, *_ = msg.k
        self._depth_intrinsics = (float(fx), float(fy), float(cx), float(cy),
                                   msg.header.frame_id)

    @staticmethod
    def _decode_depth(msg):
        """Return a 2D float32 depth array in METERS, or None if unsupported."""
        h, w = msg.height, msg.width
        if msg.encoding in ('16UC1', 'mono16'):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            return arr.astype(np.float32) * 1e-3
        if msg.encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
        return None

    def _depth_at_bbox(self, bbox_cx, bbox_cy, bbox_w, bbox_h):
        """Return a robust depth value (meters) for a YOLO bbox.

        Samples a small central patch (≤25 % of the bbox area, capped at
        21×21 px) and takes the median of valid (>0, finite) depth values.
        Median is robust to a few NaN/0 pixels and to high-frequency
        depth-image noise around object edges.
        """
        if self._latest_depth is None:
            return 0.0
        depth = self._decode_depth(self._latest_depth)
        if depth is None:
            return 0.0
        h, w = depth.shape

        # Sample a tight patch around the bbox center.
        patch_half_w = int(min(bbox_w * 0.25, 10))
        patch_half_h = int(min(bbox_h * 0.25, 10))
        x0 = max(0, int(round(bbox_cx)) - patch_half_w)
        x1 = min(w, int(round(bbox_cx)) + patch_half_w + 1)
        y0 = max(0, int(round(bbox_cy)) - patch_half_h)
        y1 = min(h, int(round(bbox_cy)) + patch_half_h + 1)
        if x1 <= x0 or y1 <= y0:
            return 0.0

        patch = depth[y0:y1, x0:x1]
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return 0.0
        # Robust against noise + occasional bad pixels.
        return float(np.median(valid))

    def _annotate_distances(self, image, detections, distances):
        """Draw a 'X.Y m' tag on each annotated bbox.

        Ultralytics' result.plot() already wrote class+score; we add a
        distance-only line below it.
        """
        for det, dist_m in zip(detections, distances):
            if dist_m <= 0.0:
                continue
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            sw = det.bbox.size_x
            sh = det.bbox.size_y
            x = int(cx - sw / 2)
            # Place under the bbox (bottom-left), or above if not enough room.
            y_below = int(cy + sh / 2 + 18)
            y = y_below if y_below < image.shape[0] - 4 else int(cy - sh / 2 - 4)
            cv2.putText(
                image,
                f'{dist_m:.2f} m',
                (max(2, x), max(14, y)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def _detect_timer_cb(self):
        """Pull latest frame and run inference. Skip if no new frame available."""
        if self.model is None:
            return
        msg = self._latest_frame
        if msg is None or msg is self._last_processed_frame:
            return
        self._last_processed_frame = msg
        self._process_frame(msg)

    def _process_frame(self, msg):
        """Run YOLO inference on a single frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Run inference
        results = self.model.predict(
            cv_image,
            conf=self.conf_threshold,
            device=self.device,
            classes=self.filter_classes if self.filter_classes else None,
            verbose=False,
        )

        if not results:
            return

        result = results[0]
        boxes = result.boxes

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        # Track per-detection distance for both the message pose and
        # the annotated overlay.
        distances_m = []

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                det = Detection2D()
                det.header = msg.header

                # Bounding box (center + size)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = float((x1 + x2) / 2)
                cy_px = float((y1 + y2) / 2)
                bw = float(x2 - x1)
                bh = float(y2 - y1)
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy_px
                det.bbox.size_x = bw
                det.bbox.size_y = bh

                # Hypothesis (class + confidence)
                # vision_msgs v4 (Jazzy): hyp.hypothesis.class_id / .score
                cls_idx = int(box.cls[0].item())
                cls_name = result.names.get(cls_idx, str(cls_idx))

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = float(box.conf[0].item())

                # Back-project bbox center to a 3D point in the color
                # camera optical frame using the aligned depth + intrinsics.
                # Convention: x=right, y=down, z=forward (depth).
                dist_m = self._depth_at_bbox(cx, cy_px, bw, bh)
                if dist_m > 0.0 and self._depth_intrinsics is not None:
                    fx, fy, k_cx, k_cy, _ = self._depth_intrinsics
                    hyp.pose.pose.position.x = (cx - k_cx) * dist_m / fx
                    hyp.pose.pose.position.y = (cy_px - k_cy) * dist_m / fy
                    hyp.pose.pose.position.z = dist_m
                # Identity quaternion (we have no orientation info; just position)
                hyp.pose.pose.orientation.w = 1.0

                det.results.append(hyp)
                det_array.detections.append(det)
                distances_m.append(dist_m)

        self.detect_pub.publish(det_array)

        # Publish annotated image (with distance overlays added on top of
        # ultralytics' own class+score labels)
        if self.annotate:
            annotated = result.plot()  # ultralytics draws boxes+labels
            self._annotate_distances(annotated, det_array.detections, distances_m)
            try:
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                )
            except Exception as e:
                self.get_logger().warn(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
