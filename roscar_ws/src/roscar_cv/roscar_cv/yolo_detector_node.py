"""YOLO object detection node for ROScar1."""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        # Rate limiting
        self.min_period = 1.0 / self.detection_rate if self.detection_rate > 0 else 0.0
        self.last_detect_time = 0.0

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

        # Subscriber
        self.create_subscription(Image, '/image_raw', self.image_cb, 1)

    def image_cb(self, msg):
        """Run YOLO inference on incoming frame."""
        if self.model is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_detect_time) < self.min_period:
            return
        self.last_detect_time = now

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

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                det = Detection2D()
                det.header = msg.header

                # Bounding box (center + size)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                det.bbox.center.position.x = float((x1 + x2) / 2)
                det.bbox.center.position.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                # Hypothesis (class + confidence)
                # vision_msgs Jazzy: ObjectHypothesisWithPose wraps ObjectHypothesis
                # with fields hypothesis.class_id (string) and hypothesis.score
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0].item()))
                hyp.hypothesis.score = float(box.conf[0].item())
                det.results.append(hyp)

                # Add class name as id for convenience
                cls_idx = int(box.cls[0].item())
                if cls_idx < len(result.names):
                    det.id = result.names[cls_idx]

                det_array.detections.append(det)

        self.detect_pub.publish(det_array)

        # Publish annotated image
        if self.annotate:
            annotated = result.plot()  # ultralytics draws boxes+labels
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
