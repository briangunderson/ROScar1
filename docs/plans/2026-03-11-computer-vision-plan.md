# Computer Vision Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add ArUco marker detection and YOLO object detection to ROScar1, processing on a remote RTX 4070 PC via WSL2, with dashboard integration for feed switching and detection overlays.

**Architecture:** Two ROS2 Python nodes (`aruco_detector_node`, `yolo_detector_node`) in a new `roscar_cv` ament_python package, running on WSL2. They subscribe to `/image_raw` from the Pi over CycloneDDS, process with OpenCV/CUDA, and publish annotated images + detections back. The AIO dashboard gets a new `aio-cv.js` module for feed toggling and detection display.

**Tech Stack:** ROS2 Jazzy, Python, OpenCV (ArUco), Ultralytics YOLOv8, PyTorch CUDA, cv_bridge, vision_msgs, roslibjs

**Design Doc:** `docs/plans/2026-03-11-computer-vision-design.md`

---

## Task 1: Package Skeleton — `roscar_cv`

**Files:**
- Create: `roscar_ws/src/roscar_cv/package.xml`
- Create: `roscar_ws/src/roscar_cv/setup.py`
- Create: `roscar_ws/src/roscar_cv/setup.cfg`
- Create: `roscar_ws/src/roscar_cv/resource/roscar_cv`
- Create: `roscar_ws/src/roscar_cv/roscar_cv/__init__.py`
- Create: `roscar_ws/src/roscar_cv/config/aruco_params.yaml`
- Create: `roscar_ws/src/roscar_cv/config/yolo_params.yaml`

**Step 1: Create directory structure**

```bash
cd ~/ROScar1/roscar_ws/src
mkdir -p roscar_cv/{roscar_cv,config,launch,resource,test}
touch roscar_cv/resource/roscar_cv
touch roscar_cv/roscar_cv/__init__.py
```

**Step 2: Write `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>roscar_cv</name>
  <version>0.1.0</version>
  <description>Computer vision nodes for ROScar1 (ArUco + YOLO). Runs on remote GPU PC.</description>
  <maintainer email="brian@example.com">Brian</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>tf2_ros</depend>
  <depend>image_transport</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 3: Write `setup.py`**

Follow the `roscar_driver` pattern — `data_files` for config + launch, `entry_points` for both nodes.

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roscar_cv'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brian',
    maintainer_email='brian@example.com',
    description='Computer vision nodes for ROScar1 (ArUco + YOLO)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = roscar_cv.aruco_detector_node:main',
            'yolo_detector = roscar_cv.yolo_detector_node:main',
        ],
    },
)
```

**Step 4: Write `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/roscar_cv
[install]
install_scripts=$base/lib/roscar_cv
```

**Step 5: Write `config/aruco_params.yaml`**

```yaml
aruco_detector:
  ros__parameters:
    dictionary: "DICT_4X4_50"
    marker_size: 0.05          # meters
    publish_tf: true
    tf_prefix: "aruco_"
    detection_rate: 15.0       # Hz — skip frames if camera is faster
```

**Step 6: Write `config/yolo_params.yaml`**

```yaml
yolo_detector:
  ros__parameters:
    model: "yolov8n.pt"
    confidence_threshold: 0.5
    device: "0"                # CUDA device index
    detection_rate: 15.0       # Hz
    classes: []                # empty = all COCO classes
    annotate_image: true
```

**Step 7: Verify the skeleton builds**

```bash
cd ~/roscar_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roscar_cv
```

Expected: Build succeeds (no nodes yet, but package structure is valid).

**Step 8: Commit**

```bash
git add roscar_ws/src/roscar_cv/
git commit -m "feat(cv): create roscar_cv package skeleton with configs"
```

---

## Task 2: ArUco Detector Node

**Files:**
- Create: `roscar_ws/src/roscar_cv/roscar_cv/aruco_detector_node.py`

**Step 1: Write the ArUco detector node**

This node:
- Subscribes to `/image_raw` (sensor_msgs/Image) and `/camera_info` (sensor_msgs/CameraInfo)
- Uses cv_bridge to convert ROS Image → OpenCV
- Detects ArUco markers using `cv2.aruco`
- If camera intrinsics available, estimates 6-DOF pose per marker
- Publishes: `/aruco/image` (annotated debug image), `/aruco/markers` (MarkerArray for rviz), TF (camera_link → aruco_{id})
- Rate-limits detection via `detection_rate` parameter

```python
"""ArUco marker detector node for ROScar1."""

import cv2
import numpy as np

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
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Rate limiting
        self.min_period = 1.0 / self.detection_rate if self.detection_rate > 0 else 0.0
        self.last_detect_time = 0.0

        # Publishers
        self.image_pub = self.create_publisher(Image, '/aruco/image', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco/markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Image, '/image_raw', self.image_cb, 1)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, 1)

        self.get_logger().info(
            f'ArUco detector started: dict={dict_name}, '
            f'marker_size={self.marker_size}m, rate={self.detection_rate}Hz'
        )

    def camera_info_cb(self, msg):
        """Cache camera intrinsics from /camera_info."""
        if self.camera_matrix is not None:
            return  # Already have intrinsics
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info('Camera intrinsics received')

    def image_cb(self, msg):
        """Process incoming camera frame for ArUco markers."""
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self.last_detect_time) < self.min_period:
            return
        self.last_detect_time = now

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
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                for i, marker_id in enumerate(ids.flatten()):
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]

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
        """Broadcast TF: camera_link → aruco_{id}."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'{self.tf_prefix}{marker_id}'

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        # Convert rotation vector to quaternion
        rot_mat, _ = cv2.Rodrigues(rvec)
        # Rotation matrix → quaternion (wxyz)
        qw = np.sqrt(1 + rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2]) / 2
        if qw > 1e-6:
            qx = (rot_mat[2, 1] - rot_mat[1, 2]) / (4 * qw)
            qy = (rot_mat[0, 2] - rot_mat[2, 0]) / (4 * qw)
            qz = (rot_mat[1, 0] - rot_mat[0, 1]) / (4 * qw)
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

    def _make_rviz_marker(self, stamp, marker_id, tvec, index):
        """Create an rviz Marker (cube) for a detected ArUco marker."""
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'camera_link'
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
```

**Step 2: Build and verify syntax**

```bash
cd ~/roscar_ws
colcon build --packages-select roscar_cv
```

Expected: Build succeeds.

**Step 3: Commit**

```bash
git add roscar_ws/src/roscar_cv/roscar_cv/aruco_detector_node.py
git commit -m "feat(cv): implement aruco_detector_node with marker detection and TF publishing"
```

---

## Task 3: YOLO Detector Node

**Files:**
- Create: `roscar_ws/src/roscar_cv/roscar_cv/yolo_detector_node.py`

**Step 1: Write the YOLO detector node**

This node:
- Subscribes to `/image_raw`
- Runs YOLOv8 inference using `ultralytics` with CUDA
- Publishes `/image_annotated` (bounding boxes drawn on image) and `/detections` (vision_msgs/Detection2DArray)
- Rate-limits inference to avoid overloading

```python
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
        self.declare_parameter('classes', [])
        self.declare_parameter('annotate_image', True)

        model_path = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        self.detection_rate = self.get_parameter('detection_rate').value
        self.filter_classes = self.get_parameter('classes').value
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
```

**Step 2: Build and verify syntax**

```bash
cd ~/roscar_ws
colcon build --packages-select roscar_cv
```

Expected: Build succeeds.

**Step 3: Commit**

```bash
git add roscar_ws/src/roscar_cv/roscar_cv/yolo_detector_node.py
git commit -m "feat(cv): implement yolo_detector_node with CUDA inference and Detection2DArray"
```

---

## Task 4: Launch File

**Files:**
- Create: `roscar_ws/src/roscar_cv/launch/cv.launch.py`

**Step 1: Write the launch file**

Launches both CV nodes with configurable arguments. Follows `roscar_bringup` launch pattern.

```python
"""Launch ArUco and YOLO CV nodes for ROScar1."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('roscar_cv')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('use_aruco', default_value='true',
                              description='Launch ArUco detector'),
        DeclareLaunchArgument('use_yolo', default_value='true',
                              description='Launch YOLO detector'),
        DeclareLaunchArgument('yolo_model', default_value='yolov8n.pt',
                              description='YOLO model file (n/s/m/l/x)'),

        # ── ArUco Detector ──
        Node(
            package='roscar_cv',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[
                os.path.join(pkg_share, 'config', 'aruco_params.yaml'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_aruco')),
        ),

        # ── YOLO Detector ──
        Node(
            package='roscar_cv',
            executable='yolo_detector',
            name='yolo_detector',
            parameters=[
                os.path.join(pkg_share, 'config', 'yolo_params.yaml'),
                {'model': LaunchConfiguration('yolo_model')},
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_yolo')),
        ),
    ])
```

**Step 2: Build and verify**

```bash
cd ~/roscar_ws
colcon build --packages-select roscar_cv
source install/setup.bash
# Verify launch file is installed:
ros2 launch roscar_cv cv.launch.py --show-args
```

Expected: Shows `use_aruco`, `use_yolo`, `yolo_model` arguments.

**Step 3: Commit**

```bash
git add roscar_ws/src/roscar_cv/launch/cv.launch.py
git commit -m "feat(cv): add cv.launch.py with conditional ArUco/YOLO node launch"
```

---

## Task 5: Dashboard — `aio-cv.js` Module

**Files:**
- Create: `roscar_ws/src/roscar_web/web/js/aio-cv.js`
- Modify: `roscar_ws/src/roscar_web/web/js/aio-app.js` (add import + init call)
- Modify: `roscar_ws/src/roscar_web/web/js/aio-camera.js` (add feed toggle support)
- Modify: `roscar_ws/src/roscar_web/web/aio.html` (add CV toggle button to camera panel)
- Modify: `roscar_ws/src/roscar_web/web/css/aio.css` (add detection overlay styles)

**Step 1: Write `aio-cv.js`**

This module:
- Subscribes to `/detections` (vision_msgs/Detection2DArray) via rosbridge
- Shows a detection summary overlay on the camera panel (e.g., "2 person, 1 chair")
- Provides a feed toggle: switch camera `<img>` between `/image_raw` and `/image_annotated`
- Exports `initCV(getRosFn)` following the existing module pattern

```javascript
/**
 * aio-cv.js — Computer vision overlay + feed toggle.
 * Subscribes to /detections for object detection results,
 * provides toggle between raw and annotated camera feeds.
 */

import { HOST, PORTS, onAppEvent } from './aio-app.js';

let getRos;
let detectSub = null;
let cvFeedActive = false;  // false = /image_raw, true = /image_annotated
let lastDetections = [];   // [{class: 'person', count: 2}, ...]

const PANEL = '#panel-camera';

export function initCV(getRosFn) {
  getRos = getRosFn;
  setupCVToggle();
  onAppEvent((ev) => {
    if (ev === 'connected') subscribeDetections();
  });
}

/** Switch camera feed between raw and annotated. */
export function setCVFeed(on) {
  cvFeedActive = on;
  updateFeedToggleUI();
  // Trigger camera stream restart with new topic
  const event = new CustomEvent('cv-feed-change', { detail: { annotated: on } });
  window.dispatchEvent(event);
}

// ── Subscriptions ────────────────────────────────────────────────────────────
function subscribeDetections() {
  const ros = getRos(); if (!ros) return;
  if (detectSub) { try { detectSub.unsubscribe(); } catch (_) {} }
  detectSub = new ROSLIB.Topic({
    ros,
    name: '/detections',
    messageType: 'vision_msgs/Detection2DArray',
    throttle_rate: 500,  // 2 Hz — UI doesn't need faster
  });
  detectSub.subscribe((msg) => {
    lastDetections = summarizeDetections(msg.detections || []);
    updateDetectionOverlay();
  });
}

function summarizeDetections(detections) {
  const counts = {};
  for (const det of detections) {
    // det.id contains class name (set by yolo_detector_node)
    const name = det.id || 'unknown';
    counts[name] = (counts[name] || 0) + 1;
  }
  return Object.entries(counts)
    .map(([cls, count]) => ({ cls, count }))
    .sort((a, b) => b.count - a.count);
}

// ── UI ───────────────────────────────────────────────────────────────────────
function setupCVToggle() {
  const btn = document.getElementById('cv-feed-toggle');
  if (btn) {
    btn.addEventListener('click', () => {
      setCVFeed(!cvFeedActive);
    });
  }
}

function updateFeedToggleUI() {
  const btn = document.getElementById('cv-feed-toggle');
  if (btn) {
    btn.classList.toggle('active', cvFeedActive);
    btn.textContent = cvFeedActive ? 'CV' : 'RAW';
    btn.title = cvFeedActive ? 'Showing annotated feed' : 'Showing raw feed';
  }
}

function updateDetectionOverlay() {
  const el = document.getElementById('cv-detections');
  if (!el) return;
  if (lastDetections.length === 0) {
    el.textContent = '';
    el.style.display = 'none';
    return;
  }
  el.style.display = '';
  el.textContent = lastDetections
    .map(d => `${d.count} ${d.cls}`)
    .join(' · ');
}

/** Get the current CV feed topic (used by aio-camera.js). */
export function getCVTopic() {
  return cvFeedActive ? '/image_annotated' : '/image_raw';
}
```

**Step 2: Modify `aio-camera.js` — integrate CV feed toggle**

Add support for the `cv-feed-change` event so the camera switches between topics.

Find the existing `TOPIC` constant and `streamUrl()` function. Replace the hard-coded topic with a dynamic one:

```javascript
// At the top of aio-camera.js, replace:
//   const TOPIC = '/image_raw';
// with:
let topic = '/image_raw';

// Add event listener in initCamera():
//   window.addEventListener('cv-feed-change', (e) => {
//     topic = e.detail.annotated ? '/image_annotated' : '/image_raw';
//     startStream();
//   });

// In streamUrl(), replace TOPIC with topic:
//   `topic=${topic}&quality=...`
```

Exact edits (use Edit tool):

1. Change `const TOPIC = '/image_raw';` → `let topic = '/image_raw';`
2. In `initCamera()`, after `onAppEvent(...)`, add:
   ```javascript
   window.addEventListener('cv-feed-change', (e) => {
     topic = e.detail.annotated ? '/image_annotated' : '/image_raw';
     startStream();
   });
   ```
3. In `streamUrl()`, change `topic=${TOPIC}` → `topic=${topic}`

**Step 3: Modify `aio-app.js` — import and init CV module**

Add to the imports section:
```javascript
import { initCV } from './aio-cv.js';
```

Add to the init sequence (after `initCamera()`):
```javascript
initCV(() => ros);
```

**Step 4: Modify `aio.html` — add CV toggle button and detection overlay to camera panel**

In the camera panel controls section (`<div class="camera-ctrls">`), add a CV feed toggle button and a detection overlay:

```html
<!-- After the existing ctrl-groups, add: -->
<div class="ctrl-group">
  <button class="sm-btn" id="cv-feed-toggle" title="Toggle raw/annotated feed">RAW</button>
</div>
```

Add a detection overlay element inside the camera panel (after the camera img wrapper):

```html
<div id="cv-detections" class="cv-detection-overlay" style="display:none;"></div>
```

**Step 5: Modify `aio.css` — add detection overlay styles**

```css
/* CV detection overlay on camera panel */
.cv-detection-overlay {
  position: absolute;
  bottom: 2.2rem;
  left: 0.4rem;
  right: 0.4rem;
  padding: 0.2rem 0.5rem;
  background: rgba(0, 0, 0, 0.7);
  color: #0f0;
  font: 600 0.7rem 'Rajdhani', sans-serif;
  letter-spacing: 0.04em;
  border-radius: 3px;
  pointer-events: none;
  z-index: 5;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}
```

**Step 6: Build, verify no JS syntax errors**

Open `http://<robot-ip>:8888/aio.html` in browser, check console for import errors.

**Step 7: Commit**

```bash
git add roscar_ws/src/roscar_web/web/js/aio-cv.js
git add roscar_ws/src/roscar_web/web/js/aio-camera.js
git add roscar_ws/src/roscar_web/web/js/aio-app.js
git add roscar_ws/src/roscar_web/web/aio.html
git add roscar_ws/src/roscar_web/web/css/aio.css
git commit -m "feat(cv): add aio-cv.js dashboard module with feed toggle and detection overlay"
```

---

## Task 6: Install Dependencies in WSL2

**This task runs on the dev PC, NOT the Pi.**

**Step 1: Install ROS2 CV packages**

```bash
# In WSL2 (Ubuntu-24.04):
sudo apt update
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-image-transport
```

**Step 2: Install Python CV packages**

```bash
pip3 install opencv-contrib-python ultralytics
```

**Step 3: Install PyTorch with CUDA**

Check https://pytorch.org for the current install command. As of early 2026:

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

Verify CUDA is working:
```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}, device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"none\"}')"
```

Expected: `CUDA available: True, device: NVIDIA GeForce RTX 4070`

**Step 4: Pre-download YOLO model**

```bash
python3 -c "from ultralytics import YOLO; m = YOLO('yolov8n.pt'); print('Model loaded')"
```

This downloads `yolov8n.pt` (~6MB) so it's cached for the first node launch.

**Step 5: Copy package to WSL2 workspace and build**

```bash
# In WSL2:
cd ~/roscar_ws/src
# Option A: symlink from Windows mount
ln -sf /mnt/d/localrepos/ROScar1/roscar_ws/src/roscar_cv .

# Option B: copy
cp -r /mnt/d/localrepos/ROScar1/roscar_ws/src/roscar_cv .

cd ~/roscar_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roscar_cv
source install/setup.bash
```

Expected: Build succeeds.

---

## Task 7: End-to-End Test

**Step 1: Start the robot in teleop or SLAM mode** (on Pi)

```bash
# SSH to Pi:
ros2 launch roscar_bringup teleop.launch.py
```

Verify `/image_raw` is publishing:
```bash
ros2 topic hz /image_raw --no-daemon
```

Expected: ~30 Hz

**Step 2: Launch CV nodes** (on WSL2 dev PC)

```bash
# In WSL2 terminal:
source ~/roscar_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 launch roscar_cv cv.launch.py
```

Expected output:
```
[aruco_detector] ArUco detector started: dict=DICT_4X4_50, marker_size=0.05m, rate=15.0Hz
[yolo_detector] YOLO model loaded: yolov8n.pt on device=0, conf=0.5, rate=15.0Hz
```

**Step 3: Verify topics are flowing back to Pi**

```bash
# On Pi or WSL2:
ros2 topic hz /image_annotated --no-daemon
ros2 topic echo /detections --once --no-daemon
```

Expected: `/image_annotated` at ~15 Hz, `/detections` shows Detection2DArray messages.

**Step 4: Test dashboard feed toggle**

1. Open `http://<robot-ip>:8888/aio.html`
2. Camera should show raw feed by default
3. Click "RAW" button → should switch to "CV" and show annotated feed with bounding boxes
4. Detection overlay should show detected objects (e.g., "1 person · 2 chair")
5. Click "CV" button → back to raw feed

**Step 5: Hold an ArUco marker in front of the camera**

Print a 4x4_50 ArUco marker (ID 0-49). Hold it ~30cm from the camera.

```bash
# Verify marker detection:
ros2 topic echo /aruco/markers --once --no-daemon
```

Expected: MarkerArray with at least one marker.

**Step 6: Commit any fixes and tag**

```bash
git add -A
git commit -m "feat(cv): end-to-end CV pipeline verified"
```

---

## Task 8: Deploy Dashboard Changes to Pi

**Step 1: Push, pull, copy, build, restart** (standard deploy)

```bash
# On dev PC:
cd ~/ROScar1
git push origin claude/frosty-moser

# SSH to Pi:
cd ~/ROScar1 && git pull origin claude/frosty-moser
cp -r ~/ROScar1/roscar_ws/src/roscar_web/web/ ~/roscar_ws/src/roscar_web/
cd ~/roscar_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select roscar_web
sudo systemctl restart roscar-web
```

**Step 2: Verify dashboard loads cleanly**

Open `http://<robot-ip>:8888/aio.html`, check browser console for errors.

---

## Task 9: Update CLAUDE.md and Lessons

**Files:**
- Modify: `CLAUDE.md` (add roscar_cv package docs, new topics, CV launch instructions)
- Modify: `tasks/lessons.md` (add CV-related lessons if any)

**Step 1: Add roscar_cv to CLAUDE.md packages table**

Add to the Packages section:
```markdown
| `roscar_cv` | ament_python | CV nodes: ArUco + YOLO (runs on remote GPU PC, not Pi) |
```

**Step 2: Add new topics to Key Topics table**

```markdown
| `/aruco/markers` | MarkerArray | aruco_detector_node | Detected ArUco marker visualizations |
| `/aruco/image` | Image | aruco_detector_node | Debug view with marker outlines |
| `/detections` | Detection2DArray | yolo_detector_node | YOLO detection results |
| `/image_annotated` | Image | yolo_detector_node | Camera feed with bounding boxes |
```

**Step 3: Add CV section to CLAUDE.md**

Document how to launch CV nodes, dependencies, and the feed toggle feature.

**Step 4: Commit**

```bash
git add CLAUDE.md tasks/lessons.md
git commit -m "docs: add roscar_cv package to CLAUDE.md, update topics and launch docs"
```
