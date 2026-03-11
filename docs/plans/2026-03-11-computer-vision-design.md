# Computer Vision Stack Design for ROScar1

**Date**: 2026-03-11
**Status**: Approved

## Overview

Add computer vision capabilities to the ROScar1 robot: ArUco marker detection and YOLO object detection. All CV processing runs on a remote Windows PC with an RTX 4070 GPU, connected to the robot via the existing CycloneDDS unicast bridge.

## Architecture

### Processing Location: Remote PC (RTX 4070)

The RPi5 lacks the compute for real-time neural network inference. The RTX 4070 dev machine already runs ROS2 Jazzy via WSL2 with proven CycloneDDS connectivity to the Pi.

```
RPi5 (Robot)                          Windows PC (WSL2)
┌─────────────┐    CycloneDDS     ┌──────────────────────────┐
│ v4l2_camera  │───/image_raw────>│ aruco_detector_node      │
│              │                   │   -> /aruco/markers      │
│              │                   │   -> /aruco/image         │
│              │                   │   -> TF (marker poses)   │
│              │                   │                          │
│              │                   │ yolo_detector_node       │
│              │                   │   -> /detections          │
│              │                   │   -> /image_annotated     │
│              │                   ├──────────────────────────┤
│ web_video    │<──/image_annotated│ GPU: RTX 4070 (CUDA)    │
│ _server      │                   └──────────────────────────┘
│ (port 8080)  │
└─────────────┘
      │
  Dashboard (browser)
  shows annotated feed
```

### Data Flow

1. Pi publishes `/image_raw` (640x480, 30fps) and `/camera_info`
2. CycloneDDS transports frames to WSL2 (unicast, already working)
3. `aruco_detector_node` and `yolo_detector_node` subscribe to `/image_raw`
4. Each node processes frames using GPU-accelerated libraries
5. `yolo_detector_node` publishes `/image_annotated` with bounding boxes + labels drawn
6. `/image_annotated` flows back to Pi via CycloneDDS
7. Pi's `web_video_server` serves the annotated stream over MJPEG
8. Dashboard displays the annotated feed

### Latency Budget

| Segment | Estimate |
|---------|----------|
| Pi -> PC (CycloneDDS, LAN) | ~2-5ms |
| ArUco detection (OpenCV) | ~1-3ms |
| YOLO inference (RTX 4070) | ~5-15ms |
| Image annotation + publish | ~1-2ms |
| PC -> Pi (CycloneDDS, LAN) | ~2-5ms |
| Total round-trip | ~15-30ms |

At 30fps input, expect ~25-30fps annotated output. Acceptable for teleoperation and monitoring.

## ROS2 Package: `roscar_cv`

**Type**: ament_python
**Location**: `roscar_ws/src/roscar_cv/`
**Runs on**: WSL2 dev machine (not the Pi)

### Package Structure

```
roscar_ws/src/roscar_cv/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/roscar_cv
├── config/
│   ├── aruco_params.yaml
│   └── yolo_params.yaml
├── launch/
│   └── cv.launch.py            # Launch both nodes
├── roscar_cv/
│   ├── __init__.py
│   ├── aruco_detector_node.py
│   └── yolo_detector_node.py
└── test/
```

### Node 1: aruco_detector_node

**Purpose**: Detect ArUco markers in camera frames, estimate their 6-DOF pose relative to the camera, and publish results as TF transforms and visualization markers.

**Subscriptions**:
| Topic | Type | Notes |
|-------|------|-------|
| `/image_raw` | sensor_msgs/Image | Camera frames from Pi |
| `/camera_info` | sensor_msgs/CameraInfo | Intrinsics for pose estimation |

**Publications**:
| Topic | Type | Notes |
|-------|------|-------|
| `/aruco/markers` | visualization_msgs/MarkerArray | RViz-compatible 3D marker visualization |
| `/aruco/image` | sensor_msgs/Image | Debug image with detected markers drawn |
| `/tf` | tf2_msgs/TFMessage | Camera-relative pose per detected marker |

**Parameters** (`config/aruco_params.yaml`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `dictionary` | `DICT_4X4_50` | ArUco dictionary type |
| `marker_size` | `0.05` | Physical marker side length (meters) |
| `publish_tf` | `true` | Publish marker poses as TF |
| `tf_prefix` | `aruco_` | TF frame prefix (e.g., `aruco_0`, `aruco_1`) |
| `detection_rate` | `15.0` | Max detection rate (Hz), skip frames if input is faster |

**Implementation Notes**:
- Uses `cv2.aruco` from `opencv-contrib-python`
- Pose estimation via `cv2.aruco.estimatePoseSingleMarkers()` requires camera intrinsics from `/camera_info`
- Camera calibration is a prerequisite (currently on the TODO list). Without calibration, marker detection still works but pose estimation will be inaccurate.
- TF frames: `camera_link` -> `aruco_{id}` for each detected marker

### Node 2: yolo_detector_node

**Purpose**: Run YOLOv8 object detection on camera frames, publish detection results and an annotated image with bounding boxes and labels.

**Subscriptions**:
| Topic | Type | Notes |
|-------|------|-------|
| `/image_raw` | sensor_msgs/Image | Camera frames from Pi |

**Publications**:
| Topic | Type | Notes |
|-------|------|-------|
| `/detections` | vision_msgs/Detection2DArray | Structured detection results |
| `/image_annotated` | sensor_msgs/Image | Annotated image (bounding boxes + labels) |

**Parameters** (`config/yolo_params.yaml`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `yolov8n.pt` | YOLO model file (n/s/m/l/x) |
| `confidence_threshold` | `0.5` | Minimum detection confidence |
| `device` | `0` | CUDA device index (0 = first GPU) |
| `detection_rate` | `15.0` | Max inference rate (Hz) |
| `classes` | `[]` | Filter to specific COCO classes (empty = all) |
| `annotate_image` | `true` | Whether to draw boxes on /image_annotated |

**Implementation Notes**:
- Uses `ultralytics` Python package with PyTorch CUDA backend
- `yolov8n.pt` (nano) recommended to start; upgrade to `yolov8s.pt` if GPU headroom allows
- Detection2DArray message includes: class ID, class name, confidence score, bounding box (center + size)
- Model downloads automatically on first run. Pre-download with `yolo predict model=yolov8n.pt` to avoid startup delay.

### Launch File: cv.launch.py

Launches both CV nodes with configurable parameters.

```python
# Launch args:
#   use_aruco:=true|false (default: true)
#   use_yolo:=true|false (default: true)
#   yolo_model:=yolov8n.pt (default: yolov8n.pt)
```

## Web Dashboard Integration

### Camera Panel Changes

The AIO dashboard camera panel (`aio-camera.js`) needs a toggle to switch between raw and annotated feeds:

- **Raw feed**: `http://<robot-ip>:8080/stream?topic=/image_raw` (current)
- **Annotated feed**: `http://<robot-ip>:8080/stream?topic=/image_annotated`
- UI: dropdown or toggle button in the camera panel header
- Falls back to raw feed if `/image_annotated` is not available

### Detection Overlay

Add a lightweight detection summary to the status panel or camera panel:

- Subscribe to `/detections` via rosbridge
- Show count of detected objects by class (e.g., "2 person, 1 chair")
- Optional: ArUco marker IDs currently visible from `/aruco/markers`

### New Frontend Files

```
roscar_ws/src/roscar_web/web/js/
├── aio-cv.js           # CV module: feed toggle, detection overlay, ArUco status
```

## Dependencies

### On the PC (WSL2)

```bash
# ROS2 packages
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-image-transport

# Python packages (in WSL2, not in a venv — ROS2 nodes need system Python)
pip3 install opencv-contrib-python ultralytics

# PyTorch with CUDA (check https://pytorch.org for current install command)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### On the Pi

No new dependencies. The Pi only needs to receive `/image_annotated` (a standard Image message) and serve it via the existing `web_video_server`.

### Prerequisite: Camera Calibration

ArUco pose estimation requires camera intrinsics. The Logitech webcam needs calibration:

```bash
# On the Pi (with camera running):
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/image_raw -r camera:=/camera
```

This produces a calibration YAML that v4l2_camera loads to populate `/camera_info`. Without this, ArUco detection works but pose output is unreliable.

## Topics Summary (New)

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/aruco/markers` | MarkerArray | aruco_detector_node | Detected ArUco marker visualizations |
| `/aruco/image` | Image | aruco_detector_node | Debug view with marker outlines |
| `/detections` | Detection2DArray | yolo_detector_node | YOLO detection results |
| `/image_annotated` | Image | yolo_detector_node | Camera feed with bounding boxes |

## Future: CV-Augmented Lidar

Potential directions for combining CV detections with lidar data (not in scope for initial implementation):

1. **Obstacle classification**: Match lidar point clusters with YOLO bounding boxes to label obstacles (person vs furniture vs wall). Requires projecting 3D lidar points into the 2D camera frame using the camera-lidar extrinsic transform.

2. **Blind spot coverage**: The RPLIDAR C1 scans a single horizontal plane. The camera sees above and below that plane. CV detections could warn about obstacles the lidar misses (e.g., table overhangs, low objects below scan height).

3. **Semantic costmap layer**: Feed classified obstacles into Nav2 as a custom costmap layer plugin. Different object types get different costs (e.g., high cost near people for social distancing, lower cost near static furniture). This would be a Nav2 `CostmapLayer` plugin subscribing to `/detections`.

## Implementation Order

1. Create `roscar_cv` package skeleton with `package.xml`, `setup.py`, configs
2. Implement `aruco_detector_node` (simpler, validates the cross-machine image pipeline)
3. Implement `yolo_detector_node` (depends on CUDA setup being correct)
4. Write `cv.launch.py`
5. Add `aio-cv.js` dashboard module (feed toggle + detection overlay)
6. Test end-to-end: Pi camera -> PC processing -> Pi web dashboard
7. Camera calibration (enables accurate ArUco pose estimation)
