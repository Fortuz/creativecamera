# CreativeCamera

Containerised ROS 2 Humble workspace for the Creative Room camera project. The stack ships the ei_fetcher package, which blends classic HSV tennis-ball tracking with an optional Ultralytics YOLO people detector that runs through ONNXRuntime.

## Prerequisites
- Docker Desktop with Compose v2
  - On Windows, install WSL 2 (run `wsl --install`), enable the Windows "Virtual Machine Platform" feature, and ensure hardware virtualization is enabled in BIOS/UEFI.
  - In Docker Desktop settings, enable "Use the WSL 2 based engine" and share the drive that contains this repository.
  - Restart Docker Desktop after installation and verify `docker version` and `docker compose version` run without errors.
- Optional: a webcam exposed as /dev/video0 for live capture
- Optional: Foxglove Studio for visualisation

## Nodes and Topics
| Node | Type | Subscriptions | Publications | Notes |
| --- | --- | --- | --- | --- |
| `video_publisher` | Python | (none) | `/camera/image_raw` | Streams a video file (default media/sample.mp4) when use_video:=true |
| `v4l2_camera_node` | C++ (external) | (none) | `/camera/image_raw`, `/camera/camera_info` | Starts when use_video:=false and a V4L2 device is available |
| `ball_tracker_rgb` | Python | `/camera/image_raw`, `/camera/camera_info` | `/detections/ball`, `/camera/image_debug`, `/camera/image_mask` | HSV segmentation; debug image shows bounding boxes, mask is mono8 |
| `people_detector` | Python | `/camera/image_raw`, `/camera/camera_info` | `/detections/person` | Provide a YOLO .onnx model path for detections |
| `detection_overlay` | Python | `/camera/image_raw`, `/detections/person` | `/image_people_debug` | Optional overlay of people detections on RGB frames |

## Quick Start
```bash
docker compose up --build -d
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /ws/ros2_ws && colcon build"
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/ros2_ws/install/setup.bash && ros2 launch ei_fetcher camera_and_detectors.launch.py"
```

### Use a Real Camera
```bash
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/ros2_ws/install/setup.bash && ros2 launch ei_fetcher camera_and_detectors.launch.py use_video:=false video_device:=/dev/video0"
```

### Stream a Different Video Clip
```bash
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/ros2_ws/install/setup.bash && ros2 launch ei_fetcher camera_and_detectors.launch.py video_path:=/ws/ros2_ws/media/your_video.mp4"
```

## Visualisation
- Bridge to Foxglove Studio: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`
- In Foxglove, connect to `ws://localhost:8765` and look for `/camera/image_raw`, `/camera/image_debug`, `/camera/image_mask`, `/detections/ball`, and `/detections/person`
- Helpful CLI commands:
  - `ros2 topic hz /camera/image_raw`
  - `ros2 topic echo /detections/ball`
  - `ros2 topic echo /detections/person`

## Configuration
Runtime parameters live in `ros2_ws/src/ei_fetcher/config/params.yaml`:
- `ball_tracker_rgb` parameters cover HSV bounds, minimum contour area, and topic names (`debug_image_topic`, `mask_image_topic`)
- `people_detector` expects `model_path` to point at a YOLO ONNX file; adjust `input_size`, `conf_threshold`, `iou_threshold`, and `infer_every_n` to suit your model and hardware

To try one-off overrides without editing the file:
```bash
ros2 run ei_fetcher ball_tracker_rgb --ros-args -p hsv_lower:="[30, 120, 120]" -p hsv_upper:="[52, 255, 255]"
```

## Development Workflow
1. Enter the container shell: `docker compose exec ros2 bash`
2. Build with symlinks for faster iteration: `colcon build --symlink-install`
3. Source the workspace overlay: `source install/setup.bash`
4. Run nodes directly, e.g. `ros2 run ei_fetcher ball_tracker_rgb`

Clean rebuild:
```bash
rm -rf /ws/ros2_ws/build /ws/ros2_ws/install /ws/ros2_ws/log && colcon build
```

## Troubleshooting
- **Debug topics missing**: ensure `ball_tracker_rgb` started (`ros2 topic list` should include `/camera/image_debug` and `/camera/image_mask`). If not, inspect `/ws/ros2_ws/log/latest/ball_tracker_rgb-*/stderr.log`.
- **No people detections**: set `-p model_path:=/ws/ros2_ws/models/person.onnx` (or similar) when launching so the ONNX model loads.
- **Installing extra Python packages**: keep `numpy<2` (`numpy==1.26.4` is pre-installed for `cv_bridge` compatibility`).
- **Foxglove cannot connect**: confirm port `8765` is exposed in `docker-compose.yml` and that the container is running `foxglove_bridge`.

## Repository Layout
- `docker-compose.yml`: Compose service definition
- `Dockerfile`: ROS 2 Humble base with OpenCV, ONNXRuntime, PyTorch, and Ultralytics
- `ros2_ws/src/ei_fetcher`: Source code, launch files, and configuration
- `media/`: Demo video assets
- `models/`: Place ONNX models here and mount into the container

## License
MIT License. See `LICENSE` for the full text.

