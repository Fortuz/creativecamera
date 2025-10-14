# Mecanumbot Camera

Containerised ROS 2 Humble workspace for the Creative Room camera project. The stack ships the mecanumbot_camera package, which blends classic HSV tennis-ball tracking with an optional Ultralytics YOLO people detector that runs through ONNXRuntime.

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
| `video_publisher` | Python | (none) | `/camera/image_raw`, `/camera/camera_info` | Streams a video file (default `mecanumbot_camera/media/sample.mp4`) when `use_video:=true`; publishes a synthetic `CameraInfo`. |
| `v4l2_camera_node` | C++ (external) | (none) | `/camera/image_raw`, `/camera/camera_info` | Starts when `use_video:=false` and the host exposes a V4L2 device. |
| `ball_tracker_rgb` | Python | `/camera/image_raw`, `/camera/camera_info` | `/detections/ball`, `/camera/image_debug`, `/camera/image_mask` | HSV segmentation with tunable morphology, circularity, and blur filters. |
| `people_detector` | Python | `/camera/image_raw`, `/camera/camera_info` | `/detections/person`, `/camera/annotations`, `/camera/people_debug` | Runs a YOLO ONNX model via ONNX Runtime and publishes Foxglove annotations plus a debug image. |
| `overlay_fused` | Python | `/camera/image_raw`, `/detections/person`, `/detections/ball` | `/camera/fused_debug` | Combines ball and people detections into a single debug image (enabled by default). |
| `detection_overlay` | Python (optional) | `/camera/image_raw`, `/detections/person` | `/image_people_debug` | Legacy overlay retained for reference; disable `overlay_fused` if you prefer it. |

## Quick Start
```bash
docker compose up --build -d
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /ws/mecanumbot_camera && colcon build"
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/mecanumbot_camera/install/setup.bash && ros2 launch mecanumbot_camera camera_and_detectors.launch.py"
```

### Use a Real Camera
1. **Windows + WSL**: Attach the USB webcam to the WSL VM that backs Docker Desktop.
   ```powershell
   usbipd wsl list
   usbipd wsl attach --busid <BUSID>
   ```
   Verify `/dev/video0` exists inside WSL (`ls -l /dev/video*`).
2. **Start the stack with the USB override** (adds `/dev/video0` to the container's device list):
   ```bash
   docker compose -f docker-compose.yml -f docker-compose.usb-camera.yml up --build -d
   ```
   (On native Linux you can skip step 1; ensure the user running Docker has access to `/dev/video0`.)
3. **Launch with the real camera**:
   ```bash
   docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/mecanumbot_camera/install/setup.bash && ros2 launch mecanumbot_camera camera_and_detectors.launch.py use_video:=false video_device:=/dev/video0"
   ```

### Stream a Different Video Clip
```bash
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/mecanumbot_camera/install/setup.bash && ros2 launch mecanumbot_camera camera_and_detectors.launch.py video_path:=/ws/mecanumbot_camera/media/your_video.mp4"
```

## Visualisation
- Bridge to Foxglove Studio: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`
- In Foxglove, connect to `ws://localhost:8765` and look for `/camera/image_raw`, `/camera/image_debug`, `/camera/image_mask`, `/camera/people_debug`, `/camera/fused_debug`, `/camera/annotations`, `/detections/ball`, and `/detections/person`
- Helpful CLI commands:
  - `ros2 topic hz /camera/image_raw`
  - `ros2 topic echo /detections/ball`
  - `ros2 topic echo /detections/person`
  - `ros2 topic echo /camera/annotations`

## Configuration
Runtime parameters live in `mecanumbot_camera/config/params.yaml`:
- `ball_tracker_rgb` covers HSV bounds, minimum contour area, optional circularity filtering (`min_circularity`), morphology controls (`morph_kernel`, `morph_iters`), median blur (`median_ksize`), whether to use an enclosing circle, and the debug/mask topics.
- `people_detector` sets the ONNX `model_path` (defaults to `mecanumbot_camera/models/yolov8n.onnx`), `input_size`, thresholds, inference stride (`infer_every_n`), class IDs, and debug outputs (`ann_topic`, `people_debug_topic`).
- `overlay_fused` selects the detection topics to merge and the fused debug topic.

To try one-off overrides without editing the file:
```bash
ros2 run mecanumbot_camera ball_tracker_rgb --ros-args -p hsv_lower:="[30, 120, 120]" -p hsv_upper:="[52, 255, 255]"
# Example: point the people detector at a different ONNX file
ros2 run mecanumbot_camera people_detector --ros-args -p model_path:=/ws/mecanumbot_camera/models/your_model.onnx -p infer_every_n:=1
```

## Development Workflow
1. Enter the container shell: `docker compose exec ros2 bash`
2. Build with symlinks for faster iteration: `colcon build --symlink-install`
3. Source the workspace overlay: `source install/setup.bash`
4. Run nodes directly, e.g. `ros2 run mecanumbot_camera ball_tracker_rgb`

Clean rebuild:
```bash
rm -rf /ws/mecanumbot_camera/build /ws/mecanumbot_camera/install /ws/mecanumbot_camera/log && colcon build
```

## Troubleshooting
- **Debug topics missing**: ensure `ball_tracker_rgb` started (`ros2 topic list` should include `/camera/image_debug` and `/camera/image_mask`). If not, inspect `/ws/mecanumbot_camera/log/latest/ball_tracker_rgb-*/stderr.log`.
- **No people detections**: ensure `people_detector` has a valid ONNX path, e.g. `ros2 run mecanumbot_camera people_detector --ros-args -p model_path:=/ws/mecanumbot_camera/models/yolov8n.onnx`.
- **Installing extra Python packages**: keep `numpy<2` (`numpy==1.26.4` is pre-installed for `cv_bridge` compatibility).
- **Foxglove cannot connect**: confirm port `8765` is exposed in `docker-compose.yml` and that the container is running `foxglove_bridge`.

## Repository Layout
- `docker-compose.yml`: Compose service definition
- `docker-compose.usb-camera.yml`: Optional override that exposes `/dev/video0` to the container
- `Dockerfile`: ROS 2 Humble base with OpenCV, ONNXRuntime, PyTorch, and Ultralytics
- `mecanumbot_camera/`: ROS 2 package sources, launch files, configuration, and assets
- `mecanumbot_camera/media/`: Demo video assets
- `mecanumbot_camera/models/`: Default YOLO ONNX models

## License
Apache 2.0 License. See `LICENSE` for the full text.

