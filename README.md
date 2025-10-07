# creativecamera
ROS2 package to use the camera in the Creative Room

# ei_fetcher (ROS 2 Humble)

Baseline perception package for the Mecanumbot project.

## Features
- Tennis ball detection (HSV) publishes `vision_msgs/Detection2DArray` on `/detections/ball`
- People detector (YOLO ONNXRuntime, optional) publishes `/detections/person`
- Video source: either external USB camera (`v4l2_camera`) or a file-based video publisher node
- Debug outputs on `/camera/image_debug` and `/camera/image_mask`

## Nodes & Topics
- `ball_tracker_rgb`
  - Sub: `/camera/image_raw`, `/camera/camera_info`
  - Pub: `/detections/ball`, `/camera/image_debug`, `/camera/image_mask`
- `people_detector`
  - Sub: `/camera/image_raw`, `/camera/camera_info`
  - Pub: `/detections/person`
- `video_publisher`
  - Pub: `/camera/image_raw`

## Quickstart
```bash
docker compose up --build -d
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /ws/ros2_ws && colcon build"
docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/ros2_ws/install/setup.bash && ros2 launch ei_fetcher camera_and_detectors.launch.py"
```
- Launch with a real camera: append `use_video:=false video_device:=/dev/video0`
- Launch with a different clip: `video_path:=/ws/ros2_ws/media/your_video.mp4`

## Runtime notes
- The Docker image pins `numpy==1.26.4` to stay compatible with `cv_bridge`. When installing extra Python packages inside the container, include the constraint, e.g. `pip3 install --no-cache-dir some_pkg "numpy<2"`.
- `ultralytics<9.0.0` is pre-installed; you can replace it with another version if needed, but keep the NumPy pin.
- Always run commands through `docker compose exec ros2 bash -lc "<command>"` so the shell exits automatically after the command finishes.

## Useful commands
- Inspect frame rates: `ros2 topic hz /camera/image_raw`
- Check detections: `ros2 topic echo /detections/ball`
- Visualize overlays: `ros2 run image_view image_view --ros-args -r image:=/camera/image_debug`
- Start Foxglove bridge: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`

## Development
1. `docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && cd /ws/ros2_ws && colcon build --symlink-install"`
2. `docker compose exec ros2 bash -lc "source /opt/ros/humble/setup.bash && source /ws/ros2_ws/install/setup.bash"`
3. Run individual nodes, e.g. `ros2 run ei_fetcher ball_tracker_rgb`

Clean builds:
```bash
docker compose exec ros2 bash -lc "rm -rf /ws/ros2_ws/build /ws/ros2_ws/install /ws/ros2_ws/log"
```

Troubleshooting tips:
- If `onnxruntime` warns about GPU discovery, it is safe to ignore when running on CPU-only hosts.
- When tuning HSV thresholds, monitor `/camera/image_mask` in Foxglove or `image_view` to see the binary mask.
