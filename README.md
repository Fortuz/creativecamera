# creativecamera
ROS2 package to use the camera in the Creative Room

# ei_fetcher (ROS 2 Humble)

Baseline perception package for the Mecanumbot project.

## Features
- **Tennis ball detection (HSV, RGB-only)** → publishes `vision_msgs/Detection2DArray` on `/detections/ball`
- **People detector (YOLO ONNXRuntime, optional)** → publishes `/detections/person`
- **Video source**: either external USB camera (`v4l2_camera`) or a file-based **video publisher** node
- **Debug image** on `/image_debug`

## Nodes & Topics
- `ball_tracker_rgb`  
  - Sub: `/camera/image_raw`, `/camera/camera_info`  
  - Pub: `/detections/ball`, `/image_debug`
- `people_detector` (runs only if `model_path` is set)  
  - Sub: `/camera/image_raw`, `/camera/camera_info`  
  - Pub: `/detections/person`
- `video_publisher` (test source from MP4)  
  - Pub: `/camera/image_raw`

## Launch
- `camera_and_detectors.launch.py`  
  - Swap source:
    - **USB/real camera**: use `v4l2_camera_node` (set `video_device`)
    - **Video file**: use `video_publisher` with `video_path`

## Quickstart
```bash
source /opt/ros/humble/setup.bash
cd /ws/ros2_ws
colcon build
source install/setup.bash

# Run with video file (set your path below in the launch)
ros2 launch ei_fetcher camera_and_detectors.launch.py

# Debug image
ros2 run image_view image_view --ros-args -r image:=/image_debug

# Detections
ros2 topic echo /detections/ball
ros2 topic echo /detections/person


































# 1) start the container
docker compose up --build -d
docker exec -it creativecamera_ros2 bash

# 2) build the workspace
cd /ws/ros2_ws
apt-get update
apt-get install -y ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-image-transport ros-humble-foxglove-bridge python3-pip
pip3 install ultralytics
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source /ws/ros2_ws/install/setup.bash

# 3) run
ros2 launch ei_fetcher camera_and_detectors.launch.py

# 3a) run with video file
ros2 launch ei_fetcher camera_and_detectors.launch.py use_video:=true \
  video_path:=/ws/ros2_ws/media/sample.mp4

# 3b) run with real camera
# (Make sure /dev/video0 is passed through in docker-compose)
ros2 launch ei_fetcher camera_and_detectors.launch.py use_video:=false \
  video_device:=/dev/video0

# Visualize
#ros2 run image_view image_view --ros-args -r image:=/image_debug
apt-get update
apt-get install -y ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

# See detections
ros2 topic echo /detections/ball
ros2 topic echo /detections/person
