FROM osrf/ros:humble-desktop

# System dependencies for creativecamera workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    python3-opencv \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-v4l2-camera \
    ros-humble-image-view \
    ros-humble-foxglove-bridge \
 && rm -rf /var/lib/apt/lists/*

# Python dependencies (ultralytics optional but useful for model export)
RUN pip3 install --no-cache-dir --upgrade pip
RUN pip3 install --no-cache-dir "numpy==1.26.4"
RUN pip3 install --no-cache-dir onnxruntime ultralytics

# Workspace mount point
WORKDIR /ws

# Helpful: source ROS in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
