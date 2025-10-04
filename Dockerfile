FROM osrf/ros:humble-desktop

# Basic utilities
RUN apt-get update && apt-get install -y \
    ros-humble-v4l2-camera \
    ros-humble-image-view \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

# ONNXRuntime CPU (change to -gpu package if you need GPU)
RUN pip3 install --no-cache-dir onnxruntime

# Workspace mount point
WORKDIR /ws

# Helpful: source ROS in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
