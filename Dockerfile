# ===== Base image ============================================================
FROM osrf/ros:humble-desktop

# Convenience env vars + noninteractive apt
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1

# ===== System deps (ROS + runtime libs) =====================================
# NOTE: intentionally do not install python3-opencv/python3-numpy via apt
# to avoid mixing them with the pip-installed packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-v4l2-camera \
    ros-humble-image-view \
    ros-humble-foxglove-bridge \
    ffmpeg libsm6 libxext6 libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-foxglove-msgs && rm -rf /var/lib/apt/lists/*

# ===== Cleanup: remove apt sympy (conflicts with Torch) ======================
RUN apt-get update && apt-get purge -y python3-sympy || true \
 && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# ===== Pip basics ============================================================
RUN python3 -m pip install --upgrade pip

# ===== Pinned NumPy + OpenCV (cv_bridge compatible) ==========================
# OpenCV wheel 4.9.0.80 works well with NumPy 1.26.4 and Ultralytics.
RUN python3 -m pip install --no-cache-dir \
    "numpy==1.26.4" "opencv-python-headless==4.9.0.80"

# ===== PyTorch CPU (x86_64, no CUDA) ========================================
# If you target ARM, set the platform via buildx or install an ARM wheel.
RUN python3 -m pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision torchaudio

# ===== ONNX Runtime + Ultralytics ===========================================
# The --upgrade-strategy only-if-needed avoids unnecessary upgrades.
RUN python3 -m pip install --no-cache-dir --upgrade-strategy only-if-needed \
    onnxruntime ultralytics

# ===== Final guard: keep NumPy and OpenCV pinned =============================
RUN python3 -m pip install --no-cache-dir --force-reinstall \
    "numpy==1.26.4" "opencv-python-headless==4.9.0.80"

# ===== Workspace =============================================================
WORKDIR /ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Expose Foxglove bridge when launching it from inside the container
EXPOSE 8765
