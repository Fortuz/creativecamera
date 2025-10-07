# ===== Base image ============================================================
FROM osrf/ros:humble-desktop

# K?nyelmi env-k + noninteractive apt
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# ===== System deps a creativecamera-hoz =====================================
# ROS ?s alap Python/vision csomagok
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
    # OpenCV/FFmpeg futtat?si libek (pip-es csomagoknak is kellhet)
    ffmpeg libsm6 libxext6 libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# ===== Pip alapok ===========================================================
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install "numpy==1.26.4" onnxruntime

# ===== ?tk?z?s elh?r?t?sa: Sympy (apt) -> PyTorch ?j Sympy ==================
# A base image-ben jellemz?en van apt-os python3-sympy (1.9),
# amit a pip nem tud elt?vol?tani. Purgel?nk, hogy a pip feltehesse az ?jat.
RUN apt-get update && apt-get purge -y python3-sympy && \
    apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# ===== PyTorch CPU (x86_64, CUDA n?lk?l) ====================================
# Ha ARM/Apple Silicon buildet c?lzol, itt platformot kellhet ?ll?tani buildx-szel,
# vagy k?l?n ARM-kompatibilis torch wheelt haszn?lni.
RUN python3 -m pip install --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision torchaudio

# ===== Ultralytics ==========================================================
# Ha b?rmi OpenCV konfliktust tapasztalsz, kipr?b?lhatod headless-szel:
#   python3 -m pip install --upgrade opencv-python-headless
RUN python3 -m pip install ultralytics

# ===== Workspace =============================================================
WORKDIR /ws

# Forr?soljuk a ROS-t minden shellben
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

