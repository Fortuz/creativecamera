# ===== Base image ============================================================
FROM osrf/ros:humble-desktop

# Kényelmi env + noninteractive apt
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1

# ===== System deps (ROS + runtime libs) =====================================
# NOTE: szándékosan NEM telepítünk apt-ból python3-opencv, python3-numpy-t
# hogy ne keveredjenek a pip-es csomagokkal.
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

# ===== Takarítás: az apt-os sympy eltávolítása (ütközik a Torch-csal) =======
RUN apt-get update && apt-get purge -y python3-sympy || true \
 && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# ===== Pip alapok ===========================================================
RUN python3 -m pip install --upgrade pip

# ===== Pinned NumPy + OpenCV (cv_bridge kompatibilis) =======================
# OpenCV wheel 4.9.0.80 jól működik NumPy 1.26.4-gyel és Ultralytics-szel is.
RUN python3 -m pip install --no-cache-dir \
    "numpy==1.26.4" "opencv-python-headless==4.9.0.80"

# ===== PyTorch CPU (x86_64, CUDA nélkül) ====================================
# Ha ARM-ra célzol, buildx-szel állíts platformot, vagy használj ARM-kompatibilis wheel-t.
RUN python3 -m pip install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision torchaudio

# ===== ONNX Runtime + Ultralytics ===========================================
# Az --upgrade-strategy only-if-needed visszafogja a felülírásokat.
RUN python3 -m pip install --no-cache-dir --upgrade-strategy only-if-needed \
    onnxruntime ultralytics

# ===== Végső “guard”: tartsuk lent a NumPy-t és az OpenCV-t ================
RUN python3 -m pip install --no-cache-dir --force-reinstall \
    "numpy==1.26.4" "opencv-python-headless==4.9.0.80"

# ===== Workspace =============================================================
WORKDIR /ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Hasznos, ha Foxglove-ot a konténerből indítod:
EXPOSE 8765
