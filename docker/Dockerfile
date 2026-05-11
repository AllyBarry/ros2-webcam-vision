FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        python3-opencv \
        ros-humble-usb-cam \
        ros-humble-vision-msgs \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-image-common \
        ros-humble-camera-info-manager \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-foxglove-bridge \
        v4l-utils \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

# CPU-only torch keeps the image small. Override with a GPU base + CUDA torch
# wheels if you want to run the detector on an NVIDIA GPU inside WSL.
#
# numpy is locked to <2 across the whole resolution pass because the
# apt-shipped ros-humble-cv-bridge boost extension is built against the
# NumPy 1.x C ABI; loading it with NumPy 2.x throws
# `AttributeError: _ARRAY_API not found`. Installing in stages let later
# `pip install` calls bump numpy when torch/ultralytics pulled it as a
# transitive dep, so we install everything in one resolver run instead.
ARG PIP_OPTS="--retries 10 --timeout 120"
RUN pip3 install $PIP_OPTS \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        'numpy<2' \
        'opencv-python>=4.6,<4.13' \
        torch torchvision \
        ultralytics \
 && python3 -c "import numpy, sys; v = numpy.__version__; \
        sys.exit(0 if v.startswith('1.') else \
                 (print(f'numpy {v} leaked into the image', file=sys.stderr) or 1))"

WORKDIR /ros_ws/src
COPY . /ros_ws/src/ros2_vlm_vision

WORKDIR /ros_ws
RUN . /opt/ros/humble/setup.sh \
 && colcon build --packages-select ros2_vlm_vision --symlink-install

# Pre-fetch the default YOLOv8 weights so first run is offline-safe.
# WORKDIR is /ros_ws, so the file lands directly where the entrypoint cds to.
RUN python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
ENV YOLO_CONFIG_DIR=/ros_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros2_vlm_vision", "vision_stack.launch.py", "device:=cpu"]
