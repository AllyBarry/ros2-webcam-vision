#!/usr/bin/env bash
set -e

# osrf's ros:humble-* (CPU build) places setup.bash at /opt/ros/humble/.
# dustynv's L4T images (Jetson build) build ROS from source and place it
# at /opt/ros/humble/install/setup.bash. Source whichever exists.
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/humble/install/setup.bash ]; then
    source /opt/ros/humble/install/setup.bash
fi
# Jetson overlay: cv_bridge, image_pipeline, v4l2_camera, apriltag_ros
# source-built against dustynv's OpenCV. Absent on the CPU build, so
# this is conditional.
if [ -f /opt/ros_overlay/install/setup.bash ]; then
    source /opt/ros_overlay/install/setup.bash
fi
# App workspace: only present once the app's colcon build has run.
# In the simplified Jetson Dockerfile the colcon step is commented out
# until the base image's overlay is ready, so this is conditional too.
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

# v4l2_camera owns the V4L2 controls: it declares a ROS parameter for
# every control and pushes the parameter value to the device at startup,
# which silently overwrites anything `v4l2-ctl` set out of band. The
# launch loads camera_info/v4l2_params.yaml directly. If that file is
# missing (fresh checkout, accidental delete), seed it with auto modes
# OFF so v4l2_camera comes up cleanly. Two reasons not to leave it empty:
#   1. v4l2_camera reads every control's value at startup; auto-mode-
#      inhibited manual controls (focus_absolute when autofocus is on,
#      exposure_time_absolute under auto exposure, ...) return EACCES
#      and the driver logs a noisy [ERROR] "Permission denied" per
#      control. The values are still "set" -- it's read-back that fails.
#   2. YOLO and the size-heuristic depth back-projection both want
#      stable framing; autofocus hunting and auto-exposure ramps shift
#      sharpness and brightness between frames. Run `tune` once to dial
#      these in for your actual lighting and `save-settings` to persist.
PARAMS_FILE=/root/.ros/camera_info/v4l2_params.yaml
if [ ! -f "$PARAMS_FILE" ]; then
    mkdir -p "$(dirname "$PARAMS_FILE")"
    cat > "$PARAMS_FILE" <<'YAML'
v4l2_camera:
  ros__parameters:
    auto_exposure: 1            # 1 = manual exposure (3 = aperture priority/auto)
    exposure_time_absolute: 250
    exposure_dynamic_framerate: false
    focus_automatic_continuous: false
    focus_absolute: 0
    white_balance_automatic: true
    power_line_frequency: 2     # 1 = 50 Hz, 2 = 60 Hz; adjust to your mains
YAML
fi

cd /ros_ws
exec "$@"
