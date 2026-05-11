# ros2_vlm_vision (app image)
# FROMs the prebuilt base; only the package source + colcon build live
# here. Layers are ordered so the layers most likely to change come
# last: editing only entrypoint.sh or a script does not re-run colcon.
#
# Build order:
#   docker compose --profile build build base    # heavy, run once
#   docker compose build vlm                     # fast, run after edits
ARG BASE_IMAGE=ros2_vlm_vision-base:latest
FROM ${BASE_IMAGE}

# ---- ROS package: source + manifest, then colcon build -----------------
# Keeping each COPY narrow ensures editing e.g. only the README doesn't
# invalidate this layer.
WORKDIR /ros_ws/src/ros2_vlm_vision
COPY package.xml setup.py setup.cfg ./
COPY resource     ./resource
COPY ros2_vlm_vision ./ros2_vlm_vision
COPY launch       ./launch
COPY config       ./config

WORKDIR /ros_ws
RUN . /opt/ros/humble/setup.sh \
 && colcon build --packages-select ros2_vlm_vision --symlink-install

# ---- Helper scripts + entrypoint (frequent edits) ---------------------
# Placed after the colcon build so touching them doesn't trigger a rebuild.
COPY docker/entrypoint.sh /entrypoint.sh
COPY scripts/save_camera_settings.sh  /usr/local/bin/save_camera_settings.sh
RUN chmod +x /entrypoint.sh \
        /usr/local/bin/save_camera_settings.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros2_vlm_vision", "vision_stack.launch.py", "device:=cpu"]
