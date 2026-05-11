#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# v4l2_camera owns the V4L2 controls: it declares a ROS parameter for
# every control and pushes the parameter value to the device at startup,
# which silently overwrites anything `v4l2-ctl` set out of band. The
# launch loads camera_info/v4l2_params.yaml directly. If that file is
# missing (fresh checkout, accidental delete), create an empty stub so
# v4l2_camera still starts -- the user can tune and save afterwards.
PARAMS_FILE=/root/.ros/camera_info/v4l2_params.yaml
if [ ! -f "$PARAMS_FILE" ]; then
    mkdir -p "$(dirname "$PARAMS_FILE")"
    cat > "$PARAMS_FILE" <<'YAML'
# Auto-generated stub. Tune with `docker compose run --rm tune` then
# `docker compose run --rm save-settings` to populate.
v4l2_camera:
  ros__parameters: {}
YAML
fi

cd /ros_ws
exec "$@"
