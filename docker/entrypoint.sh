#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# The default YOLO weights live next to the workspace; cd there so ultralytics
# finds them without re-downloading.
cd /ros_ws
exec "$@"
