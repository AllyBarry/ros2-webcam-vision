#!/usr/bin/env bash
# Bring up the vision stack with the precomputed calibration.
#
# Wraps `docker compose run vlm ros2 launch vision_stack.launch.py` with
# the defaults that match the ros2_ws calibration we imported via
# scripts/import_ros2_ws_calibration.sh:
#
#   device:=cuda:0          GPU inference on the Jetson iGPU
#   apriltag:=true          run apriltag_node + extrinsics_publisher +
#                           apriltag_world_publisher; loads the static
#                           world->camera TF and republishes tag
#                           detections in world frame
#   image_width:=1280       must match camera_info/camera.yaml resolution
#   image_height:=720       (the ros2_ws intrinsic was calibrated at this
#                           size; capturing smaller publishes a K that
#                           rectify_node rejects as "uncalibrated")
#
# Usage:
#   scripts/bringup.sh                              # defaults + foxglove
#   scripts/bringup.sh image_width:=640 image_height:=360  # half-res
#   scripts/bringup.sh apriltag:=false              # YOLO only, no tags
#   scripts/bringup.sh io_method:=mmap              # if YUYV throttles at HD
#   scripts/bringup.sh device:=cpu                  # CPU fallback
#   BRINGUP_VIZ=0 scripts/bringup.sh                # skip foxglove
#
# Extra positional args are passed through to the launch file verbatim.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# Sanity-check the calibration files are in place. The launch can still
# come up without them (extrinsics_publisher exits non-zero; the rest
# still runs), but warn early so the user knows to import first.
missing=()
[ -f camera_info/camera.yaml ]     || missing+=("camera_info/camera.yaml")
[ -f camera_info/extrinsics.yaml ] || missing+=("camera_info/extrinsics.yaml")
if [ ${#missing[@]} -gt 0 ]; then
    echo "[bringup] WARNING: missing calibration files:" >&2
    for f in "${missing[@]}"; do echo "  - $f" >&2; done
    echo "[bringup] Run: scripts/import_ros2_ws_calibration.sh" >&2
    echo "[bringup] Continuing anyway -- the stack will start but" \
         "world-frame projection won't be available." >&2
fi

# Start the Foxglove WebSocket bridge in the background so a browser
# can connect to ws://<jetson>:8765 while the vlm launch runs in the
# foreground. --no-deps because foxglove's depends_on: [vlm] would
# otherwise spin up a *second* vlm container in detached mode,
# colliding with the foreground vlm started below. With host
# networking, both share the same DDS stack -- foxglove will discover
# vlm's topics the moment the launch publishes them.
#
# Non-fatal if it fails (e.g. port 8765 already bound by a previous
# session): the vlm launch still comes up; you just don't get viz.
# Override with BRINGUP_VIZ=0 to skip foxglove entirely.
foxglove_started=0
if [ "${BRINGUP_VIZ:-1}" = 1 ]; then
    if docker compose -f docker-compose.jetson.yml --profile viz \
            up -d --no-deps foxglove >/dev/null 2>&1; then
        foxglove_started=1
        echo "[bringup] foxglove bridge up on :8765 (ws://localhost:8765)"
    else
        echo "[bringup] WARNING: foxglove failed to start -- continuing without viz." >&2
        echo "[bringup]   Check: docker compose -f docker-compose.jetson.yml logs foxglove" >&2
    fi
fi

# Stop foxglove on exit so the next bringup starts cleanly. Comment
# out the trap if you'd rather keep foxglove running between vlm
# restarts (so the browser doesn't have to reconnect).
cleanup() {
    if [ "$foxglove_started" = 1 ]; then
        docker compose -f docker-compose.jetson.yml stop foxglove >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

# Foreground vlm run -- Ctrl-C / docker compose stops propagate
# through, then the trap stops foxglove.
docker compose -f docker-compose.jetson.yml --profile viz run vlm \
    ros2 launch ros2_vlm_vision vision_stack.launch.py \
        device:=cuda:0 \
        apriltag:=true \
        image_width:=1280 \
        image_height:=720 \
        "$@"
