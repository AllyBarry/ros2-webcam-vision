#!/usr/bin/env bash
# Import the camera calibration produced by /home/jetson/ros2_ws into
# this project's camera_info/ directory (bind-mounted into the vlm
# container at /root/.ros/camera_info).
#
# What it imports:
#   - intrinsics.yaml       -> camera_info/camera.yaml
#     (converted from cv2.calibrateCamera format into ROS
#      sensor_msgs/CameraInfo format that v4l2_camera understands)
#   - T_world_camera.yaml   -> camera_info/extrinsics.yaml
#     (copied verbatim; extrinsics_publisher auto-detects the schema)
#
# Run this once after each re-calibration in the source workspace.
#
# Usage:
#   scripts/import_ros2_ws_calibration.sh [SOURCE_DIR]
#
# SOURCE_DIR defaults to /home/jetson/ros2_ws. Override if the source
# workspace lives elsewhere.

set -euo pipefail

SRC="${1:-/home/jetson/ros2_ws}"
DST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/camera_info"

if [ ! -d "$SRC" ]; then
    echo "ERROR: source directory not found: $SRC" >&2
    exit 1
fi
if [ ! -f "$SRC/intrinsics.yaml" ]; then
    echo "ERROR: $SRC/intrinsics.yaml missing -- nothing to convert." >&2
    exit 1
fi
if [ ! -f "$SRC/T_world_camera.yaml" ]; then
    echo "ERROR: $SRC/T_world_camera.yaml missing -- run calibrate.py first." >&2
    exit 1
fi

mkdir -p "$DST"

echo "[import] source workspace: $SRC"
echo "[import] target camera_info/: $DST"

# Convert intrinsics.yaml (cv2.calibrateCamera layout) -> ROS CameraInfo.
# Done in Python because we need to reshape the K matrix and pad the
# distortion coefficients; using a heredoc keeps everything self-contained.
python3 - "$SRC/intrinsics.yaml" "$DST/camera.yaml" <<'PY'
import sys, yaml
from pathlib import Path
import numpy as np

src_path = Path(sys.argv[1])
dst_path = Path(sys.argv[2])

src = yaml.safe_load(src_path.read_text())
K = np.asarray(src["camera_matrix"], dtype=float).reshape(3, 3)
D = list(src["dist_coeffs"])
width = int(src["image_width"])
height = int(src["image_height"])

# plumb_bob expects [k1, k2, p1, p2, k3]; pad if the source dropped k3.
while len(D) < 5:
    D.append(0.0)

P = np.hstack([K, np.zeros((3, 1))])  # zero baseline (mono)

out = {
    "image_width": width,
    "image_height": height,
    "camera_name": src.get("camera_name", "imported"),
    "camera_matrix": {
        "rows": 3, "cols": 3,
        "data": [float(x) for x in K.flatten().tolist()],
    },
    "distortion_model": "plumb_bob",
    "distortion_coefficients": {
        "rows": 1, "cols": len(D),
        "data": [float(x) for x in D],
    },
    "rectification_matrix": {
        "rows": 3, "cols": 3,
        "data": np.eye(3).flatten().tolist(),
    },
    "projection_matrix": {
        "rows": 3, "cols": 4,
        "data": [float(x) for x in P.flatten().tolist()],
    },
}

dst_path.write_text(yaml.safe_dump(out, default_flow_style=None, sort_keys=False))

print(f"[import] camera.yaml: {width}x{height}  "
      f"fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  "
      f"cx={K[0,2]:.1f}  cy={K[1,2]:.1f}  "
      f"k1={D[0]:.3f}  k2={D[1]:.3f}")
PY

# Copy the extrinsic verbatim -- extrinsics_publisher accepts the schema
# directly. Copy not symlink so it resolves inside the vlm container,
# which doesn't bind-mount the source workspace.
cp -f "$SRC/T_world_camera.yaml" "$DST/extrinsics.yaml"
python3 -c "
import yaml, sys
d = yaml.safe_load(open('$DST/extrinsics.yaml'))
t = d.get('translation_xyz', [0,0,0])
print(f'[import] extrinsics.yaml: parent={d.get(\"parent_frame\")}  '
      f'camera={d.get(\"camera_frame\")}  '
      f't=[{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}] m')
"

# Resolution sanity check -- the launch must capture at the same
# resolution the calibration was performed at.
python3 - "$DST/camera.yaml" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
w, h = d["image_width"], d["image_height"]
print(f"")
print(f"[import] DONE. Launch the vision stack at the matching resolution:")
print(f"  docker compose -f docker-compose.jetson.yml run --rm vlm \\")
print(f"      ros2 launch ros2_vlm_vision vision_stack.launch.py \\")
print(f"          device:=cuda:0 apriltag:=true \\")
print(f"          image_width:={w} image_height:={h}")
PY
