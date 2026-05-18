# ros2_vlm_vision

Vision module for a VLM stack on ROS 2 Humble. Consumes the colour stream
from a UVC webcam (Logitech C920 / C270 / Brio etc.) via `v4l2_camera`, runs
YOLOv8 on each frame, and back-projects detections to 3D using a
class-size heuristic — i.e. for each known COCO class the node assumes a
canonical real-world height and solves Z from the bbox vertical extent
through the pinhole model.

Includes intrinsic and extrinsic calibration helpers and a Docker Compose
setup that runs the whole stack from a Windows host with optional
in-browser visualisation.

> **Limitations of the size heuristic.** Accuracy is bounded by intra-class
> variance (a sedan vs SUV) and by pose (a person standing vs lying down).
> For viewpoint-robust 3D from a single camera, swap the detector for a
> monocular-depth pipeline (e.g. Depth Anything V2). The heuristic is a
> pragmatic default for development against a webcam.

## Contents

- [Pipeline](#pipeline)
- [Topics](#topics)
- [Quick start (Docker, Windows / WSL2)](#quick-start-docker-windows--wsl2)
- [Quick start (Docker, Jetson)](#quick-start-docker-jetson)
- [Native build (Ubuntu 22.04 + ROS 2 Humble)](#native-build-ubuntu-2204--ros-2-humble)
- [AprilTag world tracking](#apriltag-world-tracking)
- [Selecting target objects](#selecting-target-objects)
- [Camera settings (focus, exposure, white balance)](#camera-settings-focus-exposure-white-balance)
- [Calibration](#calibration)
- [Per-class dimensions for picking-grade 3D output](#per-class-dimensions-for-picking-grade-3d-output)
- [Parameters](#parameters)
- [Troubleshooting](#troubleshooting)

## Pipeline

```
Logitech UVC webcam ──► v4l2_camera (io_method=read, YUYV)
                            │
                            ├─ /image_raw       (sensor_msgs/Image)
                            └─ /camera_info     (sensor_msgs/CameraInfo)
                                       │
                                       ▼
                            vlm_detector_node  (YOLOv8 + size heuristic
                                                OR ray-plane projection
                                                if extrinsics loaded)
                                       │
              ┌────────────────────────┼────────────────────────────┐
              ▼                        ▼                            ▼
   ~/detections              ~/markers                    ~/debug_image
   (Detection3DArray         (MarkerArray                 (annotated BGR)
    frame_id=world or         cubes sized from
    camera, see below)        per-class YAML)

Optional, apriltag:=true:

  /image_raw ──► rectify_node ──► apriltag_node ──┐
                                                  ├── /detections  (apriltag_msgs)
                                                  └── /tf  camera→tag36h11:<id>

  extrinsics.yaml ──► extrinsics_publisher ──── /tf static  world→camera

                              ↓ tf2 composition
                  apriltag_world_publisher
                              │
                  /apriltag_world_publisher/world_detections (Detection3DArray)
                  /apriltag_world_publisher/world_poses      (PoseArray)
                  /apriltag_world_publisher/markers          (MarkerArray)
```

The capture node is resolved at launch time. By default (`video_device:=auto`)
the launch picks a stable per-device symlink from `/dev/v4l/by-id/` matching
the `device_name_match` substring (default `BRIO`), so `/dev/videoN`
renumbering across replug / usbipd reattach is handled automatically.

Two depth-estimation paths, picked per-frame based on what's available:

- **World-frame ray-plane projection** (active when the static
  `world → camera` TF is loaded). Each detection's bottom-bbox pixel is
  ray-cast onto the world-frame table plane `z = table_z_world`,
  yielding world (X, Y) at the contact point. The centroid Z is lifted
  by `height / 2` using the per-class height from
  [`config/object_dimensions.yaml`](config/object_dimensions.yaml).
  `BoundingBox3D.size` is filled with real metres from the YAML.
- **Size heuristic fallback** (when no extrinsic TF is available).
  `Z = real_h_m * fy / bbox_h_px` with `real_h_m` from the same YAML's
  `height` field; bbox centre back-projects to camera-frame XY via
  the standard pinhole relation. Less accurate (intra-class size
  variance, atypical pose) but always available.

See [Per-class dimensions for picking-grade 3D output](#per-class-dimensions-for-picking-grade-3d-output)
for which fields the YAML uses and how the two modes interact.

## Topics

Published:

| Topic                                  | Type                              | Notes                                                     |
|---------------------------------------|-----------------------------------|-----------------------------------------------------------|
| `/vlm_detector_node/detections`        | `vision_msgs/Detection3DArray`    | Filtered by target list (or all if no filter)             |
| `/vlm_detector_node/markers`           | `visualization_msgs/MarkerArray`  | Sphere + text label per target detection (3D)             |
| `/vlm_detector_node/debug_image`       | `sensor_msgs/Image`               | Annotated frame with header overlay (lazy — needs a sub)  |
| `/vlm_detector_node/active_targets`    | `std_msgs/String`                 | Latched: current filter, empty string = no filter         |

Published when launched with `apriltag:=true` (see [AprilTag world tracking](#apriltag-world-tracking)):

| Topic                                          | Type                              | Notes                                                     |
|-----------------------------------------------|-----------------------------------|-----------------------------------------------------------|
| `/detections`                                  | `apriltag_msgs/AprilTagDetectionArray` | From `apriltag_node`, frame_id=camera                |
| `/image_rect`                                  | `sensor_msgs/Image`               | Rectified frame consumed by `apriltag_node`               |
| `/tf` (static)  `world → camera`               | from `extrinsics_publisher` (loads `extrinsics.yaml`)                        |
| `/tf` (live)    `camera → tag36h11:<id>`       | from `apriltag_node` per detected tag                                        |
| `/apriltag_world_publisher/world_detections`   | `vision_msgs/Detection3DArray`    | Tag poses in **world** frame; class_id="tag36h11:<id>"    |
| `/apriltag_world_publisher/world_poses`        | `geometry_msgs/PoseArray`         | Pose-only view, world frame                               |
| `/apriltag_world_publisher/markers`            | `visualization_msgs/MarkerArray`  | Cube + label per tag for Foxglove                         |

Subscribed:

| Topic                                  | Type                              | Notes                                                     |
|---------------------------------------|-----------------------------------|-----------------------------------------------------------|
| `/image_raw`                           | `sensor_msgs/Image`               | Webcam colour stream                                      |
| `/camera_info`                         | `sensor_msgs/CameraInfo`          | Used for `fx, fy, cx, cy` + frame_id                      |
| `/vlm_detector_node/target_classes`    | `std_msgs/String`                 | Latched filter input — see [Selecting target objects](#selecting-target-objects) |

**Detection frame depends on calibration state.** When the static
`world → camera` TF is available (from `extrinsics_publisher` loading
`camera_info/extrinsics.yaml`) and `use_table_plane=true` (default), the
detector projects each detection onto the world-frame table plane and
publishes detections with `header.frame_id=world`. Without that TF, it
falls back to the size-heuristic depth and publishes in `camera` frame.
The topic shape is identical either way — only `frame_id` and accuracy
differ. See [Per-class dimensions for picking-grade 3D output](#per-class-dimensions-for-picking-grade-3d-output).

## Quick start (Docker, Windows / WSL2)

**Prerequisites**

1. Docker Desktop with the WSL2 backend enabled.
2. [usbipd-win](https://github.com/dorssel/usbipd-win) installed on Windows.
3. A WSL2 kernel with `uvcvideo` (built in to kernels ≥ 5.15.90.1; if
   missing, run `wsl --update`).
4. Optional viz: [Foxglove Studio](https://foxglove.dev/download) (desktop
   or the web app at `https://app.foxglove.dev`).

**Attach the webcam to WSL** (admin PowerShell). Run `usbipd list`; for a
Logitech device VID is `046d`. The BUSID is in the first column (format
like `2-3`). Then:

```powershell
usbipd bind   --busid <BUSID>          # one-time, persists across reboots
usbipd attach --wsl  --busid <BUSID>   # required after each Windows boot
```

Verify inside WSL:

```bash
lsusb | grep -i logitech
v4l2-ctl --list-devices                # should show /dev/video0 (and friends)
```

**Build and run** (from the package root inside WSL):

```bash
# One-time: build the heavy base image (~10 min: apt + torch + ultralytics
# + YOLO weights). Only re-run when the dependency list in
# docker/Dockerfile.base changes.
docker compose --profile build build base

# Subsequent: fast app rebuild (~30 s) -- just colcon + COPYs.
docker compose build vlm
docker compose up vlm

# Or both in one go for the very first run:
docker compose --profile build build base && docker compose up --build vlm

docker compose --profile viz up --build        # adds foxglove bridge on :8765
```

After editing Python source / launch / config, just `docker compose build vlm`
re-uses the `ros2_vlm_vision-base:latest` image and only rebuilds the app
layer. The `base` profile keeps the heavy image hidden from the default
`docker compose up` so it isn't accidentally rebuilt.

To visualise, open Foxglove Studio on Windows, choose **Open connection →
Foxglove WebSocket**, and connect to `ws://localhost:8765`. Useful panels:

- **Image** panel → `/vlm_detector_node/debug_image`
- **3D** panel → `/vlm_detector_node/markers`
- **Raw Messages** panel → `/vlm_detector_node/detections`

**Picking the right capture node.** The Brio (and other multi-stream UVC
cameras) exposes several `/dev/videoN` nodes, and the numbering changes
on replug / usbipd reattach. The launch resolves this automatically:

```bash
# Default: auto-detect via /dev/v4l/by-id/, matching "BRIO" in the name.
docker compose up vlm

# Different camera (e.g. Logitech C920) -- change the name substring:
docker compose run --rm vlm \
    ros2 launch ros2_vlm_vision vision_stack.launch.py \
    device:=cpu device_name_match:=C920

# Any UVC capture device (no name filter):
docker compose run --rm vlm \
    ros2 launch ros2_vlm_vision vision_stack.launch.py \
    device:=cpu device_name_match:=

# Force a specific node (skip auto-detection entirely):
docker compose run --rm vlm \
    ros2 launch ros2_vlm_vision vision_stack.launch.py \
    device:=cpu video_device:=/dev/video2
```

The chosen path is logged once at startup as
`[ros2_vlm_vision] resolved video_device=/dev/v4l/by-id/usb-Logitech_BRIO_...-video-index0`.

The default container in `docker-compose.yml` runs CPU inference
(`device:=cpu`) because most x86 / WSL2 hosts lack the NVIDIA container
runtime. For native Jetson GPU inference, see
[Quick start (Docker, Jetson)](#quick-start-docker-jetson) — the separate
`docker-compose.jetson.yml` defaults to `device:=cuda:0` and wires up
the iGPU.

## Quick start (Docker, Jetson)

For NVIDIA Jetson (Orin Nano / NX / AGX) with JetPack 6.1 / 6.2 (L4T
r36.4.x). Older JetPacks need `BASE_IMAGE` and `PYPI_INDEX_URL`
overrides; see the top-of-file comments in
[docker-compose.jetson.yml](docker-compose.jetson.yml).

**Prerequisites**

1. JetPack 6.1 or 6.2 flashed (provides CUDA 12.6 + cuDNN 9 + TensorRT 10).
   Verify with `cat /etc/nv_tegra_release | head -1` — expect `R36 ... REVISION: 4.x`.
2. The NVIDIA container runtime registered with Docker:
   `docker info | grep -i 'Runtimes:'` should list `nvidia`.
3. A UVC webcam plugged in (CSI cameras need a different capture stack).

**Build and run** (from the package root):

```bash
# One-time: build the base image (~20-30 min on Orin NX). Pulls the
# ~7 GB l4t-jetpack base, apt-installs ROS Humble, pip-installs torch
# from the Jetson AI Lab cu126 mirror, pre-fetches YOLO weights.
docker compose -f docker-compose.jetson.yml --profile build build base

# Subsequent: fast app build (~5 s) -- just colcon + COPYs on top.
docker compose -f docker-compose.jetson.yml build vlm

# Bring up the vision stack -- wrapped in a script that sets the
# right defaults for the imported calibration (1280x720, cuda:0,
# apriltag tracking on). Extra args pass through:
#   scripts/bringup.sh                          # defaults
#   scripts/bringup.sh image_width:=640 image_height:=360
#   scripts/bringup.sh apriltag:=false io_method:=mmap
#   scripts/bringup.sh device:=cpu
scripts/bringup.sh

# Or directly:
docker compose -f docker-compose.jetson.yml up vlm

# In-browser visualisation (foxglove_bridge on :8765):
docker compose -f docker-compose.jetson.yml --profile viz up
```

**Importing a precomputed calibration.** If you already have intrinsics
+ extrinsics from another workspace (e.g. a `cv2.calibrateCamera` /
hand-eye flow that produced `intrinsics.yaml` and `T_world_camera.yaml`),
the importer converts them into ROS `CameraInfo` and copies them into
this repo's `camera_info/`:

```bash
# Default source: /home/jetson/ros2_ws
scripts/import_ros2_ws_calibration.sh

# Or point at a different workspace
scripts/import_ros2_ws_calibration.sh /path/to/source_ws
```

The script writes `camera_info/camera.yaml` (intrinsics, in ROS
`CameraInfo` format) and `camera_info/extrinsics.yaml` (extrinsic
transform; `extrinsics_publisher` auto-detects the schema — see
[Calibration](#calibration)). Re-run after each fresh calibration in
the source workspace. The script's final line prints the matching
launch invocation for your calibration's resolution.

**Sanity-check GPU access** through the container runtime:

```bash
docker compose -f docker-compose.jetson.yml run --rm --entrypoint python3 vlm \
    -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# expect: True Orin
```

**GUI apps from a remote PC (qv4l2 tuning, rviz2, etc.).** Per-SSH-login
setup: the X11 forwarded DISPLAY changes every login, so the container
needs a fresh xauth cookie each time. Source the helper:

```bash
source scripts/prepare_gui_session.sh
```

It auto-detects SSH-X11 vs local-desktop and exports `XAUTH` for the
`tune` service. Then:

```bash
docker compose -f docker-compose.jetson.yml --profile tune run --rm tune
```

If `prepare_gui_session.sh` reports "no xauth entry" or the SSH session
hasn't forwarded X11, re-SSH with `-Y` (and ensure your jump host
allows it):

```bash
ssh -Y -J user@jumphost user@jetson
```

Verify forwarding with `xeyes` before sourcing the helper — if `xeyes`
doesn't pop on your local PC, the SSH side isn't carrying X11 yet.

**Foxglove access from a remote PC.** When you're SSH'd into the Jetson
(directly, or through a jump host / bastion), the Foxglove WebSocket is
plain TCP, so SSH local port forwarding tunnels it the same as any
service. From your local PC:

```bash
# Direct SSH:
ssh -N -L 8765:localhost:8765 user@jetson

# Through a jump host (works with any number of hops, e.g. -J h1,h2,h3):
ssh -N -L 8765:localhost:8765 -J user@jumphost user@jetson
```

Or add it once to `~/.ssh/config`:

```ssh-config
Host jetson
  ProxyJump jumphost
  LocalForward 8765 localhost:8765
```

then just `ssh -N jetson`. With the tunnel up, open Foxglove (desktop
app or <https://app.foxglove.dev>), **Open connection → Foxglove
WebSocket**, and connect to `ws://localhost:8765`. The browser hits
your PC's `localhost`, SSH carries the WebSocket frames through the
jump host, Docker forwards them to the bridge container.

If `app.foxglove.dev` refuses the `ws://` URL on its HTTPS page (some
browsers' mixed-content rule), use the **Foxglove Studio desktop app**
instead — it has no such restriction. And confirm the bridge is
actually up on the Jetson before connecting:
`docker compose -f docker-compose.jetson.yml --profile viz ps`.

## Native build (Ubuntu 22.04 + ROS 2 Humble)

```bash
# System deps
sudo apt install \
    ros-humble-v4l2-camera \
    ros-humble-vision-msgs ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-image-common \
    ros-humble-image-proc ros-humble-apriltag-ros \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-foxglove-bridge python3-opencv v4l-utils

# Python deps. numpy is pinned because the apt-shipped cv_bridge boost
# extension is built against the NumPy 1.x ABI.
pip3 install 'numpy<2' 'opencv-python>=4.6,<4.13' ultralytics torch
# add --index-url https://download.pytorch.org/whl/cpu for CPU-only torch

# Build
cd <ros2_ws>/src && ln -s /path/to/ros2_vlm_vision .
cd .. && colcon build --packages-select ros2_vlm_vision
source install/setup.bash

# Confirm v4l2 sees the camera. The launch's auto-detect prefers the
# stable by-id symlinks; list them too if present.
v4l2-ctl --list-devices
ls /dev/v4l/by-id/ 2>/dev/null    # stable per-device names

# Run
ros2 launch ros2_vlm_vision vision_stack.launch.py            # GPU, auto-detect device
ros2 launch ros2_vlm_vision vision_stack.launch.py device:=cpu
ros2 launch ros2_vlm_vision vision_stack.launch.py \
    image_width:=1920 image_height:=1080
```

If `v4l2_camera` is already running elsewhere, pass `launch_camera:=false`.

## AprilTag world tracking

Once extrinsic calibration has been run (i.e. `camera_info/extrinsics.yaml`
exists), the vision stack can continuously detect AprilTags in the live
stream and republish their poses in the **world** frame — useful for
verifying calibration, tracking tagged trays / fixtures / robot
end-effectors, or providing ground-truth pick anchors. Off by default;
opt in per-launch:

```bash
# Native:
ros2 launch ros2_vlm_vision vision_stack.launch.py apriltag:=true

# In Docker compose (override the service's default CMD):
docker compose -f docker-compose.jetson.yml run --rm vlm \
    ros2 launch ros2_vlm_vision vision_stack.launch.py device:=cuda:0 apriltag:=true
```

What gets wired up when `apriltag:=true`:

1. `image_proc/rectify_node` consumes `/image_raw` + `/camera_info`, publishes `/image_rect`.
2. `apriltag_ros/apriltag_node` consumes `/image_rect` + `/camera_info`, publishes:
   - `/detections` (apriltag_msgs/AprilTagDetectionArray) — per-detection corner pixels.
   - One TF per detected tag: `camera -> tag36h11:<id>` (live, PnP-derived).
3. `extrinsics_publisher` reads `extrinsics.yaml` (written by the latcher) and broadcasts the static `world -> camera` TF.
4. `apriltag_world_publisher` listens on `/detections`, composes `world -> camera -> tag` via tf2, and publishes the result on the three topics in the table above.

The world-frame detection topic is intentionally shaped like the YOLO
detector's (`vision_msgs/Detection3DArray` with `class_id` and `pose`),
so a downstream picker can treat both sources uniformly: subscribe to
both, dedupe by class, send the chosen 3D pose to the robot.

**Tag family & size** are configured in
[config/apriltag.yaml](config/apriltag.yaml) — the defaults match the
extrinsic calibration's anchor tag (36h11, 162 mm edge). To track
multiple tag families simultaneously, edit the YAML to add families
and update `apriltag_world_publisher`'s `tag_frame_prefix` parameter
accordingly.

**If you launch with `apriltag:=true` but `extrinsics.yaml` doesn't exist,**
`extrinsics_publisher` exits non-zero and the rest of the AprilTag chain
still runs but emits no world-frame topics (TF lookups fail). Re-run
the extrinsic calibration first:

```bash
docker compose -f docker-compose.jetson.yml --profile calibrate run --rm calibrate \
    ros2 launch ros2_vlm_vision calibration.launch.py mode:=extrinsic
```

## Selecting target objects

The detector accepts a live, comma-separated list of class names on
`/vlm_detector_node/target_classes` (`std_msgs/String`). The topic is
latched (TRANSIENT_LOCAL QoS) so the filter persists across Foxglove
reconnects and detector restarts.

**From Foxglove Studio.** Add a **Publish** panel, set:

- Topic: `/vlm_detector_node/target_classes`
- Datatype: `std_msgs/String`
- Message: `{ "data": "person, cup" }`

Click **Publish**. The debug image's top-left header switches from
`Targets: ALL` to `Targets: person, cup` and only those classes get drawn
in the bold target colour (and only those generate 3D markers /
detections). Other detections stay visible in grey unless
`show_non_targets` is false. To clear:

```json
{ "data": "" }
```

(or publish `"*"` — same effect).

**From a terminal.**

```bash
ros2 topic pub --once /vlm_detector_node/target_classes \
    std_msgs/msg/String '{data: "person, dog"}'

ros2 topic pub --once /vlm_detector_node/target_classes \
    std_msgs/msg/String '{data: ""}'         # clear
```

**On startup.** Set the `initial_target_classes` parameter in
[config/detector.yaml](config/detector.yaml):

```yaml
initial_target_classes: "person, cup, bottle"
```

The current active filter is republished on `~/active_targets` (also
latched) so you can verify the state in a Foxglove **Raw Messages**
panel.

What gets drawn:

- **Debug image** has a header overlay: `Targets: X | Matching: cls(n)
  | Detected: cls(n) ...`. Target boxes are green and thick; non-target
  boxes are grey and thin. Header counts update each frame so the
  operator can quickly see which class names are available to type into
  the filter.
- **3D markers** are spheres + `TEXT_VIEW_FACING` labels above each
  target, so the class name and confidence are readable in Foxglove's 3D
  panel without hovering.

## Camera settings (focus, exposure, white balance)

UVC webcams ship with autofocus, auto-exposure, and auto-white-balance
on. Those autos drift between frames and during calibration, which
poisons intrinsics. **Lock the camera settings before calibrating** and
before relying on detection distances.

The settings live in
[`camera_info/v4l2_params.yaml`](camera_info/v4l2_params.yaml) as a ROS
parameter file (`v4l2_camera: ros__parameters: {...}`). The launch
passes this file to `v4l2_camera` so the controls are set through the
node's own parameter system. Setting controls externally with
`v4l2-ctl` doesn't work --- `v4l2_camera` declares a ROS parameter for
every V4L2 control and pushes the parameter's default to the device at
startup, overwriting any pre-set value.

Types matter: boolean controls (e.g. `white_balance_automatic`) must be
`true`/`false`, integer / menu controls must be integers. Run
`docker compose exec vlm v4l2-ctl -d /dev/video0 --list-ctrls` to see
each control's type on your camera.

**Tune interactively (qv4l2).** WSL2/WSLg provides the X server, so
GUI apps in the container work without setup:

```bash
docker compose run --rm tune        # opens qv4l2 with live preview
```

On Jetson, run the GUI session helper once per SSH login first to
set up the XAUTH cookie:

```bash
source scripts/prepare_gui_session.sh
docker compose -f docker-compose.jetson.yml --profile tune run --rm tune
```

qv4l2 over SSH X11 paints via Mesa's software GL (llvmpipe) — the
`tune` service in `docker-compose.jetson.yml` already exports
`LIBGL_ALWAYS_SOFTWARE=1`, `QT_OPENGL=software`, and
`__GLX_VENDOR_LIBRARY_NAME=mesa` for this. Do **not** add
`QT_XCB_GL_INTEGRATION=none` — that disables Qt's GL integration
entirely and qv4l2 crashes with `Cannot create platform OpenGL
context, neither GLX nor EGL are enabled`. The current compose
defaults are the right combination.

You can also skip the GUI entirely — set V4L2 controls headlessly via
`v4l2-ctl` inside the container, then persist with `save_camera_settings.sh`:

```bash
docker compose -f docker-compose.jetson.yml --profile tune run --rm \
    --entrypoint sh tune -c '
        v4l2-ctl -d /dev/video1 --list-ctrls
        v4l2-ctl -d /dev/video1 -c focus_automatic_continuous=0 -c focus_absolute=80
        v4l2-ctl -d /dev/video1 -c auto_exposure=1 -c exposure_time_absolute=250
        v4l2-ctl -d /dev/video1 -c white_balance_automatic=0 -c white_balance_temperature=4500
        /usr/local/bin/save_camera_settings.sh /dev/video1
    '
```

In qv4l2:

1. **Controls** tab → disable every "auto" first (autofocus,
   auto-exposure, auto-white-balance).
2. Move sliders until the preview looks right. For YOLO inference,
   reasonable starting points: exposure short enough to freeze motion,
   focus dialled in on objects at your typical operating distance,
   white balance fixed.
3. Close the window when satisfied.

**Persist the chosen values.** qv4l2 doesn't save anywhere by itself;
dump the camera's current state to the ROS parameter YAML:

```bash
docker compose run --rm save-settings
```

The file at `camera_info/v4l2_params.yaml` is rewritten with every
typed control. Manual edits to that file work too --- just keep the
type discipline (booleans as `true`/`false`, ints as ints).

**Apply outside the GUI.** The next `docker compose up vlm` (or
`docker compose restart vlm`) reapplies the file. To change a single
control at runtime without restarting:

```bash
docker compose exec vlm bash -c \
    'source /opt/ros/humble/setup.bash && \
     ros2 param set /v4l2_camera exposure_time_absolute 200'
```

`v4l2_camera` propagates the new parameter to the device immediately
(this is the supported path; `v4l2-ctl -c` is overwritten on the next
parameter sync).

## Calibration

UVC webcams ship without usable factory intrinsics; **run the intrinsic
calibration before relying on the 3D output** — the heuristic's accuracy
depends on a correct `fy`. Calibration must happen *after* you've locked
the camera settings (focus especially — calibrating with autofocus on
produces garbage intrinsics).

Print a chessboard (defaults: 9×6 inner corners, 25 mm squares) and adjust
`board_cols`, `board_rows`, `square_size_m` in
[config/calibration.yaml](config/calibration.yaml) if yours differs.

```bash
# Intrinsics — collects diverse views and writes camera_info/camera.yaml
# (auto-loaded by v4l2_camera on next start).
ros2 launch ros2_vlm_vision calibration.launch.py mode:=intrinsic

# Extrinsic — rectify -> apriltag_ros -> apriltag_extrinsic_latcher.
# Tape an AprilTag where you want the world origin to be, run the
# command, the latcher collects N stable detections of the anchor tag,
# inverts cam->tag to get world->camera, writes
# camera_info/extrinsics.yaml, and broadcasts a static TF.
ros2 launch ros2_vlm_vision calibration.launch.py mode:=extrinsic
```

**Anchor tag setup.** The default anchor is `tag36h11:0`. Print an
AprilTag from the 36h11 family (the Wiki has PDFs:
<https://github.com/AprilRobotics/apriltag-imgs>), measure its black
square edge length precisely, and set `apriltag.size` in
[config/apriltag.yaml](config/apriltag.yaml) to that value in metres
(default `0.162`). The position you tape the tag at *is* the world
origin; the tag plane is the world X-Y, with +Z out of the tag.

Override the anchor tag ID via the launch arg if you'd rather use a
different one (e.g. `apriltag_extrinsic_latcher:anchor_tag_frame:=tag36h11:5`).

**What gets published.** Once the latcher finalises:

- Static TF: `world -> camera` (the camera's pose in the world frame).
- YAML at `camera_info/extrinsics.yaml` with both `world_T_camera` and
  `camera_T_world` for callers that need either direction without
  re-inverting.

### Reusing a calibration produced elsewhere

`extrinsics_publisher` accepts two YAML layouts so the same node loads
either calibrations produced by this repo's `apriltag_extrinsic_latcher`
or by external tools (e.g. `cv2.calibrateCamera` + wrist-tag PnP
workflows like the one in `/home/jetson/ros2_ws`):

- **Schema A** — nested `world_T_camera.{translation_xyz_m, rotation_xyzw}` block (what `apriltag_extrinsic_latcher` writes).
- **Schema B** — top-level `translation_xyz` + `rotation_quat_xyzw` with `parent_frame` / `camera_frame` siblings (the `ros2_ws/calibrate.py` format).

The auto-detection picks whichever is present and logs the schema used.
To import an external calibration, the simplest path is
[scripts/import_ros2_ws_calibration.sh](scripts/import_ros2_ws_calibration.sh):

```bash
scripts/import_ros2_ws_calibration.sh                       # default: /home/jetson/ros2_ws
scripts/import_ros2_ws_calibration.sh /path/to/source_ws    # override
```

This converts the source `intrinsics.yaml` into ROS `CameraInfo` format
at `camera_info/camera.yaml` and copies `T_world_camera.yaml` to
`camera_info/extrinsics.yaml`. Re-run after each fresh calibration; the
final line prints the exact launch invocation matching the calibration
resolution.

`vlm_detector_node` publishes its detections in `frame_id=camera`. With
the static TF in place, any tf2 listener (or Foxglove's 3D panel set to
`world` as the display frame) sees the detections in world coordinates
automatically.

The `intrinsic_calibration_node` writes to `camera_info/camera.yaml`,
which is exactly where `v4l2_camera`'s `camera_info_url` already points by
default — so the calibrated intrinsics get loaded automatically on the
next `docker compose up vlm`. To use a calibration from a different path:

```bash
ros2 launch ros2_vlm_vision vision_stack.launch.py \
    camera_info_url:=file:///root/.ros/camera_info/my_camera.yaml
```

Run them through the dedicated compose service (it shares the same
`camera_info/` volume, so the produced YAML lands next to your settings
file and is auto-loaded by the `vlm` service on its next start):

```bash
docker compose --profile calibrate run --rm calibrate                      # intrinsic by default
docker compose --profile calibrate run --rm calibrate \
    ros2 launch ros2_vlm_vision calibration.launch.py mode:=extrinsic      # override for extrinsic
```

Recommended order on a fresh setup:

1. `docker compose run --rm tune` — adjust focus / exposure / white balance.
2. `docker compose run --rm save-settings` — persist to `camera_info/v4l2_params.yaml`.
3. `docker compose --profile calibrate run --rm calibrate` — produce intrinsics.
4. `docker compose --profile viz up` — run the stack with both applied.

## Per-class dimensions for picking-grade 3D output

Once extrinsic calibration is loaded, the detector publishes detections
in the **world frame** (`frame_id=world`). The 3D estimate uses **ray-
plane back-projection against the calibrated table**, not the class-size
heuristic — so the (X, Y) is geometry-driven and accurate to the
extrinsic's RMSE at the pick location. The **per-class dimensions YAML**
enhances this in three concrete ways:

1. **Lifts the centroid above the table** by `height / 2`, so the pose's
   Z is the object centroid (what most grasp planners want), not the
   table contact point.
2. **Populates `BoundingBox3D.size`** with real metres (width × depth ×
   height) — MoveIt collision avoidance and any other consumer that
   reads `vision_msgs/Detection3D` gets real dimensions, not pixel-
   derived approximations.
3. **Drives Foxglove markers as real-size CUBEs** at the correct world
   pose, so what you see in the 3D panel is the actual occupied volume.

Dimensions live in
[`config/object_dimensions.yaml`](config/object_dimensions.yaml). Edit
to add custom classes (when using non-COCO YOLO weights) or to tune for
your domain — e.g., if you only see baby carrots, drop `carrot.width`
from 0.18 m to 0.07 m. Each entry:

```yaml
classes:
  carrot:    { height: 0.025, width: 0.18, depth: 0.03, shape: cylinder }
  apple:     { height: 0.075, width: 0.075, depth: 0.075, shape: sphere }
  # ... height is required; width/depth/shape are optional.
```

Classes not in the YAML still get a world-frame pose via the
`defaults` block; a once-per-class warning is logged so you know which
entries to add. The `shape` field is informational at the moment —
planners can use it to choose an approach strategy (top-pick for boxes,
side-pinch for cylinders, etc.).

**When extrinsic calibration *isn't* loaded** (no `world → camera` TF),
the detector falls back to the legacy depth-from-height heuristic in
the camera frame, using the same YAML's `height` values for `Z =
real_h × fy / bbox_h_px`. Detections still publish; accuracy is just
the pre-calibration class-size-heuristic level. Once you've run the
extrinsic calibration and `extrinsics_publisher` is in the launch tree,
the detector silently upgrades to the world-frame ray-plane path on
the next frame.

The `~/detections` topic's shape (`vision_msgs/Detection3DArray`) does
not change between the two modes — only the `header.frame_id` (camera
vs world) and the precision of the contents. Downstream code that
subscribes can be written for either frame and just tf2-transform if
needed.

## Parameters

Detector ([config/detector.yaml](config/detector.yaml)):

| Parameter         | Default        | Description                                         |
|-------------------|----------------|-----------------------------------------------------|
| `model`           | `yolov8n.pt`   | Any ultralytics-loadable weight                     |
| `device`          | `cuda:0`       | `cpu`, `cuda:0`, `mps`                              |
| `conf_threshold`  | `0.35`         | YOLO confidence threshold                           |
| `iou_threshold`   | `0.45`         | YOLO NMS IoU threshold                              |
| `max_distance`    | `25.0`         | Drop detections whose estimated Z exceeds this (m)  |
| `image_topic`     | `/image_raw`   |                                                     |
| `info_topic`      | `/camera_info` |                                                     |
| `camera_frame`    | `camera`       | Fallback frame_id if camera_info has none           |
| `initial_target_classes` | `""`    | Startup filter; live overrides via `~/target_classes` |
| `show_non_targets` | `true`        | Draw non-target detections faded in the debug image |
| `use_table_plane`  | `true`        | When the static `world → camera` TF is loaded, project detections onto the world-frame table plane (ray-plane intersection). Falls back to size-heuristic in camera frame if TF unavailable. |
| `table_z_world`    | `0.0`         | Height (m) of the table surface in the world frame. 0 is correct when the apriltag anchor is taped flat on the table during extrinsic calibration. |
| `world_frame`      | `world`       | TF frame to transform detections into when `use_table_plane` is active. |
| `contact_point`    | `bottom_center` | Pixel inside the bbox treated as the table contact: `bottom_center` `(u_mid, y2)` or `center` `(u_mid, v_mid)`. |
| `dimensions_path`  | `""` (bundled) | Path to per-class dimensions YAML. Empty = use the file shipped with the package. |

Camera-driver controls live in
[camera_info/v4l2_params.yaml](camera_info/v4l2_params.yaml) and are loaded
into the `v4l2_camera` node at launch.

Launch arguments (`vision_stack.launch.py --show-args` lists everything):

| Argument            | Default                                              | Notes                                                |
|---------------------|------------------------------------------------------|------------------------------------------------------|
| `video_device`      | `auto`                                               | `auto` resolves via `/dev/v4l/by-id/`; or pass a literal `/dev/videoN`. |
| `device_name_match` | `BRIO`                                               | Substring matched against by-id entries when auto-resolving. Empty = first capture device. |
| `image_width`       | `640`                                                | Must match a mode the camera supports. **Use `1280` if your `camera_info/camera.yaml` was calibrated at 1280×720** (otherwise `rectify_node` rejects K as "uncalibrated"). |
| `image_height`      | `480`                                                | See above for matched-resolution constraint. |
| `pixel_format`      | `YUYV`                                               | Safe default — the apt-shipped `ros-humble-v4l2-camera` lacks MJPG decode. Cameras that only stream MJPG at high resolution will throttle to a few Hz under YUYV. |
| `apriltag`          | `false`                                              | When `true`, runs `rectify_node` + `apriltag_node` + `extrinsics_publisher` + `apriltag_world_publisher` (see [AprilTag world tracking](#apriltag-world-tracking)). |
| `extrinsics_yaml`   | `/root/.ros/camera_info/extrinsics.yaml`             | Used when `apriltag:=true`; consumed by `extrinsics_publisher` to broadcast `world → camera` static TF. |
| `use_table_plane`   | `true`                                               | World-frame ray-plane projection vs. legacy size heuristic. See detector params table above. |
| `table_z_world`     | `0.0`                                                | Table height in world frame, metres. |
| `world_frame`       | `world`                                              | Output TF frame when table-plane projection is active. |
| `io_method`         | `read`                                               | `read \| mmap \| userptr` — `read` is most portable. |
| `camera_info_url`   | `file:///root/.ros/camera_info/camera.yaml`          | Intrinsic YAML; auto-loaded if present.              |
| `launch_camera`     | `true`                                               | Set `false` if `v4l2_camera` is running externally.  |
| `device`            | `cuda:0`                                             | Torch device for YOLOv8 (`cpu`, `cuda:0`, `mps`).    |

Calibration parameters in [config/calibration.yaml](config/calibration.yaml).

## Troubleshooting

**`Failed opening device /dev/videoN: No such file or directory`.** The
camera detached from WSL. From admin PowerShell: `usbipd list`, then
`usbipd attach --wsl --busid <BUSID>`. The attach is lost on Windows
reboot and after a physical unplug. Re-check with `lsusb` and
`v4l2-ctl --list-devices` inside WSL.

**`Failed opening device ...` even though `lsusb` shows the camera.** The
device renumbered (the Brio in particular shuffles between `/dev/video0..3`
across reattach). The launch's `video_device:=auto` default already
handles this via `/dev/v4l/by-id/`; check the startup log for the
`[ros2_vlm_vision] resolved video_device=...` line to see what got
picked. If by-id symlinks are missing on your system, override with an
explicit `video_device:=/dev/videoN`.

**Webcam attaches but no `/dev/video*` appears.** The WSL2 kernel is too
old to expose `uvcvideo`. Run `wsl --update` from PowerShell, then
`wsl --shutdown` and restart.

**`Select timeout` / `topic hz` shows nothing on `/image_raw`.** The
streaming layer is stuck — `v4l2_camera`'s `read()` is blocked waiting
for a frame. Usually a usbipd-win isochronous-transfer limitation. Try a
lower resolution (`image_width:=640 image_height:=480`), confirm no
other app on Windows is holding the camera, and `wsl --update`. The
detector's `/vlm_detector_node/active_targets` is latched and published
at startup, so that should always be visible even when the camera is
stuck — its absence indicates a process-level failure instead.

**Capture starts but frames are corrupted / black.** The default pixel
format is `YUYV`. Some cameras only stream high-resolution modes via
`MJPG`; the apt-shipped `ros-humble-v4l2-camera` lacks MJPG decode, so
you'll see frame drops or black output. Drop resolution
(`image_width:=640 image_height:=480`) to stay in YUYV, or source-build
v4l2_camera with MJPG decode for full-res capture.

**Autofocus / autoexposure won't stay off.** `v4l2_camera` declares
every V4L2 control as a ROS parameter and pushes the parameter's
default to the device on startup; setting controls via `v4l2-ctl -c` is
silently overwritten. Edit `camera_info/v4l2_params.yaml` instead (or
run `docker compose run --rm save-settings` after tuning).

**Detections feel close/far by a constant factor.** Almost always wrong
intrinsics — `v4l2_camera`'s default is a placeholder when no
calibration YAML exists. Run the intrinsic calibration; the result
auto-loads next start.

**Distances are accurate for some classes but wildly off for others.**
The per-class dimensions don't match your specimens. Edit
[`config/object_dimensions.yaml`](config/object_dimensions.yaml) and
relaunch — see [Per-class dimensions for picking-grade 3D output](#per-class-dimensions-for-picking-grade-3d-output).

**Detector starts but no detections.** Confirm `camera_info` is arriving
(`ros2 topic hz /camera_info`); the node waits for it before processing
images. Check `~/debug_image` — if it's blank, the model isn't seeing any
classes above `conf_threshold`.

**`No dimensions for class 'X' in object_dimensions.yaml; using defaults`.**
YOLO returned a class that isn't in your per-class dimensions YAML
(custom weights or an out-of-COCO class). The detector still publishes
a 3D pose using the default dimensions, but the centroid height +
bbox size are placeholders. Add an entry for the class to
[`config/object_dimensions.yaml`](config/object_dimensions.yaml) and
relaunch.

**Foxglove can't connect.** Confirm port 8765 is published
(`docker compose --profile viz ps`) and that nothing else on Windows is
binding the same port. The bridge logs every connection attempt.

**Wrong torch device.** The Docker default is `device:=cpu`. Override at
the service level (`command:` in compose) or pass `device:=cuda:0` to the
launch file if running natively.

**NumPy 1.x vs 2.x ABI error** (`_ARRAY_API not found`). Rebuild without
the build cache so the `numpy<2` pin in the Dockerfile takes effect:

```bash
docker compose build --no-cache vlm
docker compose run --rm --entrypoint python3 vlm -c "import numpy; print(numpy.__version__)"
# expect 1.26.x
```

**Jetson: `NvMapMemAllocInternalTagged: error 12` / `NVML_SUCCESS == r`
on first GPU allocation.** The Jetson iGPU shares its memory pool with
the system through the NvMap / CMA allocator. When other GPU-using
processes on the host (camera nodes doing hardware MJPEG decode,
`image_proc` rectification, anything calling `cv::cuda::*`) are
holding the pool, the container can't get a contiguous block and
torch's caching allocator aborts. `import torch` and
`torch.cuda.is_available()` still succeed — it's only the first real
allocation that fails. Find culprits on the host (not in the
container) with:

```bash
ps aux --sort=-%mem | head -15      # look for *_node processes hogging RAM/CPU
```

Stop the offending host processes, or reboot to reset CMA fragmentation.
This is not a Dockerfile / image issue — the same container works fine
once the iGPU pool is free.

**Jetson: torch `libcudss.so.0` / `libcusparse_lt.so.X` /
`libopenblas.so.0: cannot open shared object file`.** A torch version
that pulls in a runtime dep the host's JetPack doesn't ship. The pinned
`torch==2.8.0` + `torchvision==0.23.0` in
[docker/Dockerfile.base.jetson](docker/Dockerfile.base.jetson) is the
last cu126 line that doesn't need cuDSS or cuSPARSELt; do not bump it
without first confirming the new wheel's dlopen list against the
host's JetPack release. `libopenblas0` is already in the apt list.

**`rectify_node`: "Rectified topic '/image_rect' requested but camera
publishing '/camera_info' is uncalibrated".** `v4l2_camera` couldn't
load a valid calibration YAML — either `camera_info/camera.yaml`
is missing, or the resolution in the YAML doesn't match the launch's
`image_width`/`image_height`. After importing a 1280×720 calibration,
launch with `image_width:=1280 image_height:=720` (or use
`scripts/bringup.sh`, which sets these by default). The `camera_name`
mismatch warning (e.g. `[logitech_brio] does not match imported`) is
cosmetic — only `image_width/height` and `camera_matrix` matter.

**ROS 2 topics across PCs: same `ROS_DOMAIN_ID` but topics not visible
on either side.** Most often, the publishing container is on a Docker
**bridge** network (the default `networks: [ros]` in the `vlm`
service): DDS multicast doesn't traverse NAT cleanly. Three fixes,
fastest to most invasive:

1. **Switch `vlm` to `network_mode: host`** in
   [docker-compose.jetson.yml](docker-compose.jetson.yml) — the container
   gets the host's network stack, multicast discovery just works:

   ```yaml
   vlm:
     <<: *vlm_base
     container_name: vlm_vision
     network_mode: host        # was: networks: [ros]
   ```

2. **Check `ROS_LOCALHOST_ONLY`** on both PCs. If `1`, DDS is
   loopback-only — `unset ROS_LOCALHOST_ONLY` on the affected side.

3. **Match `RMW_IMPLEMENTATION`** on both PCs (e.g.
   `rmw_fastrtps_cpp` everywhere). Different RMWs interoperate via
   RTPS in theory but have had bugs.

If `network_mode: host` doesn't fix it, also check firewalls
(default Fast DDS uses UDP 7400 for discovery + dynamic UDP 7410+ for
data) and that the LAN switch isn't blocking multicast — `iperf -u
-B 239.255.0.1` can confirm multicast reaches the peer.
