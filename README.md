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
- [Selecting target objects](#selecting-target-objects)
- [Camera settings (focus, exposure, white balance)](#camera-settings-focus-exposure-white-balance)
- [Calibration](#calibration)
- [Customising the size table](#customising-the-size-table)
- [Parameters](#parameters)
- [Troubleshooting](#troubleshooting)

## Pipeline

```
Logitech UVC webcam ──► v4l2_camera (io_method=read, MJPG)
                            │
                            ├─ /image_raw       (sensor_msgs/Image)
                            └─ /camera_info     (sensor_msgs/CameraInfo)
                                       │
                                       ▼
                            vlm_detector_node  (YOLOv8 + size heuristic)
                                       │
              ┌────────────────────────┼────────────────────────────┐
              ▼                        ▼                            ▼
   ~/detections              ~/markers                    ~/debug_image
   (Detection3DArray)        (MarkerArray)                (annotated BGR)
```

The capture node is resolved at launch time. By default (`video_device:=auto`)
the launch picks a stable per-device symlink from `/dev/v4l/by-id/` matching
the `device_name_match` substring (default `BRIO`), so `/dev/videoN`
renumbering across replug / usbipd reattach is handled automatically.

Distance is computed per detection as `Z = real_h_m * fy / bbox_h_px`,
where `real_h_m` comes from [`utils/object_sizes.py`](ros2_vlm_vision/utils/object_sizes.py)
and `fy` from `camera_info`. The bbox centre is then back-projected:
`X = (u - cx) * Z / fx`, `Y = (v - cy) * Z / fy`. Detections whose class is
not in the size table are dropped from the 3D output (logged once).

## Topics

Published:

| Topic                                  | Type                              | Notes                                                     |
|---------------------------------------|-----------------------------------|-----------------------------------------------------------|
| `/vlm_detector_node/detections`        | `vision_msgs/Detection3DArray`    | Filtered by target list (or all if no filter)             |
| `/vlm_detector_node/markers`           | `visualization_msgs/MarkerArray`  | Sphere + text label per target detection (3D)             |
| `/vlm_detector_node/debug_image`       | `sensor_msgs/Image`               | Annotated frame with header overlay (lazy — needs a sub)  |
| `/vlm_detector_node/active_targets`    | `std_msgs/String`                 | Latched: current filter, empty string = no filter         |

Subscribed:

| Topic                                  | Type                              | Notes                                                     |
|---------------------------------------|-----------------------------------|-----------------------------------------------------------|
| `/image_raw`                           | `sensor_msgs/Image`               | Webcam colour stream                                      |
| `/camera_info`                         | `sensor_msgs/CameraInfo`          | Used for `fx, fy, cx, cy` + frame_id                      |
| `/vlm_detector_node/target_classes`    | `std_msgs/String`                 | Latched filter input — see [Selecting target objects](#selecting-target-objects) |

Header `frame_id` defaults to `camera` (overridable via the `camera_frame`
parameter or by `camera_info.header.frame_id`).

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
docker compose -f docker-compose.jetson.yml up vlm

# In-browser visualisation (foxglove_bridge on :8765):
docker compose -f docker-compose.jetson.yml --profile viz up
```

**Sanity-check GPU access** through the container runtime:

```bash
docker compose -f docker-compose.jetson.yml run --rm --entrypoint python3 vlm \
    -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# expect: True Orin
```

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

## Customising the size table

The default table in
[`ros2_vlm_vision/utils/object_sizes.py`](ros2_vlm_vision/utils/object_sizes.py)
covers all 80 COCO classes with median upright heights. Two reasons to
edit it:

- **Domain-specific instances.** If you only see compact cars in your
  testbed, lower `'car'` from 1.5 m to 1.4 m.
- **Custom YOLO weights** with non-COCO classes. Add entries keyed by your
  model's class names (matched against `result.names`).

Detections whose class is not in the table are dropped from the 3D output
with a one-time warning per class.

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

Camera-driver controls live in
[camera_info/v4l2_params.yaml](camera_info/v4l2_params.yaml) and are loaded
into the `v4l2_camera` node at launch.

Launch arguments (`vision_stack.launch.py --show-args` lists everything):

| Argument            | Default                                              | Notes                                                |
|---------------------|------------------------------------------------------|------------------------------------------------------|
| `video_device`      | `auto`                                               | `auto` resolves via `/dev/v4l/by-id/`; or pass a literal `/dev/videoN`. |
| `device_name_match` | `BRIO`                                               | Substring matched against by-id entries when auto-resolving. Empty = first capture device. |
| `image_width`       | `1280`                                               | Must match a mode the camera supports.               |
| `image_height`      | `720`                                                |                                                      |
| `pixel_format`      | `MJPG`                                               | `YUYV` is the fallback for cameras without MJPG.     |
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
format is `MJPG`. Some older Logitech models only stream `YUYV` —
override with `pixel_format:=YUYV` (and use `image_width:=640
image_height:=480` so it fits in USB bandwidth).

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
The size in `object_sizes.py` doesn't match what you're seeing. Edit it.

**Detector starts but no detections.** Confirm `camera_info` is arriving
(`ros2 topic hz /camera_info`); the node waits for it before processing
images. Check `~/debug_image` — if it's blank, the model isn't seeing any
classes above `conf_threshold`.

**`No size entry for class 'X'`.** Either YOLO is returning a class the
size table doesn't cover (custom weights), or a class genuinely outside
COCO. Add it to `object_sizes.py`.

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
