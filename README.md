# ros2_vlm_vision

Vision module for a VLM stack on ROS 2 Humble. Consumes the colour stream
from a UVC webcam (Logitech C920 / C270 / Brio etc.) via `usb_cam`, runs
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
- [Native build (Ubuntu 22.04 + ROS 2 Humble)](#native-build-ubuntu-2204--ros-2-humble)
- [Selecting target objects](#selecting-target-objects)
- [Camera settings (focus, exposure, white balance)](#camera-settings-focus-exposure-white-balance)
- [Calibration](#calibration)
- [Customising the size table](#customising-the-size-table)
- [Parameters](#parameters)
- [Troubleshooting](#troubleshooting)

## Pipeline

```
Logitech UVC webcam ──► usb_cam
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

If your webcam enumerates capture on a node other than `/dev/video0`
(common on multi-stream models like the Brio):

```bash
docker compose run --rm vlm \
    ros2 launch ros2_vlm_vision vision_stack.launch.py \
    device:=cpu video_device:=/dev/video2
```

The default container runs CPU inference (`device:=cpu`). For NVIDIA GPU
inference, swap the base image to a CUDA-enabled ROS Humble image, install
a CUDA torch wheel, drop the `device:=cpu` override, and add the NVIDIA
runtime reservation to the `vlm` service in `docker-compose.yml`.

## Native build (Ubuntu 22.04 + ROS 2 Humble)

```bash
# System deps
sudo apt install \
    ros-humble-usb-cam \
    ros-humble-vision-msgs ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-image-common \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-foxglove-bridge python3-opencv v4l-utils

# Python deps
pip3 install ultralytics torch  # add --index-url https://download.pytorch.org/whl/cpu for CPU-only

# Build
cd <ros2_ws>/src && ln -s /path/to/ros2_vlm_vision .
cd .. && colcon build --packages-select ros2_vlm_vision
source install/setup.bash

# Confirm v4l2 sees the camera
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext   # supported resolutions/fps

# Run
ros2 launch ros2_vlm_vision vision_stack.launch.py            # GPU
ros2 launch ros2_vlm_vision vision_stack.launch.py device:=cpu
ros2 launch ros2_vlm_vision vision_stack.launch.py \
    video_device:=/dev/video2 image_width:=1920 image_height:=1080
```

If `usb_cam` is already running elsewhere, pass `launch_camera:=false`.

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

To plumb the calibrated intrinsics back into `usb_cam`, point its
`camera_info_url` at the YAML you produced:

```bash
ros2 launch ros2_vlm_vision vision_stack.launch.py \
    camera_info_url:=file:///tmp/rgb_intrinsics.yaml
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
2. `docker compose run --rm save-settings` — persist to `camera_info/camera_settings.txt`.
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

Calibration parameters in [config/calibration.yaml](config/calibration.yaml).
Launch-time arguments (`video_device`, `image_width`, `image_height`,
`framerate`, `pixel_format`, `camera_info_url`, `device`, `launch_camera`)
are documented in the `--show-args` output of the launch files.

## Troubleshooting

**`Failed to open /dev/video0`.** USB isn't visible inside the container.
From admin PowerShell: `usbipd list`, then `usbipd attach --wsl --busid
<BUSID>`. The attach is lost on Windows reboot and after a physical unplug.
Re-check with `lsusb` and `v4l2-ctl --list-devices` inside WSL.

**Webcam attaches but no `/dev/video*` appears.** The WSL2 kernel is too
old to expose `uvcvideo`. Run `wsl --update` from PowerShell, then
`wsl --shutdown` and restart.

**`Select timeout, exiting...`** usb_cam started streaming but the
kernel never delivered a frame. Usually a usbipd-win isochronous-transfer
limitation. Try a lower resolution (`image_width:=640 image_height:=480`),
make sure no other app on Windows is holding the camera, and update
usbipd-win and WSL to the latest versions.

**Capture starts but frames are corrupted / black.** The default pixel
format is `mjpeg2rgb`. Some older Logitech models only stream `yuyv` —
override with `pixel_format:=yuyv2rgb` (and use `image_width:=640
image_height:=480` so it fits in USB bandwidth).

**Detections feel close/far by a constant factor.** Almost always wrong
intrinsics — `usb_cam` defaults are placeholders. Run the intrinsic
calibration and feed the result back via `camera_info_url:=file://...`.

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
