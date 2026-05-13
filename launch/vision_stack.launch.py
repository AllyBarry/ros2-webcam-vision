"""Bring up v4l2_camera for a UVC webcam and the VLM detector.

Uses the v4l2_camera node (not usb_cam) with `io_method=read`. The
synchronous read() path is more tolerant of the kernel/USB quirks that
make usb_cam's mmap+select pipeline time out on some hosts.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _resolve_video_device(explicit: str, name_match: str) -> str:
    """Find the right /dev/videoN, preferring stable per-device symlinks.

    The numeric /dev/videoN node a UVC webcam ends up on can change
    across replug, usbipd attach, or even within a session, but udev
    creates a stable symlink under /dev/v4l/by-id/ for each device's
    capture interfaces. 'video-index0' is the primary capture node.
    """
    if explicit and explicit != 'auto':
        return explicit

    by_id = Path('/dev/v4l/by-id')
    if by_id.is_dir():
        # Prefer entries whose name contains `name_match` (case-insensitive)
        # so a system with several UVC cameras still picks the right one.
        candidates = sorted(p for p in by_id.iterdir() if 'video-index0' in p.name)
        named = [
            p for p in candidates
            if name_match and name_match.lower() in p.name.lower()
        ]
        if named:
            return str(named[0])
        if candidates:
            return str(candidates[0])

    # No by-id symlinks (older udev, stripped container, etc.): scan
    # numeric nodes and return the first that exists.
    for n in range(10):
        p = Path(f'/dev/video{n}')
        if p.exists():
            return str(p)

    # Nothing found -- return the default and let v4l2_camera report the
    # missing-device error clearly.
    return '/dev/video0'


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('ros2_vlm_vision')
    detector_cfg = PathJoinSubstitution([pkg_share, 'config', 'detector.yaml'])

    # image_size must reach ROS as an IntegerArray. LaunchConfigurations
    # resolve to strings, so resolve and cast here at substitution time.
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))

    explicit_device = LaunchConfiguration('video_device').perform(context)
    name_match = LaunchConfiguration('device_name_match').perform(context)
    video_device = _resolve_video_device(explicit_device, name_match)

    # v4l2_camera will load any V4L2 control parameters from this YAML
    # at startup (e.g. focus_absolute, auto_exposure, brightness, ...).
    # Without it, v4l2_camera pushes its own defaults to the device,
    # which re-enables every "auto" control. Path is the bind-mounted
    # host-side camera_info/ directory.
    v4l2_params_yaml = '/root/.ros/camera_info/v4l2_params.yaml'

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[
            v4l2_params_yaml,
            {
                'video_device': video_device,
                'image_size': [image_width, image_height],
                'pixel_format': LaunchConfiguration('pixel_format'),
                'io_method': LaunchConfiguration('io_method'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'camera_frame_id': 'camera',
            },
        ],
        remappings=[
            ('image_raw', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    detector_node = Node(
        package='ros2_vlm_vision',
        executable='vlm_detector_node',
        name='vlm_detector_node',
        output='screen',
        parameters=[detector_cfg, {'device': LaunchConfiguration('device')}],
    )

    return [
        LogInfo(msg=f'[ros2_vlm_vision] resolved video_device={video_device}'),
        camera_node,
        detector_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='Set false if v4l2_camera is already running.'),
        DeclareLaunchArgument(
            'device', default_value='cuda:0',
            description='Torch device for YOLOv8 (e.g. cuda:0, cpu).'),
        DeclareLaunchArgument(
            'video_device', default_value='auto',
            description='V4L2 capture node. "auto" picks a stable per-device '
                        'symlink under /dev/v4l/by-id/ matching '
                        'device_name_match (capture interface = "-video-index0"), '
                        'or falls back to the first existing /dev/videoN. '
                        'Pass an explicit path (e.g. /dev/video2) to override.'),
        DeclareLaunchArgument(
            'device_name_match', default_value='BRIO',
            description='Substring matched (case-insensitive) against entries '
                        'in /dev/v4l/by-id/ when video_device=auto. Use an '
                        'empty string to accept any UVC capture device.'),
        DeclareLaunchArgument(
            'image_width', default_value='640',
            description='Capture width (set to a mode the camera supports).'),
        DeclareLaunchArgument(
            'image_height', default_value='480',
            description='Capture height (set to a mode the camera supports).'),
        DeclareLaunchArgument(
            'pixel_format', default_value='YUYV',
            description='V4L2 fourcc. YUYV is the safe default: the '
                        'apt-shipped ros-humble-v4l2-camera lacks MJPG '
                        'decode and aborts on the conversion path. YUYV '
                        '@ 640x480 fits USB bandwidth comfortably. Override '
                        'to MJPG only if you replace v4l2_camera with a '
                        'build that decodes MJPG.'),
        DeclareLaunchArgument(
            'io_method', default_value='read',
            description='V4L2 I/O method: read | mmap | userptr. read is the '
                        'most compatible across kernels/USB transports.'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='file:///root/.ros/camera_info/camera.yaml',
            description='Calibrated CameraInfo YAML. Defaults to the '
                        'host-mounted camera_info/ volume; produced by the '
                        'intrinsic calibration service. v4l2_camera warns '
                        'and falls back to zero intrinsics if absent.'),
        OpaqueFunction(function=_launch_setup),
    ])
