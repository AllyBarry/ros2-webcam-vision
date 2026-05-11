"""Bring up v4l2_camera for a UVC webcam and the VLM detector.

Uses the v4l2_camera node (not usb_cam) with `io_method=read`. The
synchronous read() path is more tolerant of the kernel/USB quirks that
make usb_cam's mmap+select pipeline time out on some hosts.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('ros2_vlm_vision')
    detector_cfg = PathJoinSubstitution([pkg_share, 'config', 'detector.yaml'])

    # image_size must reach ROS as an IntegerArray. LaunchConfigurations
    # resolve to strings, so resolve and cast here at substitution time.
    image_width = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))

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
                'video_device': LaunchConfiguration('video_device'),
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

    return [camera_node, detector_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='Set false if v4l2_camera is already running.'),
        DeclareLaunchArgument(
            'device', default_value='cuda:0',
            description='Torch device for YOLOv8 (e.g. cuda:0, cpu).'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='V4L2 capture node. Multi-stream webcams (e.g. Logitech '
                        'Brio) expose several nodes; use `v4l2-ctl --list-devices` '
                        'to identify the capture one (typically the first '
                        'listed under the camera name).'),
        DeclareLaunchArgument(
            'image_width', default_value='1280',
            description='Capture width (set to a mode the camera supports).'),
        DeclareLaunchArgument(
            'image_height', default_value='720',
            description='Capture height (set to a mode the camera supports).'),
        DeclareLaunchArgument(
            'pixel_format', default_value='MJPG',
            description='V4L2 fourcc. MJPG is the right default at 720p+: '
                        'uncompressed YUYV at high res saturates USB '
                        'bandwidth (the Brio caps YUYV @ 1280x720 at 10 fps) '
                        'and streams freeze under WSL2/usbipd. Fall back to '
                        'YUYV @ 640x480 for cameras without MJPG.'),
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
