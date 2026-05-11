"""Bring up usb_cam for a UVC webcam (Logitech etc.) and the VLM detector."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vlm_vision')
    detector_cfg = PathJoinSubstitution([pkg_share, 'config', 'detector.yaml'])

    launch_camera = DeclareLaunchArgument(
        'launch_camera', default_value='true',
        description='Set false if usb_cam is already running.')
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Torch device for YOLOv8 (e.g. cuda:0, cpu).')
    video_device = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='V4L2 device node for the webcam.')
    image_width = DeclareLaunchArgument(
        'image_width', default_value='1280',
        description='Capture width (set to a mode the camera supports).')
    image_height = DeclareLaunchArgument(
        'image_height', default_value='720',
        description='Capture height (set to a mode the camera supports).')
    framerate = DeclareLaunchArgument(
        'framerate', default_value='30.0',
        description='Capture framerate (fps).')
    # Default to MJPG: YUYV is uncompressed and saturates USB bandwidth at
    # 720p (most UVC webcams cap YUYV at 10 fps for that resolution). For
    # cameras without MJPG support, fall back to pixel_format:=yuyv2rgb
    # image_width:=640 image_height:=480.
    pixel_format = DeclareLaunchArgument(
        'pixel_format', default_value='mjpeg2rgb',
        description='usb_cam pixel format. Common values: mjpeg2rgb (modern '
                    'UVC webcams, best at >=720p), yuyv2rgb (fallback for '
                    'cameras without MJPG, use with 640x480). Run '
                    '`v4l2-ctl -d /dev/video0 --list-formats-ext` to verify.')
    camera_info_url = DeclareLaunchArgument(
        'camera_info_url', default_value='',
        description='file:// URL to a calibrated CameraInfo YAML. '
                    'Leave empty for the (poor) usb_cam default.')

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'camera_name': 'camera',
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'frame_id': 'camera',
        }],
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

    return LaunchDescription([
        launch_camera,
        device_arg,
        video_device,
        image_width,
        image_height,
        framerate,
        pixel_format,
        camera_info_url,
        usb_cam_node,
        detector_node,
    ])
