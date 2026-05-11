"""Run a single calibration helper. Select with the 'mode' arg.

Modes:
  intrinsic  -- chessboard intrinsic calibration; writes camera.yaml
  extrinsic  -- apriltag_ros + extrinsic latcher; writes extrinsics.yaml
                and broadcasts a static world -> camera TF

Examples:
  ros2 launch ros2_vlm_vision calibration.launch.py mode:=intrinsic
  ros2 launch ros2_vlm_vision calibration.launch.py mode:=extrinsic
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_vlm_vision')
    cfg = PathJoinSubstitution([pkg_share, 'config', 'calibration.yaml'])
    apriltag_cfg = PathJoinSubstitution([pkg_share, 'config', 'apriltag.yaml'])

    mode = LaunchConfiguration('mode')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    is_intrinsic = IfCondition(PythonExpression(["'", mode, "' == 'intrinsic'"]))
    is_extrinsic = IfCondition(PythonExpression(["'", mode, "' == 'extrinsic'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='intrinsic',
            description='intrinsic | extrinsic'),
        DeclareLaunchArgument(
            'image_topic', default_value='/image_raw',
            description='Raw image topic from the camera driver.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera_info',
            description='CameraInfo topic (used for rectification + PnP).'),

        # ----- intrinsic mode -----
        Node(
            package='ros2_vlm_vision',
            executable='intrinsic_calibration_node',
            name='intrinsic_calibration_node',
            output='screen',
            parameters=[cfg],
            condition=is_intrinsic,
        ),

        # ----- extrinsic mode: rectify -> apriltag_node -> latcher -----
        # apriltag_ros expects a rectified image stream, so we run an
        # image_proc rectifier off the raw stream first.
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify',
            output='screen',
            remappings=[
                ('image', image_topic),
                ('camera_info', camera_info_topic),
                ('image_rect', '/image_rect'),
            ],
            condition=is_extrinsic,
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[apriltag_cfg],
            remappings=[
                ('image_rect', '/image_rect'),
                ('camera_info', camera_info_topic),
            ],
            condition=is_extrinsic,
        ),
        Node(
            package='ros2_vlm_vision',
            executable='apriltag_extrinsic_latcher',
            name='apriltag_extrinsic_latcher',
            output='screen',
            parameters=[cfg],
            condition=is_extrinsic,
        ),
    ])
