"""Run a single calibration node. Select with the 'mode' arg.

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

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='intrinsic',
            description='intrinsic | extrinsic'),
        Node(
            package='ros2_vlm_vision',
            executable='intrinsic_calibration_node',
            name='intrinsic_calibration_node',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'intrinsic'"])),
        ),
        Node(
            package='ros2_vlm_vision',
            executable='extrinsic_calibration_node',
            name='extrinsic_calibration_node',
            output='screen',
            parameters=[cfg],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'extrinsic'"])),
        ),
    ])
