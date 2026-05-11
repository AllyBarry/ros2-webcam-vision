from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_vlm_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'ultralytics>=8.0.0',
        'torch',
    ],
    zip_safe=True,
    maintainer='Wits Robotics',
    maintainer_email='1056862@students.wits.ac.za',
    description='YOLOv8 detection on a UVC webcam (Logitech etc.) with size-heuristic 3D position estimation for a VLM stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_detector_node = ros2_vlm_vision.vlm_detector_node:main',
            'intrinsic_calibration_node = ros2_vlm_vision.calibration.intrinsic_calibration_node:main',
            'extrinsic_calibration_node = ros2_vlm_vision.calibration.extrinsic_calibration_node:main',
        ],
    },
)
