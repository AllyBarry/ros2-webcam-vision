"""Chessboard-based intrinsic calibration for the RGB stream.

Subscribes to a colour image topic, detects a chessboard pattern, and
collects diverse views (only frames whose centre differs sufficiently from
already-stored views are kept). When the requested sample count is reached
it runs cv2.calibrateCamera, writes the K and distortion vector to a YAML
file, and reports reprojection error.

UVC webcams generally don't publish reliable factory intrinsics --- the
camera_info from usb_cam is a placeholder until calibration is performed.
Run this once per physical camera (more often if the lens has been
disturbed) and the resulting YAML can be fed back to usb_cam via its
camera_info_url parameter.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image


class IntrinsicCalibrationNode(Node):

    def __init__(self) -> None:
        super().__init__('intrinsic_calibration_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('square_size_m', 0.025)
        self.declare_parameter('samples_required', 25)
        self.declare_parameter('min_centre_distance_px', 60.0)
        self.declare_parameter('output_path',
                               '/tmp/rgb_intrinsics.yaml')

        cols = int(self.get_parameter('board_cols').value)
        rows = int(self.get_parameter('board_rows').value)
        self._pattern = (cols, rows)
        sq = float(self.get_parameter('square_size_m').value)
        self._target_samples = int(self.get_parameter('samples_required').value)
        self._min_dist = float(self.get_parameter('min_centre_distance_px').value)
        self._output_path = Path(str(self.get_parameter('output_path').value))

        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
        self._objp = objp

        self._bridge = CvBridge()
        self._object_points: list[np.ndarray] = []
        self._image_points: list[np.ndarray] = []
        self._centres: list[tuple[float, float]] = []
        self._image_size: tuple[int, int] | None = None
        self._done = False

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.get_logger().info(
            f'Looking for {cols}x{rows} chessboard, need '
            f'{self._target_samples} diverse views.')

    def _on_image(self, msg: Image) -> None:
        if self._done:
            return
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._image_size is None:
            self._image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(
            gray, self._pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return

        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                      30, 1e-3))
        centre = tuple(corners.mean(axis=0).ravel())
        if any((centre[0] - c[0]) ** 2 + (centre[1] - c[1]) ** 2 < self._min_dist ** 2
               for c in self._centres):
            return

        self._image_points.append(corners)
        self._object_points.append(self._objp)
        self._centres.append(centre)
        self.get_logger().info(
            f'Captured view {len(self._image_points)}/{self._target_samples} '
            f'at ({centre[0]:.0f}, {centre[1]:.0f})')

        if len(self._image_points) >= self._target_samples:
            self._calibrate()
            self._done = True

    def _calibrate(self) -> None:
        assert self._image_size is not None
        rms, K, dist, _, _ = cv2.calibrateCamera(
            self._object_points, self._image_points,
            self._image_size, None, None)
        out = {
            'image_width': int(self._image_size[0]),
            'image_height': int(self._image_size[1]),
            'camera_matrix': {
                'rows': 3, 'cols': 3,
                'data': [float(v) for v in K.flatten()],
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1, 'cols': int(dist.size),
                'data': [float(v) for v in dist.flatten()],
            },
            'rms_reprojection_error_px': float(rms),
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with self._output_path.open('w') as f:
            yaml.safe_dump(out, f, default_flow_style=False)
        self.get_logger().info(
            f'Calibration written to {self._output_path} '
            f'(RMS reprojection error = {rms:.3f} px)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntrinsicCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
