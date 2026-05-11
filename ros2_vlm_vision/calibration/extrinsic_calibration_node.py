"""Extrinsic calibration: camera_optical_frame -> a reference frame.

The reference frame is anchored at the chessboard origin. Operator places
the board at a known pose relative to the robot (or world), runs this
node, and averages many PnP solutions. On stop the resulting transform is
written to YAML and published as a static TF.

This is the right approach when a single rigid mount exists between the
camera and the rest of the robot. For end-effector-mounted cameras use a
hand-eye solver (cv2.calibrateHandEye) — out of scope here.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

from ros2_vlm_vision.utils.geometry import PinholeModel


def _rmat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> quaternion (x, y, z, w). Shepperd's method."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class ExtrinsicCalibrationNode(Node):

    def __init__(self) -> None:
        super().__init__('extrinsic_calibration_node')

        self.declare_parameter('color_topic', '/image_raw')
        self.declare_parameter('info_topic', '/camera_info')
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('square_size_m', 0.025)
        self.declare_parameter('reference_frame', 'calibration_target')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('samples_required', 40)
        self.declare_parameter('output_path',
                               '/tmp/camera_extrinsics.yaml')

        cols = int(self.get_parameter('board_cols').value)
        rows = int(self.get_parameter('board_rows').value)
        sq = float(self.get_parameter('square_size_m').value)
        self._pattern = (cols, rows)
        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
        self._objp = objp

        self._target_samples = int(self.get_parameter('samples_required').value)
        self._output_path = Path(str(self.get_parameter('output_path').value))
        self._ref_frame = str(self.get_parameter('reference_frame').value)
        self._cam_frame = str(self.get_parameter('camera_frame').value)

        self._bridge = CvBridge()
        self._intrinsics: Optional[PinholeModel] = None
        self._dist: Optional[np.ndarray] = None
        self._translations: list[np.ndarray] = []
        self._rotations: list[np.ndarray] = []
        self._done = False
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(
            CameraInfo,
            self.get_parameter('info_topic').value,
            self._on_camera_info,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.create_subscription(
            Image,
            self.get_parameter('color_topic').value,
            self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.get_logger().info(
            f'Collecting {self._target_samples} PnP samples; will publish '
            f'static TF {self._ref_frame} -> {self._cam_frame} on completion.')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._intrinsics is None:
            self._intrinsics = PinholeModel.from_camera_info(msg)
            self._dist = np.array(msg.d, dtype=np.float32).reshape(-1, 1) \
                if msg.d else np.zeros((5, 1), dtype=np.float32)

    def _on_image(self, msg: Image) -> None:
        if self._done or self._intrinsics is None:
            return
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, self._pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return
        cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                      30, 1e-3))

        K = np.array([[self._intrinsics.fx, 0, self._intrinsics.cx],
                      [0, self._intrinsics.fy, self._intrinsics.cy],
                      [0, 0, 1]], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            self._objp, corners, K, self._dist,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return
        R, _ = cv2.Rodrigues(rvec)
        self._translations.append(tvec.reshape(3))
        self._rotations.append(R)

        if len(self._translations) % 5 == 0:
            self.get_logger().info(
                f'PnP samples: {len(self._translations)}/{self._target_samples}')

        if len(self._translations) >= self._target_samples:
            self._finalise()
            self._done = True

    def _finalise(self) -> None:
        # cam_T_board for each sample. Average translation directly and the
        # rotation through quaternion mean (Markley's eigen method).
        t_mean = np.mean(self._translations, axis=0)
        quats = np.array(
            [_rmat_to_quat(R) for R in self._rotations], dtype=np.float64)
        # Force consistent hemisphere before averaging.
        ref = quats[0]
        for i in range(1, len(quats)):
            if np.dot(quats[i], ref) < 0:
                quats[i] = -quats[i]
        M = quats.T @ quats
        eigvals, eigvecs = np.linalg.eigh(M)
        q = eigvecs[:, -1]
        if q[3] < 0:
            q = -q
        qx, qy, qz, qw = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

        # The averaged transform is cam_T_board (board pose expressed in the
        # camera frame). Invert to publish ref_T_cam if you'd rather anchor
        # in the calibration frame.
        out = {
            'parent_frame': self._cam_frame,
            'child_frame': self._ref_frame,
            'translation_xyz_m': [float(t_mean[0]), float(t_mean[1]), float(t_mean[2])],
            'rotation_xyzw': [qx, qy, qz, qw],
            'samples': len(self._translations),
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with self._output_path.open('w') as f:
            yaml.safe_dump(out, f, default_flow_style=False)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._cam_frame
        tf.child_frame_id = self._ref_frame
        tf.transform.translation.x = float(t_mean[0])
        tf.transform.translation.y = float(t_mean[1])
        tf.transform.translation.z = float(t_mean[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)
        self.get_logger().info(
            f'Extrinsic saved to {self._output_path}; static TF published.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExtrinsicCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
