"""Latch a static world -> camera transform from apriltag_ros detections.

apriltag_ros publishes a TF from the camera optical frame to each tag it
detects (e.g. camera_optical_frame -> tag36h11:0). This node:

  1. Picks a designated 'anchor' tag whose physical location defines the
     world frame.
  2. Collects N stable TF samples of camera -> anchor_tag.
  3. Averages them (mean translation + Markley quaternion eigen-mean) to
     suppress per-frame PnP noise.
  4. Inverts to world_T_camera and broadcasts a static TF
     <world_frame> -> <camera_frame>.
  5. Writes both directions to a YAML so downstream nodes can re-load
     the calibration without re-running the procedure.

The TF the detector publishes (frame_id=camera) can then be looked up
in the world frame through the regular tf2_ros API.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def _rmat_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
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


def _quat_to_rmat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ])


def _quaternion_mean(quats: np.ndarray) -> np.ndarray:
    """Markley's eigen-method: principal eigenvector of Σ q q^T."""
    ref = quats[0]
    for i in range(1, len(quats)):
        if np.dot(quats[i], ref) < 0:
            quats[i] = -quats[i]
    M = quats.T @ quats
    _, eigvecs = np.linalg.eigh(M)
    q = eigvecs[:, -1]
    if q[3] < 0:
        q = -q
    return q


class AprilTagExtrinsicLatcher(Node):

    def __init__(self) -> None:
        super().__init__('apriltag_extrinsic_latcher')

        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('anchor_tag_frame', 'tag36h11:0')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('samples_required', 30)
        self.declare_parameter('sample_rate_hz', 5.0)
        self.declare_parameter('output_path',
                               '/root/.ros/camera_info/extrinsics.yaml')

        self._cam = str(self.get_parameter('camera_frame').value)
        self._anchor = str(self.get_parameter('anchor_tag_frame').value)
        self._world = str(self.get_parameter('world_frame').value)
        self._target = int(self.get_parameter('samples_required').value)
        self._output_path = Path(str(self.get_parameter('output_path').value))

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._broadcaster = StaticTransformBroadcaster(self)
        self._translations: list[np.ndarray] = []
        self._quaternions: list[np.ndarray] = []
        self._last_stamp: Optional[int] = None
        self._done = False

        rate = float(self.get_parameter('sample_rate_hz').value)
        self.create_timer(1.0 / rate, self._poll)

        self.get_logger().info(
            f"Waiting for apriltag TF '{self._cam}' -> '{self._anchor}'; "
            f'will collect {self._target} samples then latch '
            f"'{self._world}' -> '{self._cam}'.")

    def _poll(self) -> None:
        if self._done:
            return
        try:
            tf = self._buffer.lookup_transform(self._cam, self._anchor, Time())
        except Exception:
            return

        stamp_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
        if stamp_ns == self._last_stamp:
            return
        self._last_stamp = stamp_ns

        t = tf.transform.translation
        r = tf.transform.rotation
        self._translations.append(np.array([t.x, t.y, t.z]))
        self._quaternions.append(np.array([r.x, r.y, r.z, r.w]))

        n = len(self._translations)
        if n % 5 == 0 or n == self._target:
            self.get_logger().info(f'  samples {n}/{self._target}')

        if n >= self._target:
            self._finalise()
            self._done = True

    def _finalise(self) -> None:
        # cam_T_anchor averaged.
        t_ca = np.mean(self._translations, axis=0)
        q_ca = _quaternion_mean(np.array(self._quaternions, dtype=np.float64))
        R_ca = _quat_to_rmat(q_ca)

        # world_T_camera = (cam_T_anchor)^-1, with world == anchor tag.
        R_wc = R_ca.T
        t_wc = -R_wc @ t_ca
        qx, qy, qz, qw = _rmat_to_quat(R_wc)

        out = {
            'samples': len(self._translations),
            'anchor_tag_frame': self._anchor,
            'world_T_camera': {
                'parent_frame': self._world,
                'child_frame': self._cam,
                'translation_xyz_m': [float(t_wc[0]), float(t_wc[1]), float(t_wc[2])],
                'rotation_xyzw': [float(qx), float(qy), float(qz), float(qw)],
            },
            'camera_T_world': {
                'parent_frame': self._cam,
                'child_frame': self._world,
                'translation_xyz_m': [float(t_ca[0]), float(t_ca[1]), float(t_ca[2])],
                'rotation_xyzw': [
                    float(q_ca[0]), float(q_ca[1]),
                    float(q_ca[2]), float(q_ca[3]),
                ],
            },
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with self._output_path.open('w') as f:
            yaml.safe_dump(out, f, default_flow_style=False, sort_keys=False)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._world
        tf.child_frame_id = self._cam
        tf.transform.translation.x = float(t_wc[0])
        tf.transform.translation.y = float(t_wc[1])
        tf.transform.translation.z = float(t_wc[2])
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        self._broadcaster.sendTransform(tf)

        self.get_logger().info(
            f'Latched {self._world} -> {self._cam} '
            f'(t = [{t_wc[0]:+.3f}, {t_wc[1]:+.3f}, {t_wc[2]:+.3f}] m) '
            f'and saved to {self._output_path}.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagExtrinsicLatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
