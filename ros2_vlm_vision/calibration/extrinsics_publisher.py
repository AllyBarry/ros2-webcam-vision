"""Load a saved extrinsics YAML and broadcast world -> camera as static TF.

Companion to apriltag_extrinsic_latcher.py and to any other tool that
writes an extrinsic transform to YAML. Reads the saved file once at
startup, broadcasts the static TF, and is done.

Two schemas are accepted -- whichever the source tool happens to write:

A. apriltag_extrinsic_latcher format (this package):

    world_T_camera:
      parent_frame: world
      child_frame:  camera
      translation_xyz_m: [x, y, z]
      rotation_xyzw:     [x, y, z, w]

B. Wrist-tag PnP / easy_handeye-style format (e.g. the calibrate.py
   in /home/jetson/ros2_ws and many other community tools):

    parent_frame: world
    camera_frame: camera
    translation_xyz:      [x, y, z]
    rotation_quat_xyzw:   [x, y, z, w]

The loader auto-detects which schema is present and prefers the
top-level `world_T_camera` block if both happen to coexist. Either way,
the static TF published is `parent_frame -> camera_frame` (or in
schema A, `world_T_camera.parent_frame -> world_T_camera.child_frame`).

If the YAML is missing or malformed the node exits non-zero -- the
caller (vision_stack.launch.py) decides whether that's fatal.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _expand(p: str) -> Path:
    return Path(os.path.expanduser(os.path.expandvars(p)))


class ExtrinsicsPublisher(Node):

    def __init__(self) -> None:
        super().__init__('extrinsics_publisher')

        self.declare_parameter(
            'yaml_path', '/root/.ros/camera_info/extrinsics.yaml')
        path = _expand(str(self.get_parameter('yaml_path').value))

        if not path.is_file():
            raise FileNotFoundError(
                f'extrinsics YAML not found: {path}\n'
                "Run extrinsic calibration first:\n"
                "  ros2 launch ros2_vlm_vision calibration.launch.py mode:=extrinsic")

        with open(path) as f:
            data = yaml.safe_load(f) or {}

        parent, child, t, q, schema = self._extract_transform(data, path)
        self._schema_used = schema

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(tf)

        samples = data.get('samples')
        anchor = data.get('anchor_tag_frame')
        provenance = [f'schema={self._schema_used}']
        if samples is not None:
            provenance.append(f'{samples} samples')
        if anchor is not None:
            provenance.append(f"anchor='{anchor}'")
        suffix = f' ({", ".join(provenance)})'

        self.get_logger().info(
            f"Published static TF '{parent}' -> '{child}' from {path}{suffix}\n"
            f'  translation: [{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}] m\n'
            f'  rotation:    [{q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f}]')

    @staticmethod
    def _extract_transform(data: dict, path: Path
                           ) -> tuple[str, str, list, list, str]:
        """Pull (parent, child, translation, quaternion, schema_name) from
        either supported YAML layout.

        Schema A: apriltag_extrinsic_latcher -- a nested `world_T_camera`
        block with `translation_xyz_m` / `rotation_xyzw`.

        Schema B: top-level `translation_xyz` + `rotation_quat_xyzw`
        with `parent_frame` / `camera_frame` siblings (the format
        written by /home/jetson/ros2_ws/calibrate.py and many similar
        community wrist-tag tools).
        """
        # Schema A -- preferred when present (carries explicit samples/
        # anchor provenance).
        wtc = data.get('world_T_camera')
        if isinstance(wtc, dict) and 'translation_xyz_m' in wtc:
            parent = str(wtc.get('parent_frame', 'world'))
            child = str(wtc.get('child_frame', 'camera'))
            t = wtc.get('translation_xyz_m')
            q = wtc.get('rotation_xyzw')
            if (isinstance(t, (list, tuple)) and len(t) == 3
                    and isinstance(q, (list, tuple)) and len(q) == 4):
                return parent, child, list(t), list(q), 'world_T_camera-block'

        # Schema B -- top-level translation_xyz + rotation_quat_xyzw.
        t = data.get('translation_xyz')
        q = data.get('rotation_quat_xyzw')
        if (isinstance(t, (list, tuple)) and len(t) == 3
                and isinstance(q, (list, tuple)) and len(q) == 4):
            parent = str(data.get('parent_frame', 'world'))
            # Both spellings turn up in the wild: camera_frame (ros2_ws
            # convention) and child_frame (REP-105 convention).
            child = str(data.get('camera_frame',
                                 data.get('child_frame', 'camera')))
            return parent, child, list(t), list(q), 'top-level-quaternion'

        raise ValueError(
            f"{path}: could not find a recognisable extrinsic transform. "
            f"Expected either a 'world_T_camera' block with "
            f"'translation_xyz_m' + 'rotation_xyzw', or top-level "
            f"'translation_xyz' + 'rotation_quat_xyzw' with "
            f"'parent_frame' / 'camera_frame' siblings.")


def main(args=None) -> int:
    rclpy.init(args=args)
    try:
        node = ExtrinsicsPublisher()
    except (FileNotFoundError, ValueError) as e:
        print(f'[extrinsics_publisher] FATAL: {e}', file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())