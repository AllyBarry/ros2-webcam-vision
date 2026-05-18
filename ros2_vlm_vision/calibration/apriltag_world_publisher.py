"""Republish apriltag_ros detections in the world frame.

apriltag_ros publishes two things per detected tag:

  - /tf: camera -> tag<family>:<id> (live; PnP-derived pose).
  - /detections (apriltag_msgs/AprilTagDetectionArray): 2D corner +
    decoded ID per detection, header.frame_id = camera frame.

With a static `world -> camera` TF in place (from extrinsics_publisher
loading the YAML written by apriltag_extrinsic_latcher), tf2 can compose
the two to get world -> tag<family>:<id>. This node does that on every
/detections callback and republishes the result as easy-to-consume
topics, mirroring vlm_detector_node's output shape so a downstream
robot executor can treat YOLO and AprilTag detections uniformly.

Topics published (namespaced under the node):

  ~/world_detections  vision_msgs/Detection3DArray
      One Detection3D per tag, frame_id=world. class_id encodes the
      tag (e.g. "tag36h11:5"), score = decision_margin from apriltag_ros.
  ~/world_poses       geometry_msgs/PoseArray
      Lightweight pose-only view, frame_id=world.
  ~/markers           visualization_msgs/MarkerArray
      Cube + label per tag for Foxglove's 3D panel.

Requires: the world -> camera static TF must be already published. If
it isn't, TF lookups fail and the node logs a throttled warning instead
of publishing stale or wrong-frame data.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray


class AprilTagWorldPublisher(Node):

    def __init__(self) -> None:
        super().__init__('apriltag_world_publisher')

        self.declare_parameter('world_frame', 'world')
        # apriltag_ros names tag TF children "tag<family>:<id>"; the prefix
        # is set by the apriltag family in apriltag.yaml. Keep this in sync.
        self.declare_parameter('tag_frame_prefix', 'tag36h11:')
        # Visual marker side length (cube). Defaults to the physical tag
        # edge from apriltag.yaml; override here if you want bigger or
        # smaller markers without changing the detection.
        self.declare_parameter('marker_size_m', 0.162)
        # How long a TF lookup may wait. Detections arrive at camera rate;
        # tf2 normally has the matching camera->tag TF in the same buffer
        # by the time we look (apriltag_ros publishes them in the same
        # callback that fires /detections).
        self.declare_parameter('lookup_timeout_s', 0.1)

        self._world = str(self.get_parameter('world_frame').value)
        self._prefix = str(self.get_parameter('tag_frame_prefix').value)
        self._marker_size = float(self.get_parameter('marker_size_m').value)
        self._timeout = float(self.get_parameter('lookup_timeout_s').value)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)

        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self._on_detections,
            10,
        )

        self._det_pub = self.create_publisher(
            Detection3DArray, '~/world_detections', 10)
        self._pose_pub = self.create_publisher(
            PoseArray, '~/world_poses', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '~/markers', 10)

        self.get_logger().info(
            f"AprilTag world publisher ready. Republishing tags into "
            f"'{self._world}' frame from /detections; expects child frame "
            f"names like '{self._prefix}<id>'.")

    def _lookup(self, tag_frame: str, stamp) -> Optional[tuple]:
        """world -> tag_frame as (translation, quaternion-xyzw), or None."""
        try:
            tf = self._buffer.lookup_transform(
                self._world, tag_frame, Time.from_msg(stamp),
                rclpy.duration.Duration(seconds=self._timeout))
        except Exception:
            # Try latest available if timestamped lookup failed (TF
            # filter window can race the detection stamp).
            try:
                tf = self._buffer.lookup_transform(
                    self._world, tag_frame, Time())
            except Exception as e:
                self.get_logger().warn(
                    f"No TF '{self._world}' -> '{tag_frame}': {e}",
                    throttle_duration_sec=2.0)
                return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        if not msg.detections:
            # Still emit an empty Detection3DArray so consumers see the
            # "no detections this frame" event (useful for staleness checks).
            empty = Detection3DArray()
            empty.header.stamp = msg.header.stamp
            empty.header.frame_id = self._world
            self._det_pub.publish(empty)
            return

        det_array = Detection3DArray()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = self._world

        pose_array = PoseArray()
        pose_array.header = det_array.header

        markers = MarkerArray()
        clear = Marker()
        clear.header = det_array.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0
        for d in msg.detections:
            # apriltag_msgs/AprilTagDetection.family is e.g. "36h11";
            # apriltag_ros names the TF child frame "tag<family>:<id>".
            # Build the same string here.
            tag_frame = f'{self._prefix}{d.id}'
            looked_up = self._lookup(tag_frame, msg.header.stamp)
            if looked_up is None:
                continue
            (tx, ty, tz), (qx, qy, qz, qw) = looked_up

            pose = Pose()
            pose.position = Point(x=tx, y=ty, z=tz)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pose_array.poses.append(pose)

            det = Detection3D()
            det.header = det_array.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = tag_frame
            # apriltag_msgs uses decision_margin (~0..>100, larger = more
            # confident); not a [0,1] score but the closest analogue.
            hyp.hypothesis.score = float(d.decision_margin)
            hyp.pose.pose = pose
            det.results.append(hyp)
            det.bbox = BoundingBox3D()
            det.bbox.center = pose
            det.bbox.size.x = self._marker_size
            det.bbox.size.y = self._marker_size
            det.bbox.size.z = 0.001  # flat tag
            det_array.detections.append(det)

            markers.markers.extend(
                self._make_markers(det_array.header, marker_id, pose, tag_frame))
            marker_id += 1

        self._det_pub.publish(det_array)
        self._pose_pub.publish(pose_array)
        self._marker_pub.publish(markers)

    def _make_markers(self, header, idx: int, pose: Pose,
                      label: str) -> list[Marker]:
        cube = Marker()
        cube.header = header
        cube.ns = 'apriltag_world'
        cube.id = idx * 2
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = pose
        cube.scale.x = self._marker_size
        cube.scale.y = self._marker_size
        cube.scale.z = 0.002
        cube.color = ColorRGBA(r=0.95, g=0.85, b=0.1, a=0.85)
        cube.lifetime.sec = 1

        text = Marker()
        text.header = header
        text.ns = 'apriltag_world'
        text.id = idx * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.position.x
        text.pose.position.y = pose.position.y
        text.pose.position.z = pose.position.z + self._marker_size * 0.7
        text.pose.orientation.w = 1.0
        text.scale.z = max(0.04, self._marker_size * 0.5)
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.lifetime.sec = 1
        text.text = label
        return [cube, text]


def main(args=None) -> int:
    rclpy.init(args=args)
    node = AprilTagWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
