"""YOLOv8 + size-heuristic 3D detector node (RGB-only).

Subscribes to a colour image topic and its camera_info from a UVC webcam
(e.g. Logitech via usb_cam), runs YOLOv8 on each frame, and back-projects
each detection to 3D by assuming the bounding-box vertical extent maps to
a canonical real-world height per class (see utils/object_sizes.py).

The math is the standard pinhole relation
  bbox_h_px / fy = real_h_m / Z   =>   Z = real_h * fy / bbox_h_px
followed by  X = (u - cx) * Z / fx,  Y = (v - cy) * Z / fy.

Target filter
-------------
Publishing a comma-separated list of class names to ~/target_classes
restricts the 3D output (and the highlighted markers) to those classes.
A latched (TRANSIENT_LOCAL) QoS profile is used so the filter survives
restarts of the bridge / Foxglove. Examples (from a Foxglove Publish
panel or `ros2 topic pub`):

  data: "person"            -> only 'person' detections
  data: "person, cup, dog"  -> any of the three
  data: ""                  -> clear filter (show everything)
  data: "*"                 -> same as empty

Non-target detections are drawn faded in the debug image so the operator
can discover what's available; set show_non_targets:=false to hide them.
"""
from __future__ import annotations

from collections import Counter
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSPresetProfiles,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from ros2_vlm_vision.utils.geometry import PinholeModel
from ros2_vlm_vision.utils.object_sizes import distance_from_height


# (B, G, R) for OpenCV drawing.
_TARGET_BGR = (0, 220, 0)
_NONTARGET_BGR = (140, 140, 140)
_HEADER_BGR = (255, 255, 255)
_HEADER_BG_BGR = (40, 40, 40)


class VLMDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('vlm_detector_node')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_distance', 25.0)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('info_topic', '/camera_info')
        self.declare_parameter('camera_frame', 'camera')
        # Comma-separated class names to restrict the 3D output on startup.
        # Live updates come via the ~/target_classes topic and override this.
        self.declare_parameter('initial_target_classes', '')
        # If false, non-target detections are omitted from the debug image.
        self.declare_parameter('show_non_targets', True)

        self._conf = float(self.get_parameter('conf_threshold').value)
        self._iou = float(self.get_parameter('iou_threshold').value)
        self._max_distance = float(self.get_parameter('max_distance').value)
        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._show_non_targets = bool(
            self.get_parameter('show_non_targets').value)

        # None == no filter; set == active filter, lowercased class names.
        self._targets: Optional[set[str]] = self._parse_targets(
            str(self.get_parameter('initial_target_classes').value))

        self._bridge = CvBridge()
        self._model = self._load_yolo()
        self._intrinsics: Optional[PinholeModel] = None
        self._unknown_classes: set[str] = set()

        self.create_subscription(
            CameraInfo,
            self.get_parameter('info_topic').value,
            self._on_camera_info,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        # Latched so a filter published before the node started is delivered
        # on subscription, and survives any consumer reconnects.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, '~/target_classes', self._on_targets, latched)

        self._det_pub = self.create_publisher(
            Detection3DArray, '~/detections', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '~/markers', 10)
        self._debug_pub = self.create_publisher(
            Image, '~/debug_image', 1)
        # Publish the active filter so other tools can show its state.
        self._active_pub = self.create_publisher(
            String, '~/active_targets', latched)
        self._publish_active_targets()

        self.get_logger().info(
            f"VLM detector ready (model={self.get_parameter('model').value}, "
            f"device={self.get_parameter('device').value}, "
            f"targets={self._format_targets()})")

    def _load_yolo(self):
        from ultralytics import YOLO
        model = YOLO(self.get_parameter('model').value)
        model.to(self.get_parameter('device').value)
        return model

    @staticmethod
    def _parse_targets(raw: str) -> Optional[set[str]]:
        raw = raw.strip()
        if not raw or raw == '*':
            return None
        return {c.strip().lower() for c in raw.split(',') if c.strip()}

    def _format_targets(self) -> str:
        if self._targets is None:
            return 'ALL'
        return ', '.join(sorted(self._targets)) or 'ALL'

    def _publish_active_targets(self) -> None:
        msg = String()
        msg.data = '' if self._targets is None else ','.join(sorted(self._targets))
        self._active_pub.publish(msg)

    def _on_targets(self, msg: String) -> None:
        new = self._parse_targets(msg.data)
        if new == self._targets:
            return
        self._targets = new
        self.get_logger().info(f'Target filter updated: {self._format_targets()}')
        self._publish_active_targets()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self._intrinsics is None:
            self._intrinsics = PinholeModel.from_camera_info(msg)
            if msg.header.frame_id:
                self._camera_frame = msg.header.frame_id
            self.get_logger().info(
                f"Got intrinsics in frame '{self._camera_frame}': "
                f"fx={self._intrinsics.fx:.2f}, fy={self._intrinsics.fy:.2f}, "
                f"cx={self._intrinsics.cx:.2f}, cy={self._intrinsics.cy:.2f}")

    def _on_image(self, msg: Image) -> None:
        if self._intrinsics is None:
            return
        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self._model.predict(
            rgb, conf=self._conf, iou=self._iou, verbose=False)
        if not results:
            return
        result = results[0]
        names = result.names

        det_array = Detection3DArray()
        det_array.header = msg.header
        det_array.header.frame_id = self._camera_frame

        markers = MarkerArray()
        clear = Marker()
        clear.header = det_array.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        boxes = result.boxes
        debug_rows: list[tuple] = []
        visible_counts: Counter[str] = Counter()
        target_counts: Counter[str] = Counter()

        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            cls_ids = boxes.cls.cpu().numpy().astype(int)

            marker_id = 0
            for box, conf, cls_id in zip(xyxy, confs, cls_ids):
                x1, y1, x2, y2 = box
                label = names.get(int(cls_id), str(int(cls_id)))
                visible_counts[label] += 1
                bbox_h_px = float(y2 - y1)
                z = distance_from_height(label, bbox_h_px, self._intrinsics.fy)

                if z is None:
                    if label not in self._unknown_classes:
                        self._unknown_classes.add(label)
                        self.get_logger().warn(
                            f"No size entry for class '{label}'; "
                            f'omitting 3D output.')
                    debug_rows.append(
                        (x1, y1, x2, y2, label, float(conf), None, False))
                    continue
                if z <= 0.0 or z > self._max_distance:
                    continue

                is_target = (self._targets is None
                             or label.lower() in self._targets)

                u = (x1 + x2) * 0.5
                v = (y1 + y2) * 0.5
                X, Y, Z = self._intrinsics.deproject(u, v, z)

                if is_target:
                    target_counts[label] += 1

                    det = Detection3D()
                    det.header = det_array.header
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = label
                    hyp.hypothesis.score = float(conf)
                    hyp.pose.pose.position = Point(x=X, y=Y, z=Z)
                    hyp.pose.pose.orientation.w = 1.0
                    det.results.append(hyp)

                    bbox = BoundingBox3D()
                    bbox.center = Pose()
                    bbox.center.position = Point(x=X, y=Y, z=Z)
                    bbox.center.orientation.w = 1.0
                    bbox.size.x = float(abs(x2 - x1) * z / self._intrinsics.fx)
                    bbox.size.y = float(abs(y2 - y1) * z / self._intrinsics.fy)
                    bbox.size.z = 0.10
                    det.bbox = bbox
                    det_array.detections.append(det)

                    markers.markers.extend(
                        self._make_markers(det_array.header, marker_id,
                                           X, Y, Z, label, conf))
                    marker_id += 1

                debug_rows.append(
                    (x1, y1, x2, y2, label, float(conf), z, is_target))

        self._det_pub.publish(det_array)
        self._marker_pub.publish(markers)
        self._publish_debug(rgb, debug_rows, visible_counts,
                            target_counts, msg.header)

    @staticmethod
    def _make_markers(header, idx: int, x: float, y: float, z: float,
                      label: str, conf: float) -> list[Marker]:
        sphere = Marker()
        sphere.header = header
        sphere.ns = 'vlm_objects'
        sphere.id = idx * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=x, y=y, z=z)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
        sphere.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.9)
        sphere.lifetime.sec = 1

        text = Marker()
        text.header = header
        text.ns = 'vlm_objects'
        text.id = idx * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(x=x, y=y - 0.10, z=z)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.lifetime.sec = 1
        text.text = f'{label} {conf:.2f}'

        return [sphere, text]

    def _publish_debug(self, rgb: np.ndarray, rows,
                       visible: Counter, targets: Counter, header) -> None:
        if self._debug_pub.get_subscription_count() == 0:
            return
        import cv2
        out = rgb.copy()
        h, w = out.shape[:2]

        for x1, y1, x2, y2, label, conf, z, is_target in rows:
            if not is_target and not self._show_non_targets:
                continue
            colour = _TARGET_BGR if is_target else _NONTARGET_BGR
            thickness = 2 if is_target else 1
            cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)),
                          colour, thickness)
            z_str = f'~{z:.2f}m' if z is not None else 'no-3D'
            tag = f'{label} {conf:.2f} {z_str}'
            (tw, th), _ = cv2.getTextSize(
                tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            y_text = max(int(y1) - 4, th + 2)
            cv2.rectangle(out, (int(x1), y_text - th - 2),
                          (int(x1) + tw + 4, y_text + 2), colour, -1)
            cv2.putText(out, tag, (int(x1) + 2, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
                        cv2.LINE_AA)

        target_str = self._format_targets()
        if target_str == 'ALL':
            header_line = f'Targets: ALL  |  Detected: {self._counter_str(visible)}'
        else:
            header_line = (f'Targets: {target_str}  '
                           f'|  Matching: {self._counter_str(targets)}  '
                           f'|  Detected: {self._counter_str(visible)}')

        (tw, th), _ = cv2.getTextSize(
            header_line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(out, (0, 0), (min(tw + 12, w), th + 12),
                      _HEADER_BG_BGR, -1)
        cv2.putText(out, header_line, (6, th + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, _HEADER_BGR, 1,
                    cv2.LINE_AA)

        msg = self._bridge.cv2_to_imgmsg(out, encoding='bgr8')
        msg.header = header
        self._debug_pub.publish(msg)

    @staticmethod
    def _counter_str(c: Counter) -> str:
        if not c:
            return 'none'
        return ', '.join(f'{k}({v})' for k, v in c.most_common())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLMDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
