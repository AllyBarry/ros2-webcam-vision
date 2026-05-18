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
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSPresetProfiles,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from ros2_vlm_vision.utils.geometry import (
    PinholeModel,
    quat_to_rotmat,
    ray_plane_intersect_z,
)
from ros2_vlm_vision.utils.object_sizes import (
    DimensionTable,
    distance_from_height,
)


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

        # ---- World-frame back-projection (off by default) ---------
        # When use_table_plane is True and a static world->camera TF is
        # available, detections are republished in the world frame using
        # ray-plane intersection against z = table_z_world (in metres,
        # world frame). Per-class dimensions from dimensions_path lift
        # the centroid above the table and fill BoundingBox3D.size.
        # When the TF or table param is missing, falls back to the
        # legacy depth-from-height heuristic in the camera frame.
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('use_table_plane', True)
        self.declare_parameter('table_z_world', 0.0)
        # Pixel inside the bbox used as the table-contact projection
        # point. 'bottom_center' (u_mid, y2) is the safest default for
        # any object sitting on the table -- the lowest image pixel of
        # the object's silhouette lies on the table plane. 'center'
        # (u_mid, v_mid) is better only if you've offset table_z_world
        # to the centroid height for a known-flat scene.
        self.declare_parameter('contact_point', 'bottom_center')
        # Path to per-class dimensions YAML. Default: bundled config
        # under share/ros2_vlm_vision/config/object_dimensions.yaml.
        self.declare_parameter('dimensions_path', '')

        self._conf = float(self.get_parameter('conf_threshold').value)
        self._iou = float(self.get_parameter('iou_threshold').value)
        self._max_distance = float(self.get_parameter('max_distance').value)
        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._world_frame = str(self.get_parameter('world_frame').value)
        self._use_table_plane = bool(
            self.get_parameter('use_table_plane').value)
        self._table_z = float(self.get_parameter('table_z_world').value)
        self._contact_point = str(self.get_parameter('contact_point').value)
        self._show_non_targets = bool(
            self.get_parameter('show_non_targets').value)

        # None == no filter; set == active filter, lowercased class names.
        self._targets: Optional[set[str]] = self._parse_targets(
            str(self.get_parameter('initial_target_classes').value))

        dims_path = str(self.get_parameter('dimensions_path').value)
        if not dims_path:
            dims_path = str(Path(
                get_package_share_directory('ros2_vlm_vision'),
                'config', 'object_dimensions.yaml'))
        self._dims = DimensionTable(dims_path)
        self.get_logger().info(
            f'Loaded dimensions for {len(self._dims)} class(es) from {dims_path}')

        # tf2 buffer for the static world->camera lookup. Cached on
        # first use since the transform is static.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._world_T_camera_R: Optional[np.ndarray] = None
        self._world_T_camera_t: Optional[np.ndarray] = None
        # Throttle the "no world->camera TF yet" warnings.
        self._tf_warn_count = 0

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

    def _ensure_world_T_camera(self) -> bool:
        """Cache the static world->camera TF on first lookup.

        Static TFs don't change, so we look up once and store R, t. Returns
        True once the cache is populated. Logs a one-shot debug-throttled
        warning while waiting (extrinsic calibration not yet loaded).
        """
        if self._world_T_camera_R is not None:
            return True
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._camera_frame, Time())
        except Exception:
            self._tf_warn_count += 1
            # Once every ~5 s (assuming ~30 Hz callback). Don't spam.
            if self._tf_warn_count % 150 == 1:
                self.get_logger().warn(
                    f"No static TF '{self._world_frame}' -> "
                    f"'{self._camera_frame}' yet -- falling back to "
                    f"camera-frame output via size heuristic. "
                    f'Did extrinsics_publisher load extrinsics.yaml?')
            return False
        t = tf.transform.translation
        r = tf.transform.rotation
        self._world_T_camera_t = np.array([t.x, t.y, t.z])
        self._world_T_camera_R = quat_to_rotmat(r.x, r.y, r.z, r.w)
        self.get_logger().info(
            f"Cached static TF '{self._world_frame}' -> "
            f"'{self._camera_frame}'  "
            f't=[{t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}] m')
        return True

    def _project_to_world(self, u: float, v: float
                          ) -> Optional[tuple[float, float, float]]:
        """Pixel (u, v) -> world-frame (X, Y, table_z) via ray-plane intersect.

        Requires the static world->camera TF and table_z_world. Returns
        None if the ray points up / parallel to the table.
        """
        if not self._ensure_world_T_camera():
            return None
        dir_cam = self._intrinsics.ray_direction(u, v)
        dir_world = self._world_T_camera_R @ dir_cam
        hit = ray_plane_intersect_z(
            self._world_T_camera_t, dir_world, self._table_z)
        if hit is None:
            return None
        return float(hit[0]), float(hit[1]), float(hit[2])

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

        # Decide once per frame which back-projection path is active.
        # The decision is based on configuration AND TF availability;
        # we re-check the TF on every frame until it's cached.
        world_mode = (
            self._use_table_plane
            and self._ensure_world_T_camera()
        )
        output_frame = self._world_frame if world_mode else self._camera_frame

        det_array = Detection3DArray()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = output_frame

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

                dims = self._dims.get(label)
                if not dims.known and label not in self._unknown_classes:
                    self._unknown_classes.add(label)
                    self.get_logger().warn(
                        f"No dimensions for class '{label}' in object_dimensions.yaml; "
                        f'using defaults (h={dims.height:.2f} m). Edit the YAML to '
                        f'improve accuracy for this class.')

                is_target = (self._targets is None
                             or label.lower() in self._targets)

                # Two projection paths -- one expects table-plane and a
                # world->camera TF; the other is the legacy depth-from-
                # height heuristic in the camera frame.
                u_mid = (x1 + x2) * 0.5
                if world_mode:
                    v_pick = (y2 if self._contact_point == 'bottom_center'
                              else (y1 + y2) * 0.5)
                    proj = self._project_to_world(float(u_mid), float(v_pick))
                    if proj is None:
                        continue
                    X, Y, Z_contact = proj
                    # Lift the centroid above the table by half-height.
                    Z = Z_contact + dims.height * 0.5
                    # Pose distance from camera (for max_distance gating).
                    dx = X - self._world_T_camera_t[0]
                    dy = Y - self._world_T_camera_t[1]
                    dz = Z - self._world_T_camera_t[2]
                    dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
                else:
                    bbox_h_px = float(y2 - y1)
                    z = distance_from_height(label, bbox_h_px,
                                             self._intrinsics.fy, self._dims)
                    if z is None or z <= 0.0:
                        continue
                    dist = z
                    v_mid = (y1 + y2) * 0.5
                    X, Y, Z = self._intrinsics.deproject(u_mid, v_mid, z)

                if dist > self._max_distance:
                    continue

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
                    if world_mode:
                        # Real-world dimensions from the YAML.
                        bbox.size.x = float(dims.width)
                        bbox.size.y = float(dims.depth)
                        bbox.size.z = float(dims.height)
                    else:
                        # Pixel-derived approximation (legacy path).
                        bbox.size.x = float(abs(x2 - x1) * dist
                                            / self._intrinsics.fx)
                        bbox.size.y = float(abs(y2 - y1) * dist
                                            / self._intrinsics.fy)
                        bbox.size.z = float(dims.height)
                    det.bbox = bbox
                    det_array.detections.append(det)

                    markers.markers.extend(
                        self._make_markers(det_array.header, marker_id,
                                           X, Y, Z, label, conf, dims))
                    marker_id += 1

                debug_rows.append(
                    (x1, y1, x2, y2, label, float(conf), dist, is_target))

        self._det_pub.publish(det_array)
        self._marker_pub.publish(markers)
        self._publish_debug(rgb, debug_rows, visible_counts,
                            target_counts, msg.header)

    @staticmethod
    def _make_markers(header, idx: int, x: float, y: float, z: float,
                      label: str, conf: float, dims) -> list[Marker]:
        """Build a real-size cube + label marker pair for one detection.

        Cube dimensions come from the per-class dimensions YAML so the
        marker matches the object's actual extent in the world; downstream
        viewers (Foxglove) render it as a proper 3D box.
        """
        cube = Marker()
        cube.header = header
        cube.ns = 'vlm_objects'
        cube.id = idx * 2
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position = Point(x=x, y=y, z=z)
        cube.pose.orientation.w = 1.0
        cube.scale.x = float(dims.width)
        cube.scale.y = float(dims.depth)
        cube.scale.z = float(dims.height)
        cube.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.6)
        cube.lifetime.sec = 1

        text = Marker()
        text.header = header
        text.ns = 'vlm_objects'
        text.id = idx * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        # Float the label above the cube.
        text.pose.position = Point(
            x=x, y=y, z=z + max(0.04, float(dims.height) * 0.6))
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.lifetime.sec = 1
        text.text = f'{label} {conf:.2f}'

        return [cube, text]

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
