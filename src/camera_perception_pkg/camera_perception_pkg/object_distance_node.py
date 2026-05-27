#!/usr/bin/env python3
# object_distance_node.py
# ros2 topic pub --once /manipulator_perception/target_button std_msgs/msg/String "{data: 'OUTSIDE_DOWN'}"

from typing import Optional, Tuple, List
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, String

from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

import message_filters

from amr_msgs.msg import DetectionArray  # DetectionArray.header, Detection[] detections


class ObjectDistanceNode(Node):
    """
    DetectionArray(2D bbox) + Depth Image + Depth CameraInfo 로
    bbox 중심 픽셀의 depth를 읽어 3D 점을 만들고 Marker를 publish.
    (현재 존재하는 토픽 기준으로 동작하도록 구성)
    """

    @staticmethod
    def _as_string_list(value) -> List[str]:
        if value is None:
            return []

        if isinstance(value, str):
            return [value] if value else []

        try:
            return [str(v) for v in value if str(v)]
        except TypeError:
            return [str(value)] if str(value) else []

    def __init__(self):
        super().__init__('object_distance_node')

        # ---------------- Parameters (실제 토픽 기본값) ----------------
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('depth_camera_info_topic', '/camera/camera/depth/camera_info')

        # 타깃 클래스 필터 (prefix 기반)
        self.declare_parameter('target_label_prefixes', ['btn_down'])
        self.declare_parameter('target_button_topic', '/manipulator_perception/target_button')

        # Runtime target presets.
        # task_manager sends one of:
        #   OUTSIDE_DOWN -> external down button
        #   INSIDE_B1    -> B1 button inside elevator
        #   INSIDE_3F    -> 3F button inside elevator
        self.declare_parameter('outside_down_prefixes', ['btn_down'])
        self.declare_parameter('inside_b1_prefixes', ['elevator_btn_under1'])
        self.declare_parameter('inside_3f_prefixes', ['elevator_btn_3'])

        # TF frame
        self.declare_parameter('base_frame', 'camera_link')   # 로봇 기준 프레임
        self.declare_parameter('fallback_camera_frame', 'camera_depth_optical_frame')  # info에 frame_id 없을 때만 사용

        # depth handling
        self.declare_parameter('depth_unit_scale', 0.001)  # 16UC1이면 mm->m (RealSense 기본)
        self.declare_parameter('depth_min_m', 0.15)
        self.declare_parameter('depth_max_m', 3.0)

        # depth sampling
        self.declare_parameter('depth_mode', 'median')  # 'center' or 'median'
        self.declare_parameter('roi_sample_grid', 5)

        # sync
        self.declare_parameter('sync_queue', 10)
        self.declare_parameter('sync_slop_sec', 0.08)

        # publish
        self.declare_parameter('marker_topic', '/object_3d_marker')
        self.declare_parameter('point_topic', '/object_3d_point')
        self.declare_parameter('marker_scale', 0.03)
        self.declare_parameter('marker_lifetime_sec', 0.2)

        # debug logs
        self.declare_parameter('debug_log', True)

        # ---------------- Load params ----------------
        self.det_topic = self.get_parameter('detections_topic').value
        self.depth_topic = self.get_parameter('depth_image_topic').value
        self.info_topic = self.get_parameter('depth_camera_info_topic').value

        self.target_prefixes = self._as_string_list(
            self.get_parameter('target_label_prefixes').value
        )
        self.target_button_topic = self.get_parameter('target_button_topic').value

        self.outside_down_prefixes = self._as_string_list(
            self.get_parameter('outside_down_prefixes').value
        )
        self.inside_b1_prefixes = self._as_string_list(
            self.get_parameter('inside_b1_prefixes').value
        )
        self.inside_3f_prefixes = self._as_string_list(
            self.get_parameter('inside_3f_prefixes').value
        )

        self.target_preset_map = {
            'OUTSIDE_DOWN': self.outside_down_prefixes,
            'INSIDE_B1': self.inside_b1_prefixes,
            'INSIDE_3F': self.inside_3f_prefixes,
        }

        self.base_frame = self.get_parameter('base_frame').value
        self.fallback_camera_frame = self.get_parameter('fallback_camera_frame').value

        self.depth_unit_scale = float(self.get_parameter('depth_unit_scale').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)

        self.depth_mode = self.get_parameter('depth_mode').value
        self.roi_sample_grid = int(self.get_parameter('roi_sample_grid').value)

        self.sync_queue = int(self.get_parameter('sync_queue').value)
        self.sync_slop = float(self.get_parameter('sync_slop_sec').value)

        marker_topic = self.get_parameter('marker_topic').value
        point_topic = self.get_parameter('point_topic').value
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.marker_lifetime_sec = float(self.get_parameter('marker_lifetime_sec').value)

        self.debug_log = bool(self.get_parameter('debug_log').value)

        # ---------------- Helpers ----------------
        self.bridge = CvBridge()

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.point_pub = self.create_publisher(PointStamped, point_topic, 10)

        # Dynamic target button subscriber
        self.target_button_sub = self.create_subscription(
            String,
            self.target_button_topic,
            self._target_button_cb,
            10
        )

        # QoS: RealSense sensor topics는 대부분 BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # message_filters subscribers (QoS 명시)
        sub_det = message_filters.Subscriber(self, DetectionArray, self.det_topic, qos_profile=sensor_qos)
        sub_dep = message_filters.Subscriber(self, Image, self.depth_topic, qos_profile=sensor_qos)
        sub_inf = message_filters.Subscriber(self, CameraInfo, self.info_topic, qos_profile=sensor_qos)

        ats = message_filters.ApproximateTimeSynchronizer(
            [sub_det, sub_dep, sub_inf],
            queue_size=self.sync_queue,
            slop=self.sync_slop
        )
        ats.registerCallback(self._sync_cb)

        self.get_logger().info(f"[object_distance_node] det={self.det_topic}")
        self.get_logger().info(f"[object_distance_node] depth={self.depth_topic}")
        self.get_logger().info(f"[object_distance_node] info={self.info_topic}")
        self.get_logger().info(f"[object_distance_node] depth_mode={self.depth_mode}, roi_grid={self.roi_sample_grid}")
        self.get_logger().info(f"[object_distance_node] base_frame={self.base_frame}")
        self.get_logger().info(f"[object_distance_node] initial target_prefixes={self.target_prefixes}")
        self.get_logger().info(f"[object_distance_node] target_button_topic={self.target_button_topic}")
        self.get_logger().info(f"[object_distance_node] preset OUTSIDE_DOWN={self.outside_down_prefixes}")
        self.get_logger().info(f"[object_distance_node] preset INSIDE_B1={self.inside_b1_prefixes}")
        self.get_logger().info(f"[object_distance_node] preset INSIDE_3F={self.inside_3f_prefixes}")

    # ---------------- Dynamic target update ----------------
    def _target_button_cb(self, msg: String) -> None:
        target = msg.data.strip()

        if not target:
            self.get_logger().warn("[object_distance_node] empty target button ignored")
            return

        target_key = target.upper()

        if target_key in self.target_preset_map:
            prefixes = self.target_preset_map[target_key]
            if not prefixes:
                self.get_logger().warn(
                    f"[object_distance_node] preset '{target_key}' has no prefixes"
                )
                return

            self.target_prefixes = list(prefixes)
            self.get_logger().info(
                f"[object_distance_node] target preset updated: "
                f"{target_key} -> {self.target_prefixes}"
            )
            return

        # Direct mode:
        #   /manipulator_perception/target_button = "btn_down"
        #   /manipulator_perception/target_button = "elevator_btn_under1,elevator_btn_3"
        # In direct mode, the received string itself is used as prefix list.
        direct_prefixes = [
            item.strip()
            for item in target.split(',')
            if item.strip()
        ]

        if not direct_prefixes:
            self.get_logger().warn(
                f"[object_distance_node] invalid direct target ignored: '{target}'"
            )
            return

        self.target_prefixes = direct_prefixes

        self.get_logger().info(
            f"[object_distance_node] target direct updated: {self.target_prefixes}"
        )

    # ---------------- Target selection ----------------
    def _is_target(self, class_name: str) -> bool:
        for pref in self.target_prefixes:
            if class_name.startswith(pref):
                return True
        return False

    def _pick_best_target(self, det_arr: DetectionArray):
        best = None
        best_score = -1.0
        for d in det_arr.detections:
            if not self._is_target(d.class_name):
                continue
            s = float(d.score)
            if s > best_score:
                best = d
                best_score = s
        return best

    # ---------------- BBox parsing (너희 구조 확정) ----------------
    def _bbox_center_and_roi(self, det) -> Tuple[int, int, Tuple[int, int, int, int]]:
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
        sx = float(det.bbox.size.x)
        sy = float(det.bbox.size.y)

        xmin = int(cx - sx * 0.5)
        xmax = int(cx + sx * 0.5)
        ymin = int(cy - sy * 0.5)
        ymax = int(cy + sy * 0.5)

        u = int(round(cx))
        v = int(round(cy))
        return u, v, (xmin, ymin, xmax, ymax)

    # ---------------- Depth utils ----------------
    def _depth_to_m(self, depth_cv: np.ndarray, encoding: str) -> np.ndarray:
        if encoding in ('16UC1', 'mono16'):
            return depth_cv.astype(np.float32) * self.depth_unit_scale
        if encoding in ('32FC1',):
            return depth_cv.astype(np.float32)
        return depth_cv.astype(np.float32)

    def _depth_at_center(self, depth_m: np.ndarray, u: int, v: int) -> Optional[float]:
        h, w = depth_m.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None
        z = float(depth_m[v, u])
        if np.isfinite(z) and (self.depth_min_m <= z <= self.depth_max_m):
            return z
        return None

    def _depth_median_in_roi(self, depth_m: np.ndarray, roi: Tuple[int, int, int, int]) -> Optional[float]:
        xmin, ymin, xmax, ymax = roi
        h, w = depth_m.shape[:2]

        xmin = max(0, min(w - 1, xmin))
        xmax = max(0, min(w - 1, xmax))
        ymin = max(0, min(h - 1, ymin))
        ymax = max(0, min(h - 1, ymax))

        if xmax <= xmin or ymax <= ymin:
            return None

        g = max(1, int(self.roi_sample_grid))
        xs = np.linspace(xmin, xmax, g).astype(int)
        ys = np.linspace(ymin, ymax, g).astype(int)

        vals: List[float] = []
        for yy in ys:
            for xx in xs:
                z = float(depth_m[yy, xx])
                if np.isfinite(z) and (self.depth_min_m <= z <= self.depth_max_m):
                    vals.append(z)

        if not vals:
            return None
        return float(np.median(np.array(vals, dtype=np.float32)))

    # ---------------- Projection ----------------
    @staticmethod
    def _deproject(u: int, v: int, z: float, K: np.ndarray) -> Tuple[float, float, float]:
        fx = float(K[0, 0]); fy = float(K[1, 1])
        cx = float(K[0, 2]); cy = float(K[1, 2])
        X = (float(u) - cx) / fx * z
        Y = (float(v) - cy) / fy * z
        return X, Y, z

    # ---------------- Main sync callback ----------------
    def _sync_cb(self, det_msg: DetectionArray, depth_msg: Image, info_msg: CameraInfo):
        if self.debug_log:
            self.get_logger().debug("SYNC_CB fired")

        det = self._pick_best_target(det_msg)
        if det is None:
            if self.debug_log:
                self.get_logger().debug("No target detection")
            return

        u, v, roi = self._bbox_center_and_roi(det)

        # depth image -> cv
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            if self.debug_log:
                self.get_logger().warn(f"Depth decode failed: {e}")
            return

        depth_m = self._depth_to_m(depth_cv, depth_msg.encoding)

        # depth pick
        if self.depth_mode == 'center':
            z = self._depth_at_center(depth_m, u, v)
        else:
            z = self._depth_median_in_roi(depth_m, roi)

        if z is None:
            if self.debug_log:
                self.get_logger().debug("Depth invalid (hole/out-of-range)")
            return

        # intrinsics
        K = np.array(info_msg.k, dtype=np.float32).reshape(3, 3)

        # camera frame 3D
        X, Y, Z = self._deproject(u, v, z, K)

        # camera frame id: camera_info 우선
        cam_frame = info_msg.header.frame_id if info_msg.header.frame_id else self.fallback_camera_frame

        p_cam = PointStamped()
        p_cam.header = Header()
        p_cam.header.stamp = depth_msg.header.stamp
        p_cam.header.frame_id = cam_frame
        p_cam.point.x = float(X)
        p_cam.point.y = float(Y)
        p_cam.point.z = float(Z)

        # transform -> base
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                p_cam.header.frame_id,
                rclpy.time.Time.from_msg(p_cam.header.stamp),
                timeout=Duration(seconds=0.1)
            )
            p_base = do_transform_point(p_cam, tf)
        except TransformException as e:
            if self.debug_log:
                self.get_logger().warn(f"TF failed: {e} (base={self.base_frame}, src={p_cam.header.frame_id})")
            return

        # publish point
        self.point_pub.publish(p_base)

        # publish marker
        mk = Marker()
        mk.header = p_base.header
        mk.ns = "object_3d"
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD

        mk.pose.position.x = p_base.point.x
        mk.pose.position.y = p_base.point.y
        mk.pose.position.z = p_base.point.z
        mk.pose.orientation.w = 1.0

        mk.scale.x = self.marker_scale
        mk.scale.y = self.marker_scale
        mk.scale.z = self.marker_scale

        mk.color.a = 1.0
        mk.color.r = 1.0
        mk.color.g = 0.2
        mk.color.b = 0.2

        mk.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()

        self.marker_pub.publish(mk)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()  