# Copyright (C) 2023  Miguel Ángel González Santamarta
# Modified to support multi DetectionArray topics merge via `models_topics` param.
# Default: subscribe & merge ["detections/best", "detections/best_person"].

import cv2
import random
import numpy as np
from typing import Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState

import message_filters
from cv_bridge import CvBridge
from ultralytics.utils.plotting import Annotator, colors

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from interfaces_pkg.msg import BoundingBox2D, KeyPoint2D, KeyPoint3D, Detection, DetectionArray


class Yolov8VisualizerNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("yolov8_visualizer_node")
        self._class_to_color = {}
        self.cv_bridge = CvBridge()

        # parameters
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.RELIABLE)
        # 기본값을 두 토픽으로 설정 → ros2 run 만으로 다중 토픽 처리
        self.declare_parameter("models_topics", ["detections/best", "detections/best_person"])  # string[]

        self.get_logger().info("Yolov8VisualizerNode created")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter("image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # pubs
        self._dbg_pub = self.create_publisher(Image, "yolov8_visualized_img", 10)
        self._bb_markers_pub = self.create_publisher(MarkerArray, "dgb_bb_markers", 10)
        self._kp_markers_pub = self.create_publisher(MarkerArray, "dgb_kp_markers", 10)

        # subs/filters 핸들 보관용
        self.image_sub = None
        self.multi_dets_subs: List[message_filters.Subscriber] = []
        self._sync = None

        # 파라미터 읽기
        self.models_topics: List[str] = list(
            self.get_parameter("models_topics").get_parameter_value().string_array_value
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        # Image 구독
        self.image_sub = message_filters.Subscriber(self, Image, "image_raw", qos_profile=self.image_qos_profile)

        # DetectionArray 구독 (단일/다중)
        if self.models_topics:  # multi-topic mode
            self.get_logger().info(f"Subscribing to multiple detection topics: {self.models_topics}")
            self.multi_dets_subs = [
                message_filters.Subscriber(self, DetectionArray, topic, qos_profile=10)
                for topic in self.models_topics
            ]
        else:  # single-topic mode (default fallback: "detections")
            self.get_logger().info("Subscribing to single detection topic: detections")
            self.multi_dets_subs = [
                message_filters.Subscriber(self, DetectionArray, "detections", qos_profile=10)
            ]

        # 동기화 구성
        if len(self.multi_dets_subs) == 1:
            # 이미지 + 단일 detections 동기화
            self._sync = message_filters.ApproximateTimeSynchronizer(
                (self.image_sub, self.multi_dets_subs[0]), queue_size=20, slop=0.5
            )
            self._sync.registerCallback(lambda img, dets: self.detections_cb(img, dets))
        else:
            # 이미지 + 다중 detections 동기화 → merged DetectionArray 생성
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub] + self.multi_dets_subs, queue_size=50, slop=0.6
            )

            def multi_cb(*msgs):
                img_msg: Image = msgs[0]
                det_arrays: List[DetectionArray] = list(msgs[1:])

                merged = DetectionArray()
                merged.header = img_msg.header
                for da in det_arrays:
                    # 타 토픽에서 넘어온 헤더 타임스탬프가 약간씩 달라도 이미지 기준으로 머지
                    merged.detections.extend(da.detections)

                self.detections_cb(img_msg, merged)

            self._sync.registerCallback(multi_cb)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        # 동기화 객체 제거
        if self._sync is not None:
            del self._sync
            self._sync = None

        # 구독 해제
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub.sub)
                self.image_sub = None
        except Exception:
            pass

        for sub in self.multi_dets_subs:
            try:
                self.destroy_subscription(sub.sub)
            except Exception:
                pass
        self.multi_dets_subs.clear()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        self.destroy_publisher(self._dbg_pub)
        self.destroy_publisher(self._bb_markers_pub)
        self.destroy_publisher(self._kp_markers_pub)

        return TransitionCallbackReturn.SUCCESS

    # ----------------- Drawing helpers -----------------

    def draw_box(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:
        label = detection.class_name
        score = detection.score
        box_msg: BoundingBox2D = detection.bbox
        track_id = detection.id

        min_pt = (round(box_msg.center.position.x - box_msg.size.x / 2.0),
                  round(box_msg.center.position.y - box_msg.size.y / 2.0))
        max_pt = (round(box_msg.center.position.x + box_msg.size.x / 2.0),
                  round(box_msg.center.position.y + box_msg.size.y / 2.0))

        cv2.rectangle(cv_image, min_pt, max_pt, color, 2)
        label_text = f"{label} ({track_id}) ({score:.3f})"
        cv2.putText(cv_image, label_text, (min_pt[0] + 5, min_pt[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)
        return cv_image

    def draw_mask(self, cv_image: np.array, detection: Detection, color: Tuple[int]) -> np.array:
        mask_msg = detection.mask
        if not mask_msg.data:
            return cv_image

        mask_array = np.array([[int(ele.x), int(ele.y)] for ele in mask_msg.data])
        layer = cv_image.copy()
        layer = cv2.fillPoly(layer, pts=[mask_array], color=color)
        cv2.addWeighted(cv_image, 0.4, layer, 0.6, 0, cv_image)
        cv_image = cv2.polylines(cv_image, [mask_array], isClosed=True, color=color, thickness=2, lineType=cv2.LINE_AA)
        return cv_image

    def draw_keypoints(self, cv_image: np.array, detection: Detection) -> np.array:
        keypoints_msg = detection.keypoints
        if not keypoints_msg.data:
            return cv_image

        ann = Annotator(cv_image)
        # 포인트
        for kp in keypoints_msg.data:
            color_k = [int(x) for x in ann.kpt_color[kp.id - 1]] if len(keypoints_msg.data) == 17 else colors(kp.id - 1)
            cv2.circle(cv_image, (int(kp.point.x), int(kp.point.y)), 5, color_k, -1, lineType=cv2.LINE_AA)

        # 스켈레톤 연결
        def get_pk_pose(kp_id: int):
            for kp in keypoints_msg.data:
                if kp.id == kp_id:
                    return (int(kp.point.x), int(kp.point.y))
            return None

        for i, sk in enumerate(ann.skeleton):
            kp1_pos = get_pk_pose(sk[0])
            kp2_pos = get_pk_pose(sk[1])
            if kp1_pos is not None and kp2_pos is not None:
                cv2.line(cv_image, kp1_pos, kp2_pos, [int(x) for x in ann.limb_color[i]], thickness=2, lineType=cv2.LINE_AA)

        return cv_image

    def create_bb_marker(self, detection: Detection, color: Tuple[int]) -> Marker:
        bbox3d = detection.bbox3d
        marker = Marker()
        marker.header.frame_id = bbox3d.frame_id

        marker.ns = "yolov8_3d"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = bbox3d.center.position.x
        marker.pose.position.y = bbox3d.center.position.y
        marker.pose.position.z = bbox3d.center.position.z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = bbox3d.size.x
        marker.scale.y = bbox3d.size.y
        marker.scale.z = bbox3d.size.z

        marker.color.b = color[0] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.r = color[2] / 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = detection.class_name

        return marker

    def create_kp_marker(self, keypoint: KeyPoint3D) -> Marker:
        marker = Marker()
        marker.ns = "yolov8_3d"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = False

        marker.pose.position.x = keypoint.point.x
        marker.pose.position.y = keypoint.point.y
        marker.pose.position.z = keypoint.point.z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.b = keypoint.score * 255.0
        marker.color.g = 0.0
        marker.color.r = (1.0 - keypoint.score) * 255.0
        marker.color.a = 0.4

        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.text = str(keypoint.id)

        return marker

    # ----------------- Core callback -----------------

    def detections_cb(self, img_msg: Image, detection_msg: DetectionArray) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)

        bb_marker_array = MarkerArray()
        kp_marker_array = MarkerArray()

        for detection in detection_msg.detections:
            # class별 고정 색상
            label = detection.class_name
            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)
            color = self._class_to_color[label]

            cv_image = self.draw_box(cv_image, detection, color)
            cv_image = self.draw_mask(cv_image, detection, color)
            cv_image = self.draw_keypoints(cv_image, detection)

            # 3D 마커(있을 때만)
            if detection.bbox3d.frame_id:
                marker = self.create_bb_marker(detection, color)
                marker.header.stamp = img_msg.header.stamp
                marker.id = len(bb_marker_array.markers)
                bb_marker_array.markers.append(marker)

            if detection.keypoints3d.frame_id:
                for kp in detection.keypoints3d.data:
                    marker = self.create_kp_marker(kp)
                    marker.header.frame_id = detection.keypoints3d.frame_id
                    marker.header.stamp = img_msg.header.stamp
                    marker.id = len(kp_marker_array.markers)
                    kp_marker_array.markers.append(marker)

        # publish outputs
        self._dbg_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding=img_msg.encoding))
        self._bb_markers_pub.publish(bb_marker_array)
        self._kp_markers_pub.publish(kp_marker_array)


def main():
    rclpy.init()
    node = Yolov8VisualizerNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
