import os
from typing import Dict, List, Optional, Set

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from ament_index_python.packages import get_package_share_directory
import yaml


class YamlWaypointFollower(Node):
    def __init__(self):
        super().__init__('yaml_waypoint_follower')
        default_yaml = os.path.join(
            get_package_share_directory('amr_navigator'),
            'params',
            '1F_IT_building.yaml',
        )
        self.declare_parameter('waypoints_file', default_yaml)
        # NOTE: An empty list is ambiguous in rclpy and gets inferred as BYTE_ARRAY.
        # Use an empty string instead and parse into a list when provided.
        self.declare_parameter('sequence', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('repeat', False)
        self.declare_parameter('repeat_delay_sec', 0.0)

        self.declare_parameter('arrive_topic', 'serial_tx')
        self.declare_parameter('done_topic', 'serial_rx')
        self.declare_parameter('arrive_prefix', 'ARRIVED')
        self.declare_parameter('done_prefix', 'DONE')
        self.declare_parameter('wait_for_done', True)
        self.declare_parameter('require_done_match', True)
        self.declare_parameter('continue_on_miss', False)

        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._goal_handle = None
        self._goal_accept = False
        self._repeat_timer = None

        self._arrive_prefix = self.get_parameter('arrive_prefix').value
        self._done_prefix = self.get_parameter('done_prefix').value
        self._wait_for_done = bool(self.get_parameter('wait_for_done').value)
        self._require_done_match = bool(self.get_parameter('require_done_match').value)
        self._continue_on_miss = bool(self.get_parameter('continue_on_miss').value)

        arrive_topic = self.get_parameter('arrive_topic').value
        done_topic = self.get_parameter('done_topic').value
        self._arrive_pub = self.create_publisher(String, arrive_topic, 10)
        self._done_sub = self.create_subscription(String, done_topic, self._done_callback, 10)

        self._waypoints_map, self._sequence, self._frame_id = self._load_waypoints()
        self._reset_sequence_state()

        if self.get_parameter('auto_start').value:
            self._start_sequence()
        else:
            self.get_logger().info('auto_start is false; waiting for manual start.')

    def _reset_sequence_state(self):
        self._current_index = 0
        self._current_name: Optional[str] = None
        self._waiting_for_done = False
        self._expected_done: Optional[str] = None
        self._done_names: Set[str] = set()
        self._done_any = False

    def _load_waypoints(self):
        path = self.get_parameter('waypoints_file').value
        frame_id = self.get_parameter('frame_id').value
        if not os.path.exists(path):
            self.get_logger().error(f'Waypoints file not found: {path}')
            return {}, [], frame_id

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints file: {exc}')
            return {}, [], frame_id

        if isinstance(data, dict) and data.get('frame_id'):
            frame_id = data.get('frame_id')

        waypoints_map = data.get('waypoints', {}) if isinstance(data, dict) else {}

        seq_param = self.get_parameter('sequence').value
        if seq_param:
            if isinstance(seq_param, str):
                try:
                    parsed = yaml.safe_load(seq_param)
                    if isinstance(parsed, list):
                        sequence = parsed
                    else:
                        cleaned = seq_param.strip().strip('[]')
                        sequence = [item.strip() for item in cleaned.split(',') if item.strip()]
                except Exception:
                    cleaned = seq_param.strip().strip('[]')
                    sequence = [item.strip() for item in cleaned.split(',') if item.strip()]
            else:
                sequence = list(seq_param)
        else:
            sequence = data.get('sequence', list(waypoints_map.keys())) if isinstance(data, dict) else []

        return waypoints_map, sequence, frame_id

    def _start_sequence(self):
        if not self._sequence:
            self.get_logger().warn('No waypoint sequence configured.')
            return

        self._reset_sequence_state()
        self._send_current_goal()

    def _send_current_goal(self):
        if self._current_index >= len(self._sequence):
            self.get_logger().info('Waypoint sequence complete.')
            if self.get_parameter('repeat').value:
                self._schedule_repeat()
            return

        name = self._sequence[self._current_index]
        entry = self._waypoints_map.get(name)
        if entry is None:
            self.get_logger().warn(f'Waypoint "{name}" not found in YAML; skipping.')
            self._advance_to_next()
            return

        pose = self._entry_to_pose(name, entry)
        if pose is None:
            self.get_logger().warn(f'Waypoint "{name}" invalid; skipping.')
            self._advance_to_next()
            return

        self._current_name = name
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending waypoint "{name}" ({self._current_index + 1}/{len(self._sequence)})')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _entry_to_pose(self, name: str, entry: Dict) -> Optional[PoseStamped]:
        try:
            if 'pose' in entry:
                pose_list = entry['pose']
                if len(pose_list) != 7:
                    raise ValueError('pose list must have 7 elements')
                x, y, z, qx, qy, qz, qw = pose_list
            else:
                position = entry.get('position', {})
                orientation = entry.get('orientation', {})
                x = position.get('x')
                y = position.get('y')
                z = position.get('z', 0.0)
                qx = orientation.get('x', 0.0)
                qy = orientation.get('y', 0.0)
                qz = orientation.get('z', 0.0)
                qw = orientation.get('w', 1.0)

            if x is None or y is None:
                raise ValueError('position.x and position.y are required')

            pose_stamped = PoseStamped(
                header=Header(frame_id=self._frame_id),
                pose=Pose(
                    position=Point(x=float(x), y=float(y), z=float(z)),
                    orientation=Quaternion(
                        x=float(qx), y=float(qy), z=float(qz), w=float(qw)
                    ),
                ),
            )
            return pose_stamped
        except Exception as exc:
            self.get_logger().error(f'Invalid waypoint "{name}": {exc}')
            return None

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        self._goal_accept = self._goal_handle.accepted
        if not self._goal_accept:
            self.get_logger().warn('Goal rejected :(')
            if self._continue_on_miss:
                self._advance_to_next()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        try:
            result = future.result().result
            missed_waypoints = result.missed_waypoints
            if missed_waypoints:
                self.get_logger().warn(f'Waypoint "{self._current_name}" missed: {missed_waypoints}')
                if self._continue_on_miss:
                    self._advance_to_next()
                return

            if self._current_name is not None:
                self._handle_waypoint_arrived(self._current_name)
        except Exception as exc:
            self.get_logger().error(f'Exception in get_result_callback: {exc}')
        finally:
            self._goal_handle = None
            self._goal_accept = False

    def _handle_waypoint_arrived(self, name: str):
        self._publish_arrival(name)
        if self._wait_for_done:
            self._waiting_for_done = True
            self._expected_done = name if self._require_done_match else None
            if self._is_done_ready(name):
                self._consume_done(name)
                self._advance_to_next()
        else:
            self._advance_to_next()

    def _publish_arrival(self, name: str):
        msg = String()
        if self._arrive_prefix:
            msg.data = f'{self._arrive_prefix}:{name}'
        else:
            msg.data = name
        self._arrive_pub.publish(msg)
        self.get_logger().info(f'Arrival flag sent: {msg.data}')

    def _done_callback(self, msg: String):
        done_name = self._parse_done(msg.data)
        if done_name is None:
            if not self._require_done_match:
                self._done_any = True
        else:
            self._done_names.add(done_name)

        if self._waiting_for_done and self._current_name is not None:
            if self._is_done_ready(self._current_name):
                self._consume_done(self._current_name)
                self._advance_to_next()

    def _parse_done(self, data: str) -> Optional[str]:
        text = data.strip()
        if not text:
            return None
        if not self._done_prefix:
            return text
        if text == self._done_prefix:
            return None
        prefix = f'{self._done_prefix}:'
        if text.startswith(prefix):
            return text[len(prefix):].strip() or None
        return None

    def _is_done_ready(self, name: str) -> bool:
        if not self._wait_for_done:
            return True
        if self._require_done_match:
            return name in self._done_names
        return self._done_any or name in self._done_names

    def _consume_done(self, name: str):
        if self._require_done_match:
            if name in self._done_names:
                self._done_names.remove(name)
        else:
            if self._done_any:
                self._done_any = False
            if name in self._done_names:
                self._done_names.remove(name)

    def _advance_to_next(self):
        self._waiting_for_done = False
        self._expected_done = None
        self._current_index += 1
        self._send_current_goal()

    def _schedule_repeat(self):
        delay = float(self.get_parameter('repeat_delay_sec').value)

        def _restart():
            if self._repeat_timer is not None:
                self._repeat_timer.cancel()
                self._repeat_timer = None
            self._start_sequence()

        if delay <= 0.0:
            _restart()
        else:
            self._repeat_timer = self.create_timer(delay, _restart)


def main(args=None):
    rclpy.init(args=args)
    node = YamlWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
