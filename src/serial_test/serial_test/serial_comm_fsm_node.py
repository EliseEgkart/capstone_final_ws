import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SerialCommFsmNode(Node):
    def __init__(self):
        super().__init__('serial_comm_fsm_node')
        self.declare_parameter('role', 'autonomy')
        self.declare_parameter('wp_arrive_topic', 'wp_arrived')
        self.declare_parameter('wp_done_topic', 'wp_done')
        self.declare_parameter('serial_tx_topic', 'serial_tx')
        self.declare_parameter('serial_rx_topic', 'serial_rx')
        self.declare_parameter('arrive_prefix', 'ARRIVED')
        self.declare_parameter('done_prefix', 'DONE')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('line_ending', '\n')
        self.declare_parameter('ws_map_yaml', '{}')
        self.declare_parameter('task_plan_yaml', '{}')
        self.declare_parameter('task_policy', 'ws_plan')  # ws_plan | first_pick_then_place_pick
        self.declare_parameter('pick_task', 'PICK3')
        self.declare_parameter('place_task', 'PLACE3')
        self.declare_parameter('done_timeout_sec', 0.0)
        self.declare_parameter('resend_sec', 0.0)
        self.declare_parameter('max_resends', 0)
        self.declare_parameter('fail_policy', 'stop')

        role = str(self.get_parameter('role').value)
        if role != 'autonomy':
            self.get_logger().warn(f'Role "{role}" not supported; running autonomy-only behavior.')

        self._arrive_prefix = str(self.get_parameter('arrive_prefix').value)
        self._done_prefix = str(self.get_parameter('done_prefix').value)

        self._done_timeout_sec = float(self.get_parameter('done_timeout_sec').value)
        self._resend_sec = float(self.get_parameter('resend_sec').value)
        self._max_resends = int(self.get_parameter('max_resends').value)
        self._fail_policy = str(self.get_parameter('fail_policy').value).lower()

        self._task_policy = str(self.get_parameter('task_policy').value).lower()
        self._pick_task = str(self.get_parameter('pick_task').value)
        self._place_task = str(self.get_parameter('place_task').value)

        self._ws_map = self._normalize_ws_map(self._load_yaml_dict(self.get_parameter('ws_map_yaml').value))
        self._task_plan = self._normalize_task_plan(
            self._load_yaml_dict(self.get_parameter('task_plan_yaml').value)
        )

        wp_arrive_topic = self.get_parameter('wp_arrive_topic').value
        wp_done_topic = self.get_parameter('wp_done_topic').value
        serial_tx_topic = self.get_parameter('serial_tx_topic').value
        serial_rx_topic = self.get_parameter('serial_rx_topic').value

        self._wp_arrive_sub = self.create_subscription(
            String, wp_arrive_topic, self._on_wp_arrive, 10
        )
        self._wp_done_pub = self.create_publisher(String, wp_done_topic, 10)
        self._serial_tx_pub = self.create_publisher(String, serial_tx_topic, 10)
        self._serial_rx_sub = self.create_subscription(
            String, serial_rx_topic, self._on_serial_rx, 10
        )

        self._current_wp = None
        self._current_ws = None
        self._tasks = []
        self._task_index = 0
        self._waiting_for_done = False
        self._resend_count = 0
        self._last_arrived_msg = None
        self._resend_timer = None
        self._timeout_timer = None

        # Global policy state: the very first stop should be PICK only. After that, always PLACE then PICK.
        self._did_first_pick = False

        self.get_logger().info(
            f'Serial comm FSM ready (policy={self._task_policy}, ARRIVED->DONE, FAIL ignored).'
        )

    def _load_yaml_dict(self, value):
        if isinstance(value, dict):
            return value
        if value is None:
            return {}
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return {}
            try:
                data = yaml.safe_load(text)
            except Exception as exc:
                self.get_logger().warn(f'Failed to parse YAML param: {exc}')
                return {}
            return data if isinstance(data, dict) else {}
        return {}

    def _normalize_ws_map(self, data):
        mapping = {}
        if not isinstance(data, dict):
            return mapping
        for key, value in data.items():
            if key is None or value is None:
                continue
            mapping[str(key)] = str(value)
        return mapping

    def _normalize_task_plan(self, data):
        plan = {}
        if not isinstance(data, dict):
            return plan
        for ws, tasks in data.items():
            if ws is None:
                continue
            ws_id = str(ws)
            if tasks is None:
                plan[ws_id] = []
            elif isinstance(tasks, str):
                plan[ws_id] = [tasks]
            else:
                plan[ws_id] = [str(task) for task in tasks]
        return plan

    def _tasks_for_ws(self, ws_id: str):
        if self._task_policy == 'first_pick_then_place_pick':
            if not self._did_first_pick:
                return [self._pick_task]
            return [self._place_task, self._pick_task]

        # Default: configured per-workstation plan.
        return self._task_plan.get(ws_id, [])

    def _on_wp_arrive(self, msg: String):
        wp_name = self._parse_wp_arrive(msg.data)
        if wp_name is None:
            self.get_logger().warn(f'Ignored arrival payload: "{msg.data}"')
            return

        if self._waiting_for_done:
            if wp_name == self._current_wp:
                self.get_logger().info(f'Duplicate arrival for "{wp_name}" ignored.')
            else:
                self.get_logger().warn(
                    f'Received arrival "{wp_name}" while busy with "{self._current_wp}".'
                )
            return

        self._start_waypoint(wp_name)

    def _parse_wp_arrive(self, data):
        text = data.strip()
        if not text:
            return None
        if self._arrive_prefix:
            prefix = f'{self._arrive_prefix}:'
            if text.startswith(prefix):
                return text[len(prefix):].strip() or None
        return text

    def _start_waypoint(self, wp_name):
        ws_id = self._ws_map.get(wp_name)
        if ws_id is None:
            self.get_logger().warn(f'No WS mapping for waypoint "{wp_name}". Skipping tasks.')
            self._publish_wp_done(wp_name)
            return

        tasks = self._tasks_for_ws(ws_id)
        if not tasks:
            self.get_logger().info(f'No tasks for WS "{ws_id}". Skipping.')
            self._publish_wp_done(wp_name)
            return

        self._current_wp = wp_name
        self._current_ws = ws_id
        self._tasks = list(tasks)
        self._task_index = 0
        self._waiting_for_done = True
        self._resend_count = 0

        self._send_arrived()
        self._start_resend_timer()
        self._start_timeout_timer()

    def _send_arrived(self):
        if not self._waiting_for_done or self._current_ws is None:
            return
        task = self._tasks[self._task_index]
        msg = String()
        msg.data = f'ARRIVED,{self._current_ws},{task}'
        self._serial_tx_pub.publish(msg)
        self._last_arrived_msg = msg.data
        self.get_logger().info(f'TX: {msg.data}')

    def _on_serial_rx(self, msg: String):
        parsed = self._parse_serial_done(msg.data)
        if parsed is None:
            return
        ws_id, task = parsed
        if not self._waiting_for_done or self._current_ws is None:
            return
        if ws_id != self._current_ws:
            return

        expected_task = self._tasks[self._task_index]
        if task != expected_task:
            self.get_logger().warn(
                f'Unexpected DONE "{task}" (expected "{expected_task}").'
            )
            return

        self._advance_task()

    def _parse_serial_done(self, data):
        text = data.strip()
        if not text:
            return None
        parts = [part.strip() for part in text.split(',') if part.strip()]
        if len(parts) < 3:
            return None
        msg_type, ws_id, task = parts[0], parts[1], parts[2]
        if msg_type != 'DONE':
            return None
        return ws_id, task

    def _advance_task(self):
        completed_task = self._tasks[self._task_index]
        if self._task_policy == 'first_pick_then_place_pick' and not self._did_first_pick:
            if completed_task == self._pick_task:
                self._did_first_pick = True

        self._task_index += 1
        if self._task_index >= len(self._tasks):
            self._finish_waypoint()
            return

        self._resend_count = 0
        self._send_arrived()
        self._start_timeout_timer()

    def _finish_waypoint(self):
        if self._task_policy == 'first_pick_then_place_pick' and not self._did_first_pick:
            # First stop completed; from the next stop onward we will always PLACE then PICK.
            self._did_first_pick = True
        if self._current_wp is not None:
            self._publish_wp_done(self._current_wp)
        self._reset_state()

    def _publish_wp_done(self, wp_name):
        msg = String()
        if self._done_prefix:
            msg.data = f'{self._done_prefix}:{wp_name}'
        else:
            msg.data = wp_name
        self._wp_done_pub.publish(msg)
        self.get_logger().info(f'WP done published: {msg.data}')

    def _reset_state(self):
        self._waiting_for_done = False
        self._current_wp = None
        self._current_ws = None
        self._tasks = []
        self._task_index = 0
        self._resend_count = 0
        self._last_arrived_msg = None
        self._cancel_resend_timer()
        self._cancel_timeout_timer()

    def _start_resend_timer(self):
        if self._resend_sec <= 0.0:
            return
        if self._resend_timer is None:
            self._resend_timer = self.create_timer(self._resend_sec, self._resend_cb)

    def _cancel_resend_timer(self):
        if self._resend_timer is not None:
            self._resend_timer.cancel()
            self._resend_timer = None

    def _resend_cb(self):
        if not self._waiting_for_done or not self._last_arrived_msg:
            return
        if self._max_resends > 0 and self._resend_count >= self._max_resends:
            if self._fail_policy == 'skip_waypoint':
                self.get_logger().warn('Max resends reached; skipping waypoint.')
                self._finish_waypoint()
            else:
                self.get_logger().error('Max resends reached; holding position.')
                self._cancel_resend_timer()
            return

        msg = String()
        msg.data = self._last_arrived_msg
        self._serial_tx_pub.publish(msg)
        self._resend_count += 1
        self.get_logger().info(f'Resend ({self._resend_count}): {msg.data}')

    def _start_timeout_timer(self):
        if self._done_timeout_sec <= 0.0:
            return
        self._cancel_timeout_timer()
        self._timeout_timer = self.create_timer(self._done_timeout_sec, self._timeout_cb)

    def _cancel_timeout_timer(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    def _timeout_cb(self):
        self._cancel_timeout_timer()
        if not self._waiting_for_done:
            return
        if self._fail_policy == 'skip_waypoint':
            self.get_logger().warn('DONE timeout reached; skipping waypoint.')
            self._finish_waypoint()
        else:
            self.get_logger().error('DONE timeout reached; holding position.')
            self._cancel_resend_timer()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommFsmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
