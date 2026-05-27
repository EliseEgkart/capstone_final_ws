import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def yaw_to_quaternion(yaw: float):
    """
    yaw(rad) -> (qx,qy,qz,qw)
    roll=pitch=0 가정
    """
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


class Navigator(Node):
    def __init__(self):
        # YAML에서 override한 파라미터를 자동 선언하도록 설정
        super().__init__(
            'navigator',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # ★ 여기 수정: Parameter 객체에서 .value만 꺼낸다
        action_param = self.get_parameter_or(
            'action_server_name',
            Parameter('action_server_name', Parameter.Type.STRING, 'navigate_to_pose')
        )
        cmd_param = self.get_parameter_or(
            'cmd_topic',
            Parameter('cmd_topic', Parameter.Type.STRING, 'destination')
        )

        self.action_server_name = action_param.value   # str
        self.cmd_topic = cmd_param.value               # str

        # Nav2 액션 클라이언트
        self.client = ActionClient(self, NavigateToPose, self.action_server_name)

        # 목적지 명령 구독 (std_msgs/String)
        self.cmd_sub = self.create_subscription(
            String,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        # 상태 관리
        self.goal_handle = None
        self.goal_in_progress = False
        self.pending_cmd = None  # 서버 준비 안 됐을 때 대기

        # points 로드
        self.points = self.load_points()

        # 서버 준비 체크 타이머 (blocking 피하기)
        self.ready_timer = self.create_timer(0.5, self.check_server_ready)

        self.get_logger().info(
            f'Navigator up. cmd_topic={self.cmd_topic}, action={self.action_server_name}'
        )
        self.get_logger().info(f'Loaded points: {list(self.points.keys())}')

    def load_points(self):
        """
        points.<name>.x/y/z/yaw 파라미터를 읽어 dict로 구성
        """
        raw = self.get_parameters_by_prefix('points')
        # raw key 예: "elevator_in.x", "204.yaw" ...
        points = {}
        for full_key, param in raw.items():
            # full_key = "<name>.<field>"
            if '.' not in full_key:
                continue
            name, field = full_key.split('.', 1)
            points.setdefault(name, {})
            points[name][field] = param.value

        # 필수 필드 체크
        valid = {}
        for name, fields in points.items():
            if all(k in fields for k in ['x', 'y', 'z', 'yaw']):
                valid[name] = fields
            else:
                self.get_logger().warn(f'Point "{name}" missing fields: {fields.keys()}')
        return valid

    def check_server_ready(self):
        if self.client.server_is_ready():
            # 대기 중인 명령이 있으면 즉시 처리
            if self.pending_cmd is not None and not self.goal_in_progress:
                cmd = self.pending_cmd
                self.pending_cmd = None
                self.get_logger().info(f'Action ready. Sending pending cmd: {cmd}')
                self.start_navigation(cmd)

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f'Received cmd: {cmd}')

        if cmd not in self.points:
            self.get_logger().warn(
                f'Unknown cmd "{cmd}". Known: {list(self.points.keys())}'
            )
            return

        # 서버 준비 안 됐으면 보류
        if not self.client.server_is_ready():
            self.pending_cmd = cmd
            self.get_logger().warn('Action server not ready. Command queued.')
            return

        # 이미 이동 중이면 선점(preempt): 기존 goal cancel 후 새 goal
        if self.goal_in_progress and self.goal_handle is not None:
            self.get_logger().warn(f'Preempting current goal with new cmd: {cmd}')
            self.pending_cmd = cmd
            self.cancel_current_goal()
            return

        self.start_navigation(cmd)

    def start_navigation(self, cmd: str):
        p = self.points[cmd]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(p['x'])
        pose.pose.position.y = float(p['y'])
        pose.pose.position.z = float(p['z'])

        qx, qy, qz, qw = yaw_to_quaternion(float(p['yaw']))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.goal_in_progress = True
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut: self.goal_response_callback(fut, cmd))

    def goal_response_callback(self, future, cmd: str):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected for cmd: {cmd}')
            self.reset_goal_state()
            return

        self.get_logger().info(f'Goal accepted for cmd: {cmd}')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.result_callback(fut, cmd))

    def result_callback(self, future, cmd: str):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrived at {cmd}')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Goal canceled: {cmd}')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f'Goal aborted: {cmd}')
        else:
            self.get_logger().warn(
                f'Goal finished with status={status} for cmd={cmd}'
            )

        self.reset_goal_state()

        # cancel 선점으로 pending_cmd가 쌓여있으면 이어서 실행
        if self.pending_cmd is not None:
            next_cmd = self.pending_cmd
            self.pending_cmd = None
            if self.client.server_is_ready():
                self.get_logger().info(f'Sending queued cmd: {next_cmd}')
                self.start_navigation(next_cmd)

    def cancel_current_goal(self):
        if self.goal_handle is None:
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda fut: self.get_logger().info('Cancel requested.')
        )

    def reset_goal_state(self):
        self.goal_in_progress = False
        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
