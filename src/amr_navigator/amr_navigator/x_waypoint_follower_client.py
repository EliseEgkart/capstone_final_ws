import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String, Float32MultiArray
from action_msgs.msg import GoalStatus

class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        # from control_center
        self.control_from_center = self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)
        # to control_center
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'destination',
            self.destination_callback,
            10)
        self.waypoints = []
        self.goal_accept = False
        self.msg_center = None
        self.goal_handle = None
        self.cmd_old = None

    def destination_callback(self, msg):
        self.get_logger().error(f'---------------------------')
        self.get_logger().error(f'msg_center: {self.msg_center}')
        self.get_logger().error(f'cmd_old: {self.cmd_old}')
        self.get_logger().error(f'---------------------------')
        if(not self.goal_accept):
            self.waypoints = self.parse_waypoints(msg.data)
            if self.waypoints:
                self.get_logger().info(f'Received waypoints: {self.waypoints}')
                if self.msg_center in ['d1_start', 'd2_start', 'd3_start']:

                    self.send_goal(self.waypoints)
                    msg1 = String()
                    msg1.data = 'driving_start'
                    self.control_from_amr.publish(msg1)
                else:
                    self.get_logger().warn('No valid command received before sending goal')
            else:
                self.get_logger().warn('No valid waypoints received')

    def control_from_center_callback(self, msg):  #None 들어오는건 안받음.!
        self.get_logger().info(f'msg_center = {self.msg_center} from control_from_center_callback')
        if msg.data in ['d1_start', 'd2_start', 'd3_start']:
            self.get_logger().info(f'self.msg_center = {self.msg_center}')
            self.get_logger().info(f'self.msg_center_old = {self.cmd_old}')
            
            if self.cmd_old!= msg.data:
                self.msg_center = msg.data
                self.cmd_old = msg.data


        elif msg.data == 'kill':
            self.get_logger().info('Received kill command, cancelling current goal.')
            self.cancel_goal()

    def parse_waypoints(self, data):
        waypoints = []
        for i in range(0, len(data), 7):
            x, y, z, qx, qy, qz, qw = data[i:i+7]
            waypoints.append(self.create_pose_stamped(x, y, z, qx, qy, qz, qw))
        return waypoints

    def create_pose_stamped(self, x, y, z, qx, qy, qz, qw):
        pose_stamped = PoseStamped(
            header=Header(frame_id='map'),
            pose=Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
            )
        )
        return pose_stamped

    def send_goal(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        self.goal_accept = self.goal_handle.accepted
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            missed_waypoints = result.missed_waypoints

            if not missed_waypoints:
                self.get_logger().info('All waypoints followed successfully.')
                
            else:
                self.get_logger().info(f'Waypoints missed: {missed_waypoints}')


            self.msg_center = None
            self.get_logger().info(f'--------------------------------------------------')
            self.get_logger().info(f'--------------------------------------------------')    
            self.get_logger().info(f'--------------------------------------------------')    
            self.get_logger().info(f'msg_center = {self.msg_center} get_result_callback')    
            self.get_logger().info(f'--------------------------------------------------')    
            self.get_logger().info(f'--------------------------------------------------')    
            self.get_logger().info(f'--------------------------------------------------')    
            msg = String()
            msg.data = 'driving_done'
            self.control_from_amr.publish(msg)
            self.waypoints = []
            self.goal_accept = False
            self.msg_center = None
            self.goal_handle = None

        except Exception as e:
            self.get_logger().error(f'Exception in get_result_callback: {e}')
            self.reset_state()

    def cancel_goal(self):
        if self.goal_handle is not None and self.goal_accept:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().warn('No goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal successfully cancelled.')
        elif cancel_response.return_code == GoalStatus.STATUS_UNKNOWN:
            self.get_logger().warn('Goal cancellation failed: unknown goal.')
        else:
            self.get_logger().warn(f'Goal cancellation returned with code: {cancel_response.return_code}')
        
        self.reset_state()



    def reset_state(self):
        self.waypoints = []
        self.goal_accept = False
        self.msg_center = None
        self.goal_handle = None
        self.cmd_old = None

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
