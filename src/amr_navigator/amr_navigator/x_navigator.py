import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

amr_number = os.getenv('AMR_NUMBER', '0')

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        # from control_center 
        self.control_from_center = self.create_subscription(String, '/control_from_center', self.control_from_center_callback, 10)
        # to control_center 
        self.control_from_amr = self.create_publisher(String, '/control_from_amr', 10)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(
            String,
            'destination',
            self.destination_callback,
            10)
        self.goal_pose = PoseStamped()
        self.destinations = {
            'd1': self.create_pose(3.8832424844533313, 0.0704704581537811, 0.0, 0.0, 0.0, -0.6249009942792362, 0.7807040075142576),
            'd2': self.create_pose(11.769884431441493, -1.4092172146102284, 0.0, 0.0, 0.0, -0.9946918895141301, 0.10289822610137385),
            'd3': self.create_pose(13.957017221667478, -3.118006990215112, 0.0, 0.0, 0.0, 0.054463256746689934,  0.9985157753708972),
        }

        self.goal_accept = False
        self.destination = None

    def create_pose(self, x, y, z, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def destination_callback(self, msg):
        destination = msg.data
        self.get_logger().info(f'Received destination command: {destination}')
        if destination in self.destinations:
            self.send_goal(self.destinations[destination])
        else:
            self.get_logger().warn(f'Unknown destination: {destination}')
            
    def send_goal(self, pose):
        self.goal_pose = pose
        self.client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.goal_accept = goal_handle.accepted


        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            status = future.result().status

            if status == 4:  # STATUS_SUCCEEDED
                self.goal_accept = False  # 초기화 필요
                self.get_logger().info('Goal reached successfully.')

                msg = String()
                if self.destination == 'd1':
                    msg.data = 'd1_done'
                    self.control_from_amr.publish(msg)
                    self.destination = None

                if self.destination == 'd2':
                    msg.data = 'd2_done'
                    self.control_from_amr.publish(msg)
                    self.destination = None

                if self.destination == 'd3':
                    msg.data = 'd3_done'
                    self.control_from_amr.publish(msg)
                    self.destination = None
                

            else:
                self.get_logger().info(f'Goal failed with status: {status}')
                self.handle_failure(status)
                
        except Exception as e:
            self.get_logger().error(f'Exception in get_result_callback: {e}')
            self.handle_failure(e)
    
    def handle_failure(self, reason):
        self.get_logger().info(f'실패함 ㅠ: {reason}')



    def control_from_center_callback(self, data):
        message = data.data
        if message == 'd1_start':
            if(not self.goal_accept):
                self.destination = 'd1'
                self.get_logger().info(f'Received destination command: {self.destination}')

                if self.destination in self.destinations:
                    self.send_goal(self.destinations[self.destination])
                else:
                    self.get_logger().warn(f'Unknown destination: {self.destination}')
                
            
            msg = String()
            msg.data = 'd1_start'
            self.control_from_amr.publish(msg)

        if message == 'd2_start':

            if(not self.goal_accept):
                self.destination = 'd2'
                self.get_logger().info(f'Received destination command: {self.destination}')

                if self.destination in self.destinations:
                    self.send_goal(self.destinations[self.destination])
                else:
                    self.get_logger().warn(f'Unknown destination: {self.destination}') 
            msg = String()
            msg.data = 'd2_start'
            self.control_from_amr.publish(msg)

        if message == 'd3_start':

            if(not self.goal_accept):
                self.destination = 'd3'
                self.get_logger().info(f'Received destination command: {self.destination}')

                if self.destination in self.destinations:
                    self.send_goal(self.destinations[self.destination])
                else:
                    self.get_logger().warn(f'Unknown destination: {self.destination}') 
            msg = String()
            msg.data = 'd3_start'
            self.control_from_amr.publish(msg)

        elif message == 'stop':
            self.lifecycle = False
            msg = String()
            msg.data = 'stop'
            self.control_from_amr.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationClient()
    rclpy.spin(navigation_client)
    
    navigation_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()