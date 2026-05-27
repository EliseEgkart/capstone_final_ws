import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Joy
from amr_msgs.msg import WheelMotor
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_euler,euler_from_quaternion
from rclpy.time import Time
import numpy as np


from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sklearn.cluster import DBSCAN
import struct
import sensor_msgs_py.point_cloud2 as pc2


from serial_test.motor_driver import MotorDriver


import serial  ###5/5 update (STM)



class Nodelet(Node):
    def __init__(self):
        super().__init__('serial_test')
        self.pub = self.create_publisher(WheelMotor, '/wheelmotor', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        #################################################
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.VOLATILE,
        #     depth=10
        # )     
        
        # self.lidar_scan = self.create_subscription(LaserScan,'/scan',self.ladar_callback,qos_profile_sensor_data)
   
        # self.point_cloud_pub_ = self.create_publisher(PointCloud2, 'clustered_points', qos_profile)
       #################################################

        self.dt = 0.02
        self.timer_ = self.create_timer(self.dt, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
       
        ####6/19 global coordinate####
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ############## 5/5 UPDATE (STM) ##############
        self.serial_port = serial.Serial(
            port='/dev/ttyACM1',  # STM32가 연결된 포트
            baudrate=115200,      # 통신 속도
            timeout=1            # 타임아웃 설정
        )
        # self.timer_stm = self.create_timer(self.dt, self.check_serial)  
        self.joy_lift_up=0
        self.joy_lift_down=0
        ##############################################

        self.loopcnt = 0

        self.firstloop = True
        self.JOY_CONTROL = True

        # Motor driver class
        self.md = MotorDriver()

        # PID related variables
        self.p_gain = 1.
        self.i_gain = 0.
        self.d_gain = 0.01
        self.forget = 0.99

        self.err1_prev, self.err1_i = 0., 0.
        self.err2_prev, self.err2_i = 0., 0.
        self.torque1, self.torque2 = 0, 0
        self.velocity1, self.velocity2 = 0, 0

        # target position
        self.target_pos1, self.target_pos2 = 0, 0

        # joy gain
        self.joy_fb = 0
        self.joy_lr = 0
        self.v_gain = 10
        self.w_gain = 5

        self.joy_r2 = 0
        self.joy_l2 = 0
        self.change_mode = 0
        self.joy_stop = 0
        self.joy_lift_down_old = 0
        self.joy_lift_up_old = 0

        self.msg_wheelmotor = WheelMotor()

        # Change BAUDRATE
        # self.md.write_BAUD()

        #odom param
        self.wheel_separation = 0.298  # Adjust as necessary
        self.wheel_diameter = 0.17    # Adjust as necessary
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.last_time = self.get_clock().now()
        

         # Encoder positions
        self.last_pos1 = 0.0
        self.last_pos2 = 0.0
        self.cur_pos1 = 0.0
        self.cur_pos2 = 0.0
        self.del_pos1 = 0.0
        self.del_pos2 = 0.0

         # cmd_vel param
        self.cmd_vel_r = 0.0
        self.cmd_vel_l = 0.0
        
        self.vel_input1 = 0.0
        self.vel_input2 = 0.0

        #lowpass filter
        self.v_motor_last = 0.0
        self.w_motor_last = 0.0
        self.alpha = 0.5

        # lift_leg_distance
        self.lift_leg_distance = 0.47   # 전체 길이 50cm - 다리 두께 1.5 *2 (다리 두께 절반이 1.5)

    def timer_callback(self):
        self.loopcnt += 1

        # self.md.version_check()
        # self.md.recv_read_this()

        if self.firstloop:
            # self.md.send_torque_cmd(self.torque1, self.torque2)
            self.md.send_vel_cmd(self.velocity1, self.velocity2)
            #self.md.send_position_cmd(self.md.pos1, self.md.pos2, int(60), int(60))
            self.md.recv_motor_state()
            self.target_pos1 = self.md.pos1
            self.target_pos2 = self.md.pos2
            # self.target_pos1 = 320
            # self.target_pos2 =  800
 
            
            self.del_pos1 = self.md.pos1
            self.del_pos2 = self.md.pos2

            self.firstloop = False
            return

        if self.JOY_CONTROL:
            self.vel_input1 =  self.v_gain*self.joy_fb
            self.vel_input1 -= self.w_gain*self.joy_lr
            self.vel_input2 =  self.v_gain*self.joy_fb
            self.vel_input2 += self.w_gain*self.joy_lr

            if self.joy_stop == 1 :
                self.md.send_position_cmd(self.md.pos1, self.md.pos2, int(60), int(60))
                self.get_logger().info(f'stop')
            else :
                self.md.send_vel_cmd(self.vel_input1, self.vel_input2)
                #self.get_logger().info(f'vel_input : {self.vel_input1} , {self.vel_input2}')

            # self.target_pos1 += self.v_gain * self.joy_fb
            # self.target_pos1 += self.w_gain * self.joy_lr

            # self.target_pos2 += self.v_gain * self.joy_fb
            # self.target_pos2 -= self.w_gain * self.joy_lr

            # self.target_pos1 -= 5 * self.joy_fb
            # self.target_pos1 += 5 * self.joy_lr

            # self.target_pos2 -= 5 * self.joy_fb
            # self.target_pos2 -= 5 * self.joy_lr
            
            # self.md.send_position_cmd(int(self.target_pos1),int(self.target_pos2), int(10),int(10))
            
            # self.position_control(int(self.target_pos1), int(self.target_pos2))
            self.msg_wheelmotor.target1 = int(self.vel_input1)
            self.msg_wheelmotor.target2 = int(self.vel_input2)
            # self.msg_wheelmotor.target1 = int(self.target_pos1)
            # self.msg_wheelmotor.target2 = int(self.target_pos2)
        
        else:
            self.md.send_position_cmd(int(self.target_pos1), int(self.target_pos2), int(60), int(60))
            # self.md.send_vel_cmd(self.cmd_vel_r, self.cmd_vel_l)


            # vel_input = 0
            # time_stop = 100
            # time_go1 = 150
            # time_go2 = 200
            # if self.loopcnt < time_stop:
            #     vel_input = 0
            # elif self.loopcnt < time_go1:
            #     vel_input = int(0.5*(self.loopcnt - time_stop))
            # elif self.loopcnt < time_go2:
            #     vel_input = 0.5*(time_go2 - time_go1) - int(0.5*(self.loopcnt - time_go1))
            # else:
            #     self.loopcnt = 0
            # self.md.send_vel_cmd(vel_input, vel_input)
            # self.msg_wheelmotor.target1 = int(self.cmd_vel_r)
            # self.msg_wheelmotor.target2 = int(self.cmd_vel_l)
            
            
            self.msg_wheelmotor.target1 = int(self.target_pos1)
            self.msg_wheelmotor.target2 = int(self.target_pos2)
            # self.msg_wheelmotor.target1 = int(1)  # for plot target line (nmpc_test)
            # self.msg_wheelmotor.target2 = int(self.target_pos2)
        


        self.md.recv_motor_state()



        self.msg_wheelmotor.position1 = self.md.pos1
        self.msg_wheelmotor.position2 = self.md.pos2
        self.msg_wheelmotor.velocity1 = self.md.rpm1
        self.msg_wheelmotor.velocity2 = self.md.rpm2
        self.msg_wheelmotor.current1 = int(self.md.current1)
        self.msg_wheelmotor.current2 = int(self.md.current2)
        # self.msg_wheelmotor.target1 = int(self.vel_input1)
        # self.msg_wheelmotor.target2 = int(self.vel_input2)
              
        self.pub.publish(self.msg_wheelmotor)

        #############################odom 5/30################################        
        
       
        self.cur_pos1 = self.md.pos1 - self.del_pos1
        self.cur_pos2 = self.md.pos2 - self.del_pos2


        # Calculate change in encoder values
        delta_pos1 = self.cur_pos1 - self.last_pos1
        delta_pos2 = self.cur_pos2 - self.last_pos2

        # Update last encoder positions
        self.last_pos1 = self.cur_pos1
        self.last_pos2 = self.cur_pos2

        # Calculate wheel displacements
        left_wheel_disp = (delta_pos1 /self.md.encoder_gain) * (np.pi * self.wheel_diameter)
        right_wheel_disp = (delta_pos2 /self.md.encoder_gain) * (np.pi * self.wheel_diameter)

        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate linear and angular velocities
        linear_velocity =  (left_wheel_disp + right_wheel_disp) / (2.0 * dt)
        angular_velocity = (right_wheel_disp - left_wheel_disp) / (self.wheel_separation * dt)


        

        # Update pose
        self.pose_x += linear_velocity * np.cos(self.pose_theta) *dt
        self.pose_y += linear_velocity * np.sin(self.pose_theta) *dt
        self.pose_theta += angular_velocity * dt
        if (self.pose_theta > np.pi):
            self.pose_theta = self.pose_theta - (2 * np.pi)
        if (self.pose_theta < -np.pi):
            self.pose_theta = self.pose_theta + (2 * np.pi)


        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y


        q = quaternion_from_euler(0, 0, self.pose_theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = linear_velocity      #기존코드
        odom_msg.twist.twist.angular.z = angular_velocity
        
        ###################lowpass filter#######################3
        # v_motor = (self.md.rpm1 + self.md.rpm2)/2.0
        # v_motor_lpf = self.Lowpass_filter(v_motor, self.v_motor_last ,self.alpha)
        # self.v_motor_last = v_motor
        # w_motor = (self.md.rpm1 - self.md.rpm2)/self.wheel_separation
        # w_motor_lpf = self.Lowpass_filter(w_motor, self.w_motor_last ,self.alpha)
        # self.w_motor_last = w_motor


        # odom_msg.twist.twist.linear.x = ( v_motor_lpf * self.wheel_diameter) / 60.0 * (2 * np.pi)   #nmpc test 모터에서 읽은 속도

        # odom_msg.twist.twist.angular.z = w_motor_lpf / 60 * (2 * np.pi)
        

        # self.get_logger().info(f"linear_vel ,w : {( v_motor_lpf * self.wheel_diameter) / 60.0 * (2 * np.pi)}, {odom_msg.twist.twist.angular.z}")
        # self.get_logger().info(f"rpm1 ,rpm2 : {( self.md.rpm1)}, {self.md.rpm2}")
        # self.get_logger().info(f"posx ,posy : {(self.pose_x)}, {self.pose_y}")

        self.odom_pub.publish(odom_msg)

        # Publish transform over 
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.translation.z = 0.
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

        # # Publish lidar transform over TF
        # lidar_transform = TransformStamped()
        # lidar_transform.header.stamp = current_time.to_msg()
        # lidar_transform.header.frame_id = 'chassis'
        # lidar_transform.child_frame_id = 'lidar1_link'
        #  # Set the translation (position) of the lidar
        # lidar_transform.transform.translation.x = 0.24698
        # lidar_transform.transform.translation.y = -0.143555
        # lidar_transform.transform.translation.z = 0.2
        # # Set the rotation of the lidar (180 degrees around x-axis)
        # quat =quaternion_from_euler(3.14159, 0, 0)
        # lidar_transform.transform.rotation.x = quat[0]
        # lidar_transform.transform.rotation.y = quat[1]
        # lidar_transform.transform.rotation.z = quat[2]
        # lidar_transform.transform.rotation.w = quat[3]








        # self.tf_broadcaster.sendTransform(lidar_transform)


        #########6/19 global coordinate-----by using only wheel odometry########
         # Get the map to odom transform
        # try:
        #     trans = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        #     map_x, map_y, map_theta = self.transform_pose_to_map(self.pose_x, self.pose_y, self.pose_theta, trans.transform)
        #     # self.get_logger().info(f"Global Position -> x: {map_x}, y: {map_y}, theta: {map_theta}")
        # except Exception as e:
        #     # self.get_logger().warn(f'Could not transform map to odom: {e}')


    def transform_pose_to_map(self, x, y, theta, transform):
        tx = transform.translation.x
        ty = transform.translation.y

        q = transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Transformation matrix from odom to map
        transformation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), tx],
            [np.sin(yaw),  np.cos(yaw), ty],
            [0,           0,           1]
        ])

        # Pose in odom frame
        pose_odom = np.array([x, y, 1])

        # Transform pose to map frame
        pose_map = np.dot(transformation_matrix, pose_odom)

        # Add the yaw (theta) component
        theta_map = theta + yaw
        if (theta_map > np.pi):
            theta_map = theta_map - (2 * np.pi)
        if (theta_map < -np.pi):
            theta_map = theta_map + (2 * np.pi)


        return pose_map[0], pose_map[1], theta_map



    def cmd_vel_callback(self, msg):
        if not self.JOY_CONTROL:
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z
            control_dt = 0.05
            # Differential drive kinematics
            velocity_right = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity)
            velocity_left = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity)
            # Convert velocities to encoder changes
            encoder_delta_right = (velocity_right * control_dt * self.md.encoder_gain) / (np.pi * self.wheel_diameter)
            encoder_delta_left = (velocity_left * control_dt * self.md.encoder_gain) / (np.pi * self.wheel_diameter)
            self.get_logger().info(f"{encoder_delta_right}")
            # Update target positions based on encoder changes
            self.target_pos1 += encoder_delta_left
            self.target_pos2 += encoder_delta_right
            # self.get_logger().info(f"{self.target_pos1},{self.encoder_delta_right}")
        
            # Send position control command
            # self.md.send_position_cmd(int(self.target_pos1), int(self.target_pos2), int(10), int(10))

        # if not self.JOY_CONTROL:    # nmpc test-> RPM -> vel input
        #     linear_velocity = -msg.linear.x
        #     angular_velocity = msg.angular.z

        #     # Differential drive kinematics
        #     velocity_right = (linear_velocity + (self.wheel_separation / 2.0) * angular_velocity)
        #     velocity_left = (linear_velocity - (self.wheel_separation / 2.0) * angular_velocity)
            
        #     # convert from rad/s to RPM
        #     self.cmd_vel_r = (velocity_right / self.wheel_diameter) * 60 / (2 * np.pi)
            
        #     self.get_logger().info(f'linear: {linear_velocity}')
        #     #self.get_logger().info(f'angular: {angular_velocity}')

        #     #self.get_logger().info(f'vel(rpm): {self.cmd_vel_r}')

        #     self.cmd_vel_l = (velocity_left/ self.wheel_diameter) * 60 / (2 * np.pi)  

    def position_control(self, target1, target2):
        err1 = target1 - self.md.pos1
        self.err1_i = self.forget * (self.err1_i + err1 * self.dt)
        err1_d = (err1 - self.err1_prev) / self.dt
        self.err1_prev = err1

        # self.torque1 = self.p_gain * err1
        # self.torque1 += self.i_gain * self.err1_i
        # self.torque1 += self.d_gain * err1_d

        self.velocity1 = self.p_gain * err1
        self.velocity1 += self.i_gain * self.err1_i
        self.velocity1 += self.d_gain * err1_d

        # if self.torque1 > 1022:
        #     self.torque1 = 1022
        # elif self.torque1 < -1022:
        #     self.torque1 = -1022
        # self.torque1 = np.array(self.torque1, dtype=np.int16)

        if self.velocity1 > 1022:
            self.velocity1 = 1022
        elif self.velocity1 < -1022:
            self.velocity1 = -1022
        self.velocity1 = np.array(self.velocity1, dtype=np.int16)

        # err2 = target2 - self.md.pos2
        # self.err2_i = self.forget * (self.err2_i + err2 * self.dt)
        # err2_d = (err2 - self.err2_prev) / self.dt
        # self.err2_prev = err2

        err2 = target2 - self.md.pos2
        self.err2_i = self.forget * (self.err2_i + err2 * self.dt)
        err2_d = (err2 - self.err2_prev) / self.dt
        self.err2_prev = err2

 
        # self.torque2 = self.p_gain * err2
        # self.torque2 += self.i_gain * self.err2_i
        # self.torque2 += self.d_gain * err2_d

        self.velocity2 = self.p_gain * err2
        self.velocity2 += self.i_gain * self.err2_i
        self.velocity2 += self.d_gain * err2_d


        # if self.torque2 > 1022:
        #     self.torque2 = 1022
        # elif self.torque2 < -1022:
        #     self.torque2 = -1022
        # self.torque2 = np.array(self.torque2, dtype=np.int16)

        if self.velocity2 > 1022:
            self.velocity2 = 1022
        elif self.velocity2 < -1022:
            self.velocity2 = -1022
        self.velocity2 = np.array(self.velocity2, dtype=np.int16)

    ############################# 5/5 UPDATE (STM) ######################################
    # def check_serial(self):
    #     data = self.receive_data()
    #     if data:
    #         self.get_logger().info(f'Received from STM32: {data}')

    # def receive_data(self):
    #     if self.serial_port.in_waiting > 0:  # 데이터가 버퍼에 있는지 확인
    #         data = self.serial_port.read(self.serial_port.in_waiting)  # 모든 버퍼 읽기
    #         return data.decode()  # 바이트를 문자열로 디코드
    #     return None
    
    def send_data(self, data):
        self.get_logger().info(f'send data check: {data}' )
        self.serial_port.write(data.encode())  # 데이터 인코딩 후 전송
   ###############################################################################
   

    def joy_callback(self, msg):
        self.joy_fb = msg.axes[1]
        self.joy_lr = msg.axes[2]

    ########################### 6/10 UPDATE (joy)######################################
        self.joy_r2 = msg.axes[4]
        self.joy_l2 = msg.axes[5]
        self.joy_stop = msg.buttons[0]
        self.joy_lift_up = msg.buttons[3]
        self.joy_lift_down = msg.buttons[1]

        if self.joy_lift_up==1 and self.joy_lift_up_old==0:
            self.send_data("UU")
        elif self.joy_lift_down==1 and self.joy_lift_down_old==0:
            self.send_data("DD")




        EPSILON = 1e-5

        if abs(self.joy_r2 + 1.0) < EPSILON and abs(self.joy_l2 + 1.0) < EPSILON and self.change_mode == 1:
            self.change_mode = 0
            self.target_pos1 = self.md.pos1
            self.target_pos2 = self.md.pos2
            self.vel_input1 = 0.0
            self.vel_input2 = 0.0

            self.JOY_CONTROL = not self.JOY_CONTROL  ## mode change
            self.get_logger().info(f"{'!!!!!!!!!!!Joystick_control!!!!!!!!!!!!' if self.JOY_CONTROL else '!!!!!!!!!!!!AUTO!!!!!!!!!!!!'}")
        
        elif abs(self.joy_r2 - 1.0) < EPSILON and abs(self.joy_l2 - 1.0) < EPSILON:
            self.change_mode = 1
            
                  
    def Lowpass_filter(self, vel_input, vel_input_1 ,alpha):
        return alpha * vel_input + (1-alpha) * vel_input_1
    ##################################################################################

    # def ladar_callback(self, msg):
        # points = []
        # min_angle = 30 * (np.pi / 180)  # 60도
        # max_angle = 150 * (np.pi / 180)  # 120도

        # for i in range(len(msg.ranges)):
        #     angle = msg.angle_min + i * msg.angle_increment
        #     distance = msg.ranges[i]
        #     if min_angle <= angle <= max_angle and msg.range_min < distance < msg.range_max:
        #         x = distance * np.cos(angle)
        #         y = distance * np.sin(angle)
        #         z = 0  # Z 값을 0.2로 설정
        #         points.append([x, y, z, angle])
        #     #         x, y, z , _ = point
        #     #         r, g, b = 255, 255, 0  # 노란색
        #     #         a = 255  # 알파 값
        #     #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #     #         clustered_points.append((x, y, z, rgb))   
        # if not points:
        #     self.get_logger().warn('No valid points found in the scan data')
        #     return

        # points = np.array(points)
        # clustering = DBSCAN(eps=0.03, min_samples=1).fit(points[:,:3])
        # labels = clustering.labels_

        # clustered_points = []
        # cluster_means = []
        # unique_labels = set(labels)
        # aspect_ratio = 0
        # for k in unique_labels:
        #     if k == -1:
        #         continue
        #     class_member_mask = (labels == k)
        #     cluster = points[class_member_mask]


        #     # min_x, min_y = np.min(cluster[:, :2], axis=0)
        #     # max_x, max_y = np.max(cluster[:, :2], axis=0)
        #     # width = max_x - min_x
        #     # height = max_y - min_y
        #     # aspect_ratio = max(width / height, height / width)

        #     # 비율을 기준으로 필터링 (예: 길이 너비 비율이 2 이하인 군집만 선택)
        #     # if aspect_ratio <= 2:
        #     #     mean_x = np.mean(cluster[:, 0])
        #     #     mean_y = np.mean(cluster[:, 1])
        #     #     mean_z = np.mean(cluster[:, 2])
        #     #     angle = np.mean(cluster[:, 3])
        #     #     cluster_means.append([mean_x, mean_y, mean_z,angle])
        #     #     for point in cluster:
        #     #         x, y, z , _ = point
        #     #         r, g, b = 255, 255, 0  # 노란색
        #     #         a = 255  # 알파 값
        #     #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #     #         clustered_points.append((x, y, z, rgb))                

        #     if 1 <= len(cluster) <= 20:  # 클러스터 크기가 15
        #         mean_x = np.mean(cluster[:, 0])
        #         mean_y = np.mean(cluster[:, 1])
        #         mean_z = np.mean(cluster[:, 2])
        #         angle = np.mean(cluster[:, 3])
        #         cluster_means.append([mean_x, mean_y, mean_z,angle,len(cluster)])
        #         for point in cluster:
        #             x, y, z , _ = point
        #             # r, g, b = 255, 255, 0  # 노란색
        #             # a = 255  # 알파 값
        #             # rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #             # clustered_points.append((x, y, z, rgb))

        # header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        # header.frame_id = msg.header.frame_id

        # fields = [
        #     pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        #     pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        # ]




        # for index, mean in enumerate(cluster_means[:3]):
        #     distance = np.linalg.norm(mean)  # 로봇이 원점(0, 0, 0)에 있다고 가정

        # min_two_leg_accurate = 10
        # two_index = [0.0, 0.0, 0.0, 0.0]

        # point_index1 = 0
        # point_index2 = 0

        # point_index_list = []

        # for i in range(len(cluster_means)):
        #     for j in range(i + 1, len(cluster_means)):
        #         mean1 = cluster_means[i]
        #         mean2 = cluster_means[j]
        #         distance1 = np.linalg.norm(mean1[:3])
        #         distance2 = np.linalg.norm(mean2[:3])
        #         angle1 = mean1[3]
        #         angle2 = mean2[3]       

        #         distnace_between_leg = self.cos_2_distance(distance1,distance2,np.fabs(angle1-angle2))
        #         error = (distnace_between_leg-self.lift_leg_distance)**2

        #         if error < min_two_leg_accurate and error < 1e-3:
        #             min_two_leg_accurate = error
        #             two_index = [i, j, angle1,angle2, distance1, distance2]
        #             point_index1 = i
        #             point_index2 = j
        #             #point_index_list.append((i,j))
        # point_index_list.append((point_index1,point_index2))

        # if cluster_means and point_index_list:
        #     for point_indices in point_index_list:
        #         x, y, z, _ , _ = cluster_means[point_indices[0]]
        #         r, g, b = 255, 0, 0  # 파란색
        #         a = 255  # 알파 값
        #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #         clustered_points.append((x, y, z, rgb))      

        #         x, y, z , _ , _= cluster_means[point_indices[1]]
        #         r, g, b = 255, 0, 0 # 빨간색
        #         a = 255  # 알파 값
        #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        #         clustered_points.append((x, y, z, rgb))                 
        #     # 나머지 처리 코드
        #     self.get_logger().info(f'갯수 : ,{cluster_means[point_index_list[0][0]][4]} 오차 : {min_two_leg_accurate}')

        # else:
        #     # cluster_means가 비어 있을 때는 아무것도 하지 않음
        #     self.get_logger().info("cluster_means is empty, no action taken.")

        # pc2_msg = pc2.create_cloud(header, fields, clustered_points)
        # self.point_cloud_pub_.publish(pc2_msg)           

    def cos_2_distance(self, r1, r2, theta):
        return np.sqrt(r1**2+r2**2-2*r1*r2*np.cos(theta))
     

    

    ########################### 5/5 UPDATE (STM)######################################
        # self.joy_lift_up = msg.buttons[3]
        # self.joy_lift_down = msg.buttons[1]
        # if self.joy_lift_up==1:
        #     self.send_data("UU")  #  데이터를 STM32에 전송
        #     self.get_logger().info(f'Received lift-up from joy-stick')
        # if self.joy_lift_down==1:
        #     self.send_data("DD")
        #     self.get_logger().info(f'Received lift-down from joy-stick')
    ###############################################################################
   

def main(args=None):
    rclpy.init(args=args)

    node = Nodelet()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

