#!/usr/bin/env python
# -*- coding: utf-8 -*-
import casadi as ca
import casadi.tools as ca_tools
from draw import Draw_MPC_tracking
import numpy as np
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from sensor_msgs.msg import ChannelFloat32


def desired_command_and_trajectory(t, T, x0_, N_, vel):
    x_ = x0_.reshape(1, -1).tolist()[0]
    m = 0.3
    w = 1
    u_ = []
    vel_ref = 0.7
    for i in range(N_):
        t_predict = t + T * i
        x_ref_ = vel_ref * t_predict 

        y_ref_ =  1  #m * np.sin(w * t_predict)
        theta_ref_ =  0 #m * w * np.cos(w * (t_predict))
        v_ref_ = 0
        omega_ref_ = 0.0
        if (x_ref_ >= 100.0):
            x_ref_ = 100.0
            v_ref_ = 0.0
        x_.append(x_ref_)
        x_.append(y_ref_)
        x_.append(theta_ref_)
        for j in range(x0_.size - 3):
            x_.append(0)
        u_.append(v_ref_)
        u_.append(omega_ref_)
    x_ = np.array(x_).reshape(N_ + 1, -1)
    u_ = np.array(u_).reshape(N_, -1)
    ref_x_ = vel_ref * t
    return x_, u_ , ref_x_



def system_dynamics(x, u, l_tr, Delta_x_cw, Delta_y_cw):
    dx1 = x[3] * np.cos(x[2])
    dx2 = x[3] * np.sin(x[2])
    dx3 = x[4]
    dx4 = u[0]
    dx5 = u[1]
    dx6 = -(1.0 / l_tr) * ((x[3] - x[4] * Delta_y_cw) * np.cos(x[5]) + x[4] * Delta_x_cw * np.sin(x[5]))  # fl front right
    dx7 = -(1.0 / l_tr) * ((x[3] + x[4] * Delta_y_cw) * np.cos(x[6]) + x[4] * Delta_x_cw * np.sin(x[6]))  # fr
    dx8 = -(1.0 / l_tr) * ((x[3] - x[4] * Delta_y_cw) * np.cos(x[7]) - x[4] * Delta_x_cw * np.sin(x[7]))  # rl
    dx9 = -(1.0 / l_tr) * ((x[3] + x[4] * Delta_y_cw) * np.cos(x[8]) - x[4] * Delta_x_cw * np.sin(x[8]))  # rr

    # correction force
    dx10 = 0.0
    dx11 = 0.0
    dx12 = 0.0
    dx13 = 0.0
    return ca.vertcat(dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8, dx9, dx10, dx11, dx12, dx13)



class NMPCNode(Node):


    def __init__(self):
        super().__init__('nmpc_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_publisher_ = self.create_publisher(Float32MultiArray, '/state_data', 10)
        self.publisher_1 = self.create_publisher(PointCloud, 'pointcloud', 10)

        self.odom_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.control_dt_ = 0.05
        self.timer_ = self.create_timer(self.control_dt_, self.timer_callback)
        self.msg_statearray = Float32MultiArray()
        self.init_nmpc()


    def init_nmpc(self):
        self.real = 0  # odom -> 0: virtual, 1: real
        T = self.control_dt_  # sampling time [s]
        N = 40  # prediction horizon
        
        rob_diam = 0.3  # [m]
        self.v_max = 0.4
        self.v_min = -self.v_max
        self.vel_traj = 0.4
        self.omega_max = np.pi / 4.0
        self.omega_min = -np.pi / 4.0
        self.wheel_d = 0.298
        self.l_tr = 0.0225
        self.Delta_x_cw = 0.2
        self.Delta_y_cw = 0.13
        self.x_see = 0
        N_state_x = 13  # Updated number of states
        N_state_u = 2
        a_min = -5
        a_max = 5
        x = ca.SX.sym('x', N_state_x)
        u = ca.SX.sym('u', N_state_u)
        self.f = ca.Function('f', [x, u], [system_dynamics(x, u, self.l_tr, self.Delta_x_cw, self.Delta_y_cw)], ['input_state', 'control_input'], ['rhs'])
        U = ca.SX.sym('U', N_state_u, N)
        X = ca.SX.sym('X', N_state_x, N + 1)
        U_ref = ca.SX.sym('U_ref', N_state_u, N)
        X_ref = ca.SX.sym('X_ref', N_state_x, N + 1)
        
        Q = np.diag([0.3, 2, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Updated Q matrix
        R = np.diag([0.01, 0.1])
        K1 = np.diag([0.0, 0.0])
        K2 = np.diag([20.0, 20.00 ,20.0 ,20.0])
        # Q = np.diag([1.0, 10.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Updated Q matrix
        # R = np.diag([0.5, 0.5])
        
        obj = 0
        g = []
        g.append(X[:, 0] - X_ref[:, 0])
        for i in range(N):
            state_error_ = X[0:3, i] - X_ref[0:3, i + 1]
            control_error_ = U[:, i]
            
            
             # Cost for caster wheels
            # x5_ss = (1.0 / self.l_tr) * ca.sqrt((X[3, i] - X[4, i] * self.Delta_y_cw)**2 + (X[4, i] * self.Delta_x_cw)**2 + 0.001)
            # x6_ss = (1.0 / self.l_tr) * ca.sqrt((X[3, i] + X[4, i] * self.Delta_y_cw)**2 + (X[4, i] * self.Delta_x_cw)**2 + 0.001)
            
            # J_angle = X[5:7, i] - ca.vertcat(x5_ss, x6_ss)
            # J_force = X[9:13, i]

            obj += ca.mtimes([state_error_.T, Q[0:3, 0:3], state_error_]) + ca.mtimes([control_error_.T, R, control_error_]) #+ ca.mtimes([J_angle.T, K1, J_angle]) + ca.mtimes([J_force.T, K2, J_force])
            x_next_ = self.f(X[:, i], U[:, i]) * T + X[:, i]
            g.append(X[:, i + 1] - x_next_)
        opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))
        opt_params = ca.vertcat(ca.reshape(U_ref, -1, 1), ca.reshape(X_ref, -1, 1))
        nlp_prob = {'f': obj, 'x': opt_variables, 'p': opt_params, 'g': ca.vertcat(*g)}
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-6, 'ipopt.acceptable_obj_change_tol': 1e-6}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
        self.T = T
        self.N = N
        self.lbg = 0
        self.ubg = 0
        self.lbx = []
        self.ubx = []
        for _ in range(N):   # control input constraints
            # self.lbx.append(-self.v_max)
            # self.lbx.append(-self.omega_max)
            # self.ubx.append(self.v_max)
            # self.ubx.append(self.omega_max)

            self.lbx.append(-1.)
            self.lbx.append(-1*np.pi)
            self.ubx.append(1.)
            self.ubx.append(1*np.pi)

        for _ in range(N + 1):
            self.lbx.append(-20.0)
            self.lbx.append(-2.0)
            self.lbx.append(-np.pi)
            self.lbx.append(self.v_min)
            self.lbx.append(self.omega_min)
            self.lbx.append(-np.pi)
            self.lbx.append(-np.pi)
            self.lbx.append(-np.pi)
            self.lbx.append(-np.pi)
            self.lbx.append(-np.inf)  # For state 10
            self.lbx.append(-np.inf)  # For state 11
            self.lbx.append(-np.inf)  # For state 12
            self.lbx.append(-np.inf)  # For state 13
            self.ubx.append(20.0)
            self.ubx.append(2.0)
            self.ubx.append(np.pi)
            self.ubx.append(self.v_max)
            self.ubx.append(self.omega_max)
            self.ubx.append(np.pi)
            self.ubx.append(np.pi)
            self.ubx.append(np.pi)
            self.ubx.append(np.pi)
            self.ubx.append(np.inf)  # For state 10
            self.ubx.append(np.inf)  # For state 11
            self.ubx.append(np.inf)  # For state 12
            self.ubx.append(np.inf)  # For state 13
        self.t0 = 0.0
        self.init_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 1.0, 1.0, 1.0, 1.0]).reshape(-1, 1)  # Updated initial state
        self.current_state = self.init_state.copy()
        self.u0 = np.array([0.0, 0.0] * N).reshape(-1, 2).T
        self.next_trajectories = np.tile(self.current_state.reshape(1, -1), N + 1).reshape(N + 1, -1)
        self.next_states = self.next_trajectories.copy()
        self.next_controls = np.zeros((N, 2))
        self.x_c = []
        self.u_c = []
        self.t_c = [self.t0]
        self.xx = []
        self.sim_time = 10.0
        self.mpciter = 0
        self.index_t = []
        self.current_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.start_time = time.time()
        self.init_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 1.0, 1.0, 1.0, 1.0]).reshape(-1, 1)  # Updated initial state
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.psi = 0.0
        self.real_current_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).reshape(-1, 1)  # Updated current state
        self.v_x = 0
        self.angular_z = 0


    def shift_movement(self, T, t0, x0, u, x_f, f, real, real_state):
            condition = 1
            castor_fiction = 0.1
            mass = 10.


            if condition != real:
                f_value = f(x0, u[:, 0])
                st = x0 + T * f_value.full()
            else:
                x0[0:5] = real_state[0:5]  # [real_state[0], real_state[1], real_state[2], real_state[3], real_state[4]]  # 0 ~ 4 is real state , 5 ~ 8 is caster wheel
                x0[5] += T * (-(1.0 / self.l_tr) * ((x0[3] - x0[4] * self.Delta_y_cw) * np.cos(x0[5]) + x0[4] * self.Delta_x_cw * np.sin(x0[5])))  # fl front right
                x0[6] += T * (-(1.0 / self.l_tr) * ((x0[3] + x0[4] * self.Delta_y_cw) * np.cos(x0[6]) + x0[4] * self.Delta_x_cw * np.sin(x0[6])))  # fr
                x0[7] += T * (-(1.0 / self.l_tr) * ((x0[3] - x0[4] * self.Delta_y_cw) * np.cos(x0[7]) - x0[4] * self.Delta_x_cw * np.sin(x0[7])))  # rl
                x0[8] += T * (-(1.0 / self.l_tr) * ((x0[3] + x0[4] * self.Delta_y_cw) * np.cos(x0[8]) - x0[4] * self.Delta_x_cw * np.sin(x0[8])))  # rr
                
                if (x0[5] > np.pi):    #angle remapping for -pi~+pi
                    x0[5] = x0[5] - (2 * np.pi)
                elif (x0[5] < -np.pi):
                    x0[5] = x0[5] + (2 * np.pi)

                if (x0[6] > np.pi):
                    x0[6] = x0[6] - (2 * np.pi)
                elif (x0[6] < -np.pi):
                    x0[6] = x0[6] + (2 * np.pi)

                if (x0[7] > np.pi):
                    x0[7] = x0[7] - (2 * np.pi)
                elif (x0[8] < -np.pi):
                    x0[8] = x0[8] + (2 * np.pi)

                if (x0[9] > np.pi):
                    x0[9] = x0[9] - (2 * np.pi)
                elif (x0[9] < -np.pi):
                    x0[9] = x0[9] + (2 * np.pi)


                

                if x0[5] > 0:    #to make theta   #to compensate coordinate angle difference
                    theta_fl = x0[5] + np.pi/2 + np.pi/2
                else :
                    theta_fl = x0[5] + np.pi/2 + np.pi/2
                
                if x0[6] > 0:
                    theta_fr = x0[6] + np.pi/2 + np.pi/2
                else :
                    theta_fr = x0[6] + np.pi/2 + np.pi/2
                
                if x0[7] > 0:
                    theta_rl = x0[7] + np.pi/2 + np.pi/2
                else :
                    theta_rl = x0[7] + np.pi/2 + np.pi/2
                
                if x0[8] > 0:
                    theta_rr = x0[8] + np.pi/2 + np.pi/2
                else :
                    theta_rr = x0[8] + np.pi/2 + np.pi/2

                
            
                tau_fl_l = np.arctan2(((self.wheel_d/2)-self.Delta_y_cw),self.Delta_x_cw)
                tau_fl_r = np.arctan2(((self.wheel_d/2)+self.Delta_y_cw),self.Delta_x_cw)
                tau_fr_l = np.arctan2(((self.wheel_d/2)+self.Delta_y_cw),self.Delta_x_cw)
                tau_fr_r = np.arctan2(((self.wheel_d/2)-self.Delta_y_cw),self.Delta_x_cw)
                
                #castor corretion force for front left castor wheel -> left motor,   front left castor -> right wheel 
                x0[9] = castor_fiction * mass/4 * np.cos(theta_fl+tau_fl_l) * np.cos(tau_fl_l)
                x0[10] = castor_fiction * mass/4 * np.cos(theta_fl-tau_fl_r) * np.cos(tau_fl_r)
                
                #castor force for right castor
                x0[11] = castor_fiction * mass/4 * np.cos(theta_fr+tau_fr_l) * np.cos(tau_fr_l)
                x0[12] = castor_fiction * mass/4 * np.cos(theta_fr-tau_fr_r) * np.cos(tau_fr_r)

                st = x0

            t = t0 + T
            u_end = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)  # for maintain number of axis --> u[:, -1:]
            x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)
            # print(st.T)
            self.msg_statearray.data = st.flatten().tolist()
            self.state_publisher_.publish(self.msg_statearray)
            #self.get_logger().info('Publishing: %s' % self.msg_statearray.data)
            return t, st, u_end, x_f
    


    def timer_callback(self):
        pointcloud_msg = PointCloud()
        pointcloud_msg.header = Header()
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg() 
        pointcloud_msg.header.frame_id = "odom"
        point = Point32()
        point.y = 1.0
        point.z = 0.0

        if self.mpciter * self.T < self.sim_time:
            current_time = self.mpciter * self.T
            # control, trajectory ==> align one line
            c_p = np.concatenate((self.next_controls.reshape(-1, 1), self.next_trajectories.reshape(-1, 1)))
            init_control = np.concatenate((self.u0.T.reshape(-1, 1), self.next_states.T.reshape(-1, 1)))
            # solver cal time / solve
            t_ = time.time()
            res = self.solver(x0=init_control, p=c_p, lbg=self.lbg, lbx=self.lbx, ubg=self.ubg, ubx=self.ubx)
            self.index_t.append(time.time() - t_)
            #self.get_logger().info(f"t, {time.time() - t_}")
            # opt _ output
            estimated_opt = res['x'].full()
            self.u0 = estimated_opt[:int(2 * self.N)].reshape(self.N, 2).T
            x_m = estimated_opt[int(2 * self.N):].reshape(self.N + 1, 13).T  # Updated reshape dimensions
            # to show variable
            self.x_c.append(x_m.T)
            self.u_c.append(self.u0[:, 0])
            self.t_c.append(self.t0)
            # current state --> next state
            self.t0, self.current_state, self.u0, self.next_states = self.shift_movement(self.T, self.t0, self.current_state, self.u0, x_m, self.f, self.real, self.real_current_state)
            self.current_state = ca.reshape(self.current_state, -1, 1)
            self.current_state = self.current_state.full()
            # to show robot state
            self.xx.append(self.current_state)
            # next_trajectories : current state(3) + next trajectory (N future)
            self.next_trajectories, self.next_controls , self. x_see = desired_command_and_trajectory(self.t0, self.T, self.current_state, self.N, self.vel_traj)
            self.mpciter += 1
            # Update current velocities by integrating acceleration and angular acceleration
            self.current_velocity += self.u0[0, 0] * self.T
            self.current_angular_velocity += self.u0[1, 0] * self.T
            # Create and publish Twist message
            cmd_vel = Twist()

            if self.current_velocity > self.v_max:
                self.current_velocity = self.v_max
            if self.current_velocity < self.v_min:
                self.current_velocity = self.v_min
            if self.current_angular_velocity > self.omega_max:
                self.current_angular_velocity = self.omega_max
            if self.current_angular_velocity < self.omega_min:
                self.current_angular_velocity = self.omega_min
            cmd_vel.linear.x = self.current_velocity
            cmd_vel.angular.z = self.current_angular_velocity
            self.publisher_.publish(cmd_vel)

          #  self.get_logger().info(f"{cmd_vel.linear.x}")
            
            # point 
            point.x = self.x_see ## v_ref 플롯을 위해
            pointcloud_msg.points.append(point)
            channel = ChannelFloat32()
            channel.name = "intensity"
            channel.values = [0.8]
            pointcloud_msg.channels.append(channel)
            self.publisher_1.publish(pointcloud_msg)

        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.publisher_.publish(cmd_vel)
            t_v = np.array(self.index_t)
            print("solver_calculate_time : ", t_v.mean())
            print("mean_period_time : ", (time.time() - self.start_time) / (self.mpciter))
            print("loop_count : ", self.mpciter)
            draw_result = Draw_MPC_tracking(rob_diam=0.3, init_state=self.init_state, robot_states=self.xx, robot_predict=self.x_c)
            
            rclpy.shutdown()



    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        roll, pitch, self.psi = euler
        #print("x", self.x_pos, "y", self.y_pos, "psi :", self.psi)
        
        self.v_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z
        self.get_logger().info(f"vx, {self.v_x}, w, {self.angular_z}")
        
        self.real_current_state = np.array([self.x_pos, self.y_pos, self.psi, self.v_x, self.angular_z, 1.0, 1.0, 1.0, 1.0]).reshape(-1, 1)  # Updated current state



def main(args=None):
    rclpy.init(args=args)
    nmpc_node = NMPCNode()
    rclpy.spin(nmpc_node)
    nmpc_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()