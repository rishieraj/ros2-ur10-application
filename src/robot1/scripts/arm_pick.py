#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import tf_transformations
import sympy as sp
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


class ArmControl(Node):

    def __init__(self):
        super().__init__('data_subpub')
        self.sub = self.create_subscription(PoseStamped, 'odom', self.pose_callback, 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        timer_period = 0.01  # seconds
        self.th1 = 0.0
        self.th2 = 0.0
        self.th3 = 0.0
        self.th4 = 0.0
        self.th5 = 0.0
        self.th6 = 0.0
        self.t = 0.0
        self.reached = False
        self.q = np.matrix([[0.0], [-0.01], [-0.01], [1.5708], [-1.5708], [0.01]])
        self.x_lst = []
        self.y_lst = []
        self.z_lst = []
        self.sub  # prevent unused variable warning
        self.timer = self.create_timer(timer_period, self.control_publisher)

    def pose_callback(self, msg):     
        if msg.header.frame_id == "link6":
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.z = msg.pose.position.z
            quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            euler_angles = tf_transformations.euler_from_quaternion(quaternion)
            self.eff_roll, self.eff_pitch, self.eff_yaw = euler_angles
            # self.get_logger().info('End-Effector odometry has been calculated!')

    def control_publisher(self):
        if self.reached:
            exit()

        omega = (2*np.pi/20)

        x_t = 0.9
        y_t = 0.0
        z_t = 0.1

        joint_positions = Float64MultiArray()

        # The lengths of the distances between frames
        l1 = 0.128
        l2 = 0.6127
        l3 = 0.5716
        l4 = 0.1639
        l5 = 0.1157
        l6 = 0.1922

        dt = 0.01

        theta1, theta2, theta3, theta4, theta5, theta6 = self.th1, self.th2, self.th3, self.th4, self.th5, self.th6

        T01 = sp.Matrix([[sp.cos(theta1), 0, -sp.sin(theta1), 0], [sp.sin(theta1), 0, sp.cos(theta1), 0], [0, -1, 0, l1], [0, 0, 0, 1]])
        T12 = sp.Matrix([[sp.cos((sp.pi)/2 + theta2), sp.sin((sp.pi)/2 + theta2), 0, -l2*sp.cos((sp.pi)/2 + theta2)], [sp.sin((sp.pi)/2 + theta2), -sp.cos((sp.pi)/2 + theta2), 0, -l2*sp.sin((sp.pi)/2 + theta2)], [0, 0, -1, 0], [0, 0, 0, 1]])
        T23 = sp.Matrix([[sp.cos(theta3), sp.sin(theta3), 0, -l3*sp.cos(theta3)], [sp.sin(theta3), -sp.cos(theta3), 0, -l3*sp.sin(theta3)], [0, 0, -1, 0], [0, 0, 0, 1]])
        T34 = sp.Matrix([[sp.cos(-(sp.pi)/2 + theta4), 0, sp.sin(-(sp.pi)/2 + theta4), 0], [sp.sin(-(sp.pi)/2 + theta4), 0, -sp.cos(-(sp.pi)/2 + theta4), 0], [0, 1, 0, l4], [0, 0, 0, 1]])
        T45 = sp.Matrix([[sp.cos(theta5), 0, -sp.sin(theta5), 0], [sp.sin(theta5), 0, sp.cos(theta5), 0], [0, -1, 0, l5], [0, 0, 0, 1]])
        T56 = sp.Matrix([[sp.cos(theta6), -sp.sin(theta6), 0, 0], [sp.sin(theta6), sp.cos(theta6), 0, 0], [0, 0, 1, l6], [0, 0, 0, 1]])

        # Transformation matrix for ith joint
        T02 = T01*T12
        T03 = T02*T23
        T04 = T03*T34
        T05 = T04*T45
        T06 = T05*T56

        # extracting z_i matrices
        z0 = sp.Matrix([[0], [0], [1]])

        z1 = T01.col(2)
        z1.row_del(3)

        z2 = T02.col(2)
        z2.row_del(3)

        z3 = T03.col(2)
        z3.row_del(3)

        z4 = T04.col(2)
        z4.row_del(3)

        z5 = T05.col(2)
        z5.row_del(3)

        # extracting o_i matrices
        o0 = sp.Matrix([[0], [0], [0]])

        o1 = T01.col(3)
        o1.row_del(3)

        o2 = T02.col(3)
        o2.row_del(3)

        o3 = T03.col(3)
        o3.row_del(3)

        o4 = T04.col(3)
        o4.row_del(3)

        o5 = T05.col(3)
        o5.row_del(3)

        o6 = T06.col(3)
        o6.row_del(3)

        # building the column vectors of Jacobian matrix
        J1 = sp.Matrix([z0.cross(o6 - o0), z0])
        J2 = sp.Matrix([z1.cross(o6 - o1), z1])
        J3 = sp.Matrix([z2.cross(o6 - o2), z2])
        J4 = sp.Matrix([z3.cross(o6 - o3), z3])
        J5 = sp.Matrix([z4.cross(o6 - o4), z4])
        J6 = sp.Matrix([z5.cross(o6 - o5), z5])

        # creating Jacobian
        J_sym = sp.Matrix.hstack(J1, J2, J3, J4, J5, J6)

        # initializing Jacobian to home orientation
        J = np.matrix(J_sym).astype(np.float64)
        J_inv = np.linalg.pinv(J)

        self.x_lst.append(self.x)
        self.y_lst.append(self.y)
        self.z_lst.append(self.z)

        # trajectory kinematic equations
        # x_dot = 0.2*omega*np.cos(omega*self.t)
        # y_dot = 0
        # z_dot = -0.2*omega*np.sin(omega*self.t)

        x_dot = (x_t - self.x_lst[0]) / 20
        y_dot = (y_t - self.y_lst[0]) / 20
        z_dot = (z_t - self.z_lst[0]) / 20
        x_rot_dot = 0
        y_rot_dot = 0
        z_rot_dot = 0

        # Xi matrix
        ksi = np.matrix([[x_dot], [y_dot], [z_dot], [x_rot_dot], [y_rot_dot], [z_rot_dot]])

        # joint angle matrix
        q_dot = J_inv*ksi
        self.q = self.q + q_dot*dt

        # joint angle extraction
        [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6] = [self.q[i].item() for i in range(6)]
        print('Angle: ', self.th1, '\t', self.th2, '\t', self.th3, '\t', self.th4, '\t', self.th5, '\t', self.th6)

        joint_positions.data = [-self.th1, -self.th2, -self.th3, self.th4, self.th5, self.th6]
        self.joint_position_pub.publish(joint_positions)

        self.t = self.t + dt

        if (x_t - 0.05 < self.x < x_t + 0.05) and (y_t - 0.05 < self.y < y_t + 0.05) and (z_t - 0.05 < self.z < z_t + 0.05):
            self.reached = True
            self.get_logger().info('Reached Waypoint-1\n')
            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='3d')  # Create a 3D subplot

            # Plotting in 3D
            ax.plot(self.x_lst, self.y_lst, self.z_lst, color='red')

            ax.set_title('Trajectory of End-Effector')
            ax.set_xlabel('X-Trajectory')
            ax.set_ylabel('Y-Trajectory')
            ax.set_zlabel('Z-Trajectory')

            # Setting the axes limits
            ax.set_ylim([-1, 1]) 

            plt.grid()
            plt.show()
        

def main(args=None):
    rclpy.init(args=args)
    data_subpub = ArmControl()
    rclpy.spin(data_subpub)
    data_subpub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

