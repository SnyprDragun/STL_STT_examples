#!/usr/bin/env python3
import torch
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class STT_Controller():
    def __init__(self, current_state_topic, vel_pub_topic, C, degree, start, end):
        self.current_state_topic = current_state_topic
        self.vel_pub_topic = vel_pub_topic

        self.C = C
        self.degree = degree
        self.dimension = int(len(C) / (2 * (degree + 1)))
        self.start = start
        self.end = end

        rospy.init_node('STT_Controller')
        
        if self.dimension == 2:
            rospy.Subscriber(self.current_state_topic, Odometry, self.omnibot_state_callback)
            self.vel_pub = rospy.Publisher(self.vel_pub_topic, Twist, queue_size=10)
            self.current_state = Odometry()
        elif self.degree == 3:
            rospy.Subscriber(self.current_state_topic, Odometry, self.uav_state_callback)
            self.vel_pub = rospy.Publisher(self.vel_pub_topic, Twist, queue_size=10)
            self.current_state = Odometry()
        else:
            raise ValueError("degree not according to C")
        

    def omnibot_state_callback(self, msg):
        """Callback function for the state subscriber."""
        self.current_state = msg

    def uav_state_callback(self, msg):
        """Callback function for the state subscriber."""
        self.current_state = msg

    def gamma(self, t):
        '''method to calculate tube boundaries at time instance 't' using torch for optimization'''
        t_tensor = torch.tensor(t, dtype=torch.float32)
        powers_of_t = torch.tensor([t_tensor**j for j in range(self.degree + 1)], dtype=torch.float32)
        C_tensor = torch.tensor(self.C, dtype=torch.float32).view(2 * self.dimension, self.degree + 1)
        real_tubes = torch.matmul(C_tensor, powers_of_t)
        return real_tubes.numpy()
    
    def normalized_error(self, x, gamma_sum, gamma_diff):
        return (2 * x - gamma_sum) / gamma_diff

    def omnibot_control(self):
        """Calculates the error as the difference between the current state and the target."""
        rate = rospy.Rate(1)

        k = 5
        t_values = np.arange(self.start, self.end + 1, 5)

        while not rospy.is_shutdown():
            for t in t_values:
                gamma = self.gamma(t)
                if self.dimension == 2:
                    gamma_xl, gamma_yl, gamma_xu, gamma_yu = gamma[0], gamma[1], gamma[2], gamma[3]
                    
                    gamma_sx = gamma_xu + gamma_xl
                    gamma_dx = gamma_xu - gamma_xl
                    gamma_sy = gamma_yu + gamma_yl
                    gamma_dy = gamma_yu - gamma_yl

                    e1 = self.normalized_error(self.current_state.pose.pose.position.x, gamma_sx, gamma_dx)
                    e2 = self.normalized_error(self.current_state.pose.pose.position.y, gamma_sy, gamma_dy)

                    e_matrix = torch.tensor([e1, e2])
                    print("e_matrix: ", e_matrix)

                    epsilon1 = math.log((1 + e1) / (1 - e1))
                    epsilon2 = math.log((1 + e2) / (1 - e2))

                    epsilon_matrix = torch.tensor([epsilon1, epsilon2])
                    gamma_d_matrix = torch.diag(torch.tensor([gamma_dx, gamma_dy]))
                    xi_matrix = 4 * torch.matmul(gamma_d_matrix.inverse(), (torch.eye(self.dimension) - torch.matmul(e_matrix.T, e_matrix)).inverse().to(torch.float64))
                    u_matrix = - k * torch.matmul(xi_matrix, epsilon_matrix.to(torch.float64))

                    v_x = u_matrix[0].item()
                    v_y = u_matrix[1].item()

                    vel_msg = Twist()
                    if -0.22 < v_x < 0.22 and -0.22 < v_y < 0.22:
                        vel_msg.linear.x = v_x
                        vel_msg.linear.y = v_y
                        self.vel_pub.publish(vel_msg)
                    else:
                        vel_msg.linear.x = 0.22
                        vel_msg.linear.y = 0.22
                        self.vel_pub.publish(vel_msg)

                    rate.sleep()

    def uav_control(self):
        """Calculates the error as the difference between the current state and the target."""
        rate = rospy.Rate(1)

        k = 5
        t_values = np.arange(self.start, self.end + 1, 5)

        while not rospy.is_shutdown():
            for t in t_values:
                gamma = self.gamma(t)
                if self.dimension == 3:
                    gamma_xl, gamma_yl, gamma_zl, gamma_xu, gamma_yu, gamma_zu = gamma[0], gamma[1], gamma[2], gamma[3], gamma[4], gamma[5]

                    gamma_sx = gamma_xu + gamma_xl
                    gamma_dx = gamma_xu - gamma_xl
                    gamma_sy = gamma_yu + gamma_yl
                    gamma_dy = gamma_yu - gamma_yl
                    gamma_sz = gamma_zu + gamma_zl
                    gamma_dz = gamma_zu - gamma_zl

                    e1 = self.normalized_error(self.current_state.pose.pose.position.x, gamma_sx, gamma_dx)
                    e2 = self.normalized_error(self.current_state.pose.pose.position.y, gamma_sy, gamma_dy)
                    e3 = self.normalized_error(self.current_state.pose.pose.position.z, gamma_sz, gamma_dz)

                    e_matrix = torch.tensor([e1, e2, e3])
                    print("e_matrix: ", e_matrix)

                    epsilon1 = math.log((1 + e1) / (1 - e1))
                    epsilon2 = math.log((1 + e2) / (1 - e2))
                    epsilon3 = math.log((1 + e3) / (1 - e3))

                    epsilon_matrix = torch.tensor([epsilon1, epsilon2, epsilon3])
                    gamma_d_matrix = torch.diag(torch.tensor([gamma_dx, gamma_dy, gamma_dz]))
                    xi_matrix = 4 * torch.matmul(gamma_d_matrix.inverse(), (torch.eye(self.dimension) - torch.matmul(e_matrix.T, e_matrix)).inverse().to(torch.float64))
                    u_matrix = - k * torch.matmul(xi_matrix, epsilon_matrix.to(torch.float64))

                    v_x = u_matrix[0].item()
                    v_y = u_matrix[1].item()
                    v_z = u_matrix[2].item()

                    vel_msg = Twist()
                    if -0.22 < v_x < 0.22 and -0.22 < v_y < 0.22:
                        vel_msg.linear.x = v_x
                        vel_msg.linear.y = v_y
                        self.vel_pub.publish(vel_msg)
                    else:
                        vel_msg.linear.x = 0.22
                        vel_msg.linear.y = 0.22
                        self.vel_pub.publish(vel_msg)

                    rate.sleep()

                
if __name__ == '__main__':
    C0 = 0.5#20797808558189697
    C1 = 0.5#16574648035233522
    C2 = -0.5#328330183376585
    C3 = -0.5#26801152659762774
    # C4 = 0.020797808558189697
    # C5 = -0.016838027770626724
    # C6 = 0.33813823872200727
    # C7 = -0.027553666189457638
    # C8 = 0.5415956171163794
    # C9 = -0.016574648035233522
    # C10 = 0.3328330183376585
    # C11 = -0.026801152659762774
    # C12 = 0.9792021914418103
    # C13 = -0.41057611837612756
    # C14 = 0.46889554184430615
    # C15 = -0.038642267452590924
    C = [C0, C1, C2, C3]#, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15]
    try:
        STT_Controller('/odom', '/cmd_vel', C, 0, 0, 30).uav_control()
    except rospy.ROSInterruptException:
        print("some error")
