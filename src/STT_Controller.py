#!/usr/bin/env python3
import torch
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class STT_Controller():
    def __init__(self, current_state_topic, vel_pub_topic, C, degree, start, end):
        self.current_state_topic = current_state_topic
        self.vel_pub_topic = vel_pub_topic

        rospy.init_node('STT_Controller')
        rospy.Subscriber(self.current_state_topic, Float32, self.state_callback)
        self.vel_pub = rospy.Publisher(self.vel_pub_topic, Twist, queue_size=10)

        self.C = C
        self.degree = degree
        self.dimension = (len(C)) / self.degree
        self.start = start
        self.end = end

    def state_callback(self, msg):
        """Callback function for the state subscriber."""
        global current_state
        current_state = msg.data

    def gamma(self, t):
        '''method to calculate tube boundaries at time instance 't' '''
        real_tubes = np.zeros(2 * self.dimension)

        for i in range(2 * self.dimension):
            power = 0
            for j in range(self.degree + 1):
                real_tubes[i] += ((self.C[j + i * (self.degree + 1)]) * (t ** power))
                power += 1
        return real_tubes
    
    def control(self):
        """Calculates the error as the difference between the current state and the target."""
        rate = rospy.Rate(10)
        k=5
        while not rospy.is_shutdown():
            for t in np.arange(self.start, self.end + 1, 1):
                if self.dimension == 2:
                    gamma_xu = self.gamma(t)[0]
                    gamma_yu = self.gamma(t)[1]
                    gamma_xl = self.gamma(t)[2]
                    gamma_yl = self.gamma(t)[3]

                    gamma_s1 = gamma_xu + gamma_xl
                    gamma_d1 = gamma_xu - gamma_xl
                    e_x = (2 * current_state.x - gamma_s1) / gamma_d1 
                    epsilon_x = math.log((1+e_x)/(1-e_x))

                    gamma_s2 = gamma_yu + gamma_yl
                    gamma_d2 = gamma_yu - gamma_yl
                    e_y = (2 * current_state.y - gamma_s2) / gamma_d2 
                    epsilon_y = math.log((1+e_y)/(1-e_y))

                    e_matrix = [e_x, e_y]
                    epsilon_matrix = [epsilon_x, epsilon_y]

                    xi_matrix = [(4/(gamma_d1 * (1-e_matrix*e_matrix)))]

                    u_matrix = - k * xi_matrix * epsilon_matrix

                    v_x = u_matrix[0]
                    v_y = u_matrix[1]

                    vel_msg = Twist()
                    vel_msg.linear.x = v_x
                    vel_msg.linear.y = v_y
                    self.vel_pub.publish(vel_msg)

                    rate.sleep()

if __name__ == '__main__':
    C = []
    try:
        STT_Controller('/state', '/cmd_vel', C, 3, 0, 15).control()
    except rospy.ROSInterruptException:
        pass



# import torch

# def gamma(self, t):
#     '''method to calculate tube boundaries at time instance 't' using torch for optimization'''
    
#     # Convert t into a tensor
#     t_tensor = torch.tensor(t, dtype=torch.float32)
    
#     # Prepare tensor for powers of t (i.e., [t^0, t^1, ..., t^degree])
#     powers_of_t = torch.tensor([t_tensor**j for j in range(self.degree + 1)], dtype=torch.float32)
    
#     # Reshape the coefficients C and powers for matrix multiplication
#     C_tensor = torch.tensor(self.C, dtype=torch.float32).view(2 * self.dimension, self.degree + 1)
    
#     # Perform the matrix multiplication between C_tensor and powers_of_t
#     real_tubes = torch.matmul(C_tensor, powers_of_t)
    
#     return real_tubes.numpy()  # Convert back to numpy array if needed

# import torch

# # Assuming self.start, self.end, and self.gamma are already defined
# # Replace self.gamma with an appropriate torch-compatible version
# # For this example, we assume gamma outputs torch tensors

# # Create a range of t values as a tensor
# t_values = torch.arange(self.start, self.end + 1, 1)

# # Initialize empty lists to store results
# e_matrix = []
# epsilon_matrix = []
# u_matrix = []

# for t in t_values:
#     if self.dimension == 2:
#         gamma = self.gamma(t)  # Assume this returns a torch tensor of shape (4,)
#         gamma_xu, gamma_yu, gamma_xl, gamma_yl = gamma[0], gamma[1], gamma[2], gamma[3]

#         gamma_s1 = gamma_xu + gamma_xl
#         gamma_d1 = gamma_xu - gamma_xl
#         e_x = (2 * current_state.x - gamma_s1) / gamma_d1
#         epsilon_x = torch.log((1 + e_x) / (1 - e_x))

#         gamma_s2 = gamma_yu + gamma_yl
#         gamma_d2 = gamma_yu - gamma_yl
#         e_y = (2 * current_state.y - gamma_s2) / gamma_d2
#         epsilon_y = torch.log((1 + e_y) / (1 - e_y))

#         e_matrix.append(torch.tensor([e_x, e_y]))
#         epsilon_matrix.append(torch.tensor([epsilon_x, epsilon_y]))

#         xi_matrix = 4 / (gamma_d1 * (1 - e_matrix[-1] * e_matrix[-1]))  # Use the last e_matrix computed
#         u_matrix.append(-k * xi_matrix * epsilon_matrix[-1])

# # Convert lists to tensors if needed
# e_matrix = torch.stack(e_matrix)
# epsilon_matrix = torch.stack(epsilon_matrix)
# u_matrix = torch.stack(u_matrix)

# # Extract velocity components if needed
# v_x = u_matrix[:, 0]
# v_y = u_matrix[:, 1]
