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

        # rospy.init_node('STT_Controller')
        # rospy.Subscriber(self.current_state_topic, Float32, self.state_callback)
        # self.vel_pub = rospy.Publisher(self.vel_pub_topic, Twist, queue_size=10)

        self.C = C
        self.degree = degree
        self.dimension = 2#len(C) / (2 * (degree + 1))
        self.start = start
        self.end = end

    # def state_callback(self, msg):
    #     """Callback function for the state subscriber."""
    #     global current_state
    #     current_state = msg.data

    def gamma(self, t):
        '''method to calculate tube boundaries at time instance 't' '''
        real_tubes = np.zeros(2 * self.dimension)

        for i in range(2 * self.dimension):
            power = 0
            for j in range(self.degree + 1):
                real_tubes[i] += ((self.C[j + i * (self.degree + 1)]) * (t ** power))
                power += 1
        return real_tubes
    
    def normalized_error(self, x, gamma_sum, gamma_diff):
        return (2 * x - gamma_sum) / gamma_diff

    def control(self):
        """Calculates the error as the difference between the current state and the target."""
        # rate = rospy.Rate(10)

        k = 5
        t_values = np.arange(self.start, self.end + 1, 1)

        # while not rospy.is_shutdown():
        for t in t_values:
            print("time: ", t)
            current_state_x = t
            current_state_y = t
            if self.dimension == 2:
                gamma = self.gamma(t)
                gamma_xl, gamma_yl, gamma_xu, gamma_yu = gamma[0], gamma[1], gamma[2], gamma[3]
                
                gamma_sx = gamma_xu + gamma_xl
                gamma_dx = gamma_xu - gamma_xl
                gamma_sy = gamma_yu + gamma_yl
                gamma_dy = gamma_yu - gamma_yl

                e1 = self.normalized_error(current_state_x, gamma_sx, gamma_dx)
                e2 = self.normalized_error(current_state_y, gamma_sy, gamma_dy)

                e_matrix = torch.tensor([e1, e2])
                print(e_matrix)

                epsilon1 = math.log((1 + e1) / (1 - e1))
                epsilon2 = math.log((1 + e2) / (1 - e2))

                epsilon_matrix = torch.tensor([epsilon1, epsilon2])
                gamma_d_matrix = torch.diag(torch.tensor([gamma_dx, gamma_dy]))
                xi_matrix = 4 * torch.matmul(gamma_d_matrix.inverse(), (torch.eye(self.dimension) - torch.matmul(e_matrix.T, e_matrix)).inverse().to(torch.float64))
                u_matrix = - k * torch.matmul(xi_matrix, epsilon_matrix.to(torch.float64))

                print(u_matrix)

                # v_x = u_matrix[0]
                # v_y = u_matrix[1]

                # print(v_x, v_y)

                    # vel_msg = Twist()
                    # vel_msg.linear.x = v_x
                    # vel_msg.linear.y = v_y
                    # self.vel_pub.publish(vel_msg)

                    # rate.sleep()

if __name__ == '__main__':
    C0 = -0.49042954184063553
    C1 = 1.0372652849612387
    C2 = 0.021418455987188483
    C3 = -0.012748711136309862
    C4 = 0.002685073375052976
    C5 = -0.0001661976669242473
    C6 = -0.49179351223981954
    C7 = -0.03529816723066544
    C8 = 1.5792618027211525
    C9 = -0.5132629040204001
    C10 = 0.059713893661179795
    C11 = -0.0023153546963445818
    C12 = 0.49397700904924696
    C13 = 1.096665134875751
    C14 = -0.044209705248240645
    C15 = -0.009055315328576095
    C16 = 0.003770471282873895
    C17 = -0.00025198927628639924
    C18 = 0.49534097944843103
    C19 = 0.018128722739422096
    C20 = 1.512815679824099
    C21 = -0.5080551039997402
    C22 = 0.06048724206716131
    C23 = -0.0023835712567075607
    C = [C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23]
    try:
        STT_Controller('/state', '/cmd_vel', C, 5, 0, 2).control()
    except rospy.ROSInterruptException:
        print("some error")



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
