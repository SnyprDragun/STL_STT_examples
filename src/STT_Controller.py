#!/usr/bin/env python3
import sys
import torch
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class STT_Controller():
    def __init__(self, C, dimension, start, end):

        self.C = C
        self.dimension = dimension
        self.degree = int((len(C) / (2 * self.dimension)) - 1)
        self.start = start
        self.end = end

        rospy.init_node('STT_Controller')
        
        if self.dimension == 2:
            self.state_sub = rospy.Subscriber('/odom', Odometry, self.omnibot_state_callback)
            self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            self.current_state = Odometry()
            self.vel_msg = Twist()
        elif self.dimension == 3:
            self.state_sub = rospy.Subscriber('/mavros/state', State, self.uav_state_callback)
            self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,self.uav_pose_callback)
            self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
            self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

            self.current_state = State()
            self.current_pose = PoseStamped()
            self.vel_msg = TwistStamped()
            self.offboard_mode_set = False

            self.gamma_u = []
            self.gamma_l = []
            self.trajectory = []
            self.control_input = []
        else:
            raise ValueError("degree not according to C")

    def omnibot_state_callback(self, msg):
        """Callback function for the state subscriber."""
        self.current_state = msg

    def uav_state_callback(self, msg):
        """Callback function for the state subscriber."""
        self.current_state = msg

    def uav_pose_callback(self, msg):
        """Callback function for the pose subscriber."""
        self.current_pose = msg

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
        """Calculates the error between the current state and the target."""
        rate = rospy.Rate(1)

        k = 5
        t_values = np.arange(self.start, self.end + 1, 5)

        while not rospy.is_shutdown():
            for t in t_values:
                gamma = self.gamma(t)
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

                if -0.22 <= v_x <= 0.22 and -0.22 <= v_y <= 0.22:
                    self.vel_msg.linear.x = v_x
                    self.vel_msg.linear.y = v_y
                    self.vel_pub.publish(self.vel_msg)
                elif v_x > 0.22 and v_y > 0.22:
                    self.vel_msg.linear.x = 0.22
                    self.vel_msg.linear.y = 0.22
                    self.vel_pub.publish(self.vel_msg)
                else:
                    self.vel_msg.linear.x = -0.22
                    self.vel_msg.linear.y = -0.22
                    self.vel_pub.publish(self.vel_msg)

                rate.sleep()

    def uav_control(self):
        """Calculates the error between the current state and the target."""
        rate = rospy.Rate(10)

        k = 1
        max_vel = 5
        t_values = np.arange(self.start, self.end + 0.5, 0.1)

        while not rospy.is_shutdown() and not self.current_state.armed:
            rospy.loginfo("Waiting for drone to be armed...")
            time.sleep(1)

        for _ in range(1):
            self.vel_msg.twist.linear.x = 0
            self.vel_msg.twist.linear.y = 0
            self.vel_msg.twist.linear.z = 0
            self.vel_pub.publish(self.vel_msg)
            rate.sleep()

        if not self.offboard_mode_set and self.current_state.mode != "OFFBOARD":
            rospy.wait_for_service('/mavros/set_mode')
            try:
                response = self.set_mode_srv(0, 'OFFBOARD')
                if response.mode_sent:
                    rospy.loginfo("OFFBOARD mode set successfully.")
                else:
                    rospy.logwarn("Failed to set OFFBOARD mode.")
                self.offboard_mode_set = True
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

        rospy.loginfo("OFFBOARD mode set. UAV ready for velocity control.")

        max_iterations = 1
        count = 0
        while not rospy.is_shutdown() and count < max_iterations:
            count += 1

            for t in t_values:
                rospy.sleep(0.1)
                gamma = self.gamma(t)
                gamma_xl, gamma_yl, gamma_zl, gamma_xu, gamma_yu, gamma_zu = gamma[0], gamma[1], gamma[2], gamma[3], gamma[4], gamma[5]
                self.gamma_u.append([gamma_xu, gamma_yu, gamma_zu])
                self.gamma_l.append([gamma_xl, gamma_yl, gamma_zl])

                gamma_sx = gamma_xu + gamma_xl
                gamma_dx = gamma_xu - gamma_xl
                gamma_sy = gamma_yu + gamma_yl
                gamma_dy = gamma_yu - gamma_yl
                gamma_sz = gamma_zu + gamma_zl
                gamma_dz = gamma_zu - gamma_zl

                self.trajectory.append([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])

                e1 = self.normalized_error(self.current_pose.pose.position.x, gamma_sx, gamma_dx)
                e2 = self.normalized_error(self.current_pose.pose.position.y, gamma_sy, gamma_dy)
                e3 = self.normalized_error(self.current_pose.pose.position.z, gamma_sz, gamma_dz)

                try:
                    print("")
                    e_matrix = torch.tensor([e1, e2, e3])
                    print("e_matrix: ", e_matrix, "time: ", t)
                    print("current pose: ", self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z)
                    print("target pose: ", gamma_sx/2, gamma_sy/2, gamma_sz/2)
                    print("--------------------------------------------------------------------")

                    #--------------------------- CONTROLLER 1 ---------------------------#
                    # epsilon1 = math.log((1 + e1) / (1 - e1))
                    # epsilon2 = math.log((1 + e2) / (1 - e2))
                    # epsilon3 = math.log((1 + e3) / (1 - e3))

                    # epsilon_matrix = torch.tensor([epsilon1, epsilon2, epsilon3])

                    # gamma_d_matrix = torch.diag(torch.tensor([gamma_dx, gamma_dy, gamma_dz]))
                    # xi_matrix = 4 * torch.matmul(gamma_d_matrix.inverse().to(torch.float64), (torch.eye(self.dimension).to(torch.float64) - torch.matmul(e_matrix.T, e_matrix)).inverse().to(torch.float64))
                    # u_matrix = torch.matmul(xi_matrix, epsilon_matrix.to(torch.float64))

                    # v_x = 3.245 * u_matrix[0].item()
                    # v_y = 1.75 * u_matrix[1].item()
                    # v_z = 0.1 * u_matrix[2].item()
                    # self.control_input.append([v_x, v_y, v_z])
                    #--------------------------------------------------------------------#

                    #--------------------------- CONTROLLER 2 ---------------------------#
                    phi_matrix = torch.tanh(k * e_matrix) * (1 - torch.exp(k * e_matrix))

                    v_x = -1.3 * phi_matrix[0].item()
                    v_y = -0.7 * phi_matrix[1].item()
                    v_z = -0.8 * phi_matrix[2].item()
                    self.control_input.append([v_x, v_y, v_z])
                    #--------------------------------------------------------------------#

                    self.vel_msg.twist.linear.x = v_x
                    self.vel_msg.twist.linear.y = v_y
                    self.vel_msg.twist.linear.z = v_z
                    self.vel_pub.publish(self.vel_msg)

                except ValueError:
                    rospy.INFO("Normalized error out of bounds!")

                rate.sleep()

        if not self.offboard_mode_set and self.current_state.mode != "AUTO.LOITER":
            rospy.wait_for_service('/mavros/set_mode')
            try:
                response = self.set_mode_srv(0, 'AUTO.LOITER')
                if response.mode_sent:
                    rospy.loginfo("AUTO.LOITER mode set successfully.")
                else:
                    rospy.logwarn("Failed to set AUTO.LOITER mode.")
                self.offboard_mode_set = True
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

        rospy.loginfo("AUTO.LOITER mode set. UAV on standby.")

        fig = plt.figure(figsize = (8,8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(0, 15)
        ax.set_ylim(0, 15)
        ax.set_zlim(0, 15)
        self.plot_cubes(self.gamma_u, self.gamma_l, ax)
        self.plot_points(self.trajectory, ax)
        self.plot_cuboid(ax, [-1,2], [-1,2], [1,4], 'green', 'green')       # start
        self.plot_cuboid(ax, [3,6], [6,9], [6,9], 'green', 'green')         # block 1
        self.plot_cuboid(ax, [6,9], [6,9], [0,15], 'red', 'red')            # obstacle
        self.plot_cuboid(ax, [9,12], [6,9], [6,9], 'green', 'green')        # block 2
        self.plot_cuboid(ax, [12,15], [12,15], [12,15], 'green', 'green')   # end

        plt.show(block = True)
        sys.exit(0)

    def create_cube_vertices(self, upper, lower):
        vertices = [
            [upper[0], upper[1], upper[2]],
            [lower[0], upper[1], upper[2]],
            [lower[0], lower[1], upper[2]],
            [upper[0], lower[1], upper[2]],
            [upper[0], upper[1], lower[2]],
            [lower[0], upper[1], lower[2]],
            [lower[0], lower[1], lower[2]],
            [upper[0], lower[1], lower[2]]
        ]
        return vertices

    def plot_cubes(self, upper_list, lower_list, ax):
        for i in np.arange(0, len(upper_list), 15):
            upper = upper_list[i]
            lower = lower_list[i]

            vertices = self.create_cube_vertices(upper, lower)
            
            faces = [[vertices[j] for j in [0, 1, 2, 3]],  # Top face
                    [vertices[j] for j in [4, 5, 6, 7]],  # Bottom face
                    [vertices[j] for j in [0, 1, 5, 4]],  # Front face
                    [vertices[j] for j in [2, 3, 7, 6]],  # Back face
                    [vertices[j] for j in [0, 3, 7, 4]],  # Left face
                    [vertices[j] for j in [1, 2, 6, 5]]
                    ]  # Right face

            ax.add_collection3d(Poly3DCollection(faces, facecolors='blue', edgecolors='blue', alpha=0.25))

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    def plot_points(self, point_list, ax):
        x = [point[0] for point in point_list]
        y = [point[1] for point in point_list]
        z = [point[2] for point in point_list]
        ax.scatter(x, y, z, c='green', marker='.', s=50, label='Points')

    def plot_cuboid(self, ax, x_range, y_range, z_range, face_color, edge_color):
        vertices = [
            [x_range[1], y_range[1], z_range[1]],  # Top front right
            [x_range[0], y_range[1], z_range[1]],  # Top front left
            [x_range[0], y_range[0], z_range[1]],  # Top back left
            [x_range[1], y_range[0], z_range[1]],  # Top back right
            [x_range[1], y_range[1], z_range[0]],  # Bottom front right
            [x_range[0], y_range[1], z_range[0]],  # Bottom front left
            [x_range[0], y_range[0], z_range[0]],  # Bottom back left
            [x_range[1], y_range[0], z_range[0]]   # Bottom back right
        ]

        faces = [[vertices[j] for j in [0, 1, 2, 3]],  # Top face
                [vertices[j] for j in [4, 5, 6, 7]],  # Bottom face
                [vertices[j] for j in [0, 1, 5, 4]],  # Front face
                [vertices[j] for j in [2, 3, 7, 6]],  # Back face
                [vertices[j] for j in [0, 3, 7, 4]],  # Left face
                [vertices[j] for j in [1, 2, 6, 5]]]  # Right face

        ax.add_collection3d(Poly3DCollection(faces, facecolors=face_color, linewidths=1, edgecolors=edge_color, alpha=0.25))

if __name__ == '__main__':

    #-----------------------------------------------------------------------------------------#
    #---------------------------------------- OMNIBOT ----------------------------------------#
    #-----------------------------------------------------------------------------------------#
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


    #-----------------------------------------------------------------------------------------#
    #----------------------------------------- DRONE -----------------------------------------#
    #-----------------------------------------------------------------------------------------#
    # C0 = -0.6155764940651274
    # C1 = -0.16747256519988604
    # C2 = 0.18354218525309415
    # C3 = -0.007637131069865292
    # C4 = -0.936214597798238
    # C5 = -0.07024607248050078
    # C6 = 0.20316323099379538
    # C7 = -0.009308739106244737
    # C8 = 1.0628853767935942
    # C9 = -0.059703865671985365
    # C10 = 0.17267342962280333
    # C11 = -0.007911726443198165
    # C12 = 1.9422117529674363
    # C13 = -0.16747256519988604
    # C14 = 0.18354218525309415
    # C15 = -0.007637131069865292
    # C16 = 1.6215736492343256
    # C17 = -0.07024607248050078
    # C18 = 0.20316323099379538
    # C19 = -0.009308739106244737
    # C20 = 3.620673623826158
    # C21 = -0.059703865671985365
    # C22 = 0.17267342962280333
    # C23 = -0.007911726443198165
    # C = [C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23]

    #------------------ DRONE OR CASE FIRST BLOCK ------------------#
    # C0 = -0.5092201124440394
    # C1 = -1.665740011109661
    # C2 = 1.542484516723121
    # C3 = -0.3243109601139466
    # C4 = 0.026244366716944464
    # C5 = -0.0007154516946045534
    # C6 = -0.5092201124440394
    # C7 = -0.9354351139789893
    # C8 = 0.8798053580061804
    # C9 = -0.15421328533867962
    # C10 = 0.011825144550615181
    # C11 = -0.00032995786104859844
    # C12 = 1.4907798875559606
    # C13 = -0.14819509743468093
    # C14 = 0.32437796115886813
    # C15 = -0.058556349964849215
    # C16 = 0.0052465859553120706
    # C17 = -0.0001699023672777245
    # C18 = 1.9953899437779803
    # C19 = -1.6883828104673564
    # C20 = 1.5955854133448955
    # C21 = -0.34080413875938786
    # C22 = 0.02799927661790214
    # C23 = -0.0007749401658234577
    # C24 = 1.9953899437779803
    # C25 = -0.9354351139789893
    # C26 = 0.8798053580061804
    # C27 = -0.15421328533867962
    # C28 = 0.011825144550615181
    # C29 = -0.00032995786104859844
    # C30 = 3.99538994377798
    # C31 = -0.14819509743468093
    # C32 = 0.32437796115886813
    # C33 = -0.058556349964849215
    # C34 = 0.0052465859553120706
    # C35 = -0.0001699023672777245
    # C = [C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23, C24, C25, C26, C27, C28, C29, C30, C31, C32, C33, C34, C35]


    #------------------ DRONE OR CASE SECOND BLOCK ------------------#
    C0 = -0.9947193621508238
    C1 = 0.4277863285546202
    C2 = 1.2140297298487956
    C3 = -0.27679682188266586
    C4 = 0.02200792560735738
    C5 = -0.0005888928692527835
    C6 = -0.5105612756983524
    C7 = -1.4667281217292212
    C8 = 1.0843122115171406
    C9 = -0.18152804910001696
    C10 = 0.013280466284529367
    C11 = -0.0003552471176615102
    C12 = 1.4894387243016476
    C13 = -1.27618875247245
    C14 = 0.655150320834794
    C15 = -0.08099965733727106
    C16 = 0.004422992408222249
    C17 = -8.864906559813607e-05
    C18 = 1.5105612756983524
    C19 = 0.4277863285546202
    C20 = 1.2140297298487956
    C21 = -0.27679682188266586
    C22 = 0.02200792560735738
    C23 = -0.0005888928692527835
    C24 = 1.994719362150824
    C25 = -1.4667281217292212
    C26 = 1.0843122115171406
    C27 = -0.18152804910001696
    C28 = 0.013280466284529367
    C29 = -0.0003552471176615102
    C30 = 3.994719362150824
    C31 = -1.27618875247245
    C32 = 0.655150320834794
    C33 = -0.08099965733727106
    C34 = 0.004422992408222249
    C35 = -8.864906559813607e-05
    C = [C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23, C24, C25, C26, C27, C28, C29, C30, C31, C32, C33, C34, C35]


    #------------------------------------------ RUN ------------------------------------------#

    try:
        STT_Controller(C, 3, 0, 15).uav_control()
    except rospy.ROSInterruptException:
        print("some error")
