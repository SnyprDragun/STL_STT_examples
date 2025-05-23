#!/usr/bin/env python3
import sys
import time
import torch
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from phasespace_msgs.msg import Rigids
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import cflib.crtp

cflib.crtp.init_drivers(enable_debug_driver=False)
URI = 'radio://0/80/2M/E7E7E7E7E7'

class STT_CrazieflieMocap:
    def __init__(self, C, dimension, start, end, link_uri):
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)
        self._cf.open_link(link_uri)

        self.C = C
        self.dimension = dimension
        self.degree = int((len(C) / (2 * self.dimension)) - 1)
        self.start = start
        self.end = end
        self.step = 0.002
        self.range = int((self.end - self.start)/self.step)

        self.setpoints = [[-1, 2, -1, 2, 1, 4, 0, 1], [3, 6, 6, 9, 6, 9, 7, 8], [9, 12, 6, 9, 6, 9, 7, 8], [12, 15, 12, 15, 12, 15, 14, 15]]
        self.obstacles = [[6, 9, 6, 9, 0, 15, 0, 15]]

        rospy.init_node('STT_Controller')

        self.pose_sub = rospy.Subscriber('/phasespace/rigids', Rigids, self.uav_pose_callback)
        self.current_pose = Rigids()
        self.gamma_u = []
        self.gamma_l = []
        self.trajectory = []
        self.control_input = []

    def uav_pose_callback(self, msg):
        """Callback function for the pose subscriber."""
        self.current_pose = msg

    def _connected(self, link_uri):
        print('Connected to %s' % link_uri)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(1)
        self._start_velocity_control()

    def gamma(self, t):
        '''method to calculate tube boundaries at time instance 't' using torch for optimization'''
        t_tensor = torch.tensor(t, dtype=torch.float32)
        powers_of_t = torch.tensor([t_tensor**j for j in range(self.degree + 1)], dtype=torch.float32)
        C_tensor = torch.tensor(self.C, dtype=torch.float32).view(2 * self.dimension, self.degree + 1)
        real_tubes = torch.matmul(C_tensor, powers_of_t)
        return real_tubes.numpy()
    
    def normalized_error(self, x, gamma_sum, gamma_diff):
        return (2 * x - gamma_sum) / gamma_diff

    def _start_velocity_control(self):
        rate = rospy.Rate(500)
        t_values = np.arange(self.start, self.end + self.step, self.step)
        max_iterations = 1
        count = 0
        while not rospy.is_shutdown() and count < max_iterations:
            count += 1

            for t in t_values:
                rospy.sleep(self.step)
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

                self.trajectory.append([self.current_pose.rigids.x, self.current_pose.rigids.y, self.current_pose.rigids.z])

                e1 = self.normalized_error(self.current_pose.rigids.x, gamma_sx, gamma_dx)
                e2 = self.normalized_error(self.current_pose.rigids.y, gamma_sy, gamma_dy)
                e3 = self.normalized_error(self.current_pose.rigids.z, gamma_sz, gamma_dz)

                print("")
                e_matrix = torch.tensor([e1, e2, e3])
                print("e_matrix: ", e_matrix, "time: ", t)
                print("current pose: ", self.current_pose.rigids.x, self.current_pose.rigids.y, self.current_pose.rigids.z)
                print("target pose: ", gamma_sx/2, gamma_sy/2, gamma_sz/2)
                print("--------------------------------------------------------------------")

                #--------------------------- CONTROLLER 1 ---------------------------#
                # k = 1
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

                #----- block 1 -----#
                kx = 5
                ky = 3
                kz = 3
                max_vel = 1
                #-------------------#
                #----- block 2 -----#
                # kx = 7
                # ky = 3
                # kz = 3
                # max_vel = 2
                #-------------------#

                k = torch.diag(torch.tensor([kx, ky, kz]))
                phi_matrix = torch.tanh(torch.matmul(k.to(torch.float32), e_matrix.to(torch.float32))) * (1 - torch.exp(- torch.pow(torch.matmul(k.to(torch.float32), e_matrix.to(torch.float32)), 2)))

                v_x = -max_vel * phi_matrix[0].item()
                v_y = -max_vel * phi_matrix[1].item()
                v_z = -max_vel * phi_matrix[2].item()
                self.control_input.append([v_x, v_y, v_z])
                #--------------------------------------------------------------------#
                
                try:
                    self._cf.commander.send_velocity_world_setpoint(v_x , v_y, v_z, 0)
                    time.sleep(0.002)
                except KeyboardInterrupt:
                    print("Interrupted. Landing!")
                finally:
                    self._cf.close_link()

                rate.sleep()

        self.plot_for_3D()
        self.store_trajectory()

        plt.show(block = True)
        sys.exit(0)

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def faces(self, i):
        vertices = [[i[0], i[2], i[4]], [i[1], i[2], i[4]], [i[1], i[3], i[4]], [i[0], i[3], i[4]],  # Bottom face
                    [i[0], i[2], i[5]], [i[1], i[2], i[5]], [i[1], i[3], i[5]], [i[0], i[3], i[5]]]   # Top face

        # Define the 6 faces of the cube using the vertices
        faces = [   [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
                    [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
                    [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
                    [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
                    [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right face
                    [vertices[0], vertices[3], vertices[7], vertices[4]]]  # Left face
        return faces

    def plot_for_3D(self):
        x_u = np.zeros(self.range)
        x_l = np.zeros(self.range)
        y_u = np.zeros(self.range)
        y_l = np.zeros(self.range)
        z_u = np.zeros(self.range)
        z_l = np.zeros(self.range)

        for i in range(self.range):
            x_u[i] = self.gamma(i * self.step)[3]
            x_l[i] = self.gamma(i * self.step)[0]
            y_u[i] = self.gamma(i * self.step)[4]
            y_l[i] = self.gamma(i * self.step)[1]
            z_u[i] = self.gamma(i * self.step)[5]
            z_l[i] = self.gamma(i * self.step)[2]

        fig1, axs = plt.subplots(3, 1, figsize=(8, 8), constrained_layout=True)
        ax, bx, cx = axs
        for i in self.setpoints:        # t1  x1/y1/z1  t2    t1  x2/y2/z2  x1
            square_x = patches.Rectangle((i[6], i[0]), i[7] - i[6], i[1] - i[0], edgecolor='green', facecolor='none')
            square_y = patches.Rectangle((i[6], i[2]), i[7] - i[6], i[3] - i[2], edgecolor='green', facecolor='none')
            square_z = patches.Rectangle((i[6], i[4]), i[7] - i[6], i[5] - i[4], edgecolor='green', facecolor='none')
            ax.add_patch(square_x)
            bx.add_patch(square_y)
            cx.add_patch(square_z)

        for i in self.obstacles:        # t1  x1/y1/z1  t2    t1  x2/y2/z2  x1
            square_x = patches.Rectangle((i[6], i[0]), i[7] - i[6], i[1] - i[0], edgecolor='red', facecolor='none')
            square_y = patches.Rectangle((i[6], i[2]), i[7] - i[6], i[3] - i[2], edgecolor='red', facecolor='none')
            square_z = patches.Rectangle((i[6], i[4]), i[7] - i[6], i[5] - i[4], edgecolor='red', facecolor='none')
            ax.add_patch(square_x)
            bx.add_patch(square_y)
            cx.add_patch(square_z)

        t = np.linspace(self.start, self.end, self.range)
        
        xt = [sublist[0] for sublist in self.trajectory]
        yt = [sublist[1] for sublist in self.trajectory]
        zt = [sublist[2] for sublist in self.trajectory]

        ax.plot(t, x_u)
        for i in range(len(t) - 1):
            if x_l[i] < xt[i] < x_u[i]:
                ax.plot(t[i:i+2], xt[i:i+2], color='green')
            else:
                ax.plot(t[i:i+2], xt[i:i+2], color='red')
        ax.plot(t, x_l)

        bx.plot(t, y_u)
        for i in range(len(t) - 1):
            if y_l[i] < yt[i] < y_u[i]:
                bx.plot(t[i:i+2], yt[i:i+2], color='green')
            else:
                bx.plot(t[i:i+2], yt[i:i+2], color='red')
        bx.plot(t, y_l)

        cx.plot(t, z_u)
        for i in range(len(t) - 1):
            if z_l[i] < zt[i] < z_u[i]:
                cx.plot(t[i:i+2], zt[i:i+2], color='green')
            else:
                cx.plot(t[i:i+2], zt[i:i+2], color='red')
        cx.plot(t, z_l)

        ax.set_title("t vs x")
        bx.set_title("t vs y")
        cx.set_title("t vs z")

        # --------------------------------------------------- 3D PLOT {X vs Y vs Z} --------------------------------------------------- #
        fig2 = plt.figure(2, figsize = (10, 8))
        dx = fig2.add_subplot(111, projection='3d')
        dx.set_xlim(0, 15) ## dx.set_xlim(self.get_x_start(), self.get_x_finish())
        dx.set_ylim(0, 15) ## dx.set_ylim(self.get_y_start(), self.get_y_finish())
        dx.set_zlim(0, 15) ## dx.set_zlim(self.getStart(), self.getFinish())
        dx.set_xlabel('X Axis')
        dx.set_ylabel('Y Axis')
        dx.set_zlabel('Z Axis')

        for i in np.arange(0, self.range, 500):
            vertices = [[x_u[i], y_u[i], z_u[i]], [x_l[i], y_u[i], z_u[i]], [x_l[i], y_l[i], z_u[i]], [x_u[i], y_l[i], z_u[i]],
                        [x_u[i], y_u[i], z_l[i]], [x_l[i], y_u[i], z_l[i]], [x_l[i], y_l[i], z_l[i]], [x_u[i], y_l[i], z_l[i]]]

            faces = [   [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
                        [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
                        [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
                        [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
                        [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right face
                        [vertices[0], vertices[3], vertices[7], vertices[4]]]  # Left face

            dx.add_collection3d(Poly3DCollection(faces, facecolors='blue', alpha=0.09))

        x = [point[0] for point in self.trajectory]
        y = [point[1] for point in self.trajectory]
        z = [point[2] for point in self.trajectory]
        for i in np.arange(0, self.range, 50):
            if (x_l[i] < x[i] < x_u[i]) and (y_l[i] < y[i] < y_u[i]) and (z_l[i] < z[i] < z_u[i]):
                dx.scatter(x[i:i+2], y[i:i+2], z[i:i+2], c='black', marker='.', s=50, label='Points')
            else:
                dx.scatter(x[i:i+2], y[i:i+2], z[i:i+2], c='red', marker='.', s=50, label='Points')
        # dx.scatter(x, y, z, c='black', marker='.', s=50, label='Points')

        for i in self.obstacles:
            dx.add_collection3d(Poly3DCollection(self.faces(i), facecolors='red', edgecolors='r', alpha=0.25))

        for i in self.setpoints:
            dx.add_collection3d(Poly3DCollection(self.faces(i), facecolors='green', edgecolors='green', alpha=0.25))

    def store_trajectory(self):
        file_name = 'trajetory_first_block.py'
        # file_name = 'trajetory_second_block.py'
        content = "[" + ',\n '.join(map(str, self.trajectory)) + "]"
        with open(file_name, 'w') as f:
            f.write(content)


if __name__ == '__main__':

    #-----------------------------------------------------------------------------------------#
    #----------------------------------------- DRONE -----------------------------------------#
    #-----------------------------------------------------------------------------------------#

    #------------------ DRONE OR CASE FIRST BLOCK ------------------#
    C0 = -0.5092201124440394
    C1 = -1.665740011109661
    C2 = 1.542484516723121
    C3 = -0.3243109601139466
    C4 = 0.026244366716944464
    C5 = -0.0007154516946045534
    C6 = -0.5092201124440394
    C7 = -0.9354351139789893
    C8 = 0.8798053580061804
    C9 = -0.15421328533867962
    C10 = 0.011825144550615181
    C11 = -0.00032995786104859844
    C12 = 1.4907798875559606
    C13 = -0.14819509743468093
    C14 = 0.32437796115886813
    C15 = -0.058556349964849215
    C16 = 0.0052465859553120706
    C17 = -0.0001699023672777245
    C18 = 1.9953899437779803
    C19 = -1.6883828104673564
    C20 = 1.5955854133448955
    C21 = -0.34080413875938786
    C22 = 0.02799927661790214
    C23 = -0.0007749401658234577
    C24 = 1.9953899437779803
    C25 = -0.9354351139789893
    C26 = 0.8798053580061804
    C27 = -0.15421328533867962
    C28 = 0.011825144550615181
    C29 = -0.00032995786104859844
    C30 = 3.99538994377798
    C31 = -0.14819509743468093
    C32 = 0.32437796115886813
    C33 = -0.058556349964849215
    C34 = 0.0052465859553120706
    C35 = -0.0001699023672777245
    C = [C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23, C24, C25, C26, C27, C28, C29, C30, C31, C32, C33, C34, C35]


    #------------------ DRONE OR CASE SECOND BLOCK ------------------#
    # C0 = -0.9947193621508238
    # C1 = 0.4277863285546202
    # C2 = 1.2140297298487956
    # C3 = -0.27679682188266586
    # C4 = 0.02200792560735738
    # C5 = -0.0005888928692527835
    # C6 = -0.5105612756983524
    # C7 = -1.4667281217292212
    # C8 = 1.0843122115171406
    # C9 = -0.18152804910001696
    # C10 = 0.013280466284529367
    # C11 = -0.0003552471176615102
    # C12 = 1.4894387243016476
    # C13 = -1.27618875247245
    # C14 = 0.655150320834794
    # C15 = -0.08099965733727106
    # C16 = 0.004422992408222249
    # C17 = -8.864906559813607e-05
    # C18 = 1.5105612756983524
    # C19 = 0.4277863285546202
    # C20 = 1.2140297298487956
    # C21 = -0.27679682188266586
    # C22 = 0.02200792560735738
    # C23 = -0.0005888928692527835
    # C24 = 1.994719362150824
    # C25 = -1.4667281217292212
    # C26 = 1.0843122115171406
    # C27 = -0.18152804910001696
    # C28 = 0.013280466284529367
    # C29 = -0.0003552471176615102
    # C30 = 3.994719362150824
    # C31 = -1.27618875247245
    # C32 = 0.655150320834794
    # C33 = -0.08099965733727106
    # C34 = 0.004422992408222249
    # C35 = -8.864906559813607e-05
    # C = [C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, C18, C19, C20, C21, C22, C23, C24, C25, C26, C27, C28, C29, C30, C31, C32, C33, C34, C35]

    try:
        STT_CrazieflieMocap(C, 3, 0, 15, URI)
    except rospy.ROSInterruptException:
        print("some error")
