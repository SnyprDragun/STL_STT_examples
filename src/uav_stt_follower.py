#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, CommandBoolRequest, SetMode, SetModeRequest


class uav_stt_follower():
    def __init__(self, C, degree):
        rospy.init_node('uav_controller', anonymous=True)

        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.pose_publisher = rospy.Publisher('/curve', PoseStamped, queue_size=10)

        self.current_pose = PoseStamped()
        self.rate = rospy.Rate(20)

        self.C = C
        self.dimension = 3
        self.degree = degree

    def state_callback(self, state_data):
        self.current_state = state_data

    def gamma(self, t):
        '''method to calculate tube equations'''
        real_tubes = np.zeros(2 * self.dimension)
        for i in range(2 * self.dimension):
            power = 0
            for j in range(self.degree + 1):
                real_tubes[i] += ((self.C[j + i * (self.degree + 1)]) * (t ** power))
                power += 1
        return real_tubes

    def trajectory(self):
        x_u, x_l, y_u, y_l, z_u, z_l = [], [], [], [], [], []
        for t in np.arange(0, 10.05, 0.05):
            x_u += self.gamma(t)[0]
            x_l += self.gamma(t)[1]
            y_u += self.gamma(t)[2]
            y_l += self.gamma(t)[3]
            z_u += self.gamma(t)[4]
            z_u += self.gamma(t)[5]

        waypoint_x = (x_u + x_l)/2
        waypoint_y = (y_u + y_l)/2
        waypoint_z = (z_u + z_l)/2

        self.current_pose.pose.position.x = waypoint_x
        self.current_pose.pose.position.y = waypoint_y
        self.current_pose.pose.position.z = waypoint_z

        self.pose_publisher.publish(self.current_pose)
        self.rate.sleep()

if __name__ == '__main__':
    C = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    while 1:
      try:
        drone = uav_stt_follower(C, 2)
        drone.trajectory()
      except:
        pass
