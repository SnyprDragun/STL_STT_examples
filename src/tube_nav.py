#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist

def gammas(degree, dimension, t):
        '''method to calculate tube equations'''
        C = [0.00416667, 0.5, 0, 0.00416667, 0.48936736, 0.00265816,
            0.50833333, 0.5, 0, 0.50833333, 0.51568315, -0.00265816]
        
        vel_tubes = np.zeros(2 * dimension)
        for i in range(2 * dimension): #for 4 tube equations
            power = 0
            for j in range(degree + 1): #each tube eq has {degree+1} terms
                if power > 0:
                    vel_tubes[i] += power * ((C[j + i * (degree + 1)]) * (t ** (power - 1)))
                else:
                    vel_tubes[i] += 0
                power += 1
        gamma1_L, gamma2_L, gamma1_U, gamma2_U = vel_tubes[0], vel_tubes[1], vel_tubes[2], vel_tubes[3]
        x_vel = (gamma1_L + gamma1_U)/2
        y_vel = (gamma2_L + gamma2_U)/2
        return x_vel, y_vel

def follow_equation(start_time, end_time):
    rospy.init_node('stt_follower', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - start_time
        # rospy.loginfo("Time stamp")
        x_vel , y_vel = gammas(2, 2, current_time)
        vel_msg = Twist()
        vel_msg.linear.x = x_vel
        vel_msg.linear.y = y_vel
        vel_msg.angular.z = 0.0
        cmd_vel_pub.publish(vel_msg)

        if current_time >= end_time:
            vel_msg.linear.x = 0.0
            cmd_vel_pub.publish(vel_msg)
            break

        rate.sleep()

if __name__ == '__main__':
    follow_equation(0, 10)
