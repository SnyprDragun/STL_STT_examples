#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,sin,cos
from tf.transformations import euler_from_quaternion
import numpy as np 

class turtlebot():
    def __init__(self, C, degree):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.callback)
        self.pose = Odometry()
        self.rate = rospy.Rate(20)
        self.C = C
        self.dimension = 2
        self.degree = degree

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data.pose.pose.position
        self.orient = data.pose.pose.orientation
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def gamma(self, t):
        real_tubes = np.zeros(2 * self.dimension)

        for i in range(2 * self.dimension): #for 4 tube equations
            power = 0
            for j in range(self.degree + 1): #each tube eq has {degree+1} terms
                real_tubes[i] += ((self.C[j + i * (self.degree + 1)]) * (t ** power))
                power += 1
        x_l = real_tubes[0]
        y_l = real_tubes[1]
        x_u = real_tubes[2]
        y_u = real_tubes[3]
        return (x_u + x_l)/2, (y_u + y_l)/2

    def move2goal(self, x, y):
        K1 = 0.2
        K2 = 0.1
        goal_pose_ = Odometry()
        goal_pose = goal_pose_.pose.pose.position
        goal_pose.x = x
        goal_pose.y = y
        distance_tolerance = 0.01
        vel_msg = Twist()
        r = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
        while r >= distance_tolerance:

            r = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            psi = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
            orientation_list = [self.orient.x, self.orient.y, self.orient.z, self.orient.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            theta = yaw
            phi = theta - psi
            if phi > np.pi:
                phi = phi - 2*np.pi
            if phi < -np.pi:
                phi = phi + 2*np.pi

            vel_msg.linear.x = K1*r*cos(phi)
            vel_msg.angular.z = -K1*sin(phi)*cos(phi)-(K2*phi)

            #Publishing input
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def traverse(self):
        for time in np.arange(0, 10.05, 0.05):
            x_target, y_target = self.gamma(time)
            self.move2goal(x_target, y_target)


if __name__ == '__main__':
   x = turtlebot()
   while 1:
      try:
        x.traverse()
      except:
        pass