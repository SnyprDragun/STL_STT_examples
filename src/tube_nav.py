#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtleBotPositionUpdater:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtlebot_position_updater')
        
        # Publisher to send velocity commands
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscriber to get the robot's current position
        self.position = {'x': 0.0, 'y': 0.0}
        self.sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # Timer to call the update_position function every 0.05 seconds
        rospy.Timer(rospy.Duration(0.05), self.update_position)
        
        # Keep the node running
        rospy.spin()

    def odom_callback(self, msg):
        # Update the position from odometry data
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y

    def update_position(self, event):
        # Calculate the desired position
        new_x = self.position['x'] + 0.1
        new_y = self.position['y'] + 0.1

        # Calculate the distance to move and the angle to turn
        distance = math.sqrt((new_x - self.position['x'])**2 + (new_y - self.position['y'])**2)
        angle = math.atan2(new_y - self.position['y'], new_x - self.position['x'])
        
        # Create a Twist message to control the TurtleBot
        twist = Twist()
        twist.linear.x = distance / 0.05  # Speed to cover the distance in 0.05 seconds
        twist.angular.z = angle - self.position['theta']  # Turn to face the new angle
        
        # Publish the velocity command
        self.pub.publish(twist)
        rospy.loginfo("Moving to new position: x=%f, y=%f", new_x, new_y)

if __name__ == '__main__':
    TurtleBotPositionUpdater()




# import rospy
# import numpy as np
# from geometry_msgs.msg import Twist

# def gammas(degree, dimension, t):
#         '''method to calculate tube equations'''
#         C = [0.00416667, 0.5, 0, 0.00416667, 0.48936736, 0.00265816,
#             0.50833333, 0.5, 0, 0.50833333, 0.51568315, -0.00265816]
    
#         vel_tubes = np.zeros(2 * dimension)
#         for i in range(2 * dimension): #for 4 tube equations
#             power = 0
#             for j in range(degree + 1): #each tube eq has {degree+1} terms
#                 if power > 0:
#                     vel_tubes[i] += power * (((C[j + i * (degree + 1)]) * (t ** (power - 1))))
#                 else:
#                     vel_tubes[i] += 0
#                 power += 1
#         gamma1_L, gamma2_L, gamma1_U, gamma2_U = vel_tubes[0], vel_tubes[1], vel_tubes[2], vel_tubes[3]
#         x_vel = (gamma1_L + gamma1_U)/2
#         y_vel = (gamma2_L + gamma2_U)/2
#         return [x_vel, y_vel]

# def publish_velocity(end_time):
#     rospy.init_node('velocity_publisher', anonymous=True)
#     velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
#     rate = rospy.Rate(20)  # 20 Hz, which is equivalent to 0.05 seconds
    
#     twist = Twist()
#     time_elapsed = 0.0

#     while not rospy.is_shutdown():
#         # Increment time
#         time_elapsed += 0.05

#         # Set linear velocities in both x and y directions
#         twist.linear.x = gammas(2, 2, time_elapsed)[0]  # Constant velocity in the x-direction
#         twist.linear.y = gammas(2, 2, time_elapsed)[1]  # Constant velocity in the y-direction
#         twist.angular.z = 0.0  # Keep angular velocity constant for simplicity

#         # Publish the velocity
#         velocity_publisher.publish(twist)

#         rospy.loginfo("Time: {:.2f}s, Publishing velocity - Linear: x: {:.2f} m/s, y: {:.2f} m/s, Angular: {:.2f} rad/s".format(
#             time_elapsed, twist.linear.x, twist.linear.y, twist.angular.z))

#         if time_elapsed >= end_time:
#             twist.linear.x = 0.0
#             twist.linear.y = 0.0
#             velocity_publisher.publish(twist)
#             break
#         # Sleep to maintain the loop at the desired rate (0.05 seconds)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         publish_velocity(10)
#     except rospy.ROSInterruptException:
#         pass



# import rospy
# import numpy as np
# from geometry_msgs.msg import Twist



# def follow_equation(start_time, end_time):
#     rospy.init_node('stt_follower', anonymous=True)
#     cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     rate = rospy.Rate(20)
#     start_time = rospy.Time.now().to_sec()

#     while not rospy.is_shutdown():
#         current_time = rospy.Time.now().to_sec() - start_time
#         if current_time % 0.05 == 0:
#             x_vel = gammas(2, 2, current_time)[0]
#             y_vel = gammas(2, 2, current_time)[1]
#             vel_msg = Twist()
#             vel_msg.linear.x = x_vel
#             vel_msg.linear.y = y_vel
#             vel_msg.angular.z = 0.0
#             cmd_vel_pub.publish(vel_msg)

#             if current_time >= end_time:
#                 vel_msg.linear.x = 0.0
#                 vel_msg.linear.y = 0.0
#                 cmd_vel_pub.publish(vel_msg)
#                 break

#         rate.sleep()

# if __name__ == '__main__':
#     follow_equation(0, 10)
