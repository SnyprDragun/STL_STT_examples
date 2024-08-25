#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def follow_equation(C0, C1, C2, final_time):
    # Initialize the ROS node
    rospy.init_node('turtlebot_follow_equation', anonymous=True)
    
    # Publisher to send velocity commands to the TurtleBot
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to loop
    rate = rospy.Rate(10)  # 10 Hz
    
    # Record the start time
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        # Get the current time
        current_time = rospy.Time.now().to_sec() - start_time
        
        # Calculate the desired position using the given equation
        x_desired = C0 + C1 * current_time + C2 * current_time**2
        
        # Calculate the derivative to find the desired velocity
        x_velocity = C1 + 2 * C2 * current_time
        
        # Create a Twist message to send velocity commands
        vel_msg = Twist()
        
        # Set the linear x velocity (assuming movement along the x-axis)
        vel_msg.linear.x = x_velocity
        
        # We assume no rotation is needed
        vel_msg.angular.z = 0.0
        
        # Publish the velocity command
        cmd_vel_pub.publish(vel_msg)
        
        # Stop the robot if the final time is reached
        if current_time >= final_time:
            vel_msg.linear.x = 0.0
            cmd_vel_pub.publish(vel_msg)
            break
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        # Define your coefficients and final time
        C0 = 0.0
        C1 = 0.3
        C2 = 0.02
        final_time = 10.0  # Time at which x = 5
        
        follow_equation(C0, C1, C2, final_time)
    except rospy.ROSInterruptException:
        pass
