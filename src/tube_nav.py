#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

def sine_wave_motion():
    # Initialize the ROS node
    rospy.init_node('turtlebot_sine_wave', anonymous=True)
    
    # Create a publisher to the '/cmd_vel' topic to control the robot's velocity
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the loop rate (Hz)
    rate = rospy.Rate(10)
    
    # Parameters for the sine wave
    amplitude = 1.0   # Amplitude of the sine wave (meters)
    frequency = 0.2   # Frequency of the sine wave (Hz)
    linear_speed = 0.5  # Constant linear speed along the x-axis (meters/second)
    
    start_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        # Calculate the current time
        current_time = rospy.Time.now().to_sec()
        
        # Time elapsed since the start of the program
        elapsed_time = current_time - start_time
        
        # Calculate the desired y position using a sine function
        y_position = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        
        # Calculate the desired y velocity (derivative of the sine function)
        y_velocity = 2 * math.pi * frequency * amplitude * math.cos(2 * math.pi * frequency * elapsed_time)
        
        # Create a Twist message and populate it
        vel_msg = Twist()
        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = y_velocity
        vel_msg.angular.z = 0.0  # No rotation
        
        # Publish the velocity message
        cmd_vel_pub.publish(vel_msg)
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        sine_wave_motion()
    except rospy.ROSInterruptException:
        pass
