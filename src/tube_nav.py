#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def follow_equation(C0, C1, C2, final_time):
    rospy.init_node('turtlebot_follow_equation', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - start_time
        x_desired = C0 + C1 * current_time + C2 * current_time**2
        x_velocity = C1 + 2 * C2 * current_time

        vel_msg = Twist()
        vel_msg.linear.x = x_velocity
        vel_msg.angular.z = 0.0
        cmd_vel_pub.publish(vel_msg)

        if current_time >= final_time:
            vel_msg.linear.x = 0.0
            cmd_vel_pub.publish(vel_msg)
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        C0 = 0.0
        C1 = 0.3
        C2 = 0.02
        final_time = 10.0
        
        follow_equation(C0, C1, C2, final_time)
    except rospy.ROSInterruptException:
        pass
