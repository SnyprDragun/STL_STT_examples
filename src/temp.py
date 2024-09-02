#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import WaypointPush, SetMode
from mavros_msgs.msg import Waypoint
from geometry_msgs.msg import PoseStamped
import time

def send_waypoints(waypoint_push_client, waypoints):
    waypoint_list = []
    for pose in waypoints:
        wp = Waypoint()
        wp.x_lat = pose.pose.position.x
        wp.y_long = pose.pose.position.y
        wp.z_alt = pose.pose.position.z
        wp.frame = Waypoint.FRAME_LOCAL_NED  # Local NED coordinate frame
        wp.command = 16   # Command to navigate to waypoint
        wp.is_current = False
        wp.autocontinue = True
        waypoint_list.append(wp)
    
    try:
        response = waypoint_push_client(waypoint_list)
        if response.success:
            rospy.loginfo("Waypoints sent successfully.")
        else:
            rospy.logerr("Failed to send waypoints.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

def main():
    rospy.init_node('waypoint_sender_node')

    # Create service proxies for sending waypoints and setting mode
    rospy.wait_for_service('/mavros/mission/push')
    waypoint_push_client = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
    
    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    # Set mode to AUTO.MISSION
    try:
        set_mode_response = set_mode_client(custom_mode="AUTO.MISSION")
        if set_mode_response.mode_sent:
            rospy.loginfo("Mode set to AUTO.MISSION.")
        else:
            rospy.logerr("Failed to set mode.")
            return
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return

    # Define waypoints
    waypoints = []

    # Example waypoints in local NED frame
    waypoint1 = PoseStamped()
    waypoint1.pose.position.x = 10.0  # NED North coordinate
    waypoint1.pose.position.y = 5.0   # NED East coordinate
    waypoint1.pose.position.z = -10.0 # NED Down coordinate
    waypoints.append(waypoint1)

    waypoint2 = PoseStamped()
    waypoint2.pose.position.x = 20.0  # NED North coordinate
    waypoint2.pose.position.y = 10.0  # NED East coordinate
    waypoint2.pose.position.z = -15.0 # NED Down coordinate
    waypoints.append(waypoint2)

    waypoint3 = PoseStamped()
    waypoint3.pose.position.x = 30.0  # NED North coordinate
    waypoint3.pose.position.y = 15.0  # NED East coordinate
    waypoint3.pose.position.z = -20.0 # NED Down coordinate
    waypoints.append(waypoint3)

    # Add more waypoints as needed
    # Ensure there are 200 waypoints in total

    # Send waypoints to the drone
    send_waypoints(waypoint_push_client, waypoints)

    # Wait for each waypoint to be reached
    for _ in waypoints:
        time.sleep(1)  # Wait for 1 second before proceeding to the next waypoint

    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    main()
