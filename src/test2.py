#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros

def publish_fake_gps():
    # Initialize the ROS node
    rospy.init_node('fake_gps_publisher', anonymous=True)

    # Create a publisher to the topic /mavros/fake_gps/mocap/tf
    pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=10)

    # Create a TransformStamped message
    transform = PoseStamped()

    # Fill in the header with current time and frame information
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "map"  # Parent frame (adjustable)
    #transform.child_frame_id = "base_link"  # Child frame (adjustable)

    # Set translation (position) to (0, 0, 0)
    transform.pose.position.x = 5.00000000000
    transform.pose.position.y = 0.00000000000
    transform.pose.position.z = 0.00000000000

    # Set rotation (orientation) to identity (no rotation)
    transform.pose.orientation.x = 0.00000000000
    transform.pose.orientation.y = 0.00000000000
    transform.pose.orientation.z = 0.00000000000
    transform.pose.orientation.w = 1.00000000000  # Identity quaternion (no rotation)

    # Publish the transform
    rospy.loginfo("Publishing fake GPS data: Position (0, 0, 0), No rotation")
    pub.publish(transform)

    # Keep publishing the transform at a fixed rate (e.g., 10 Hz)
    rate = rospy.Rate(100)  # 10 Hz
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()  # Update the timestamp
        pub.publish(transform)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_gps()
    except rospy.ROSInterruptException:
        pass

