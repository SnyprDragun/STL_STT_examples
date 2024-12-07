#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros

def publish_fake_gps():
    # Initialize the ROS node
    rospy.init_node('fake_gps_publisher', anonymous=True)

    # Create a publisher to the topic /mavros/fake_gps/mocap/tf
    pub = rospy.Publisher('/mavros/fake_gps/mocap/tf', TransformStamped, queue_size=10)

    # Create a TransformStamped message
    transform = TransformStamped()

    # Fill in the header with current time and frame information
    transform.header.stamp = rospy.Time.now()
    #transform.header.frame_id = "world"  # Parent frame (adjustable)
    #transform.child_frame_id = "mocap"  # Child frame (adjustable)

    # Set translation (position) to (0, 0, 0)
    transform.transform.translation.x = 5.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0

    # Set rotation (orientation) to identity (no rotation)
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0  # Identity quaternion (no rotation)

    # Publish the transform
    rospy.loginfo("Publishing fake GPS data: Position (0, 0, 0), No rotation")
    pub.publish(transform)

    # Keep publishing the transform at a fixed rate (e.g., 10 Hz)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        transform.header.stamp = rospy.Time.now()  # Update the timestamp
        pub.publish(transform)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_gps()
    except rospy.ROSInterruptException:
        pass

