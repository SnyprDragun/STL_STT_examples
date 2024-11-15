#!/usr/bin/env python3
'''script for mocap_to_mavros_transform'''

import rospy
from phasespace_msgs.msg import Rigids
from geometry_msgs.msg import TransformStamped

def rigids_callback(data):
    if data.rigids.id == 1:
        transform_msg = TransformStamped()

        transform_msg.transform.translation.x = data.rigids.x
        transform_msg.transform.translation.y = data.rigids.y
        transform_msg.transform.translation.z = data.rigids.z

        transform_msg.transform.rotation.x = data.rigids.qx
        transform_msg.transform.rotation.y = data.rigids.qy
        transform_msg.transform.rotation.z = data.rigids.qz
        transform_msg.transform.rotation.w = data.rigids.qw

        pub.publish(transform_msg)

if __name__ == "__main__":
    rospy.init_node("mocap_to_fake_gps", anonymous=True)
    rospy.Subscriber("/phasespace/rigids", Rigids, rigids_callback)
    pub = rospy.Publisher("/mavros/fake_gps/mocap/tf", TransformStamped, queue_size=10)
    rospy.loginfo("Mocap to Fake GPS Bridge Node started.")
    rospy.spin()
