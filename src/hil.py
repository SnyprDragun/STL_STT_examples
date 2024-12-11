#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import HilGPS
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Header

def gps_publisher():
    # Initialize the ROS node
    rospy.init_node('static_gps_publisher', anonymous=True)

    # Create a publisher for the /mavros/hil/gps topic
    pub = rospy.Publisher('/mavros/hil/gps', HilGPS, queue_size=10)

    # Set the rate (in Hz) at which to publish the message
    rate = rospy.Rate(10)  # 10 Hz (adjust as needed)

    # Create an instance of the HilGPS message
    gps_msg = HilGPS()

    while not rospy.is_shutdown():
        # Assign fixed values to the GPS message fields
        
        # Header (Timestamp)
        gps_msg.header = Header()
        gps_msg.header.stamp = rospy.Time.now()  # Current time
        gps_msg.header.frame_id = "gps"  # Set a frame ID (optional)

        # Fix type (2 = 3D fix)
        gps_msg.fix_type = 2

        # Geographic position (latitude, longitude, altitude)
        gps_msg.geo = GeoPoint()
        gps_msg.geo.latitude = -35.3621474  # Latitude in degrees
        gps_msg.geo.longitude = 149.1651746  # Longitude in degrees
        gps_msg.geo.altitude = 500  # Altitude in meters

        # Horizontal dilution of precision (HDOP) - multiply by 1000 to make it an integer
        gps_msg.eph = int(0.2 * 1000)  # Horizontal dilution of precision (HDOP) as an integer

        # Vertical dilution of precision (VDOP) - same fix, multiply by 1000 if needed
        gps_msg.epv = int(0.0 * 1000)  # Vertical dilution of precision (VDOP)

        # Ground speed in cm/s (0.0 m/s converted to cm/s)
        gps_msg.vel = 0  # Ground speed in cm/s (use integer)

        # Velocity components (north, east, down) - all are 0 in this case
        gps_msg.vn = 0  # North velocity in cm/s
        gps_msg.ve = 0  # East velocity in cm/s
        gps_msg.vd = 0  # Down velocity in cm/s

        # Course over ground (COG) in degrees (0-360)
        gps_msg.cog = 0  # Course over ground in degrees

        # Number of satellites visible
        gps_msg.satellites_visible = 20  # Number of visible satellites

        # Publish the message
        rospy.loginfo("Publishing GPS data: %s", gps_msg)
        pub.publish(gps_msg)

        # Sleep to maintain the desired publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass

