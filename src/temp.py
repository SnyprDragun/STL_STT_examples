#!/usr/bin/env python3
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import math

def latlon_to_xy(lat, lon):
    ref_lat = 37.7749
    ref_lon = -122.4194
    ref_alt = 0 

    earth_radius = 6371000  
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)

    x = earth_radius * d_lon * math.cos(math.radians(ref_lat))
    y = earth_radius * d_lat

    print(x, y)

def xy_to_latlon(x, y):
    ref_lat = 47.3977422
    ref_lon = 8.545593

    earth_radius = 6371000

    d_lat = y/(earth_radius)
    d_lon = x/(earth_radius * math.cos(math.radians(ref_lat)))

    lat = math.degrees(math.radians(ref_lat) + d_lat)
    lon = math.degrees(math.radians(ref_lon) + d_lon)

    print(lat, lon)
    return lat, lon

waypoints_latlon = [
            [37.7749, -122.4194], 
            [37.7749, -122.4194],  
            [37.7759, -122.4184], 
            [37.7759, -122.4174],  
            [37.7759, -122.4174] 
        ]

waypoints_xy = [[0.0              , 0.0               ],
                [0.0              , 0.0               ],
                [87.89107614166831, 111.19492664429958],
                [175.7821522845856, 111.19492664429958],
                [175.7821522845856, 111.19492664429958]
            ]

# for i in waypoints_latlon:
#         latlon_to_xy(i[0], i[1])
# print('---------------------------------------')
# for i in waypoints_xy:
#         xy_to_latlon(i[0], i[1])

# latitude: 47.3977422
# longitude: 8.545593
# altitude: 534.719917099309



class Modes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    def auto_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)

    def wpPush(self,index,wps):
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
            wpPushService(start_index=0,waypoints=wps)#start_index = the index at which we want the mission to start
            print ("Waypoint Pushed")
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s"%e)
    def wpPull(self,wps):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            print (wpPullService().wp_received)

            print ("Waypoint Pulled")
        except rospy.ServiceException as e:
            print ("Service Puling call failed: %s"%e)

class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        
    def stateCb(self, msg):
        self.state = msg

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
        self.wp.command = command #'''VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
        self.wp.param1=param1 # no idea what these are for but the script will work so go ahead
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt #relative altitude.

        return self.wp


def main():
    rospy.init_node('waypointMission', anonymous=True)
    rate = rospy.Rate(20.0)

    stateMt = stateMoniter()
    md = Modes()
    
    wayp0 = wpMissionCnt()
    wayp1 = wpMissionCnt()
    wayp2 = wpMissionCnt()
    wayp3 = wpMissionCnt()
    wayp4 = wpMissionCnt()
    
    wps = [] #List to story waypoints

# ------------------------------------------------------

    w = wayp0.setWaypoints(3,16,True,True,0.0,0.0,0.0,float('nan'),47.3977422, 8.545593,5)
    wps.append(w)

    w = wayp1.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),47.39776917964818, 8.54563285736424,5)
    wps.append(w)

    w = wayp2.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),47.397760186432116, 8.54564614315232,5)
    wps.append(w)

    w = wayp3.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),47.3977871660803, 8.5456594289404,5)
    wps.append(w)

# 47.3977422 8.545593
# 47.39776917964818 8.54563285736424
# 47.397760186432116 8.54564614315232
# 47.3977871660803 8.5456594289404

# -------------------------------------------------------
    print(wps)
    md.wpPush(0,wps)

    md.wpPull(wps[0])
    md.wpPull(wps[1])
    md.wpPull(wps[2])
    md.wpPull(wps[3])

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
    # Switching the state to auto mode
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode()
        rate.sleep()
        print("AUTO.MISSION")

    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


##Source: https://github.com/Mohit505Git/Mavros-AUTO.MISSION-mode
# latitude: 47.3977417
# longitude: 8.5455956
# altitude: 532.8409132385511