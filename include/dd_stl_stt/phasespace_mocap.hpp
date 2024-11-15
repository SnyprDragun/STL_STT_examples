#ifndef PHASESPACE_MOCAP_HPP
#define PHASESPACE_MOCAP_HPP

#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>

using namespace std;
using namespace ros;

class Mocap{
    private:
        NodeHandle nh;
        Subscriber phasespace_mocap_sub;
        Publisher mavros_mocap_pub;
        
    public:
        Mocap();
        void phasespace_mocap_cb(const phasespace_msgs::Rigids&);
        void init_connection();
        void mocap_to_mavros_transform();
};

#endif