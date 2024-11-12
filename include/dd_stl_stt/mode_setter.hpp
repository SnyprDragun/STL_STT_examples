#ifndef MODE_SETTER_HPP
#define MODE_SETTER_HPP

#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace ros;

class Setter{
    private:
        NodeHandle nh;
        Subscriber setter_state_sub;
        Publisher setter_position_pub;
        ServiceClient mode_setter_client;

    public:
        Setter();
        void state_cb(const mavros_msgs::State&);
        void OFFBOARD();
        void LOITER();
        void STABILIZED();
};

#endif