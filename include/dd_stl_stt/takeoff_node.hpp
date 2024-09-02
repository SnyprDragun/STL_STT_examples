#ifndef TAKEOFF_NODE_HPP
#define TAKEOFF_NODE_HPP

#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>

using namespace std;
using namespace ros;

class Takeoff{
    private:
        NodeHandle nh;
        Subscriber state_sub;
        Publisher global_pos_pub;
        ServiceClient takeoff_client;
        ServiceClient arming_client;
        ServiceClient set_mode_client;
        
    public:
        Takeoff();
        void state_cb(const mavros_msgs::State&);
        void init_connection();
        void arm();
        void takeoff(float);
};

#endif