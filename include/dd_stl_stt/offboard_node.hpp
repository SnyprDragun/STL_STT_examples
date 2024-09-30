#ifndef OFFBOARD_NODE_HPP
#define OFFBOARD_NODE_HPP

#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>

using namespace std;
using namespace ros;

class Offboard{
    private:
        NodeHandle nh;
        Subscriber state_sub;
        Publisher position_pub;
        Publisher velocity_pub;
        ServiceClient set_mode_client;
        ServiceClient waypoint_push_client;
        
    public:
        Offboard();
        void state_cb(const mavros_msgs::State&);
        void init_connection();
        void offboard(double, double, double);
        void panorama();
        // void sendWaypoints(const std::vector<geometry_msgs::PoseStamped>&);
        void mission();
        std::vector<double> gamma(double);
        void follow_stt();
};

#endif