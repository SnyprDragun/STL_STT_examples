#ifndef STT_CONTROLLER_HPP
#define STT_CONTROLLER_HPP

#include <cmath>
#include <vector>
#include <thread>
#include <chrono>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/WaypointPush.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>

using namespace std;
using namespace ros;

class Controller{
    private:
        NodeHandle nh;
        Subscriber state_sub;
        Subscriber position_sub;
        Publisher position_pub;
        Publisher velocity_pub;
        ServiceClient set_mode_client;
        
    public:
        Controller();
        void state_cb(const mavros_msgs::State&);
        void position_cb(const geometry_msgs::PoseStamped&);
        void init_connection();
        std::vector<double> gamma(double);
        double normalized_error(double, double, double);
        void controller();
};

#endif