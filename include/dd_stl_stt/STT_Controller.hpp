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
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>

using namespace std;
using namespace ros;
using namespace Eigen;

class Controller{
    private:
        NodeHandle nh;
        Subscriber state_sub;
        Subscriber position_sub;
        Publisher position_pub;
        Publisher velocity_pub;
        ServiceClient set_custom_mode_client;

        int degree;
        int dimension;
        MatrixXf C;
        double start, end, step;
        vector<vector<double>> gamma_u, gamma_l, trajectory, control_input;

    public:
        Controller(int, int, const vector<vector<double>>&, double, double, double);
        void state_cb(const mavros_msgs::State&);
        void position_cb(const geometry_msgs::PoseStamped&);
        void init_connection();
        VectorXf gamma(double);
        double normalized_error(double, double, double);
        void controller();
};

#endif