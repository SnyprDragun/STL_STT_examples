#include "dd_stl_stt/offboard_node.hpp"

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;

Controller::Controller(){
    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Controller::state_cb, this);

    string position_sub_topic = "/mavros/global_position/local";
    this->position_sub = this->nh.subscribe(position_sub_topic, 10, &Controller::position_cb, this);
    
    string position_pub_topic = "/mavros/setpoint_position/local";
    this->position_pub = this->nh.advertise<geometry_msgs::PoseStamped>(position_pub_topic, 10);

    string vel_pub_topic = "/mavros/setpoint_velocity/cmd_vel";
    this->velocity_pub = this->nh.advertise<geometry_msgs::TwistStamped>(vel_pub_topic, 10);

    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);
}

void Controller::state_cb(const mavros_msgs::State& msg){
    current_state = msg;
}

void Controller::position_cb(const geometry_msgs::PoseStamped& msg){
    current_position = msg
}

void Controller::init_connection(){
    Rate rate(20);

    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state_offboard.connected){
        ROS_INFO("Initializing controller_node...");
        spinOnce();
        rate.sleep();
        break;
    }
    ROS_INFO("Connected!");
}

std::vector<double> Offboard::gamma(double time){
    Rate rate(20.0);

    int degree = 1;
    std::vector<double> C{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::vector<double> real_tubes{0, 0, 0, 2, 2, 3};

    for (int i = 0; i < 6; ++i) {
        double power = 0;
        for (int j = 0; j <= degree; ++j) {
            real_tubes[i] += C[j + i * (degree + 1)] * std::pow(time, power);
            // ROS_INFO("Saw tube");
            // std::cout << i << ", see?" << j;
            ++power;
        }
    }
    return real_tubes;
}

void Controller::controller(){
    Rate rate(20.0);
}

void Controller::follow_stt(){
    Rate rate(20.0);

    std::vector<double> x_u;
    std::vector<double> x_l;
    std::vector<double> y_u;
    std::vector<double> y_l;
    std::vector<double> z_u;
    std::vector<double> z_l;

    for (double time=0; time<61; time+=1){
        x_u.push_back(gamma(time)[0]);
        x_l.push_back(gamma(time)[1]);
        y_u.push_back(gamma(time)[2]);
        y_l.push_back(gamma(time)[3]);
        z_u.push_back(gamma(time)[4]);
        z_l.push_back(gamma(time)[5]);
    }

    int time = 0;
    while (time <= 60){
        offboard((x_u[time] + x_l[time])/2, (y_u[time] + y_l[time])/2, (z_u[time] + z_l[time])/2);
        ROS_INFO("Here");
        time++;
    }

    spinOnce(); 
    rate.sleep();
}
