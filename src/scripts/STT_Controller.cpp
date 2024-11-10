#include "dd_stl_stt/STT_Controller.hpp"

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

    start_ = 0; 
    end_ = 100; 
    step_ = 0.1;
}

void Controller::state_cb(const mavros_msgs::State& msg){
    current_state = msg;
}

void Controller::position_cb(const geometry_msgs::PoseStamped& msg){
    current_position = msg;
}

void Controller::init_connection(){
    Rate rate(20);

    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state.connected){
        ROS_INFO("Initializing controller_node...");
        spinOnce();
        rate.sleep();
        break;
    }
    ROS_INFO("Connected!");
}

vector<double> Controller::gamma(double time){
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

double Controller::normalized_error(double x, double gamma_sum, double gamma_diff) {
    return ((2.0 * x - gamma_sum) / gamma_diff);
}

void Controller::controller(){
    Rate rate(500);
    vector<double> t_values;

    for (double t = start_; t <= end_; t += step_) {
        t_values.push_back(t);
    }

    while (ros::ok() && !current_state.armed) {
        ROS_INFO("Waiting for drone to be armed...");
        ros::Duration(1.0).sleep();
    }

    // Set initial velocity to zero
    vel_msg_.twist.linear.x = vel_msg_.twist.linear.y = vel_msg_.twist.linear.z = 0;
    vel_pub_.publish(vel_msg_);
    rate.sleep();

    // Set OFFBOARD mode
    if (!offboard_mode_set_ && current_state_.mode != "OFFBOARD") {
        mavros_msgs::SetMode offboard_set_mode;
        offboard_set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
            ROS_INFO("OFFBOARD mode set successfully.");
            offboard_mode_set_ = true;
        } else {
            ROS_WARN("Failed to set OFFBOARD mode.");
        }
    }

    ROS_INFO("OFFBOARD mode set. UAV ready for velocity control.");

    int max_iterations = 1;
    int count = 0;
    while (ok() && count < max_iterations) {
        count++;

        for (double t : t_values) {
            Duration(step_).sleep();
            Eigen::VectorXd gamma(6); // Assume gamma returns Eigen::VectorXd of size 6
            gamma = gamma(t);

            gamma_u_.push_back({gamma[3], gamma[4], gamma[5]});
            gamma_l_.push_back({gamma[0], gamma[1], gamma[2]});

            double gamma_sx = gamma[3] + gamma[0];
            double gamma_dx = gamma[3] - gamma[0];
            double gamma_sy = gamma[4] + gamma[1];
            double gamma_dy = gamma[4] - gamma[1];
            double gamma_sz = gamma[5] + gamma[2];
            double gamma_dz = gamma[5] - gamma[2];

            trajectory_.push_back({current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z});

            double e1 = normalized_error(current_pose_.pose.position.x, gamma_sx, gamma_dx);
            double e2 = normalized_error(current_pose_.pose.position.y, gamma_sy, gamma_dy);
            double e3 = normalized_error(current_pose_.pose.position.z, gamma_sz, gamma_dz);

            Eigen::Vector3d e_matrix(e1, e2, e3);
            std::cout << "\ne_matrix: " << e_matrix.transpose() << " time: " << t << std::endl;
            std::cout << "current pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y << ", " << current_pose_.pose.position.z << std::endl;
            std::cout << "target pose: " << gamma_sx / 2 << ", " << gamma_sy / 2 << ", " << gamma_sz / 2 << std::endl;
            std::cout << "--------------------------------------------------------------------" << std::endl;

            // Controller 2
            double kx = 5, ky = 3, kz = 3, max_vel = 1;
            Eigen::DiagonalMatrix<double, 3> k(kx, ky, kz);
            Eigen::Vector3d phi_matrix = (k * e_matrix).array().tanh() * (1 - (- (k * e_matrix).array().square()).exp());

            double v_x = -max_vel * phi_matrix[0];
            double v_y = -max_vel * phi_matrix[1];
            double v_z = -max_vel * phi_matrix[2];
            control_input_.push_back({v_x, v_y, v_z});

            vel_msg_.twist.linear.x = v_x;
            vel_msg_.twist.linear.y = v_y;
            vel_msg_.twist.linear.z = v_z;
            vel_pub_.publish(vel_msg_);

            rate.sleep();
        }
    }

    if (!offboard_mode_set_ && current_state_.mode != "AUTO.LOITER") {
        mavros_msgs::SetMode loiter_set_mode;
        loiter_set_mode.request.custom_mode = "AUTO.LOITER";
        if (set_mode_client_.call(loiter_set_mode) && loiter_set_mode.response.mode_sent) {
            ROS_INFO("AUTO.LOITER mode set successfully.");
            offboard_mode_set_ = true;
        } else {
            ROS_WARN("Failed to set AUTO.LOITER mode.");
        }
    }

    ROS_INFO("AUTO.LOITER mode set. UAV on standby.");

    spinOnce(); 
    rate.sleep();
}
