#include "dd_stl_stt/STT_Controller.hpp"

mavros_msgs::SetMode set_mode;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;
geometry_msgs::TwistStamped velocity_pub_msg;

Controller::Controller(int degree, int dimension, const std::vector<std::vector<float>>& C): degree(degree), dimension(dimension) {
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

    start = 0; 
    end = 100; 
    step = 0.1;
    C = tensor(C, kFloat32).view({2 * dimension_, degree_ + 1});
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

Tensor Controller::gamma(double t) {
    Tensor t_tensor = tensor({t}, kFloat32);
    Tensor powers_of_t = empty({degree + 1}, kFloat32);

    for (int j = 0; j <= degree; ++j) {
        powers_of_t[j] = pow(t_tensor, j);
    }

    Tensor real_tubes = matmul(C, powers_of_t);
    return real_tubes;
}

double Controller::normalized_error(double x, double gamma_sum, double gamma_diff) {
    return ((2.0 * x - gamma_sum) / gamma_diff);
}

void Controller::controller(){
    Rate rate(500);

    vector<double> t_values;
    for (double t = start; t <= end; t += step) {
        t_values.push_back(t);
    }

    while (ok() && !current_state.armed) {
        ROS_INFO("Waiting for drone to be armed...");
        Duration(1.0).sleep();
    }

    velocity_pub_msg.twist.linear.x = velocity_pub_msg.twist.linear.y = velocity_pub_msg.twist.linear.z = 0;
    velocity_pub.publish(velocity_pub_msg);
    rate.sleep();

    if (!offboard_mode_set && current_state.mode != "OFFBOARD") {
        set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.callset_mode) && (set_mode.response.mode_sent) {
            ROS_INFO("OFFBOARD mode set successfully.");
            offboard_mode_set_ = true;
        } else {
            ROS_WARN("Failed to set OFFBOARD mode.");
        }
    }

    ROS_INFO("UAV ready for velocity control.");

    int max_iterations = 1;
    int count = 0;
    while (ok() && count < max_iterations) {
        count++;

        for (double t : t_values) {
            Duration(step).sleep();
            Tensor gamma = gamma(t);

            gamma_u.push_back({gamma[3].item<double>(), gamma[4].item<double>(), gamma[5].item<double>()});
            gamma_l.push_back({gamma[0].item<double>(), gamma[1].item<double>(), gamma[2].item<double>()});

            double gamma_sx = gamma[3].item<double>() + gamma[0].item<double>();
            double gamma_dx = gamma[3].item<double>() - gamma[0].item<double>();
            double gamma_sy = gamma[4].item<double>() + gamma[1].item<double>();
            double gamma_dy = gamma[4].item<double>() - gamma[1].item<double>();
            double gamma_sz = gamma[5].item<double>() + gamma[2].item<double>();
            double gamma_dz = gamma[5].item<double>() - gamma[2].item<double>();

            trajectory.push_back({current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z});

            double e1 = normalized_error(current_pose.pose.position.x, gamma_sx, gamma_dx);
            double e2 = normalized_error(current_pose.pose.position.y, gamma_sy, gamma_dy);
            double e3 = normalized_error(current_pose.pose.position.z, gamma_sz, gamma_dz);

            Tensor e_matrix = tensor({e1, e2, e3}, kFloat32);
            cout << "\ne_matrix: " << e_matrix << " time: " << t << endl;
            cout << "current pose: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << endl;
            cout << "target pose: " << gamma_sx / 2 << ", " << gamma_sy / 2 << ", " << gamma_sz / 2 << endl;
            cout << "--------------------------------------------------------------------" << endl;

            //--------------------------- CONTROLLER 1 ---------------------------//
            //--------------------------------------------------------------------//

            //--------------------------- CONTROLLER 2 ---------------------------//

            //----- block 1 -----//
            double kx = 5, ky = 3, kz = 3, max_vel = 1;
            //-------------------//

            //----- block 2 -----//
            // double kx = 7, ky = 3, kz = 3, max_vel = 2;
            //-------------------//

            Tensor k = tensor({kx, ky, kz}, kFloat32).diag();
            Tensor phi_matrix = tanh(k.matmul(e_matrix)) * (1 - exp(-pow(k.matmul(e_matrix), 2)));

            double v_x = -max_vel * phi_matrix[0].item<double>();
            double v_y = -max_vel * phi_matrix[1].item<double>();
            double v_z = -max_vel * phi_matrix[2].item<double>();
            control_input_.push_back({v_x, v_y, v_z});
            //--------------------------------------------------------------------//

            velocity_pub_msg.twist.linear.x = v_x;
            velocity_pub_msg.twist.linear.y = v_y;
            velocity_pub_msg.twist.linear.z = v_z;
            vel_pub.publish(velocity_pub_msg);

            rate.sleep();
        }
    }

    if (!loiter_mode_set && current_state.mode != "AUTO.LOITER") {
        set_mode.request.custom_mode = "AUTO.LOITER";
        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
            ROS_INFO("AUTO.LOITER mode set successfully.");
            loiter_mode_set = true;
        } else {
            ROS_WARN("Failed to set AUTO.LOITER mode.");
        }
    }

    ROS_INFO("UAV on standby.");

    spinOnce(); 
    rate.sleep();
}
