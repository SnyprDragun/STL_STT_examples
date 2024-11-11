#include "dd_stl_stt/STT_Controller.hpp"

mavros_msgs::SetMode set_mode;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;
geometry_msgs::TwistStamped velocity_pub_msg;

Controller::Controller(int degree, int dimension, const vector<vector<double>>& C, double start, double end, double step)
    : degree(degree), dimension(dimension), start(start), end(end), step(step) {
    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Controller::state_cb, this);

    string position_sub_topic = "/mavros/local_position/pose";
    this->position_sub = this->nh.subscribe(position_sub_topic, 10, &Controller::position_cb, this);
    
    string position_pub_topic = "/mavros/setpoint_position/local";
    this->position_pub = this->nh.advertise<geometry_msgs::PoseStamped>(position_pub_topic, 10);

    string vel_pub_topic = "/mavros/setpoint_velocity/cmd_vel";
    this->velocity_pub = this->nh.advertise<geometry_msgs::TwistStamped>(vel_pub_topic, 10);

    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);

    this->C = MatrixXf::Zero(2 * dimension, degree + 1);
    for (int i = 0; i < 2 * dimension; ++i) {
        for (int j = 0; j <= degree; ++j) {
            this->C(i, j) = C[i][j];
        }
    }
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

VectorXf Controller::gamma(double t) {
    VectorXf powers_of_t(degree + 1);
    for (int j = 0; j <= degree; ++j) {
        powers_of_t(j) = pow(t, j);
    }

    VectorXf real_tubes = C * powers_of_t;
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

    // while (ok() && !current_state.armed) {
    //     ROS_INFO("Waiting for drone to be armed...");
    //     Duration(1.0).sleep();
    // }

    velocity_pub_msg.twist.linear.x = velocity_pub_msg.twist.linear.y = velocity_pub_msg.twist.linear.z = 0;
    velocity_pub.publish(velocity_pub_msg);
    rate.sleep();

    if (!offboard_mode_set && current_state.mode != "OFFBOARD") {
        set_mode.request.custom_mode = "OFFBOARD";
        if ((set_mode_client.call(set_mode)) && (set_mode.response.mode_sent)) {
            ROS_INFO("OFFBOARD mode set successfully.");
            offboard_mode_set = true;
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
            VectorXf gamma = this->gamma(t);

            gamma_u.push_back({gamma(3), gamma(4), gamma(5)});
            gamma_l.push_back({gamma(0), gamma(1), gamma(2)});

            double gamma_sx = gamma(3) + gamma(0);
            double gamma_dx = gamma(3) - gamma(0);
            double gamma_sy = gamma(4) + gamma(1);
            double gamma_dy = gamma(4) - gamma(1);
            double gamma_sz = gamma(5) + gamma(2);
            double gamma_dz = gamma(5) - gamma(2);

            trajectory.push_back({current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z});

            double e1 = normalized_error(current_position.pose.position.x, gamma_sx, gamma_dx);
            double e2 = normalized_error(current_position.pose.position.y, gamma_sy, gamma_dy);
            double e3 = normalized_error(current_position.pose.position.z, gamma_sz, gamma_dz);

            Vector3f e_matrix(e1, e2, e3);
            cout << "\ne_matrix: " << e_matrix.transpose() << " time: " << t << endl;
            cout << "current pose: " << current_position.pose.position.x << ", " << current_position.pose.position.y << ", " << current_position.pose.position.z << endl;
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

            Vector3f k(kx, ky, kz);
            Vector3f phi_matrix = (k.array() * e_matrix.array().tanh() * (1 - (-e_matrix.array().square()).exp())).matrix();

            double v_x = -max_vel * phi_matrix(0);
            double v_y = -max_vel * phi_matrix(1);
            double v_z = -max_vel * phi_matrix(2);
            control_input.push_back({v_x, v_y, v_z});
            //--------------------------------------------------------------------//

            velocity_pub_msg.twist.linear.x = v_x;
            velocity_pub_msg.twist.linear.y = v_y;
            velocity_pub_msg.twist.linear.z = v_z;
            velocity_pub.publish(velocity_pub_msg);

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
