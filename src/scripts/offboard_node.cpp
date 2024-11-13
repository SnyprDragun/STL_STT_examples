#include "dd_stl_stt/offboard_node.hpp"
#include "dd_stl_stt/STT_Controller.hpp"

mavros_msgs::State current_state_offboard;
geometry_msgs::PoseStamped curve;

Offboard::Offboard(){
    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Offboard::state_cb, this);
    
    string position_pub_topic = "/mavros/setpoint_position/local";
    this->position_pub = this->nh.advertise<geometry_msgs::PoseStamped>(position_pub_topic, 10);

    string vel_pub_topic = "/mavros/setpoint_velocity/cmd_vel";
    this->velocity_pub = this->nh.advertise<geometry_msgs::TwistStamped>(vel_pub_topic, 10);

    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);

    string waypoint_push_client_topic = "mavros/mission/push";
    this-> waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>(waypoint_push_client_topic);
}

void Offboard::state_cb(const mavros_msgs::State& msg){
    current_state_offboard = msg;
}

void Offboard::init_connection(){
    Rate rate(20);

    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state_offboard.connected){
        ROS_INFO("Initializing offboard_node...");
        spinOnce();
        rate.sleep();
        break;
    }
    ROS_INFO("Connected!");
}

void Offboard::offboard(double latitude, double longitude, double altitude){
    Rate rate(20.0);

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = latitude;
    pose.pose.position.y = longitude;
    pose.pose.position.z = altitude;

    for(int i = 100; ok() && i > 0; --i){
        position_pub.publish(pose);
        spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    Time last_request = Time::now();

    while(ok()){
        if( current_state_offboard.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Preparing to move...");
            }
            last_request = Time::now();
        }

        position_pub.publish(pose);
        if(pose.pose.position.x == latitude && (Time::now() - last_request > Duration(5.0))){
            ROS_INFO("Setpoint Reached!");
            offb_set_mode.request.custom_mode = "AUTO.LOITER";
            while(ok()){
                if( current_state_offboard.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Standby");
                        break;
                    }
                    last_request = Time::now();
                }
            }
            break;
        }
        spinOnce();
        rate.sleep();
    }
}

void Offboard::panorama(){
    Rate rate(20.0);

    geometry_msgs::TwistStamped pose;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    Time last_request, t = Time::now();

    while(ok()){
        if( current_state_offboard.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Preparing to take panorama...");
            }
            last_request = Time::now();
        }

        if(Time::now() - t < Duration(30.0)){
            pose.twist.angular.z = 0.2;
            velocity_pub.publish(pose);
        }
        else{
            pose.twist.angular.z = 0.0;
            velocity_pub.publish(pose);
            ROS_INFO("Panorama taken");
            offb_set_mode.request.custom_mode = "AUTO.LOITER";
            while(ok()){
                if( current_state_offboard.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Standby");
                        break;
                    }
                    last_request = Time::now();
                }
            }
            break;
        }
        spinOnce(); 
        rate.sleep();
    }
}

void Offboard::follow_stt(int degree_, int dimension_, const vector<vector<double>>& C_, double start_, double end_, double step_){
    Rate rate(20.0);

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.5;

    for(int i = 100; ok() && i > 0; --i){
        position_pub.publish(pose);
        spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    Time last_request = Time::now();

    int count = 0;

    while(ok()){
        if(current_state_offboard.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD mode set!");
            }
            last_request = Time::now();
        }

        position_pub.publish(pose);

        if(current_state_offboard.mode == "OFFBOARD"){
            ROS_INFO("Controller starting...");
            Controller* controller = new Controller(degree_, dimension_, C_, start_, end_, step_);
            controller->init_connection();
            controller->controller();

            offb_set_mode.request.custom_mode = "AUTO.LOITER";
            while(ok()){
                if(current_state_offboard.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
                    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Standby");
                        break;
                    }
                    last_request = Time::now();
                }
            }
            break;
        }

        spinOnce();
        rate.sleep();
    }
}

void Offboard::mission(){
    Rate rate(20.0);

    geometry_msgs::PoseStamped pose;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    Time last_request = Time::now();

    while(ok()){
        if( current_state_offboard.mode != "AUTO.MISSION" && (Time::now() - last_request > Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Mission mode activated.");
                Duration(3.0).sleep();
            }
            last_request = Time::now();
        }
        ROS_INFO("Ping");
        spinOnce();
        rate.sleep();
    }
}



// void Offboard::sendWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints) {
//     mavros_msgs::WaypointPush waypoint_push_srv;
    
//     // Convert waypoints to MAVROS waypoints format
//     for (const auto& pose : waypoints) {
//         mavros_msgs::Waypoint wp;
//         wp.x_lat = pose.pose.position.x;
//         wp.y_long = pose.pose.position.y;
//         wp.z_alt = pose.pose.position.z;
//         wp.frame = mavros_msgs::Waypoint::FRAME_LOCAL_NED;
//         // wp.command = mavros_msgs::Waypoint::NAV_WAYPOINT;
//         wp.is_current = false;
//         wp.autocontinue = true;
//         waypoint_push_srv.request.waypoints.push_back(wp);
//     }

//     if (waypoint_push_client.call(waypoint_push_srv)) {
//         ROS_INFO("Waypoints sent successfully.");
//     } else {
//         ROS_ERROR("Failed to send waypoints.");
//     }
// }

// void Offboard::mission(){
//     Rate rate(20.0);

//     // Define the flight mode change request
//     mavros_msgs::SetMode set_mode_srv;
//     set_mode_srv.request.custom_mode = "AUTO.MISSION"; // Set mode to mission

//     // Define waypoints
//     std::vector<geometry_msgs::PoseStamped> waypoints;
    
//     // Example waypoints (you need to replace these with actual coordinates)
//     geometry_msgs::PoseStamped waypoint1;
//     waypoint1.pose.position.x = 10.0;   // Latitude
//     waypoint1.pose.position.y = 5.0;    // Longitude
//     waypoint1.pose.position.z = -10.0;  // Altitude
//     waypoints.push_back(waypoint1);
    
//     geometry_msgs::PoseStamped waypoint2;
//     waypoint2.pose.position.x = 20;     // Latitude
//     waypoint2.pose.position.y = 10;     // Longitude
//     waypoint2.pose.position.z = -15;    // Altitude
//     waypoints.push_back(waypoint2);
    
//     geometry_msgs::PoseStamped waypoint3;
//     waypoint3.pose.position.x = 30;     // Latitude
//     waypoint3.pose.position.y = 15;     // Longitude
//     waypoint3.pose.position.z = -20;    // Altitude
//     waypoints.push_back(waypoint3);

//     Time last_request, t = Time::now();
    
//     // Ensure the drone is in the correct mode
//     while(ok()){
//         if( current_state_offboard.mode != "AUTO.MISSION" && (Time::now() - last_request > Duration(5.0))){
//             if( set_mode_client.call(set_mode_srv) && set_mode_srv.response.mode_sent){
//                 ROS_INFO("Mode set to AUTO.MISSION.");
//             }
//             last_request = Time::now();
//         }
    
//         else {
//             ROS_ERROR("Failed to set mode.");
//         }
//     }

//     // Send waypoints to the drone
//     sendWaypoints(waypoints);

//     // Wait for each waypoint to be reached
//     for (size_t i = 0; i < waypoints.size(); ++i) {
//         std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait for 1 second before sending the next waypoint
//     }

//     spinOnce(); 
//     rate.sleep();; // Keep the node running
// }



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

// void Offboard::follow_stt(){
//     Rate rate(20.0);

//     std::vector<double> x_u;
//     std::vector<double> x_l;
//     std::vector<double> y_u;
//     std::vector<double> y_l;
//     std::vector<double> z_u;
//     std::vector<double> z_l;

//     for (double time=0; time<61; time+=1){
//         x_u.push_back(gamma(time)[0]);
//         x_l.push_back(gamma(time)[1]);
//         y_u.push_back(gamma(time)[2]);
//         y_l.push_back(gamma(time)[3]);
//         z_u.push_back(gamma(time)[4]);
//         z_l.push_back(gamma(time)[5]);
//     }

//     int time = 0;
//     while (time <= 60){
//         offboard((x_u[time] + x_l[time])/2, (y_u[time] + y_l[time])/2, (z_u[time] + z_l[time])/2);
//         ROS_INFO("Here");
//         time++;
//     }

//     spinOnce(); 
//     rate.sleep();
// }
