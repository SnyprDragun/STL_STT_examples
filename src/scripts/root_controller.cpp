#include "dd_stl_stt/takeoff_node.hpp"
#include "dd_stl_stt/offboard_node.hpp"
#include "dd_stl_stt/landing_node.hpp"

mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;

int main(int argc, char **argv){
    init(argc, argv, "root_controller", init_options::AnonymousName);
    
    ROS_INFO("Initializing Root Controller...");

    float altitude = 1.0;

    Takeoff* takeoff = new Takeoff();
    takeoff->init_connection();
    takeoff->arm();
    takeoff->takeoff(altitude);

    Time last_request = Time::now();

    bool flag = true;
    while (flag){
        if (current_state.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
            ROS_INFO("Waiting to complete takeoff...");
        }
        else{
            ROS_INFO("Takeoff Completed Successfully!");
            Duration(10).sleep();

            Offboard* offboard = new Offboard();
            offboard->init_connection();
            offboard->offboard(2, 2, 2);
            ROS_INFO("This Done");
            // offboard->panorama();
            // offboard->mission();
            offboard->mission();

            flag = false;
        }
    }
    // Duration(10).sleep();
    // Land* land = new Land();
    // land->init_connection();
    // land->land();

    spin();
    return 0;
}


// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/State.h>

// ros::Publisher local_pos_pub;
// ros::ServiceClient set_mode_client;
// ros::ServiceClient arming_client;
// mavros_msgs::State current_state;

// void state_cb(const mavros_msgs::State::ConstPtr& msg) {
//     current_state = *msg;
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "mavros_offboard_control");
//     ros::NodeHandle nh;

//     // Initialize publisher and service clients
//     local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//     set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
//     arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

//     ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);

//     // Wait for the FCU connection
//     ROS_INFO("Waiting for FCU connection...");
//     ros::Rate rate(10.0);
//     while (ros::ok() && !current_state.connected) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     // Set mode to OFFBOARD
//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
//         ROS_INFO("Offboard mode enabled");
//     } else {
//         ROS_ERROR("Failed to enable offboard mode");
//         return -1;
//     }

//     // Arm the drone
//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
//         ROS_INFO("Vehicle armed");
//     } else {
//         ROS_ERROR("Failed to arm vehicle");
//         return -1;
//     }

//     // Send a position setpoint
//     geometry_msgs::PoseStamped pose;
//     pose.header.frame_id = "map";
//     pose.header.stamp = ros::Time::now();
//     pose.pose.position.x = 2.0;
//     pose.pose.position.y = 2.0;
//     pose.pose.position.z = 2.0;
//     pose.pose.orientation.w = 1.0;

//     ros::Time last_request = ros::Time::now();
//     while (ros::ok()) {
//         // Send the setpoint
//         local_pos_pub.publish(pose);

//         // Keep the mode as OFFBOARD
//         if (current_state.mode != "OFFBOARD") {
//             if ((ros::Time::now() - last_request) > ros::Duration(5.0)) {
//                 if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
//                     ROS_INFO("Offboard mode enabled");
//                 } else {
//                     ROS_ERROR("Failed to enable offboard mode");
//                     return -1;
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

