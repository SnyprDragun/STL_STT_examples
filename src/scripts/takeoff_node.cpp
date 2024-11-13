#include "dd_stl_stt/takeoff_node.hpp"

mavros_msgs::State current_state_takeoff;
geometry_msgs::PoseStamped takeoff_position;

Takeoff::Takeoff(){
    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Takeoff::state_cb, this);

    string position_sub_topic = "/mavros/local_position/pose";
    this->position_sub = this->nh.subscribe(position_sub_topic, 10, &Takeoff::position_cb, this);

    string takeoff_client_topic = "/mavros/cmd/takeoff";
    this->takeoff_client = this->nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_client_topic);

    string arming_client_topic = "/mavros/cmd/arming";
    this->arming_client = this->nh.serviceClient<mavros_msgs::CommandBool>(arming_client_topic);

    string set_mode_client_topic = "/mavros/set_mode";
    this->set_mode_client = this->nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_topic);
}

void Takeoff::state_cb(const mavros_msgs::State& msg){
    current_state_takeoff = msg;
}

void Takeoff::position_cb(const geometry_msgs::PoseStamped& msg){
    takeoff_position = msg;
}

void Takeoff::init_connection(){
    Rate rate(20);
    ROS_INFO("Connecting to FCT...");
    while(ok() && current_state_takeoff.connected){
        ROS_INFO("Initializing takeoff_node...");
        spinOnce();
        rate.sleep();
        break;
    }
    ROS_INFO("Connected!");
}

void Takeoff::arm(){
    Rate rate(20);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    Time last_request = Time::now();

    bool flag = true;
    while(flag){
        if(!current_state_takeoff.armed && (Time::now() - last_request > Duration(5.0))){
            if(this->arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Arming Vehicle");
                flag = false;
            }
            last_request = Time::now();
        }
        spinOnce();
        rate.sleep();
    }
}

void Takeoff::takeoff(float altitude){
    Rate rate(20);
    
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.latitude = nan("1");
    takeoff_cmd.request.longitude = nan("1");
    takeoff_cmd.request.altitude = altitude;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.LOITER";

    Time last_request1 = Time::now();

    while(ok()){
        if(current_state_takeoff.mode != "AUTO.LOITER" && (Time::now() - last_request1 > Duration(5.0))){
            ROS_INFO("should not be here");
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                if(current_state_takeoff.mode == "AUTO.LOITER"){
                    ROS_INFO("Standby");
                    break;
                }
            }
            last_request1 = Time::now();
        }
        else{
            break;
        }
        spinOnce();
        rate.sleep();
    }

    Time last_request2 = Time::now();

    while(ok()){
        if(current_state_takeoff.mode != "AUTO.TAKEOFF" && (Time::now() - last_request2 > Duration(5.0))){
            if(this->takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
                if(current_state_takeoff.mode == "AUTO.TAKEOFF" && takeoff_position.pose.position.x > 1){
                    ROS_INFO("Taking Off");
                    break;
                }
                else{
                    arm();
                }
            }
            last_request2 = Time::now();
        }
        else if (current_state_takeoff.mode == "AUTO.TAKEOFF"){
            ROS_INFO("Takeoff Completed Successfully!");
            break;
        }
        else{
            ROS_INFO("Latest mode not suitable. Re-attempting takeoff...");
            arm();
        }
        spinOnce();
        rate.sleep();
    }
}