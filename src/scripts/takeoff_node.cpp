#include "dd_stl_stt/takeoff_node.hpp"

mavros_msgs::State current_state_takeoff;

Takeoff::Takeoff(){
    string state_sub_topic = "/mavros/state";
    this->state_sub = this->nh.subscribe(state_sub_topic, 10, &Takeoff::state_cb, this);

    string takeoff_client_topic = "/mavros/cmd/takeoff";
    this->takeoff_client = this->nh.serviceClient<mavros_msgs::CommandTOL>(takeoff_client_topic);

    string arming_client_topic = "/mavros/cmd/arming";
    this->arming_client = this->nh.serviceClient<mavros_msgs::CommandBool>(arming_client_topic);
}

void Takeoff::state_cb(const mavros_msgs::State& msg){
    current_state_takeoff = msg;
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

    Time last_request = Time::now();

    bool flag_1 = true;
    while(flag_1){
        if(current_state_takeoff.mode != "AUTO.TAKEOFF" && (Time::now() - last_request > Duration(5.0))){
            if(this->takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
                ROS_INFO("Taking Off");
                flag_1 = false;
            }
            last_request = Time::now();
        } 
        spinOnce();
        rate.sleep();
    }

    bool flag_2 = true;
    while (flag_2){
        if (current_state_takeoff.mode != "AUTO.LOITER" && (Time::now() - last_request > Duration(5.0))){
            ROS_INFO("Waiting to complete takeoff...");
        }
        else{
            ROS_INFO("Takeoff Completed Successfully!");
            flag_2 = false;
        }
        last_request = Time::now();
    }
}