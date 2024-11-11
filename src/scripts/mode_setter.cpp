#include "dd_stl_stt/mode_setter.hpp"

mavros_msgs::SetMode custom_mode_setter;
mavros_msgs::State setter_state;

Setter::Setter(){
    string setter_state_sub_topic = "/mavros/state";
    this->setter_state_sub = this->nh.subscribe(setter_state_sub_topic, 10, &Setter::state_cb, this);

    string mode_setter_client_topic = "/mavros/set_mode";
    this->mode_setter_client = this->nh.serviceClient<mavros_msgs::SetMode>(mode_setter_client_topic);
}

void Setter::state_cb(const mavros_msgs::State& msg){
    setter_state = msg;
}

void Setter::OFFBOARD(){
    Rate rate(20.0);

    mavros_msgs::SetMode custom_mode_setter;
    custom_mode_setter.request.custom_mode = "OFFBOARD";

    Time last_request = Time::now();

    while(ok()){
        if( setter_state.mode != "OFFBOARD" && (Time::now() - last_request > Duration(5.0))){
            if( mode_setter_client.call(custom_mode_setter) && custom_mode_setter.response.mode_sent){
                ROS_INFO("OFFBOARD mode set");
            }
            last_request = Time::now();
        }

        spinOnce();
        rate.sleep();
    }
}

void Setter::LOITER(){

}

void Setter::STABILIZED(){

}
