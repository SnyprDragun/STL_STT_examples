#include "dd_stl_stt/phasespace_mocap.hpp"

phasespace_msgs::Rigids mocap_state;

Mocap::Mocap(){
    string phasespace_mocap_sub_topic = "/phasespace/rigids";
    this->phasespace_mocap_sub = this->nh.subscribe(phasespace_mocap_sub_topic, 10, &Mocap::phasespace_mocap_cb, this);

    string mavros_mocap_pub_topic = "/mavros/fake_gps/mocap/tf";
    this->mavros_mocap_pub = this->nh.advertise<geometry_msgs::TransformStamped>(mavros_mocap_pub_topic, 10);
}

void Mocap::phasespace_mocap_cb(const phasespace_msgs::Rigids&){
    mocap_state = msg;
}

void Mocap::init_connection(){
    Rate rate(20);

    ROS_INFO("Connecting to FCT...");
    while(ok() && mocap_state.connected){
        ROS_INFO("Initializing mocap_node...");
        spinOnce();
        rate.sleep();
        break;
    }
    ROS_INFO("Connected!");
}

void Mocap::mocap_to_mavros_transform(){
    Rate rate(20);

    geometry_msgs::TransformStamped mavros_mocap_msg;

    if (mocap_state.rigids.id == 1){
        mavros_mocap_msg.transform.translation.x = mocap_state.rigids.x;
        mavros_mocap_msg.transform.translation.y = mocap_state.rigids.y;
        mavros_mocap_msg.transform.translation.z = mocap_state.rigids.z;

        mavros_mocap_msg.transform.rotation.x = mocap_state.rigids.qx;
        mavros_mocap_msg.transform.rotation.y = mocap_state.rigids.qy;
        mavros_mocap_msg.transform.rotation.z = mocap_state.rigids.qz;
        mavros_mocap_msg.transform.rotation.w = mocap_state.rigids.qw;
    }
    mavros_mocap_pub.publish(mavros_mocap_msg);

    spinOnce();
    rate.sleep();
}

int main(int argc, char **argv){
    init(argc, argv, "mocap_to_mavros_transform", init_options::AnonymousName);
    
    ROS_INFO("Initializing Mocap Transform node...");

    Mocap* mocap = new Mocap();
    mocap->init_connection();
    mocap->mocap_to_mavros_transform();

    spin();
    return 0;
}