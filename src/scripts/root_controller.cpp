#include "dd_stl_stt/takeoff_node.hpp"
#include "dd_stl_stt/offboard_node.hpp"
#include "dd_stl_stt/landing_node.hpp"

//------------------ OR CASE FIRST BLOCK ------------------//
const double C0 = -0.5092201124440394;
const double C1 = -1.665740011109661;
const double C2 = 1.542484516723121;
const double C3 = -0.3243109601139466;
const double C4 = 0.026244366716944464;
const double C5 = -0.0007154516946045534;
const double C6 = -0.5092201124440394;
const double C7 = -0.9354351139789893;
const double C8 = 0.8798053580061804;
const double C9 = -0.15421328533867962;
const double C10 = 0.011825144550615181;
const double C11 = -0.00032995786104859844;
const double C12 = 1.4907798875559606;
const double C13 = -0.14819509743468093;
const double C14 = 0.32437796115886813;
const double C15 = -0.058556349964849215;
const double C16 = 0.0052465859553120706;
const double C17 = -0.0001699023672777245;
const double C18 = 1.9953899437779803;
const double C19 = -1.6883828104673564;
const double C20 = 1.5955854133448955;
const double C21 = -0.34080413875938786;
const double C22 = 0.02799927661790214;
const double C23 = -0.0007749401658234577;
const double C24 = 1.9953899437779803;
const double C25 = -0.9354351139789893;
const double C26 = 0.8798053580061804;
const double C27 = -0.15421328533867962;
const double C28 = 0.011825144550615181;
const double C29 = -0.00032995786104859844;
const double C30 = 3.99538994377798;
const double C31 = -0.14819509743468093;
const double C32 = 0.32437796115886813;
const double C33 = -0.058556349964849215;
const double C34 = 0.0052465859553120706;
const double C35 = -0.0001699023672777245;

//------------------ OR CASE SECOND BLOCK -----------------//
// const double C0 = -0.9947193621508238;
// const double C1 = 0.4277863285546202;
// const double C2 = 1.2140297298487956;
// const double C3 = -0.27679682188266586;
// const double C4 = 0.02200792560735738;
// const double C5 = -0.0005888928692527835;
// const double C6 = -0.5105612756983524;
// const double C7 = -1.4667281217292212;
// const double C8 = 1.0843122115171406;
// const double C9 = -0.18152804910001696;
// const double C10 = 0.013280466284529367;
// const double C11 = -0.0003552471176615102;
// const double C12 = 1.4894387243016476;
// const double C13 = -1.27618875247245;
// const double C14 = 0.655150320834794;
// const double C15 = -0.08099965733727106;
// const double C16 = 0.004422992408222249;
// const double C17 = -8.864906559813607e-05;
// const double C18 = 1.5105612756983524;
// const double C19 = 0.4277863285546202;
// const double C20 = 1.2140297298487956;
// const double C21 = -0.27679682188266586;
// const double C22 = 0.02200792560735738;
// const double C23 = -0.0005888928692527835;
// const double C24 = 1.994719362150824;
// const double C25 = -1.4667281217292212;
// const double C26 = 1.0843122115171406;
// const double C27 = -0.18152804910001696;
// const double C28 = 0.013280466284529367;
// const double C29 = -0.0003552471176615102;
// const double C30 = 3.994719362150824;
// const double C31 = -1.27618875247245;
// const double C32 = 0.655150320834794;
// const double C33 = -0.08099965733727106;
// const double C34 = 0.004422992408222249;
// const double C35 = -8.864906559813607e-05;

const vector<vector<double>> _C_ = {
    {C0, C1, C2, C3, C4, C5}, 
    {C6, C7, C8, C9, C10, C11}, 
    {C12, C13, C14, C15, C16, C17}, 
    {C18, C19, C20, C21, C22, C23}, 
    {C24, C25, C26, C27, C28, C29}, 
    {C30, C31, C32, C33, C34, C35}
};

int _degree_ = 5;
int _dimension_ = 3;
double _start_ = 0.0;
double _end_ = 15.0;
double _step_ = 0.1;

int main(int argc, char **argv){
    init(argc, argv, "root_controller", init_options::AnonymousName);
    
    ROS_INFO("Initializing Root Controller...");

    float altitude = 1.0;

    Takeoff* takeoff = new Takeoff();
    takeoff->init_connection();
    takeoff->arm();
    takeoff->takeoff(altitude);
    Duration(10).sleep();

    Offboard* offboard = new Offboard();
    offboard->init_connection();
    offboard->follow_stt(_degree_, _dimension_, _C_, _start_, _end_, _step_);

    Duration(10).sleep();
    Land* land = new Land();
    land->init_connection();
    land->land();

    spin();
    return 0;
}
