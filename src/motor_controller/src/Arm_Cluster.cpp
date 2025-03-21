#include "Arm_Cluster.h"

Arm_Cluster::Arm_Cluster(ros::NodeHandle &nh, Body_Part body_part)
{
    this->body_part = body_part;
    subscriber = nh.subscribe<robot_controller::controller_state>(
        "/controller_metadata", 1,
        [this](const boost::shared_ptr<const robot_controller::controller_state> &msg)
        {
            this->read_controller_data(*msg);
        });
}
void Arm_Cluster:add_motor(std::shared_ptr<Motor_Controller> motor, int motor_name)
{
    if (!motor)
    {
        ROS_ERROR("add_motor() received a null pointer!");
        return;
    }

    motors[motor->get_id()] = std::make_pair(motor, motor_name);
}

void Arm_Cluster::update_motor(std::shared_ptr<Motor_Controller> motor, const std::string &controller_key)
{
    auto change_in_position = 500;
    auto controller_value = controller_keys[controller_key];
    auto goal_position = motor->get_present_position();
    double x_goal = 0;  //left to right position of hand
    double y_goal = 0;
}

//Motors: shoulder1, shoulder2, shoulder3, elbow
double Arm_Cluster::get_elbow_X(){
    
}
double Arm_Cluster::get_elbow_Y(){

}
double Arm_Cluster::get_elbow_Z(){

}
double Arm_Cluster::get_hand_X(){

}
double Arm_Cluster::get_hand_Y(){

}
double Arm_Cluster::get_hand_Z(){

}