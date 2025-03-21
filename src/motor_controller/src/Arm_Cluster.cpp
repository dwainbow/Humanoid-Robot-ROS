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
//X is left-right, Y is up-down, Z is forward-back
double Arm_Cluster::get_elbow_Position(){
    double conversion = 4096/(2*M_PI);
    double angle1 = find_motor("shoulder1").get_present_position()*conversion;
    double angle2 = find_motor("shoulder2").get_present_position()*conversion;
    double angle3 = find_motor("shoulder3").get_present_position()*conversion;
    std::vector currentAngles(3);
    std::vector currentPosition(3);
    currentAngles[1] = angle1;
    for (int i = 0; i < 3; i++){
        currentPosition[i] += 2*cos(currentAngles[i]);
    }
    currentAngles[
    

}
double Arm_Cluster::get_elbow_Y(){
    double conversion = 4096/(2*M_PI);
    double alpha = find_motor("shoulder1").get_present_position()*conversion;
    double beta = find_motor("shoulder2").get_present_position()*conversion;
    double gamma = find_motor("shoulder3").get_present_position()*conversion;

}
double Arm_Cluster::get_elbow_Z(){

}
double Arm_Cluster::get_hand_X(){

}
double Arm_Cluster::get_hand_Y(){

}
double Arm_Cluster::get_hand_Z(){

}
Motor_Controller Arm_Cluster::find_motor(const std:: string &motor_name){
    auto it = motors.find(motor_name);
    if (it != motors.end()){
        return it->second;
    }   else{
        throw std::runtime_error("Motor not found: " + motor_name);
    }
}