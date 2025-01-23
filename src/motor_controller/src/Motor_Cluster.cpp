#include "Motor_Cluster.h"

Motor_Cluster::Motor_Cluster(ros::NodeHandle &nh, Body_Part body_part)
{
    this->body_part = body_part;
    subscriber = nh.subscribe<robot_controller::controller_state>(
    "/controller_metadata", 10,
    [this](const boost::shared_ptr<const robot_controller::controller_state>& msg) {
        this->read_controller_data(*msg);  // Dereference to access the message
    });
}

void Motor_Cluster::update_motor(int motor_id, float position_scaling_factor, float velocity_scaling_factor)
{

}

void Motor_Cluster::add_motor(Motor_Controller motor)
{
    motors[motor.get_id()] = motor;
}

void Motor_Cluster::read_controller_data(const robot_controller::controller_state &msg)
{

    //TODO: Maybe add filtering to make message reading for clusters more efficient
    switch (body_part){
        case Body_Part::LEFT_ARM: 
            for (const auto& axis : msg.axes) {
                auto key = axis.key.c_str();
                if (key == "L2"){
                    auto value = axis.value;
                    controller_keys[key] = value;
                    ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                }
            }
            break ;
        case Body_Part::RIGHT_ARM: 
            for (const auto& axis : msg.axes) {

                auto key = axis.key.c_str();
                if (key == "R2"){
                    auto value = axis.value;

                    controller_keys[key] = value;
                    ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                }
            }
            break ;
        case Body_Part::LEFT_LEG: 
            for (const auto& axis : msg.axes) {
                    auto key = axis.key.c_str();

                if (key == "Left_Stick_X" || key == "Left_Stick_Y"){
                    auto value = axis.value;

                    controller_keys[key] = value;
                    ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                }
            }
            break ;
        case Body_Part::RIGHT_LEG: 
            for (const auto& axis : msg.axes) {
                    auto key = axis.key.c_str();

                if (key == "Right_Stick_X" || key == "Right_Stick_Y"){
                    auto value = axis.value;

                    controller_keys[key] = value;
                    ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                }
            }
            break ;
    }    
}
