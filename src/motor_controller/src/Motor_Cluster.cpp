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

void Motor_Cluster::update_motor(int motor_id)
{
    //TODO This needs to be modified, the current methodoloy is not great for controlling the motor 
    auto motor = motors[motor_id];
    float value;

    switch(body_part){
    case Body_Part::LEFT_ARM: 
        value = controller_keys["L2"];
        break ;
    
    case Body_Part::RIGHT_ARM:

        value = controller_keys["R2"];
        break ;
    
    case Body_Part::LEFT_LEG:
        value = controller_keys["Left_Stick_X"];
        value = controller_keys["Left_Stick_Y"];
        break ;
    
    case Body_Part::RIGHT_LEG:
        value = controller_keys["Right_Stick_X"];
        value = controller_keys["Right_Stick_Y"];
        break ;
    }

    if (value == 1.0){
        return;
    }
    int new_position_value = motor.get_present_position() + 100*value;
    motor.set_goal_position(new_position_value);

    ROS_INFO("Motor %d present position: %d", motor_id, motor.get_present_position());
    
    motor.publish_motor_data();
}

void Motor_Cluster::add_motor(Motor_Controller motor)
{
    motors[motor.get_id()] = motor;
}

void Motor_Cluster::read_controller_data(const robot_controller::controller_state &msg)
{
    switch (body_part){
        case Body_Part::LEFT_ARM: 
            for (const auto& axis : msg.axes) {
                std::string key = axis.key.c_str();
               
                if (key == "L2"){
                    auto value = axis.value;
                    controller_keys[key] = value;
                    // ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                    break ;
                }
            }
            
            break ;
        case Body_Part::RIGHT_ARM: 
            for (const auto& axis : msg.axes) {
                std::string key = axis.key.c_str();

                if (key == "R2"){
                    auto value = axis.value;

                    controller_keys[key] = value;
                    //ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                    break ;
                }
            }
            break ;
        case Body_Part::LEFT_LEG: 
            for (const auto& axis : msg.axes) {
                std::string key = axis.key.c_str();

                if (key == "Left_Stick_X" || key == "Left_Stick_Y"){
                    auto value = axis.value;

                    controller_keys[key] = value;
                    // ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                    break ;
                }
            }
            break ;
        case Body_Part::RIGHT_LEG: 
            for (const auto& axis : msg.axes) {
                    std::string key = axis.key.c_str();

                if (key == "Right_Stick_X" || key == "Right_Stick_Y"){
                    auto value = axis.value;

                    controller_keys[key] = value;
                    //ROS_INFO("Axis key: %s, value: %f", axis.key.c_str(), axis.value);
                    break ;
                }
            }
            break ;
    }    
}
