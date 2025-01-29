#include "Motor_Cluster.h"

Motor_Cluster::Motor_Cluster(ros::NodeHandle &nh, Body_Part body_part)
{
    this->body_part = body_part;
    subscriber = nh.subscribe<robot_controller::controller_state>(
    "/controller_metadata", 1,
    [this](const boost::shared_ptr<const robot_controller::controller_state>& msg) {
        this->read_controller_data(*msg);  // Dereference to access the message
    });
}

void Motor_Cluster::update_motor(int motor_id)
{
    //TODO This needs to be modified, the current methodoloy is not great for controlling the motor 

    const auto change_in_position = 500; 
    
    auto motor_pair = motors[motor_id];
    auto motor = motor_pair.first;
    auto controller_key = motor_pair.second;
    auto controller_value = controller_keys[controller_key];
    auto goal_position = motor.get_present_position();
    if (controller_value>0){
        goal_position += change_in_position;
    } else if (controller_value<0){
        goal_position -= change_in_position;
    }

 
    motor.set_goal_position(goal_position); 

    // ROS_INFO("Motor %d present position: %d", motor_id, motor.get_present_position());
    
    motor.publish_motor_data();
}

void Motor_Cluster::add_motor(Motor_Controller motor, std::string controller_key)
{
    auto pair = std::make_pair(motor, controller_key);
    motors[motor.get_id()] = pair;
}

void Motor_Cluster::read_controller_data(const robot_controller::controller_state &msg)
{
    std::set<std::string> keys_of_interest;

    switch (body_part) {
        case Body_Part::LEFT_ARM:
            keys_of_interest = {"L2", "Left_Stick_Y"};
            break;
        case Body_Part::RIGHT_ARM:
            keys_of_interest = {"R2"};
            break;
        case Body_Part::LEFT_LEG:
            keys_of_interest = {"Left_Stick_X", "Left_Stick_Y"};
            break;
        case Body_Part::RIGHT_LEG:
            keys_of_interest = {"Right_Stick_X", "Right_Stick_Y"};
            break;
    }

    for (const auto &axis : msg.axes) {
        std::string key = axis.key.c_str();
        if (keys_of_interest.find(key) != keys_of_interest.end()) {
            controller_keys[key] = axis.value;
            ROS_INFO("Axis key: %s, value: %f", key.c_str(), axis.value);
        }
    }
}

