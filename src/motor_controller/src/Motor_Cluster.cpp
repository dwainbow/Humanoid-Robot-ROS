#include "Motor_Cluster.h"

Motor_Cluster::Motor_Cluster(ros::NodeHandle &nh, Body_Part body_part)
{
    this->body_part = body_part;
    subscriber = nh.subscribe<robot_controller::controller_state>(
        "/controller_metadata", 1,
        [this](const boost::shared_ptr<const robot_controller::controller_state> &msg)
        {
            this->read_controller_data(*msg);
        });
}

void Motor_Cluster::update_motor(std::shared_ptr<Motor_Controller> motor, const std::string &controller_key)
{

    auto change_in_position = 500;
    auto controller_value = controller_keys[controller_key];
    auto goal_position = motor->get_present_position();
    if(motor->get_id() ==11){
        ROS_INFO("GOAL POSITION %d", goal_position);
        ROS_INFO("Controller value %f", controller_value);
        ROS_INFO("-----------------------------------------------");
    }
    if (controller_value > 0)
    {
        goal_position += change_in_position;
    }
    else if (controller_value < 0)
    {
        goal_position -= change_in_position;
    }
    if(motor->get_id() ==11){
        ROS_INFO("GOAL POSITION %d", goal_position);
        ROS_INFO("Controller value %f", controller_value);
        ROS_INFO("-----------------------------------------------");
    }
    motor->set_goal_position(goal_position);
    motor->write_goal_position();
    // exit(0);
}

void Motor_Cluster::add_motor(std::shared_ptr<Motor_Controller> motor, const std::string &controller_key)
{
    if (!motor)
    {
        ROS_ERROR("add_motor() received a null pointer!");
        return;
    }

    motors[motor->get_id()] = std::make_pair(motor, controller_key);
}

void Motor_Cluster::update_motors()
{
    for (const auto &motor_pair : motors)
    {
        auto motor = motor_pair.second.first;
        auto controller_key = motor_pair.second.second;
        if (motor->get_subscriber().getNumPublishers() > 0)
        {
            // ROS_INFO("Not updating motor %d", motor->get_id());
            continue;
        }

        update_motor(motor, controller_key);
    }
}

void Motor_Cluster::read_controller_data(const robot_controller::controller_state &msg)
{
    std::set<std::string> keys_of_interest;

    switch (body_part)
    {
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

    for (const auto &axis : msg.axes)
    {
        std::string key = axis.key.c_str();
        if (keys_of_interest.find(key) != keys_of_interest.end())
        {
            controller_keys[key] = axis.value;
        }
    }
}