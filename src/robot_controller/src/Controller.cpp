#include "Controller.h"
#include <ros/ros.h>
Controller::Controller()
{
    this->buttons = {
        {"X", 0},
        {"Circle", 0},
        {"Triangle", 0},
        {"Square", 0},
        {"R1", 0},
        {"R3", 0},
        {"L3", 0},
        {"Share", 0},
        {"Options", 0},

    };

    this->axes  = { 
        {"Left_Stick_X", 0.0},
        {"Left_Stick_Y", 0.0},
        {"Right_Stick_X", 0.0},
        {"Right_Stick_Y", 0.0},
        {"DPad_X", 0.0},
        {"DPad_Y", 0.0},
        {"L2", 0.0},
        {"R2", 0.0},
    };
}

Controller::~Controller()
{
    //Leave empty for now 
}

double Controller::get_axis(const std::string &key)
{
    if (this->axes.find(key) != this->axes.end())
    {
        return this->axes[key];
    }
    return -1;
}

int Controller::get_button(const std::string &key)
{
    if (this->buttons.find(key) != this->buttons.end())
    {
        return this->buttons[key];
    }
    return -1;
}

void Controller::update(const sensor_msgs::Joy::ConstPtr& msg)
{
    axes["Left_Stick_X"] = msg->axes[0];
    axes["Left_Stick_Y"] = msg->axes[1];
    axes["L2"] = msg->axes[2];
    axes["Right_Stick_X"] = msg->axes[3];
    axes["Right_Stick_Y"] = msg->axes[4];
    axes["R2"] = msg->axes[5];
    axes["DPad_X"] = msg->axes[6];
    axes["DPad_Y"] = msg->axes[7];

    buttons["X"] = msg->buttons[0];
    buttons["Circle"] = msg->buttons[1];
    buttons["Square"] = msg->buttons[2];
    buttons["Triangle"] = msg->buttons[3];
    buttons["L1"] = msg->buttons[4];
    buttons["R1"] = msg->buttons[5];
    buttons["Share"] = msg->buttons[6];
    buttons["Options"] = msg->buttons[7];
    buttons["L3"] = msg->buttons[9];
    buttons["R3"] = msg->buttons[10];

    ROS_INFO("Axes: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
     axes["Left_Stick_X"], axes["Left_Stick_Y"], axes["L2"], axes["Right_Stick_X"], axes["Right_Stick_Y"], axes["R2"], axes["DPad_X"], axes["DPad_Y"],
        buttons["X"], buttons["Circle"], buttons["Square"], buttons["Triangle"], buttons["L1"], buttons["R1"], buttons["Share"], buttons["Options"], buttons["L3"], buttons["R3"] 
    );
}

void Controller::init_controller(ros::NodeHandle &nh)
{
    pub = nh.advertise<robot_controller::controller_state>("/controller_metadata", 10);
    sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, [this](const sensor_msgs::Joy::ConstPtr& msg) { //this is a lambda function to subsribe to the joy topic
    this->update(msg);
    });
}

void Controller::publish_data()
{
    state_msg.buttons.clear();
    state_msg.axes.clear();

    for(const auto& button_pair: buttons)
    {   
        robot_controller::Button_data button_data;
        button_data.key = button_pair.first;
        button_data.value = button_pair.second;
        state_msg.buttons.push_back(button_data);
    }

    for(const auto& axis_pair: axes)
    {
        robot_controller::Axis_data axis_data;
        axis_data.key = axis_pair.first;
        axis_data.value = axis_pair.second;
        state_msg.axes.push_back(axis_data);
    }

    pub.publish(state_msg);
}
