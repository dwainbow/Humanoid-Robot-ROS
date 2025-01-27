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

float Controller::apply_low_pass_filter(float alpha, float prev_output, float new_input)
{   

    //alpha must be beween 0 and 1
    //adjust this to smooth out fluctuations/noise generated from controller to reduce jerky movements 
    //lower values = smoother response, higher values = faster  response
    return alpha * new_input + (1 - alpha) * prev_output;
}

float Controller::normalize_input(float raw_value, float min_raw, float max_raw)
{   
    //This normalizes the inputs to [-1,1] for all inputs 
    return (raw_value - min_raw) / (max_raw - min_raw) * 2.0 - 1.0;
}

float Controller::map_to_motor(float normalized_value,float max_motor)
{   
    //Each controller key has a different range of values, this function maps the normalized values to the motor range
    return normalized_value * max_motor;
}

float Controller::apply_deadzone(float input, float deadzone_threshold)
{
    if (std:: abs(input) < deadzone_threshold)
    {
        return 0.0;
    }
    return input;
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

float Controller::process_input(float input, float min_raw, float max_raw, float alpha, float prev_output, float deadzone_threshold, float max_motor)
{
    float normalized_input = normalize_input(input, min_raw, max_raw);
    float filtered_input = apply_low_pass_filter(alpha, prev_output, normalized_input);
    float deadzoned_input = apply_deadzone(filtered_input, deadzone_threshold);
    float motor_input = map_to_motor(deadzoned_input, max_motor);
    return motor_input;
}

void Controller::update(const sensor_msgs::Joy::ConstPtr& msg)
{   

    const float alpha = 0.1;
    const float deadzone_threshold = 0.2;
    
    axes["Left_Stick_X"] = process_input(msg->axes[0], -1.0, 1.0, alpha, axes["Left_Stick_X"], deadzone_threshold, 1.0); //max_motor is dependent on the motor range
    axes["Left_Stick_Y"] = process_input(msg->axes[1], -1.0, 1.0, alpha, axes["Left_Stick_Y"], deadzone_threshold, 1.0);
    axes["L2"] = process_input(msg->axes[2], -1.0, 1.0, alpha, axes["L2"], deadzone_threshold, 1.0);
    axes["Right_Stick_X"] = process_input(msg->axes[3], -1.0, 1.0, alpha, axes["Right_Stick_X"], deadzone_threshold, 1.0);
    axes["Right_Stick_Y"] = process_input(msg->axes[4], -1.0, 1.0, alpha, axes["Right_Stick_Y"], deadzone_threshold, 1.0);
    axes["R2"] = process_input(msg->axes[5], -1.0, 1.0, alpha, axes["R2"], deadzone_threshold, 1.0);
    axes["DPad_X"] = process_input(msg->axes[6], -1.0, 1.0, alpha, axes["DPad_X"], deadzone_threshold, 1.0);
    axes["DPad_Y"] = process_input(msg->axes[7], -1.0, 1.0, alpha, axes["DPad_Y"], deadzone_threshold, 1.0);

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
    pub = nh.advertise<robot_controller::controller_state>("/controller_metadata", 1);
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
