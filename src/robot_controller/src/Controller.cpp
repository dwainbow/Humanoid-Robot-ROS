#include "Controller.h"
#include <ros/ros.h>

/// @brief Constructor for the Controller class
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

    this->axes = {
        {"Left_Stick_X", 0.0},
        {"Left_Stick_Y", 0.0},
        {"Right_Stick_X", 0.0},
        {"Right_Stick_Y", 0.0},
        {"DPad_X", 0.0},
        {"DPad_Y", 0.0},
        {"L2", 0.0},
        {"R2", 0.0},
    };

    this->motor_values = {
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

/// @brief Destructor for the Controller class
Controller::~Controller()
{
    // Leave empty for now
}

/// @brief Applies a low pass filter to smooth out fluctuations/noise
/// @param alpha Smoothing factor, must be between 0 and 1
/// @param prev_output Previous output value
/// @param new_input New input value
/// @return Filtered value
float Controller::apply_low_pass_filter(float alpha, float prev_output, float new_input)
{
    return alpha * new_input + (1 - alpha) * prev_output;
}

/// @brief Maps normalized controller values to motor range
/// @param normalized_value Normalized value from the controller
/// @param max_motor Maximum motor value
/// @return Mapped motor value
float Controller::map_to_motor(float normalized_value, float max_motor)
{
    return normalized_value * max_motor;
}

/// @brief Applies a deadzone to the input to ignore small fluctuations
/// @param input Input value
/// @param deadzone_threshold Threshold below which input is ignored
/// @return Processed input value
float Controller::apply_deadzone(float input, float deadzone_threshold)
{
    if (std::abs(input) < deadzone_threshold)
    {
        return 0.0;
    }
    return input;
}

/// @brief Gets the value of a specific axis
/// @param key Axis key
/// @return Axis value or -1 if key not found
double Controller::get_axis(const std::string &key)
{
    if (this->axes.find(key) != this->axes.end())
    {
        return this->axes[key];
    }
    return -1;
}

/// @brief Gets the value of a specific button
/// @param key Button key
/// @return Button value or -1 if key not found
int Controller::get_button(const std::string &key)
{
    if (this->buttons.find(key) != this->buttons.end())
    {
        return this->buttons[key];
    }
    return -1;
}

/// @brief Processes the input by applying low pass filter and deadzone
/// @param input Input value
/// @param alpha Smoothing factor for low pass filter
/// @param prev_output Previous output value
/// @param deadzone_threshold Threshold for deadzone
/// @return Processed input value
float Controller::process_input(float input, float alpha, float prev_output, float deadzone_threshold)
{
    float filtered_input = apply_low_pass_filter(alpha, prev_output, input);
    float deadzoned_input = apply_deadzone(filtered_input, deadzone_threshold);

    return deadzoned_input;
}

/// @brief Updates the controller state based on the received joystick message
/// @param msg Pointer to the joystick message
void Controller::update(const sensor_msgs::Joy::ConstPtr &msg)
{
    const float alpha = 0.6;
    const float deadzone_threshold = 0.59;
    const float max_motor = 1.0; // max motor value for the motor range, we probably want to define this range

    axes["Left_Stick_X"] = this->process_input(msg->axes[0], alpha, axes["Left_Stick_X"], deadzone_threshold); // max_motor is dependent on the motor range
    motor_values["Left_Stick_X"] = this->map_to_motor(axes["Left_Stick_X"], max_motor);

    axes["Left_Stick_Y"] = this->process_input(msg->axes[1], alpha, axes["Left_Stick_Y"], deadzone_threshold);
    motor_values["Left_Stick_Y"] = this->map_to_motor(axes["Left_Stick_Y"], max_motor);

    axes["L2"] = this->process_input(msg->axes[2], alpha, axes["L2"], deadzone_threshold);
    motor_values["L2"] = this->map_to_motor(axes["L2"], max_motor);

    axes["Right_Stick_X"] = this->process_input(msg->axes[3], alpha, axes["Right_Stick_X"], deadzone_threshold);
    motor_values["Right_Stick_X"] = this->map_to_motor(axes["Right_Stick_X"], max_motor);

    axes["Right_Stick_Y"] = this->process_input(msg->axes[4], alpha, axes["Right_Stick_Y"], deadzone_threshold);
    motor_values["Right_Stick_Y"] = this->map_to_motor(axes["Right_Stick_Y"], max_motor);

    axes["R2"] = this->process_input(msg->axes[5], alpha, axes["R2"], deadzone_threshold);
    motor_values["R2"] = this->map_to_motor(axes["R2"], max_motor);

    axes["DPad_X"] = this->process_input(msg->axes[6], alpha, axes["DPad_X"], deadzone_threshold);
    motor_values["DPad_X"] = this->map_to_motor(axes["DPad_X"], max_motor);

    axes["DPad_Y"] = this->process_input(msg->axes[7], alpha, axes["DPad_Y"], deadzone_threshold);
    motor_values["DPad_Y"] = this->map_to_motor(axes["DPad_Y"], max_motor);

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

    // ROS_INFO("Axes: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
    //  axes["Left_Stick_X"], axes["Left_Stick_Y"], axes["L2"], axes["Right_Stick_X"], axes["Right_Stick_Y"], axes["R2"], axes["DPad_X"], axes["DPad_Y"],
    //     buttons["X"], buttons["Circle"], buttons["Square"], buttons["Triangle"], buttons["L1"], buttons["R1"], buttons["Share"], buttons["Options"], buttons["L3"], buttons["R3"]
    // );
    // ROS_INFO("Axes: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
    //  motor_values["Left_Stick_X"], motor_values["Left_Stick_Y"], motor_values["L2"], motor_values["Right_Stick_X"], motor_values["Right_Stick_Y"], motor_values["R2"], motor_values["DPad_X"], motor_values["DPad_Y"],
    //     buttons["X"], buttons["Circle"], buttons["Square"], buttons["Triangle"], buttons["L1"], buttons["R1"], buttons["Share"], buttons["Options"], buttons["L3"], buttons["R3"]
    // );
}

/// @brief Initializes the controller by setting up ROS publishers and subscribers
/// @param nh ROS NodeHandle
void Controller::init_controller(ros::NodeHandle &nh)
{
    pub = nh.advertise<robot_controller::controller_state>("/controller_metadata", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, [this](const sensor_msgs::Joy::ConstPtr &msg) { // this is a lambda function to subscribe to the joy topic
        this->update(msg);
    });
}

/// @brief Publishes the controller state data
void Controller::publish_data()
{
    state_msg.buttons.clear();
    state_msg.axes.clear();

    for (const auto &button_pair : buttons)
    {
        robot_controller::Button_data button_data;
        button_data.key = button_pair.first;
        button_data.value = button_pair.second;
        state_msg.buttons.push_back(button_data);
    }

    for (const auto &motor_axis_pair : motor_values)
    {
        robot_controller::Axis_data axis_data;
        axis_data.key = motor_axis_pair.first;
        axis_data.value = motor_axis_pair.second;
        state_msg.axes.push_back(axis_data);
    }

    pub.publish(state_msg);
}
