
#include "Motor.h"


void Motor_Controller::update_motors(const std_msgs::Int32::ConstPtr &msg)
{
}

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id, std::string &body_part, std::string &controller_key)
{
    this->motor_id = motor_id;
    this->body_part = body_part;
    this->controller_key = controller_key;

    goal_position = 0;
    present_position = 0;
    present_velocity = 0;
    goal_velocity = 0;
    operating_mode = 0;
    torque_enabled = false;

    port_handler = PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    packet_handler = PacketHandler::getPacketHandler(2.0); //Protocol Version 2.0

    motor_subscriber = nh.subscribe(topic, 10, &Motor_Controller::update_motors, this);
}

int Motor_Controller::get_present_velocity()
{
    return present_velocity;
}

int Motor_Controller::get_present_position()
{
    return present_position;
}

int Motor_Controller::get_goal_position()
{
    return goal_position;
}

int Motor_Controller::get_goal_velocity()
{
    return goal_velocity;
}

bool Motor_Controller::torque_enabled()
{
    return torque_enabled;
}

int Motor_Controller::get_operating_mode()
{
    return operating_mode;
}

void Motor_Controller::set_goal_position(int position)
{
    goal_position = position;
}

void Motor_Controller::set_goal_velocity(int velocity)
{
    goal_velocity = velocity;
}

void Motor_Controller::set_torque(bool torque)
{
    torque_enabled = torque;
}

void Motor_Controller::set_operating_mode(int mode)
{
    operating_mode = mode;
}

void Motor_Controller::scale_velocity(float scaling_factor)
{
    goal_velocity*= scaling_factor;
}

void Motor_Controller::scale_position(float scaling_factor)
{
    goal_position*= scaling_factor; 
}

void Motor_Controller::write_torque()
{
    int dxl_error = 0;
    packet_handler->write1ByteTxRx(port_handler, motor_id, 64, torque_enabled ? 1 : 0, &dxl_error);
}

void Motor_Controller::write_goal_position()
{
    int dxl_error = 0;
    packet_handler->write4ByteTxRx(port_handler, motor_id, 116, goal_position, &dxl_error);
}

void Motor_Controller::write_goal_velocity()
{
    int dxl_error = 0;
    packet_handler_->write4ByteTxRx(port_handler_, motor_id, 104, goal_velocity, &dxl_error);
}

void Motor_Controller::update_motor(const robot_controller::controller_state msg)
{
}
