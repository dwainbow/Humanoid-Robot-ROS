#include "Motor_Controller.h"

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id, std::string cluster)
{
    this->motor_id = motor_id;

    goal_position = 0;
    present_position = 0;
    present_velocity = 0;
    goal_velocity = 0;
    operating_mode = 0;
    torque = false;

    port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0); //Protocol Version 2.0

    publisher = nh.advertise<sensor_msgs::JointState>("/motor"+ std::to_string(motor_id), 10);
   
}

int Motor_Controller::get_id()
{
    return motor_id;
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
    return torque;
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
    this->torque = torque;
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
    uint8_t dxl_error = 0;
    packet_handler->write1ByteTxRx(port_handler, motor_id, 64, torque ? 1 : 0, &dxl_error);
}

void Motor_Controller::write_goal_position()
{
    uint8_t dxl_error = 0;
    packet_handler->write4ByteTxRx(port_handler, motor_id, 116, goal_position, &dxl_error);
    present_position = goal_position;
}

void Motor_Controller::write_goal_velocity()
{
    uint8_t dxl_error = 0;
    packet_handler->write4ByteTxRx(port_handler, motor_id, 104, goal_velocity, &dxl_error);
    present_velocity = goal_velocity;
}


void Motor_Controller::publish_motor_data(float position_scaling_factor, float velocity_scaling_factor) {

        this->scale_position(position_scaling_factor);
        this->scale_velocity(velocity_scaling_factor);

        this->write_torque();
        this->write_goal_position();
        this->write_goal_velocity();

        ROS_INFO("Motor %d updated with scaled values: Position = %d, Velocity = %d",
                 motor_id, present_position, present_velocity);
}