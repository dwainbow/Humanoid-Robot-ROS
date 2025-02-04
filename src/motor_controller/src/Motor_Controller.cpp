#include "Motor_Controller.h"

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id, int baude_rate, int starting_position)
{
    this->motor_id = motor_id;
    this->baude_rate = baude_rate;
    goal_position = 0;
    protocol_version = 2.0;
    min_motor_position = -4096;
    max_motor_position= 4096;
    present_position = 0;
    this->set_starting_position(starting_position);
    torque = true ;
    reverse_position = false;
    operating_mode = 5;

    port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    packet_handler = dynamixel::PacketHandler::getPacketHandler(protocol_version); //Protocol Version 2.0

    motor_connected = this->connect_motor();

    if(motor_connected){
        this->reset_motor();
    }
}

bool Motor_Controller::connect_motor()
{
    if (!port_handler->openPort()) {
        ROS_ERROR("Failed to open port");
        return false ;
    }

    if (!port_handler->setBaudRate(baude_rate)) {
        ROS_ERROR("Failed to set baud rate: %d", baude_rate);
        port_handler->closePort();
        return true ;
    }

    uint16_t model_number = 0;
    uint8_t dxl_error = 0;

    int dxl_comm_result = packet_handler->ping(port_handler, motor_id, &model_number, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
        ROS_INFO("Motor found! ID: %d, Model Number: %d, Protocol: %.1f, Baud Rate: %d",
                    motor_id, model_number, protocol_version, baude_rate);
        ROS_INFO("Motor %d connected", motor_id);
        return true ;
    }
    
    ROS_WARN("Error connecting to Motor with ID %d: %s", motor_id, packet_handler->getTxRxResult(dxl_comm_result));
    return false ;
    

}
int Motor_Controller::get_id()
{
    return motor_id;
}


int Motor_Controller::get_present_position()
{
    uint8_t dxl_error = 0;
    int32_t position = 0;  

    int dxl_comm_result = packet_handler->read4ByteTxRx(port_handler, motor_id, 132, (uint32_t*)&position, &dxl_error);

    present_position = position;  
    return present_position;
}

int Motor_Controller::get_goal_position()
{
    return goal_position;
}

int Motor_Controller::get_starting_position()
{
    return starting_position;
}

int Motor_Controller::get_operating_mode()
{
    int dxl_comm_result = COMM_TX_FAIL; 
    uint8_t operating_mode = 0; 

    dxl_comm_result = packet_handler->read1ByteTxRx(port_handler, motor_id, 11, &operating_mode);
    
    return static_cast<int>(operating_mode); 
}

bool Motor_Controller::get_reverse()
{
    return reverse_position;
}


void Motor_Controller::set_goal_position(int position)
{
    goal_position = position;
    if(reverse_position){
        goal_position = boost::algorithm::clamp(goal_position, max_motor_position, starting_position);
        return;
    }
    goal_position = boost::algorithm::clamp(goal_position, starting_position, max_motor_position);
}

void Motor_Controller::set_torque(bool torque)
{
    this->torque = torque;
    this->write_torque();
}

void Motor_Controller::set_operating_mode(int mode)
{
    operating_mode = mode;
    this->write_operating_mode();
}

void Motor_Controller::write_operating_mode()
{
    uint8_t dxl_error = 0;
    packet_handler->write1ByteTxRx(port_handler, motor_id, 11, operating_mode, &dxl_error);
}

void Motor_Controller::set_starting_position(int position)
{   
    starting_position = reverse_position ? -4096*position/360 : 4096*position/360;
    goal_position = starting_position;
    // this->publish_motor_data();
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
}

void Motor_Controller::publish_motor_data() {
    this->write_torque();
    this->write_goal_position();
}

/// @brief This creates the degree limit for min motor position
/// @param min_motor_degrees 
void Motor_Controller::set_min_motor_degrees(int min_motor_degrees)
{
    this->min_motor_position = present_position+(min_motor_degrees*4096)/360;
}

/// @brief This creates the degree limit for max motor position
/// @param max_motor_degrees 
void Motor_Controller::set_max_motor_degrees(int max_motor_degrees)
{
    ROS_INFO("Starting Position: %d", starting_position);
    auto converted_position = (max_motor_degrees*4096)/360;
    if(reverse_position){
        this->max_motor_position = starting_position - converted_position ;      
        return;
    }
    this->max_motor_position = starting_position + converted_position; ;
}

void Motor_Controller::reset_motor()
{
    this->set_operating_mode(operating_mode);
    this->set_torque(true);
    this->publish_motor_data();
    ROS_INFO("Motor %d reset", motor_id);
}

void Motor_Controller::set_reverse(bool reverse_position)
{
    this->reverse_position = reverse_position;
}
