#include "Motor_Controller.h"

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id)
{

    //TODO: Maybe have a starting positon for some of the motors depending on their position 
    this->motor_id = motor_id;
    goal_position = 0;
    present_position = 0;
    present_velocity = 0;
    goal_velocity = 0;
    operating_mode = 0;
    torque = true ;
    baude_rate = 57600;
    protocol_version = 2.0;
    min_motor_position = -360*4096/360;
    max_motor_position= 360*4096/360;

    port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    packet_handler = dynamixel::PacketHandler::getPacketHandler(protocol_version); //Protocol Version 2.0

    motor_connected = this->connect_motor();

    if (motor_connected){
        this->reset_motor();
    }
    
}

bool Motor_Controller::connect_motor()
{
    ROS_INFO("Connecting to motor %d", motor_id);
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

int Motor_Controller::get_present_velocity()
{
    uint8_t dxl_error = 0;
    int32_t velocity = 0;  

    int dxl_comm_result = packet_handler->read4ByteTxRx(port_handler, motor_id, 128, (uint32_t*)&velocity, &dxl_error);

    // ROS_INFO("Motor %d present velocity: %d", motor_id, velocity);
    present_velocity = velocity;  
    return present_position;
}

int Motor_Controller::get_present_position()
{
    uint8_t dxl_error = 0;
    int32_t position = 0;  

    int dxl_comm_result = packet_handler->read4ByteTxRx(port_handler, motor_id, 132, (uint32_t*)&position, &dxl_error);

    // ROS_INFO("Motor %d present position: %d", motor_id, position);
    present_position = position;  
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
    goal_position = std::max(min_motor_position, std::min(goal_position, max_motor_position));
    // ROS_INFO("Setting goal position for motor %d to %d", motor_id, position);
}

void Motor_Controller::set_goal_velocity(int velocity)
{
    // ROS_INFO("Setting goal velocity for motor %d to %d", motor_id, velocity);
    goal_velocity = velocity;
}

void Motor_Controller::set_torque(bool torque)
{
    // ROS_INFO("Setting torque for motor %d to %d", motor_id, torque);
    this->torque = torque;
}

void Motor_Controller::set_operating_mode(int mode)
{
    operating_mode = mode;
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
    // present_position = goal_position;
}

void Motor_Controller::write_goal_velocity()
{
    uint8_t dxl_error = 0;
    packet_handler->write4ByteTxRx(port_handler, motor_id, 104, goal_velocity, &dxl_error);
    // present_velocity = goal_velocity;
}


void Motor_Controller::publish_motor_data() {

        this->write_torque();
        this->write_goal_position();
        this->write_goal_velocity();

        // ROS_INFO("Motor %d updated with scaled values: Position = %d, Velocity = %d",
                //  motor_id, present_position, present_velocity);
}

/// @brief This creates the degree limit for min motor position
/// @param min_motor_degrees 
void Motor_Controller::set_min_motor_degrees(int min_motor_degrees)
{
    this->min_motor_position = (min_motor_degrees*4096)/360;
}

/// @brief This creates the degree limit for max motor position
/// @param max_motor_degrees 
void Motor_Controller::set_max_motor_degrees(int max_motor_degrees)
{
    this->max_motor_position = (max_motor_degrees*4096)/360;
}

void Motor_Controller::reset_motor()
{
    goal_position = 0;
    present_position = 0;
    present_velocity = 0;
    goal_velocity = 0;
    operating_mode = 0;
    torque = true;

    ROS_INFO("Resetting motor %d", motor_id);
    this->publish_motor_data();
    ROS_INFO("Motor %d reset", motor_id);

}
