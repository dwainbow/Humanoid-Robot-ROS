#include "Motor_Controller.h"

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id, int baude_rate, int starting_position, bool reverse_position = false)
{
    port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    packet_handler = dynamixel::PacketHandler::getPacketHandler(protocol_version); //Protocol Version 2.0

    this->motor_id = motor_id;
    this->baude_rate = baude_rate;
    this->reverse_position = reverse_position;

    motor_connected = this->connect_motor();
    if(motor_connected){
        this->starting_position = this->set_starting_position(starting_position);
        // present_position = get_present_position();
        // this->set_offset();


        protocol_version = 2.0;
        max_motor_position= 4096;
        torque = true ;
        reverse_position = false;
        operating_mode = 5;
        this->reset_motor();
    }
    else{
        return;
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

int Motor_Controller::set_starting_position(int position)
{   
    // ROS_INFO("Present Position: %d", this->get_present_position())
    starting_position = reverse_position ? -4096*position/360 : 4096*position/360;
    ROS_INFO("Starting Position on Startup: %d", starting_position );
    // goal_position = starting_position;
    // this->publish_motor_data();
    return starting_position;
  
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
    ROS_INFO("Present Position: %d", this->get_present_position());
    this->write_goal_position();
}


/// @brief This creates the degree limit for max motor position
/// @param max_motor_degrees 
void Motor_Controller::set_max_motor_degrees(int max_motor_degrees)
{
    // ROS_INFO("Starting Position: %d", starting_position);
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
    this->go_to_starting_position();
    this->publish_motor_data();
    // exit(0);
    ROS_INFO("Motor %d reset", motor_id);
    ROS_INFO("Goal position %d", goal_position);
    ROS_INFO("Present Positon: %d", this->get_present_position());

    //I suspect it might be a time issue 
    // exit(0);
}

void Motor_Controller :: set_offset(){
    uint8_t dxl_error = 0;
    ROS_INFO("Reading");
    int dxl_comm_result = packet_handler->read4ByteTxRx(port_handler, motor_id, 20, (uint32_t*)&present_position, &dxl_error);
    ROS_INFO("REad offset");
}

void Motor_Controller::go_to_starting_position()
{
    int d1 = std::abs(starting_position - present_position);
    int d2 = std::abs((starting_position + 4096) - present_position);
    int d3 = std::abs((starting_position - 4096) - present_position);

    int corrected_goal = starting_position;

    if(d2<d1){
        corrected_goal = starting_position+4096;
    }
    else if (d3 < d1){
        corrected_goal = starting_position - 4096;
    }

    goal_position = corrected_goal;
    
}
