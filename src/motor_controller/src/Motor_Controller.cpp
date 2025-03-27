#include "Motor_Controller.h"

/// @brief Constructor for the Motor_Controller class
/// @param nh : NodeHandle
/// @param motor_id : ID of the motor
/// @param baude_rate : Baud Rate of the motor
/// @param starting_position : Starting position of the motor (deg)
/// @param reverse_position : Reverse the position of the motor
Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id, int baude_rate, int starting_position, int max_degrees, bool reverse_position = false)
{
    protocol_version = 2.0;
    port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");         // change to rfcomm0 for bluetooth connetcion
    packet_handler = dynamixel::PacketHandler::getPacketHandler(protocol_version); // Protocol Version 2.0

    this->motor_id = motor_id;
    this->baude_rate = baude_rate;
    this->reverse_position = reverse_position;

    publisher = nh.advertise<std_msgs::Int32>("/motor_controller_" + std::to_string(motor_id) + "/goal_position", 1000);
    motor_connected = this->connect_motor();
    if (!motor_connected)
    {
        return;
    }
    operating_mode = 4;
    drive_mode = reverse_position ? 1 : 0;
    this->set_operating_mode(operating_mode);
    this->set_drive_mode(drive_mode);
    this->starting_position = this->set_starting_position(starting_position);
    this->set_max_motor_degrees(max_degrees);
    this->set_torque(true);
    this->add_offset();
    // this->reset_motor(); 
}

/// @brief Connect to the motor
/// @return true if motor is connected, false otherwise


void Motor_Controller::add_offset()
{
    ROS_INFO("----------------------------------------------------------------------");
    auto present_position = this->get_present_position();
    
    ROS_INFO("Present position before offset %d", present_position);
    ROS_INFO("Starting Positon before offset %d", starting_position);
    ROS_INFO("max motor Positon before offset%d", max_motor_position);
    
    auto converted_position = present_position;
    if (present_position < 0){
        converted_position = std::abs(present_position);
    }
    auto diff_1 = converted_position - starting_position;
    auto diff_2 = 4096 - converted_position;

    
    ROS_INFO("DIFF 1 %d", diff_1);
    ROS_INFO("DIFF 2 %d", diff_2);

    auto offset = 0;

    if (diff_2 < diff_1){
        ROS_INFO("Applying offset");
        if (present_position < 0){
            offset -= 4096;
        }
        else{
            offset +=4096;
        }
    }
    


    starting_position += offset;
    max_motor_position += offset;
    // if (present_position >= starting_position && present_position <= max_motor_position)
    // {
    //     return;
    // }
    // else if (present_position < starting_position)
    // {
    //     return;
    // }
    // starting_position += 4069;
    // max_motor_position += 4069;

    ROS_INFO("Present position after offset %d", this->get_present_position());
    ROS_INFO("Starting Positon after offset %d", starting_position);
    ROS_INFO("max motor Positon after offset%d", max_motor_position);
    ROS_INFO("----------------------------------------------------------------------");
}
bool Motor_Controller::connect_motor()
{
    if (!port_handler->openPort())
    {
        ROS_ERROR("Failed to open port");
        return false;
    }

    if (!port_handler->setBaudRate(baude_rate))
    {
        ROS_ERROR("Failed to set baud rate: %d", baude_rate);
        port_handler->closePort();
        return true;
    }

    uint16_t model_number = 0;
    uint8_t dxl_error = 0;

    int dxl_comm_result = packet_handler->ping(port_handler, motor_id, &model_number, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS)
    {
        ROS_INFO("Motor found! ID: %d, Model Number: %d, Protocol: %.1f, Baud Rate: %d",
                 motor_id, model_number, protocol_version, baude_rate);
        ROS_INFO("Motor %d connected", motor_id);
        return true;
    }

    ROS_WARN("Error connecting to Motor with ID %d: %s", motor_id, packet_handler->getTxRxResult(dxl_comm_result));
    return false;
}

/// @brief Get the ID of the motor
/// @return motor_id
int Motor_Controller::get_id()
{
    return motor_id;
}

/// @brief Get the present position of the motor
/// @return present_position
int Motor_Controller::get_present_position()
{
    uint8_t dxl_error = 0;
    int32_t position = 0;

    int dxl_comm_result = packet_handler->read4ByteTxRx(port_handler, motor_id, 132, (uint32_t *)&position, &dxl_error);

    present_position = position;
    return present_position;
}

/// @brief Get the goal position of the motor
/// @return goal_position
int Motor_Controller::get_goal_position()
{
    return goal_position;
}

/// @brief Get the starting position of the motor
/// @return starting_position
int Motor_Controller::get_starting_position()
{
    return starting_position;
}

/// @brief Get the operating mode of the motor
/// @return operating_mode
int Motor_Controller::get_operating_mode()
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t operating_mode = 0;

    dxl_comm_result = packet_handler->read1ByteTxRx(port_handler, motor_id, 11, &operating_mode);

    return static_cast<int>(operating_mode);
}

/// @brief Get the reverse position status of the motor
/// @return reverse_position
bool Motor_Controller::get_reverse()
{
    return reverse_position;
}

/// @brief Set the goal position of the motor
/// @param position : Goal position to set
void Motor_Controller::set_goal_position(int position)
{   
    bool printing = false ;
    if (motor_id == 14){
        printing = true; 
    }

    if (printing){
        ROS_INFO("Starting Position : %d", starting_position);
        ROS_INFO("Proposed Goal position: %d", position);
        ROS_INFO("Max Motor Position Before: %d", max_motor_position);

        ROS_INFO("Goal Position Before: %d", goal_position);
    }
    goal_position = position;
    goal_position = boost::algorithm::clamp(goal_position, starting_position, max_motor_position);

    if (printing){
        ROS_INFO("Goal Position After: %d", goal_position);
        ROS_INFO("----------------------------------------------------------------------");
    }
}

/// @brief Set the torque of the motor
/// @param torque : Torque value to set
void Motor_Controller::set_torque(bool torque)
{
    this->torque = torque;
    this->write_torque();
}

/// @brief Set the operating mode of the motor
/// @param mode : Operating mode to set
void Motor_Controller::set_operating_mode(int mode)
{
    operating_mode = mode;
    this->write_operating_mode();
}

/// @brief Write the operating mode to the motor
void Motor_Controller::write_operating_mode()
{
    uint8_t dxl_error = 0;
    packet_handler->write1ByteTxRx(port_handler, motor_id, 11, operating_mode, &dxl_error);
}

/// @brief Set the starting position of the motor
/// @param position : Starting position to set
/// @return starting_position
int Motor_Controller::set_starting_position(int position)
{
    starting_position = 4096 * position / 360;
    // goal_position = starting_position;
    // this->write_goal_position();
    return starting_position;
}

/// @brief Write the torque to the motor
void Motor_Controller::write_torque()
{
    uint8_t dxl_error = 0;
    packet_handler->write1ByteTxRx(port_handler, motor_id, 64, torque ? 1 : 0, &dxl_error);
}

/// @brief Write the goal position to the motor
void Motor_Controller::write_goal_position()
{
    uint8_t dxl_error = 0;
    packet_handler->write4ByteTxRx(port_handler, motor_id, 116, goal_position, &dxl_error);
}

int Motor_Controller::process_publisher_data(int position){
    //This the input is the position data of the new motor. 
    //There is a possibility that data cannot be within the range of the other motor so it will not update 
    //Determine logic whether or not to apply an offset

    //If position is out of range then we add an offset
    //Note this only works for positive, we need to add negative logic too 
    // ROS_INFO("Position data before: %d", position);
    if(!(starting_position <= position && position <= max_motor_position)){
        position+=4096;
    }
    // ROS_INFO("Position data after: %d", position);
    return position;
}

void Motor_Controller::sync_motor_with(ros::NodeHandle &nh, Motor_Controller &leader_motor)
{
    std::string topic_name = "/motor_controller_" + std::to_string(leader_motor.get_id()) + "/goal_position";
    subscriber = nh.subscribe<std_msgs::Int32>(topic_name, 1000,
                                               [this](const std_msgs::Int32::ConstPtr &msg)
                                               {

                                                   int position_data =  this->process_publisher_data(msg->data);
                                                   this->set_goal_position(position_data);
                                                //    ROS_INFO("Present Position after sync: %d", this->get_present_position());
                        
                                                   this->write_goal_position();
                                               });
}

void Motor_Controller::publish_motor_data()
{

    // if (publisher.getNumSubscribers() == 0)
    // {
    //     // ROS_WARN("No subscribers for %s, skipping publish", publisher.getTopic().c_str());
    //     return;
    // }
    std_msgs::Int32 msg;
    msg.data = this->get_present_position();
    publisher.publish(msg);
}

ros::Subscriber Motor_Controller::get_subscriber()
{
    return subscriber;
}

ros::Publisher Motor_Controller::get_publisher()
{
    return publisher;
}

/// @brief Set the maximum motor degrees
/// @param max_motor_degrees : Maximum motor degrees to set
void Motor_Controller::set_max_motor_degrees(int max_motor_degrees)
{
    if (max_motor_degrees < 0)
    {
        ROS_ERROR("Max motor degrees cannot be negative");
        return;
    }
    auto converted_position = (max_motor_degrees * 4096) / 360;
    this->max_motor_position = starting_position + converted_position;
}

/// @brief Reset the motor
void Motor_Controller::reset_motor()
{
    goal_position = starting_position;
    ROS_INFO("Moving to starting position %d", starting_position);
    ROS_INFO("Goal Position %d", goal_position);
    ROS_INFO("Present Position %d", present_position);

    this->write_goal_position();
    ROS_INFO("Motor %d reset", motor_id);
    ROS_INFO("----------------------------------------------------------------------");
    // exit(0);

    // while (std::abs(this->get_present_position() - goal_position) > 10)
    // {
    //     ros::Duration(0.1).sleep();
    // }
    // TODO: Uncomment until we have sorted out threading
}

void Motor_Controller::set_drive_mode(int drive_mode)
{
    this->drive_mode = drive_mode;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, motor_id, 10, drive_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN("Failed to set drive mode for motor %d", motor_id);
    }
    else
    {
        ROS_INFO("Set Drive mode to %d for motor %d", drive_mode, motor_id);
    }
}
