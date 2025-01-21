
#include "Motor.h"
#include <stdexcept>

Motor_Controller::Motor_Controller(ros::NodeHandle &nh, int motor_id, const std::string &topic)
{
    this->motor_id = motor_id; 
    port_handler = PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    packet_handler = PacketHandler::getPacketHandler(2.0); //Protocol Version 2.0

    motor_subscriber = nh.subscribe(topic, 10, &Motor_Controller::update_motors, this);
    
}

void Motor_Controller::update_motors(const std_msgs::Int32::ConstPtr &msg)
{
}
