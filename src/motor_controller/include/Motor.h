#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

class Motor_Controller {
public:
    Motor_Controller(ros::NodeHandle& nh, int motor_id, const std::string& topic);

private:
    void update_motors(const std_msgs::Int32::ConstPtr& msg);

    int motor_id;
    ros::Subscriber motor_subscriber;

    // Dynamixel SDK members
    PortHandler* port_handler;
    PacketHandler* packet_handler;

    void initialize_motor();
    void set_motor_position(int position);
};

#endif // MOTOR_CONTROLLER_H