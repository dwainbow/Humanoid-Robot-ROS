#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string> 
#include <sensor_msgs/JointState.h>

class Motor_Controller {
public:
    Motor_Controller(ros::NodeHandle& nh,int motor_id);
    Motor_Controller() = default;

    int get_id();
    int get_present_velocity();
    int get_present_position();
    int get_goal_position();
    int get_goal_velocity();
    int get_operating_mode();
    bool torque_enabled();

    void connect_motor();

    void set_goal_position(int position);
    void set_goal_velocity(int velocity);
    void set_torque(bool torque);
    void set_operating_mode(int mode);

    void write_torque();
    void write_goal_position();
    void write_goal_velocity();
    void publish_motor_data();

    void reset_motor();

private:
    int motor_id;
    int goal_position;
    int present_position;
    int present_velocity;
    int goal_velocity; 
    int operating_mode;
    int baude_rate;
    float protocol_version;
    bool torque; 

    std::string controller_key;

    dynamixel::PortHandler* port_handler;
    dynamixel::PacketHandler* packet_handler;

};
#endif // MOTOR_CONTROLLER_H