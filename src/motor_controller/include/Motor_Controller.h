#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string> 
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/clamp.hpp>
#include <unistd.h> 


class Motor_Controller {
public:
    Motor_Controller(ros::NodeHandle& nh,int motor_id, int baude_rate, int starting_position);
    Motor_Controller() = default;

    int get_id();
    int get_present_position();
    int get_goal_position();
    int get_starting_position();
    int get_operating_mode();

    bool get_reverse();
    bool connect_motor();

    void set_goal_position(int position);
    void set_torque(bool torque);
    void set_operating_mode(int mode);
    void set_starting_position(int position);
   
    void write_torque();
    void write_goal_position();
    void write_operating_mode();
    void publish_motor_data();

    void set_min_motor_degrees(int min_motor_degrees);
    void set_max_motor_degrees(int max_motor_degrees);
    void reset_motor();
    void set_reverse(bool reverse_position);

private:
    int motor_id;
    int goal_position;
    int present_position;
    int present_velocity;
    int goal_velocity; 
    int operating_mode;
    int baude_rate;
    int min_motor_position;
    int max_motor_position;
    int starting_position;
    float protocol_version;
    bool torque; 
    bool motor_connected;
    bool reverse_position;

    std::string controller_key;

    dynamixel::PortHandler* port_handler;
    dynamixel::PacketHandler* packet_handler;

};
#endif // MOTOR_CONTROLLER_H