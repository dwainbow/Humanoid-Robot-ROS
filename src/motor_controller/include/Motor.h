#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <string> 
#include "robot_controller/controller_state.h"

class Motor_Controller {
public:
    Motor_Controller(ros::NodeHandle& nh,int motor_id, std:: string &body_part, std::string &controller_key);

    int get_present_velocity();
    int get_present_position();
    int get_goal_position();
    int get_goal_velocity();
    bool torque_enabled();
    int get_operating_mode();

    void set_goal_position(int position);
    void set_goal_velocity(int velocity);
    void set_torque(bool torque);
    void set_operating_mode(int mode);

    void scale_velocity(float scaling_factor);
    void scale_position(float scaling_factor);

    void write_torque();
    void write_goal_position();
    void write_goal_velocity();


private:
    void update_motor(const robot_controller::controller_state msg);

    int motor_id;
    int goal_position;
    int present_position;
    int present_velocity;
    int goal_velocity; 
    int operating_mode;
    bool torque_enabled; 

    std::string body_part ;
    std::string controller_key;

    ros::Subscriber subscriber;
    ros::Publisher publisher;

    // Dynamixel SDK members
    dynamixel::PortHandler* port_handler;
    dynamixel::PacketHandler* packet_handler;

    void initialize_motor();
    void set_motor_position(int position);
};

#endif // MOTOR_CONTROLLER_H