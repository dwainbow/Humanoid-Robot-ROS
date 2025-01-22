#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string> 
#include <sensor_msgs/JointState.h>

class Motor_Controller {
public:
    Motor_Controller(ros::NodeHandle& nh,int motor_id, std::string cluster);

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
    int motor_id;
    int goal_position;
    int present_position;
    int present_velocity;
    int goal_velocity; 
    int operating_mode;
    bool torque; 

    std::string body_part ;
    std::string controller_key;

    ros::Publisher publisher;

    // Dynamixel SDK members
    dynamixel::PortHandler* port_handler;
    dynamixel::PacketHandler* packet_handler;

    void update_motor(float position_scaling_factor = 1.0, float velocity_scaling_factor = 1.0);
};
#endif // MOTOR_CONTROLLER_H