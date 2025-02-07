#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/clamp.hpp>
#include <unistd.h>

// TODO: add hard coding, for knee and motor joints (set motor x to this positon and so on for the test)
class Motor_Controller
{
public:
    Motor_Controller(ros::NodeHandle &nh, int motor_id, int baude_rate, int starting_position, bool reverse_position);
    Motor_Controller() = default;

    int get_id();
    int get_present_position();
    int get_goal_position();
    int get_starting_position();
    int get_operating_mode();

    bool get_reverse();

    void set_max_motor_degrees(int max_motor_degrees);
    void set_goal_position(int position);
    void write_goal_position();
    void publish_motor_data();

private:
    int motor_id;
    int goal_position;
    int present_position;
    int operating_mode;
    int baude_rate;
    int max_motor_position;
    int starting_position;
    int drive_mode;
    float protocol_version;
    bool torque;
    bool motor_connected;
    bool reverse_position;

    dynamixel::PortHandler *port_handler;
    dynamixel::PacketHandler *packet_handler;

    int set_starting_position(int position);

    bool connect_motor();
    void set_torque(bool torque);
    void set_operating_mode(int mode);
    void write_torque();
    void write_operating_mode();
    void reset_motor();
    void set_drive_mode(int drive_mode);
};
#endif // MOTOR_CONTROLLER_H