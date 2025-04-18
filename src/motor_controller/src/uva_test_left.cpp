#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

void trigger_left_arm_motion(ros::NodeHandle nh, int bd_rate){

    const char* port_string = "/dev/ttyUSB0";
    std::shared_ptr<Motor_Controller> left_elbow_motor = std::make_shared<Motor_Controller>(nh,5, bd_rate, 0, 90, port_string, false);
    std::shared_ptr<Motor_Controller> left_upper_arm_motor = std::make_shared<Motor_Controller>(nh, 6, bd_rate, 0, 45, port_string,true);
    std::shared_ptr<Motor_Controller> left_lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 7, bd_rate, 0, 135,port_string,false); //Note that the motor for 7 is on the wrong way
    std::shared_ptr<Motor_Controller> left_upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 8, bd_rate, 0, 180, port_string,true);

    left_lower_shoulder_motor->set_motor_position(30);
    ros::Duration(2).sleep();
    // U config
    left_upper_shoulder_motor->set_motor_position(180);
    ros::Duration(2).sleep();
    left_lower_shoulder_motor->set_motor_position(0);
    ros::Duration(2).sleep();
    left_elbow_motor->set_motor_position(45);
    ros::Duration(2).sleep();
    // exit(0);

    // V config
    left_elbow_motor->set_motor_position(0);
    ros::Duration(2).sleep();


    // A config
    left_lower_shoulder_motor->set_motor_position(-45);
    ros::Duration(1).sleep();
    left_elbow_motor->set_motor_position(30);
    ros::Duration(2).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uva_test_left");
    ros::NodeHandle nh; 
    auto bd_rate = 3000000;
    trigger_left_arm_motion(nh,bd_rate);

    return 0;
}