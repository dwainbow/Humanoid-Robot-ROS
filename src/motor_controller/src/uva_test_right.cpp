#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

void trigger_right_arm_motion(ros::NodeHandle nh, int bd_rate){
    const char* port_string = "/dev/ttyUSB1";

    std::shared_ptr<Motor_Controller> right_elbow_motor = std::make_shared<Motor_Controller>(nh, 1, bd_rate, 0, 90, port_string,true);
    std::shared_ptr<Motor_Controller> right_upper_arm_motor = std::make_shared<Motor_Controller>(nh, 2, bd_rate, 0, 45, port_string,false);
    std::shared_ptr<Motor_Controller> right_lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 3, bd_rate, 0, 135, port_string,false);
    std::shared_ptr<Motor_Controller> right_upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 4, bd_rate, 0, 180, port_string,false);

    right_lower_shoulder_motor->set_motor_position(30);
    ros::Duration(2).sleep();

    // U config
    right_upper_shoulder_motor->set_motor_position(180);
    ros::Duration(2).sleep();
    right_lower_shoulder_motor->set_motor_position(0);
    ros::Duration(2).sleep();
    right_elbow_motor->set_motor_position(45);
    ros::Duration(2).sleep();

    // V config
    right_elbow_motor->set_motor_position(0);
    ros::Duration(2).sleep();


    // A config
    right_lower_shoulder_motor->set_motor_position(-45);
    ros::Duration(1).sleep();
    right_elbow_motor->set_motor_position(30);
    ros::Duration(2).sleep();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uva_test_right");
    ros::NodeHandle nh; 
    auto bd_rate = 3000000;
    trigger_right_arm_motion(nh,bd_rate);

    return 0;
}