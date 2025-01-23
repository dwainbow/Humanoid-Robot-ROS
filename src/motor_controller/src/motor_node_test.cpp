#include <ros/ros.h>
#include "Motor_Controller.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    Motor_Controller motor(nh, 1, "head");


    return 0;
}