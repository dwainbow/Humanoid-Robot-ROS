#include <ros/ros.h>
#include "Motor_Cluster.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    Motor_Cluster left_arm(nh, Body_Part::LEFT_ARM);


    return 0;
}