#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_LEG);

    Motor_Controller knee_motor_1 = Motor_Controller(nh, 1, 3000000, 0, true);
    Motor_Controller knee_motor_2 = Motor_Controller(nh, 2, 3000000, 0, false);
    knee_motor_1.set_max_motor_degrees(90);
    knee_motor_2.set_max_motor_degrees(90);


    Motor_Controller knee_motor_3 = Motor_Controller(nh, 3, 3000000, 0, true);
    Motor_Controller knee_motor_4 = Motor_Controller(nh, 4, 3000000, 0, false);
    knee_motor_3.set_max_motor_degrees(45);
    knee_motor_4.set_max_motor_degrees(45);

    left_leg_cluster.add_motor(knee_motor_1, "Left_Stick_Y");
    left_leg_cluster.add_motor(knee_motor_2, "Left_Stick_Y");
    left_leg_cluster.add_motor(knee_motor_3, "Right_Stick_Y");
    left_leg_cluster.add_motor(knee_motor_4, "Right_Stick_Y");

    return left_leg_cluster;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    Motor_Cluster left_leg_cluster = build_left_leg_cluster(nh);

    while (ros::ok())
    {
        left_leg_cluster.update_motors();
        ros::spinOnce();
    }
    return 0;
}