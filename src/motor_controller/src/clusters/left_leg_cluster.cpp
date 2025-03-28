#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"


Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_LEG);

    auto bd_rate = 3000000;

    std::shared_ptr<Motor_Controller> motor_2 = std::make_shared<Motor_Controller>(nh, 14, bd_rate, 0, 90, false);

    left_leg_cluster.add_motor(motor_2, "Left_Stick_Y");
    return left_leg_cluster;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "left_leg_cluster_node"); 
    ros::NodeHandle nh;

    Motor_Cluster left_leg = build_left_leg_cluster(nh);

    ros::AsyncSpinner spinner(2);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        left_leg.update_motors();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}