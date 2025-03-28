#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"


Motor_Cluster build_right_leg_cluster(ros::NodeHandle nh)
{
    Motor_Cluster right_leg_cluster = Motor_Cluster(nh, Body_Part::RIGHT_LEG);

    auto bd_rate = 3000000;

    std::shared_ptr<Motor_Controller> motor_2 = std::make_shared<Motor_Controller>(nh, 2, bd_rate, 0, 90, false);

    right_leg_cluster.add_motor(motor_2, "Left_Stick_Y");
    return right_leg_cluster;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "right_leg_cluster_node"); 
    ros::NodeHandle nh;

    Motor_Cluster right_leg = build_right_leg_cluster(nh);

    ros::AsyncSpinner spinner(2);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        right_leg.update_motors();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}