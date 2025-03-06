#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_LEG);

    auto bd_rate = 3000000;

    // Use `std::shared_ptr` to store motors dynamically
    std::shared_ptr<Motor_Controller> knee_motor_1 = std::make_shared<Motor_Controller>(nh, 1, bd_rate, 0, 120, true);
    std::shared_ptr<Motor_Controller> knee_motor_2 = std::make_shared<Motor_Controller>(nh, 2, bd_rate, 0, 120, false);
    std::shared_ptr<Motor_Controller> knee_motor_3 = std::make_shared<Motor_Controller>(nh, 3, bd_rate, 0, 120, false);

    knee_motor_2->sync_motor_with(nh, *knee_motor_1);
    knee_motor_3->sync_motor_with(nh, *knee_motor_1);

    // Add motors to cluster
    left_leg_cluster.add_motor(knee_motor_1, "Left_Stick_Y");
    left_leg_cluster.add_motor(knee_motor_2, "Left_Stick_Y");
    left_leg_cluster.add_motor(knee_motor_3, "Left_Stick_Y");

    return left_leg_cluster;
}

Motor_Cluster build_left_arm_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_arm_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);

    auto bd_rate = 3000000;

    // Use `std::shared_ptr` to store motors dynamically
    std::shared_ptr<Motor_Controller> arm_elbow_1 = std::make_shared<Motor_Controller>(nh, 11, bd_rate, 0, 45, false);
    // std::shared_ptr<Motor_Controller> knee_motor_2 = std::make_shared<Motor_Controller>(nh, 2, bd_rate, 0, 120, false);
    // std::shared_ptr<Motor_Controller> knee_motor_3 = std::make_shared<Motor_Controller>(nh, 3, bd_rate, 0, 120, false);

    // knee_motor_2->sync_motor_with(nh, *knee_motor_1);
    // knee_motor_3->sync_motor_with(nh, *knee_motor_1);

    // Add motors to cluster
    // left_leg_cluster.add_motor(knee_motor_1, "Left_Stick_Y");
    // left_leg_cluster.add_motor(knee_motor_2, "Left_Stick_Y");
    left_arm_cluster.add_motor(arm_elbow_1, "Left_Stick_Y");

    return left_arm_cluster;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_node"); 
    ros::NodeHandle nh;

    Motor_Cluster left_arm_cluster = build_left_arm_cluster(nh);

    while (ros::ok())
    {
        left_arm_cluster.update_motors();
        ros::spinOnce();
    }
    return 0;
}