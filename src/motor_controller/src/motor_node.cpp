#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

Motor_Cluster build_right_arm_cluster(ros::NodeHandle nh)
{
    Motor_Cluster right_arm_cluster = Motor_Cluster(nh, Body_Part::RIGHT_ARM);
    auto bd_rate = 3000000;

    // Use `std::shared_ptr` to store motors dynamically
    std::shared_ptr<Motor_Controller> elbow_motor = std::make_shared<Motor_Controller>(nh, 1, bd_rate, 0, 45, false);
    std::shared_ptr<Motor_Controller> upper_arm_motor = std::make_shared<Motor_Controller>(nh, 2, bd_rate, 0, 45, false);
    std::shared_ptr<Motor_Controller> lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 3, bd_rate, 0, 45, false);
    std::shared_ptr<Motor_Controller> upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 4, bd_rate, 0, 45, false);

    right_arm_cluster.add_motor(elbow_motor, "Right_Stick_X");
    right_arm_cluster.add_motor(upper_arm_motor, "Right_Stick_X");
    right_arm_cluster.add_motor(lower_shoulder_motor, "Right_Stick_Y");
    right_arm_cluster.add_motor(upper_shoulder_motor, "Right_Stick_Y");

    return right_arm_cluster;
}

Motor_Cluster build_left_arm_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_arm_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);
    auto bd_rate = 3000000;

    std::shared_ptr<Motor_Controller> elbow_motor = std::make_shared<Motor_Controller>(nh,5, bd_rate, 0, 45, true);
    std::shared_ptr<Motor_Controller> upper_arm_motor = std::make_shared<Motor_Controller>(nh, 6, bd_rate, 0, 45, true);
    std::shared_ptr<Motor_Controller> lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 7, bd_rate, 0, 45, false); //Note that the motor for 7 is on the wrong way
    std::shared_ptr<Motor_Controller> upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 8, bd_rate, 0, 45, true);

    left_arm_cluster.add_motor(elbow_motor, "Left_Stick_X");
    left_arm_cluster.add_motor(upper_arm_motor, "Left_Stick_X");
    left_arm_cluster.add_motor(lower_shoulder_motor, "Left_Stick_Y");
    left_arm_cluster.add_motor(upper_shoulder_motor, "Left_Stick_Y");

    return left_arm_cluster;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_node"); 
    ros::NodeHandle nh;

    Motor_Cluster left_arm_cluster = build_left_arm_cluster(nh);
    Motor_Cluster right_arm_cluster = build_right_arm_cluster(nh);

    while (ros::ok())
    {
        left_arm_cluster.update_motors();
        right_arm_cluster.update_motors();
        ros::spinOnce();
    }
    return 0;
}