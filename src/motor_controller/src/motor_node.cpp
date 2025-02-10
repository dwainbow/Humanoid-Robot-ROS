#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

void motor_test(ros::NodeHandle nh, Motor_Controller knee_motor_1, Motor_Controller knee_motor_2)
{

    int num_cycles = 10;                         // Number of oscillations
    auto converted_position = (90 * 4096) / 360; // 45-degree position
    auto converted_2 = (180 * 4096) / 360;       // 20-degree position

    ROS_INFO("Converted 45 degrees to position: %d", converted_position);
    ROS_INFO("Converted 20 degrees to position: %d", converted_2);

    ROS_INFO("Starting oscillation test...");

    for (int i = 0; i < num_cycles; ++i)
    {
        ROS_INFO("Cycle %d: Moving to 90 degrees", i + 1);
        knee_motor_1.set_goal_position(converted_position);
        knee_motor_2.set_goal_position(converted_position);

        knee_motor_1.write_goal_position();
        knee_motor_2.write_goal_position();

        ros::Duration(2).sleep();

        ROS_INFO("Cycle %d: Moving to 180 degrees", i + 1);
        knee_motor_1.set_goal_position(-converted_2);
        knee_motor_2.set_goal_position(-converted_2);

        knee_motor_1.write_goal_position();
        knee_motor_2.write_goal_position();

        ros::Duration(2).sleep();
    }

    ROS_INFO("Oscillation test completed.");
}
Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);

    Motor_Controller motor_1 = Motor_Controller(nh, 3, 3000000, 0, false);
    motor_1.set_max_motor_degrees(720);

    Motor_Controller motor_2 = Motor_Controller(nh, 5, 3000000, 0, true);
    motor_2.set_max_motor_degrees(720);

    left_leg_cluster.add_motor(motor_1, "Left_Stick_Y");
    left_leg_cluster.add_motor(motor_2, "Left_Stick_Y");

    return left_leg_cluster;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;

    Motor_Controller knee_motor_1 = Motor_Controller(nh, 3, 3000000, 0, false);
    knee_motor_1.set_max_motor_degrees(720);

    Motor_Controller knee_motor_2 = Motor_Controller(nh, 5, 3000000, 0, true);
    knee_motor_2.set_max_motor_degrees(720);

    Motor_Cluster left_leg_cluster = build_left_leg_cluster(nh);

    while (ros::ok())
    {
        // left_leg_cluster.update_motors();
        motor_test(nh, knee_motor_1, knee_motor_2);
        ros::spinOnce();
    }
    // ros::spin();
    return 0;
}