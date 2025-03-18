
#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

void motor_test(Motor_Controller knee_motor_1, Motor_Controller knee_motor_2, Motor_Controller knee_motor_3, Motor_Controller knee_motor_4)
{

    int num_cycles = 10;                      // Number of oscillations
    auto upper_positions = (90 * 4096) / 360; // 90-degree position
    auto lower_positions = (45 * 4096) / 360; // 180-degree position

    ROS_INFO("Starting oscillation test...");

    for (int i = 0; i < num_cycles; ++i)
    {
        ROS_INFO("Cycle %d: Moving up", i + 1);
        knee_motor_1.set_goal_position(upper_positions);
        knee_motor_2.set_goal_position(upper_positions);

        knee_motor_3.set_goal_position(lower_positions);
        knee_motor_4.set_goal_position(lower_positions);

        knee_motor_1.write_goal_position();
        knee_motor_2.write_goal_position();
        knee_motor_3.write_goal_position();
        knee_motor_4.write_goal_position();

        ros::Duration(2).sleep();

        ROS_INFO("Cycle %d: Moving down", i + 1);
        knee_motor_1.set_goal_position(0);
        knee_motor_2.set_goal_position(0);
        knee_motor_2.set_goal_position(0);
        knee_motor_2.set_goal_position(0);

        knee_motor_1.write_goal_position();
        knee_motor_2.write_goal_position();
        knee_motor_3.write_goal_position();
        knee_motor_4.write_goal_position();

        ros::Duration(2).sleep();
    }

    ROS_INFO("Oscillation test completed.");
}

void sync_test(Motor_Controller motor_1){
    int num_cycles = 10;                      // Number of oscillations
    auto upper_positions = (90 * 4096) / 360; // 90-degree position

    ROS_INFO("Starting oscillation test...");

    for (int i = 0; i < num_cycles; ++i)
    {
        ROS_INFO("Cycle %d: Moving up", i + 1);
        motor_1.set_goal_position(upper_positions);
        motor_1.write_goal_position();
   

        ros::Duration(2).sleep();

        ROS_INFO("Cycle %d: Moving down", i + 1);
        motor_1.set_goal_position(0);
        motor_1.write_goal_position();
        

        ros::Duration(2).sleep();
    }

    ROS_INFO("Oscillation test completed.");
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh; 

    auto bd_rate = 3000000;
    std::shared_ptr<Motor_Controller> arm_elbow_14 = std::make_shared<Motor_Controller>(nh, 14, bd_rate, 0, 720, false);
    std::shared_ptr<Motor_Controller> arm_elbow_15 = std::make_shared<Motor_Controller>(nh, 15, bd_rate, 0, 720, false);
    // std::shared_ptr<Motor_Controller> knee_motor_3 = std::make_shared<Motor_Controller>(nh, 3, bd_rate, 0, 120, false);

    arm_elbow_14->sync_motor_with(nh, *arm_elbow_15);
    // knee_motor_3->sync_motor_with(nh, *knee_motor_1);

    // Add motors to cluster
    // left_leg_cluster.add_motor(knee_motor_1, "Left_Stick_Y");
    // left_arm_cluster.add_motor(arm_elbow_14, "Left_Stick_Y");
    // left_arm_cluster.add_motor(arm_elbow_15, "Left_Stick_Y");
     ros::Duration(5).sleep();
    while (ros::ok())
    {
        // motor_test(knee_motor_1, knee_motor_2, knee_motor_3, knee_motor_4);
        sync_test(*arm_elbow_14);
        ros::spinOnce();
    }

    return 0;
}