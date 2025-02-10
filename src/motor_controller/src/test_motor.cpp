
#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

void motor_test(Motor_Controller knee_motor_1, Motor_Controller knee_motor_2, Motor_Controller knee_motor_3, Motor_Controller knee_motor_4)
{

    int num_cycles = 10;                         // Number of oscillations
    auto upper_positions = (90 * 4096) / 360; // 90-degree position
    auto lower_positions = (45 * 4096) / 360;       // 180-degree position

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;

    //Upper leg parts
    Motor_Controller knee_motor_1 = Motor_Controller(nh, 1, 3000000, 0, true);
    Motor_Controller knee_motor_2 = Motor_Controller(nh, 2, 3000000, 0, false);
    knee_motor_1.set_max_motor_degrees(90);
    knee_motor_2.set_max_motor_degrees(90);

    //Lower leg part 
    Motor_Controller knee_motor_3 = Motor_Controller(nh, 3, 3000000, 0, true);
    Motor_Controller knee_motor_4 = Motor_Controller(nh, 4, 3000000, 0, false);
    knee_motor_3.set_max_motor_degrees(45);
    knee_motor_4.set_max_motor_degrees(45);

    while (ros::ok())
    {
        motor_test(knee_motor_1,knee_motor_2,knee_motor_3,knee_motor_4);
        ros::spinOnce();
    }

    return 0;
}