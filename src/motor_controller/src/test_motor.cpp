
#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"

void motor_test(Motor_Controller &knee_motor_top, Motor_Controller &knee_motor_bottom)
{

    int num_cycles = 10;                      // Number of oscillations
    auto upper_position = (90 * 4096) / 360; // 90-degree position
    auto lower_position = (45 * 4096) / 360; // 45-degree position

    ROS_INFO("Starting oscillation test...");

    for (int i = 0; i < num_cycles; ++i)
    {    
        ROS_INFO("Cycle %d: Moving up", i + 1);
        knee_motor_top.set_goal_position(upper_position);
        knee_motor_bottom.set_goal_position(lower_position);
        ros::Duration(2).sleep();

        ROS_INFO("Cycle %d: Moving down", i + 1);
        knee_motor_bottom.set_goal_position(0);
        knee_motor_top.set_goal_position(0);
        ros::Duration(2).sleep();
    }

    ROS_INFO("Oscillation test completed.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;

    //Note: I would hold off on this script with sync_motor_with until we have sorted it out 
    // Upper leg parts
    Motor_Controller knee_motor_1 = Motor_Controller(nh, 1, 3000000, 0, 90, true);
    Motor_Controller knee_motor_2 = Motor_Controller(nh, 2, 3000000, 0, 90, false);
    knee_motor_2.sync_motor_with(nh, knee_motor_1);

    // Lower leg part
    Motor_Controller knee_motor_3 = Motor_Controller(nh, 3, 3000000, 0, 45, true);
    // Motor_Controller knee_motor_4 = Motor_Controller(nh, 4, 3000000, 0, 45, false);
    // knee_motor_4.sync_motor_with(nh, knee_motor_3);

    // motor_test(knee_motor_1, knee_motor_3);  
    ros::Rate loop_rate(100);  // Runs at 10 Hz )
    while (ros::ok())
    {
        ros::spinOnce();  
        loop_rate.sleep(); 
    }

    return 0;
}