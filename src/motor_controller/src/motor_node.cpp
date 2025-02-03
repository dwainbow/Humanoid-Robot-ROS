#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh){
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);
    
    Motor_Controller knee_motor_2 = Motor_Controller(nh, 7,3000000);
    knee_motor_2.set_torque(true);
    knee_motor_2.set_reverse(true);
    knee_motor_2.set_starting_position(0);
    knee_motor_2.set_max_motor_degrees(0);
   
    Motor_Controller knee_motor_1 = Motor_Controller(nh, 6, 3000000);
    knee_motor_1.set_torque(true);
    knee_motor_1.set_starting_position(0);
    knee_motor_1.set_max_motor_degrees(0);
   
    left_leg_cluster.add_motor(knee_motor_1, "Left_Stick_Y");
    left_leg_cluster.add_motor(knee_motor_2, "Left_Stick_Y");

    return left_leg_cluster;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    // ROS_INFO("Scanning motors");
    // scan_motors();
    // ROS_INFO("Finished Scanning nodes");

    Motor_Cluster left_leg_cluster = build_left_leg_cluster(nh);
    while(ros::ok()){
        left_leg_cluster.update_motors();
        ros::spinOnce();
    }
    // ros::spin();
    return 0;
}