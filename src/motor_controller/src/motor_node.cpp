#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh){
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);
    
    Motor_Controller motor_1 = Motor_Controller(nh, 1, 57600, 0);
    motor_1.set_torque(true);
    motor_1.set_reverse(true);
    motor_1.set_max_motor_degrees(90);
   
    Motor_Controller motor_3 = Motor_Controller(nh, 3, 3000000, 0);
    motor_3.set_torque(true);
    motor_3.set_max_motor_degrees(90);

    left_leg_cluster.add_motor(motor_1, "Left_Stick_Y");
    left_leg_cluster.add_motor(motor_3, "Left_Stick_Y");   


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