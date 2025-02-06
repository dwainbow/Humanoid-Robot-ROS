#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"
#include <dynamixel_sdk/dynamixel_sdk.h>




void motor_test(ros::NodeHandle nh, Motor_Controller knee_motor_1, Motor_Controller knee_motor_2) {
    

    // Motor_Controller lower_1 = Motor_Controller(nh,1,57600,0, false);
    // knee_motor_1.set_starting_position(45);
    // knee_motor_1.set_max_motor_degrees(90);

    // Motor_Controller lower_2 = Motor_Controller(nh,1,57600,0, true);
    // lower_1.set_starting_position(45);
    // lower_2.set_max_motor_degrees(90);

    ros::Duration(2).sleep();

    auto converted_position = (45*4096)/360 ;
    auto converted_2 = (20*4096)/360;

    ROS_INFO("Going to position");
    knee_motor_1.set_goal_position(converted_position);
    knee_motor_2.set_goal_position(-converted_position);

    
    knee_motor_1.write_goal_position();
    knee_motor_2.write_goal_position();

    ros::Duration(2).sleep();

    ROS_INFO("Going to zero position");
    knee_motor_1.set_goal_position(converted_2);
    knee_motor_2.set_goal_position(-converted_2);

    knee_motor_1.write_goal_position();
    knee_motor_2.write_goal_position();



    
    // lower_1.write_goal_position();
    // lower_2.write_goal_position();
}
Motor_Cluster build_left_leg_cluster(ros::NodeHandle nh){
    Motor_Cluster left_leg_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);
    
    Motor_Controller knee_motor_test = Motor_Controller(nh,1,57600,0, false);
    knee_motor_test.set_starting_position(0);
    knee_motor_test.set_max_motor_degrees(90);
   
    // Motor_Controller knee_motor_1 = Motor_Controller(nh, 6, 3000000);
    // knee_motor_1.set_torque(true);
    // // knee_motor_1.set_starting_position(0);
    // // knee_motor_1.set_max_motor_degrees(0);
   
    left_leg_cluster.add_motor( knee_motor_test, "Left_Stick_Y");
    // left_leg_cluster.add_motor(knee_motor_2, "Left_Stick_Y");

    return left_leg_cluster;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    // ROS_INFO("Scanning motors");
    // scan_motors();
    // ROS_INFO("Finished Scanning nodes");

    // Motor_Cluster left_leg_cluster = build_left_leg_cluster(nh);
    // while(ros::ok()){
    //     left_leg_cluster.update_motors();
    //     ros::spinOnce();
    // }
    Motor_Controller knee_motor_1 = Motor_Controller(nh,14,3000000,0, false);
    knee_motor_1.set_starting_position(180);
    knee_motor_1.set_max_motor_degrees(90);

    Motor_Controller knee_motor_2 = Motor_Controller(nh,17,3000000,0, true);
    knee_motor_2.set_starting_position(180);
    knee_motor_2.set_max_motor_degrees(90);
    while (ros::ok()){
        motor_test(nh, knee_motor_1, knee_motor_2);
        ros::spinOnce();
    }
    // ros::spin();
    return 0;
}