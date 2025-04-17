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
    right_arm_cluster.add_motor(upper_arm_motor, "Right_Stick_Y");
    right_arm_cluster.add_motor(lower_shoulder_motor, "Left_Stick_Y");
    right_arm_cluster.add_motor(upper_shoulder_motor, "Left_Stick_X");

    return right_arm_cluster;
}

Motor_Cluster build_left_arm_cluster(ros::NodeHandle nh)
{
    Motor_Cluster left_arm_cluster = Motor_Cluster(nh, Body_Part::LEFT_ARM);
    auto bd_rate = 3000000;

    std::shared_ptr<Motor_Controller> left_elbow_motor = std::make_shared<Motor_Controller>(nh,5, bd_rate, 0, 45, true);
    std::shared_ptr<Motor_Controller> left_upper_arm_motor = std::make_shared<Motor_Controller>(nh, 6, bd_rate, 0, 45, true);
    std::shared_ptr<Motor_Controller> left_lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 7, bd_rate, 0, 45, false); //Note that the motor for 7 is on the wrong way
    std::shared_ptr<Motor_Controller> left_upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 8, bd_rate, 0, 45, true);

    return left_arm_cluster;
}

void uva_test(ros::NodeHandle nh){
    auto bd_rate = 3000000;

    // std::shared_ptr<Motor_Controller> right_elbow_motor = std::make_shared<Motor_Controller>(nh, 1, bd_rate, 0, 90, true);
    // std::shared_ptr<Motor_Controller> right_upper_arm_motor = std::make_shared<Motor_Controller>(nh, 2, bd_rate, 0, 45, false);
    // std::shared_ptr<Motor_Controller> right_lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 3, bd_rate, 0, 135, false);
    // std::shared_ptr<Motor_Controller> right_upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 4, bd_rate, 0, 180, false);

    std::shared_ptr<Motor_Controller> left_elbow_motor = std::make_shared<Motor_Controller>(nh,5, bd_rate, 0, 90, false);
    std::shared_ptr<Motor_Controller> left_upper_arm_motor = std::make_shared<Motor_Controller>(nh, 6, bd_rate, 0, 45, true);
    std::shared_ptr<Motor_Controller> left_lower_shoulder_motor = std::make_shared<Motor_Controller>(nh, 7, bd_rate, 0, 135, false); //Note that the motor for 7 is on the wrong way
    std::shared_ptr<Motor_Controller> left_upper_shoulder_motor = std::make_shared<Motor_Controller>(nh, 8, bd_rate, 0, 180, true);
    

    left_lower_shoulder_motor->set_motor_position(20);
    ros::Duration(2).sleep();
    // U config
    left_upper_shoulder_motor->set_motor_position(180);
    ros::Duration(2).sleep();
    left_lower_shoulder_motor->set_motor_position(0);
    ros::Duration(2).sleep();
    left_elbow_motor->set_motor_position(45);
    ros::Duration(2).sleep();
    // exit(0);

    // V config
    left_elbow_motor->set_motor_position(0);
    ros::Duration(2).sleep();


    // A config
    left_lower_shoulder_motor->set_motor_position(-45);
    ros::Duration(1).sleep();
    left_elbow_motor->set_motor_position(30);
    ros::Duration(2).sleep();

    // right_lower_shoulder_motor->set_motor_position(30);
    // // U config
    // right_upper_shoulder_motor->set_motor_position(180);
    // ros::Duration(2).sleep();
    // right_lower_shoulder_motor->set_motor_position(0);
    // ros::Duration(2).sleep();
    // right_elbow_motor->set_motor_position(45);
    // ros::Duration(2).sleep();

    // // V config
    // right_elbow_motor->set_motor_position(0);
    // ros::Duration(2).sleep();


    // // A config
    // right_lower_shoulder_motor->set_motor_position(-45);
    // ros::Duration(1).sleep();
    // right_elbow_motor->set_motor_position(30);
    // ros::Duration(2).sleep();
    
    exit(0);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uva_test");
    ros::NodeHandle nh; 

    ros::Duration(2).sleep();
    uva_test(nh);
    // while (ros::ok())
    // {
    //     // motor_test(knee_motor_1, knee_motor_2, knee_motor_3, knee_motor_4);
    //     // sync_test(*arm_elbow_14);
    //     ros::spinOnce();
    // }

    return 0;
}