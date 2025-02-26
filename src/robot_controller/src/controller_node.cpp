#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Controller.h>
#include "robot_controller/controller_state.h"

int main(int argc, char **argv)
{

    // Init Ros Node
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

    // Subscribe to the /joy topic
    Controller &controller = Controller::get_instance();
    controller.init_controller(nh);

    ROS_INFO("Controller Launched");
    ros::Rate loop_rate(50); // keep this in range of 10-50 Hz
    while (ros::ok())
    {
        controller.publish_data();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}