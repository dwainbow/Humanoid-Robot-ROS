#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Controller.h>
#include "robot_controller/controller_state.h"


int main(int argc, char** argv)
{
    
    //Init Ros Node
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh;

     // Subscribe to the /joy topic
    Controller& controller = Controller::get_instance();
    controller.init_controller(nh);
    
    ros::Rate loop_rate(75); //keep this in range of 50 - 100 Hz
    while (ros::ok()) {
        // Call the publish_data method to publish messages
        controller.publish_data();

        // Process any incoming messages (e.g., joystick updates)
        ros::spinOnce();

        // Sleep to maintain the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}