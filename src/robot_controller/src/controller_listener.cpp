#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Controller.h>

int main(int argc, char** argv)
{
    Controller controller;
    
    //Init Ros Node
    ros::init(argc, argv, "controller_listener");
    ros::NodeHandle nh;


     // Subscribe to the /joy topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, 
        [&controller](const sensor_msgs::Joy::ConstPtr& msg) {
            controller.update(msg);
        }
    );

    // Spin to process incoming messages
    ros::spin();

    return 0;
}