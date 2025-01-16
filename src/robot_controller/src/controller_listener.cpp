#include <ros/ros.h>
#include <sensor_msgs/Joy.h>


void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    //Log the axes and buttons values
    ROS_INFO("Axes: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Buttons: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
             msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3], msg->axes[4], msg->axes[5], msg->axes[6],
             msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3], msg->buttons[4], msg->buttons[5],
             msg->buttons[6], msg->buttons[7], msg->buttons[8], msg->buttons[9]);
    
}


int main(int argc, char** argv)
{
    //Init Ros Node
    ros::init(argc, argv, "controller_listener");
    ros::NodeHandle nh;


     // Subscribe to the /joy topic
    ros::Subscriber sub = nh.subscribe("/joy", 10, joy_callback);

    // Spin to process incoming messages
    ros::spin();

    return 0;
}