#include <ros/ros.h>
#include "Motor_Cluster.h"
#include "Motor_Controller.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

void scan_nodes(){
    auto port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); //change to rfcomm0 for bluetooth connetcion 
    auto packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0); //Protocol Version 2.0

    port_handler->setBaudRate(57600);
    for(int i = 1 ; i <253 ; i++){
        uint8_t error = 0;
        uint16_t model_number = 0;
        auto res = packet_handler->ping(port_handler,i, &model_number, &error);
        if(res==COMM_SUCCESS){
            ROS_INFO("Motor ID: %d, Model Number: %d", i, model_number);
        }
        else{
            ROS_INFO("COMM SUCCESS %d", res);
        }
    }
    port_handler->closePort();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle nh;
    ROS_INFO("Scanning nodes");
    scan_nodes();
    ROS_INFO("Finished Scanning nodes");
    // Motor_Cluster left_arm_cluster(nh, Body_Part::LEFT_ARM);
    // Motor_Controller motor = Motor_Controller(nh, 1);
    // motor.set_torque(true);
    // motor.set_operating_mode(3);
    // motor.set_goal_position(2048);
    // motor.set_goal_velocity(50);

    // left_arm_cluster.add_motor(motor);
    // ros::spin();
    return 0;
}