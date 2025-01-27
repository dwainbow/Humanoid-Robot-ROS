#ifndef MOTOR_CLUSTER_H
#define MOTOR_CLUSTER_H

#include <ros/ros.h>
#include "Motor_Controller.h"
#include <map>
#include <string> 
#include <robot_controller/controller_state.h>


enum class Body_Part {
    LEFT_ARM,
    RIGHT_ARM,
    LEFT_LEG,
    RIGHT_LEG
};
class Motor_Cluster {
    public:
        Motor_Cluster(ros::NodeHandle& nh, Body_Part body_part);
        void update_motor(int motor_id);
        void add_motor(Motor_Controller motor);
    private:
        ros::Subscriber subscriber; 
        Body_Part body_part;
        std::map<int, Motor_Controller> motors;
        std::map<std::string, float> controller_keys;

        void read_controller_data(const robot_controller::controller_state &msg);        
};
#endif // MOTOR_CONTROLLER_H


// The motors are going ot be hard coded, but how can I set the params for the motor? 