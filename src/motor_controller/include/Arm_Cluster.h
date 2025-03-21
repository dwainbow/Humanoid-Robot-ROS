#ifndef ARM_CLUSTER_H
#define ARM_CLUSTER_H

#include <ros/ros.h>
#include "Motor_Controller.h"
#include <map>
#include <memory>  
#include <string>
#include <robot_controller/controller_state.h>
#include <cmath>

class Arm_Cluster : public Motor_Cluster
{
    public:
        Arm_Cluster(ros::NodeHandle &nh, Body_Part body_part);
        void update_motor(std::shared_ptr<Motor_Controller> motor, const std::string &motor_name);
        void add_motor(std::shared_ptr<Motor_Controller> motor, const std::string &controller_key);
        void update_motors();

    private:
        ros::Subscriber subscriber;
        Body_Part body_part;
        std::map<int, std::pair<std::shared_ptr<Motor_Controller>, std::string>> motors;
        std::map<std::string, float> controller_keys;

        double get_elbow_X();
        double get_elbow_Y();
        double get_elbow_Z();
        double get_hand_X();
        double get_hand_Y();
        double get_hand_Z();
        void read_controller_data(const robot_controller::controller_state &msg);
};
#endif // MOTOR_CLUSTER_H