#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <map>
#include <sensor_msgs/Joy.h>
#include "robot_controller/controller_state.h"
#include <ros/ros.h>


class Controller {
public:
    // Delete copy constructor and assignment operator to enforce singleton
    Controller(const Controller& obj) = delete;
    Controller& operator=(const Controller& obj) = delete;

    // Accessor for the singleton instance
    static Controller& get_instance() {
        static Controller instance;
        return instance;
    }

    // Public methods
    double get_axis(const std::string& key);
    int get_button(const std::string& key);
    void update(const sensor_msgs::Joy::ConstPtr& msg);
    void publish_data();
    void init_controller(ros::NodeHandle &nh);

private:    
    // Private constructor and member variables
    Controller() ;
    ~Controller();

    float apply_low_pass_filter(float alpha, float prev_output, float new_input);
    float normalize_input(float raw_value, float min_raw,float max_raw );
    float map_to_motor(float normalized_value,float max_motor);
    float apply_deadzone(float input, float deadzone_threshold);
    float process_input(float input, float min_raw, float max_raw, float alpha, float prev_output, float deadzone_threshold, float max_motor);
    std::map<std::string, float> axes;
    std::map<std::string, int> buttons;

    ros:: Publisher pub;
    ros:: Subscriber sub;
    ros:: NodeHandle nh;

    robot_controller::controller_state state_msg;
};

#endif // CONTROLLER_H