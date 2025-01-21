#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <map>
#include <sensor_msgs/Joy.h>

class Controller {
public:
    
    Controller();
    double get_axis(std::string key);
    int get_button(std::string key);
    void update(const sensor_msgs::Joy::ConstPtr& msg);
    

private:
    // Private member variables and methods
    std::map<std::string, float> axes;
    std::map<std::string, int> buttons;
};

#endif // CONTROLLER_H