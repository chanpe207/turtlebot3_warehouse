#ifndef TURTLEBOT3INTERFACE_H
#define TURTLEBOT3INTERFACE_H

#include "ros/ros.h"
#include "turtlebot3_warehouse/TurtleBot3.h"
#include <array>
#include <vector>

class TurtleBot3Interface {
public:
    explicit TurtleBot3Interface(ros::NodeHandle* nh, std::string include_file_path);
    std::vector<TurtleBot3*> getTurtleBotsList(void);
    int getNumTurtlebots(void);
private:
    ros::NodeHandle nh_;
    int num_turtlebots_;
    std::vector<TurtleBot3*> turtlebots_;
    std::string include_file_path_;
    std::string config_file_path_ = include_file_path_ + "/turtlebot3_warehouse/config.txt";
    int configSearch(const std::string& config_variable);
};

#endif // TURTLEBOT3INTERFACE_H
