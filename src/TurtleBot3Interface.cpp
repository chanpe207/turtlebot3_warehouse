#include "turtlebot3_warehouse/TurtleBot3Interface.h"
#include "turtlebot3_warehouse/TurtleBot3.h"
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>
using namespace std;

TurtleBot3Interface::TurtleBot3Interface(ros::NodeHandle* nh, std::string include_file_path) :
    nh_(*nh), include_file_path_(include_file_path) 
{
    // create the number of turtlebots given in config.txt
    num_turtlebots_ = configSearch("NUM_TURTLEBOTS: ");
    
    // make turtlebots
    std::string topic_string;
    int topic_num;
    for (int i = 0; i < num_turtlebots_; i++)
    {
        // create the topic_string_ in the form of "/tb3_#/amcl_pose" where # is starting from 0 and ends in i-1
        topic_num = i;
        topic_string = "/tb3_" + to_string(topic_num) + "/amcl_pose";
        turtlebots_.push_back(new TurtleBot3(&nh_, topic_string));
        std::cout<<"Created Turtlebot "<<i<<std::endl;
    }
    
    
}

std::vector<TurtleBot3*> TurtleBot3Interface::getTurtleBotsList(void)
{
    return turtlebots_;
}

int TurtleBot3Interface::getNumTurtlebots(void)
{
    return num_turtlebots_;
}

int TurtleBot3Interface::configSearch(const std::string& config_variable) {
    std::ifstream config_file(config_file_path_);
    std::string line;
    std::string value_str;
    int value = 0;

    if (config_file.is_open()) {
        std::cout << "Config file opened successfully." << std::endl;
        while (getline(config_file, line)) {
            size_t pos = line.find(config_variable);
            if (pos != std::string::npos) {
                value_str = line.substr(pos + config_variable.size());
                try {
                    value = std::stoi(value_str);
                    std::cout << config_variable << " found: " << value << std::endl;
                    return value;
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid number format in config file: " << e.what() << std::endl;
                    return -1; // or other error code
                }
            }
        }
        std::cout << config_variable << " not found in the config file." << std::endl;
    } else {
        std::cerr << "Failed to open config file." << std::endl;
    }
    return -1; // or other error code indicating failure
}

