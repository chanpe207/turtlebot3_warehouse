#ifndef MULTIBOT_H
#define MULTIBOT_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "turtlebot3_warehouse/TurtleBot3Interface.h"
#include "turtlebot3_warehouse/taskallocation.h"
#include "turtlebot3_warehouse/order.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

class MultiBot {
public:
    explicit MultiBot(ros::NodeHandle* nh, TurtleBot3Interface tb3Interface, TaskAllocation taskAllocation, std::string include_file_path);

    // void pathCallback(const nav_msgs::Path::ConstPtr& path_msg);
    double calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);
    void calculateDepotPlans();
    void calculateFuturePlans();
    void loadPackages();  // Load packages from file
    double calculatePlanDistance(const nav_msgs::Path& path);

private:
    ros::NodeHandle nh_;
    TurtleBot3Interface turtleBot3Interface_;  // Instance of TurtleBot3Interface
    TaskAllocation taskAllocation_;  // Instance of TaskAllocation
    ros::Subscriber path_sub_;
    std::vector<std::tuple<double, double, double>> packageCoordinates;
    std::vector<double> planDistances_;  // Store distances of plans
    std::string include_file_path_;
    std::string package_orders_file_path_ = include_file_path_ +"/turtlebot3_warehouse/package_orders.csv";
    std::string plans_file_path_ = include_file_path_ + "/turtlebot3_warehouse/plans.csv";
    // std::string package_orders_file_path_ = "/home/nk/catkin_ws/src/turtlebot3_warehouse/include/turtlebot3_warehouse/package_orders.csv";
    // std::string plans_file_path_ = + "/home/nk/catkin_ws/src/turtlebot3_warehouse/include/turtlebot3_warehouse/plans.csv";
};

#endif  // MULTIBOT_H
