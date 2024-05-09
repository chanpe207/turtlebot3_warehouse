#ifndef TASKALLOCATION_H
#define TASKALLOCATION_H

#include <vector>
#include "turtlebot3_warehouse/order.h"
#include "turtlebot3_warehouse/TurtleBot3Interface.h"
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
using namespace std;

// Include any other header files this class relies on

class TaskAllocation 
{
public:
    // Default Constructors
    TaskAllocation(TurtleBot3Interface tb3Interface, std::string include_file_path);

    /**
     * MultiRobot gets their goals from here [[TB1, startLoc, endLoc], [TB2, startLoc, endLoc]]
     *  
     */
    // void assignedGoals(void);
    /**
     * Takes package orders from warehouse system (.csv) and converts them to vectors of Orders [order1, order2], where an order is [pkg#, pickUpLoc, dropOffLoc]
     * Runs once in constructor to set up
     * @return true if packages have all been read correctly
    */
    bool convertPackageOrders(void);
    /**
     * @return the orders converted from the csv file
    */
    std::vector<Order> getOrders(void);

    void executeTSP(void);

    void controlGoalPasser(void);

    void goalPasser(void);
    
protected:

private:
    // Functionality not used elsewhere goes here

    /**
     * Assigns package orders to currently available turtlebots via nearest neighbour greedy allocation
    */
    // bool nearestNeighbour(void);

    std::vector<Order> orders_;

    TurtleBot3Interface turtleBot3Interface_;  // Instance of TurtleBot3Interface
    std::vector<TurtleBot3*> turtlebots_;

    std::string include_file_path_;
    std::string package_orders_file_path_ = include_file_path_ +"/turtlebot3_warehouse/package_orders.csv";
    std::string lkh_file_path_ = include_file_path_+"/turtlebot3_warehouse/LKH-3.0.6/lkh_cvrp_allocation.py";
    std::string allocations_file_path_ = include_file_path_ + "/turtlebot3_warehouse/allocations.csv";

    std::vector<std::vector<unsigned int>> packageAllocations_; //contains all the task allocations of every order stored by their packageNumber e.g. {{2,1},{3}}
    std::vector<std::vector<std::vector<double>>> goalAllocations_; //contains all the task allocations of every order stored by their goal coords e.g. {{{1,0},{3,0},{-4,0}},{{-5,3},{-4,0}}}
    unsigned int goalAllocationsIndex_ = 0;
    std::vector<std::vector<double>> singleAllocation_; //only used for controlGoalPasser
    unsigned int singleAllocationIndex_ = 0; //only used for controlGoalPasser

    geometry_msgs::Pose setPose(double x_coord, double y_coord, double z_coord, double roll, double pitch, double yaw);
    
};

#endif // TASKALLOCATION_H