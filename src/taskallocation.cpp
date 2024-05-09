#include "turtlebot3_warehouse/taskallocation.h"
#include "turtlebot3_warehouse/order.h"
#include "turtlebot3_warehouse/TurtleBot3Interface.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Pose.h>
using namespace std;

TaskAllocation::TaskAllocation(TurtleBot3Interface tb3Interface, std::string include_file_path) :
    turtleBot3Interface_(tb3Interface), include_file_path_(include_file_path)
{
    turtlebots_ = turtleBot3Interface_.getTurtleBotsList();
    convertPackageOrders();
}

bool TaskAllocation::convertPackageOrders(void){
    bool converted;
    std::vector<Order> orders;
    unsigned int orderState = 0;
    unsigned int packageNum = 0;
    std::string packageNum_str;
    unsigned int pickUpLoc = 0;
    std::string pickUpLoc_str;
    unsigned int dropOffLoc = 0;
    std::string dropOffLoc_str;
    std::ifstream package_orders_file(package_orders_file_path_);
    std::string myText;
    int counter = -1;

    if (package_orders_file.is_open())
    {
        std::cout << "package_orders.csv reads:" << std::endl;
        while (package_orders_file.good()) 
        { 
            getline(package_orders_file, myText);
            std::cout << myText << std::endl;
            orderState = 0;
            
            if (counter >= 0) // skip first line (packageNo,pickUpLoc,dropOffLoc)
            {
                for (auto myChar:myText)
                {
                    switch (orderState)
                    {
                    case 0:
                        // all characters until next comma are the packageNum
                        if (myChar == ',')
                        {
                            if (packageNum_str == "")
                            {
                                packageNum = 0;
                            }
                            else
                            {
                                packageNum = stoi(packageNum_str);
                            }
                            orderState++;
                            packageNum_str = "";
                        }
                        else
                        {
                            packageNum_str += myChar;
                        }                        
                        break;
                    case 1:
                        // all characters until next comma are the pickUpLoc
                        if (myChar == ',')
                        {
                            if (pickUpLoc_str == "")
                            {
                                pickUpLoc = 0;
                            }
                            else
                            {
                                pickUpLoc = stoi(pickUpLoc_str);
                            }
                            orderState++;
                            pickUpLoc_str = "";
                        }
                        else
                        {
                            pickUpLoc_str += myChar;
                        }
                        break;
                    case 2:
                        // all characters until next comma are the dropOffLoc
                        if (myChar == ',')
                        {
                            if (dropOffLoc_str == "")
                            {
                                dropOffLoc = 0;
                            }
                            else
                            {
                                dropOffLoc = stoi(dropOffLoc_str);
                            }
                            orderState = 0;
                            dropOffLoc_str = "";
                        }
                        else
                        {
                            dropOffLoc_str += myChar;
                        }
                        break;
                    default:
                        break;                        
                    }

                }            

                orders.push_back(Order(include_file_path_, packageNum, pickUpLoc, dropOffLoc));
                packageNum = 0;
                pickUpLoc = 0;
                dropOffLoc = 0;
                
            }
            counter++;
            // std::cout<<"good: "<<package_orders_file.good()<<std::endl;
        }        
    }
    else
    {
        std::cout << "package_orders.csv could not be read" <<std::endl;
        return converted = false;
    }  

    std::cout << "Orders are:" << std::endl;
    for (auto order:orders)
    {
        std::cout << "packageNo: " << order.getPackageNo() << ", pickUpLoc: " << order.getPickUpLoc() << ", dropOffLoc: " << order.getDropOffLoc() 
                  << ", pickUpCoords: [" << order.getPickUpCoords().front() << "," << order.getPickUpCoords().back() << "]"
                  << ", dropOffCoords: [" << order.getDropOffCoords().front() << "," << order.getDropOffCoords().back() << "]" << std::endl;
    }

    orders_ = orders;

    return converted = true;
    
}


std::vector<Order> TaskAllocation::getOrders(void){
    return orders_;
}


void TaskAllocation::executeTSP(void){
    // execute the lkh_cvrp_allocation.py
    // solve TSP using LKH-3 CVRP (Capacitated Vehicle Routing Problem)
    // makes TSP graph and solves with LKH <-- get the plans.csv and turn into a graph
    std::string filename = lkh_file_path_;
    std::string command = "python ";
    command += filename;
    system(command.c_str());

    // store the allocations.csv into a vector of vectors packageAllocations_
    std::ifstream allocations_file(allocations_file_path_);
    std::string myText;
    std::string strToInt;
    unsigned int myInt;
    std::vector<unsigned int> packageAllocation;
    unsigned int packageNum;
    std::vector<double> pickUpCoords;
    std::vector<double> dropOffCoords;
    std::vector<std::vector<double>> goalAllocation;

    if (allocations_file.is_open())
    {
        // std::cout << "allocations_file.csv reads:" << std::endl;
        while (allocations_file.good()) 
        {
            getline(allocations_file, myText);
            for (auto myChar:myText)
            {
                if (myChar == ']')
                {
                    // end the packageAllocation, start new one
                    packageAllocations_.push_back(packageAllocation);
                    packageAllocation.clear();
                    // end the goalAllocation, start new one
                    goalAllocation.push_back(dropOffCoords);
                    goalAllocations_.push_back(goalAllocation);
                    goalAllocation.clear();
                }
                else if (myChar == ' ' || myChar == '[' || myChar == ',')
                {
                    // do not add anything
                }
                else
                {
                    // add numbers to packageAllocation
                    strToInt += myChar;
                    myInt = stoi(strToInt);
                    packageAllocation.push_back(myInt-1);
                    packageNum = myInt-1;
                    strToInt = "";

                    // get the coords from orders_
                    for (auto order:orders_)
                    {
                        if (order.getPackageNo() == packageNum)
                        {
                            pickUpCoords = order.getPickUpCoords();
                            goalAllocation.push_back(pickUpCoords);
                            dropOffCoords = order.getDropOffCoords();
                        }
                    }
                    
                }
            }
            // remove the last vector in packageAllocations_ and goalAllocations_ because its doubled
            packageAllocations_.erase(packageAllocations_.end());
            goalAllocations_.erase(goalAllocations_.end());
        }
    }
    else
    {
        std::cout << "allocations.csv could not be read" <<std::endl;
    }

    std::cout<<"packageAllocations_: "<<std::endl;
    for (auto packages:packageAllocations_)
    {
        std::cout<<"[";
        for (auto package:packages)
        {
            std::cout<<package<<" ";
        }
        std::cout<<"] ";
    }
    std::cout<<std::endl;

    std::cout<<"goalAllocations_: "<<std::endl;
    for (auto packages:goalAllocations_)
    {
        for (auto package:packages)
        {
            std::cout<<"["<<package.at(0)<<","<<package.at(1)<<"] ";
        }
        std::cout<<std::endl;
    }
}

void TaskAllocation::controlGoalPasser(void){
    // works for a SINGLE turtlebot
    // only called when Control topic for a new goal is set to TRUE
    double x_coord;
    double y_coord;
    double z_coord;
    geometry_msgs::Pose pose;

    if (singleAllocation_.empty())
    { // if theres no goals allocated
        if (goalAllocationsIndex_<goalAllocations_.size())
        { // if there are still goal allocations left
            singleAllocation_ = goalAllocations_.at(goalAllocationsIndex_);
            goalAllocationsIndex_++;
        }
        // otherwise leave the singleAllocation_ empty
    }

    if (!singleAllocation_.empty()) 
    { // if the turtlebot has goals to do
        if (singleAllocationIndex_ < singleAllocation_.size()) 
        { // if theres still goals in the allocation to get, assign more
            singleAllocationIndex_++;
        }
        else 
        {
            if (goalAllocationsIndex_ < goalAllocations_.size()) 
            { // if there are no more goals in the allocation and there are still allocations left to get, assign more
                goalAllocationsIndex_++;
                singleAllocation_ = goalAllocations_.at(goalAllocationsIndex_);
                singleAllocationIndex_ = 0;
            }
            else 
            { // if theres no more allocations and no more goals, don't set up any more goals
                singleAllocation_.clear();
                singleAllocationIndex_ = 0;
            }
        }
    }

    x_coord = singleAllocation_.at(singleAllocationIndex_).front();
    y_coord = singleAllocation_.at(singleAllocationIndex_).back();
    z_coord = 0;

    // send ros geometry_msgs/Pose to move_base_simple/goal
    pose = setPose(x_coord, y_coord, z_coord, 0, 0, 0);
}

// Send goals one by one to MultiBot for a MULTIPLE turtlebot
// called during running of program
void TaskAllocation::goalPasser(void){
    std::vector<std::vector<double>> currentAllocation;
    unsigned int currentAllocationIndex;
    double x_coord;
    double y_coord;
    geometry_msgs::Pose pose;
    bool newGoalFlag;
    /*
    turtlebots all have a particular allocation (set of goals) to complete. Once complete, 
    they move on to the next available allocation that isn't already being used and isn't already finished
    */

    for (auto turtlebot:turtlebots_)
    {
        currentAllocation = turtlebot->getCurrentAllocation();
        currentAllocationIndex = turtlebot->getCurrentAllocationIndex();

        if (currentAllocation.empty())
        { // if theres no goals allocated
            if (goalAllocationsIndex_<goalAllocations_.size())
            { // if there are still goal allocations left
                currentAllocation = goalAllocations_.at(goalAllocationsIndex_);
                goalAllocationsIndex_++;
            }
            // otherwise leave the currentAllocation empty
        }

        if (!currentAllocation.empty()) 
        { // if the turtlebot has allocations to do
            // IMPLEMENT WITH MULTIBOT: check the control topic if a new goal is required

            if (newGoalFlag) //if a new goal is needed, change the goal for this turtlebot
            {
                if (currentAllocationIndex < currentAllocation.size()) 
                { // if theres still goals in the allocation to get, assign more
                    currentAllocationIndex++;
                    turtlebot->setCurrentAllocationIndex(currentAllocationIndex);
                }
                else 
                {
                    if (goalAllocationsIndex_ < goalAllocations_.size()) 
                    { // if there are no more goals in the allocation and there are still allocations left to get, assign more
                        goalAllocationsIndex_++;
                        currentAllocation = goalAllocations_.at(goalAllocationsIndex_);
                        turtlebot->setCurrentAllocation(currentAllocation);
                        currentAllocationIndex = 0;
                        turtlebot->setCurrentAllocationIndex(currentAllocationIndex);
                    }
                    else 
                    { // if theres no more allocations and no more goals, don't set up any more goals
                        currentAllocation.clear();
                        turtlebot->setCurrentAllocation(currentAllocation);
                        currentAllocationIndex = 0;
                        turtlebot->setCurrentAllocationIndex(currentAllocationIndex);
                    }
                }
            }
        }
        // IMPLEMENT WITH MULTIBOT: pass the goal for the turtlebot to execute

        // if a new goal is needed get a goal from the allocations
        x_coord = currentAllocation.at(currentAllocationIndex).front();
        y_coord = currentAllocation.at(currentAllocationIndex).back();
        
        
        
    }
}

geometry_msgs::Pose TaskAllocation::setPose(double x_coord, double y_coord, double z_coord, double roll, double pitch, double yaw) 
{
    geometry_msgs::Pose pose;
    pose.position.x = x_coord;
    pose.position.y = y_coord;
    pose.position.z = z_coord;

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose.orientation.z = cr * cp * sy - sr * sp * cy;

    return pose;
}

/*
bool TaskAllocation::nearestNeighbour(void){
    bool executed = true;
    vector<unsigned int> turtlebotNeighbourhood;
    vector<unsigned int> neighbouringPackages;
    unsigned int packageNo;
    double distance;
    vector<double> distances;
    double closestDistance;
    MultiRobot multiRobot; //MultiRobot Path Planning class


    // find available turtlebots
    for (auto turtlebot : turtlebots_)
    {
        if (!turtlebot.getHasPackage())
        {
            // find packages within vicinity of turtlebot's neighbourhood
            turtlebotNeighbourhood = turtlebot.getNeighbourhood();
            turtlebotNeighbourhood.push_back(9999);//for find algorithm to work
            for (auto order : orders_)
            {
                packageNo = order.getPackageNo();

                auto it = std::find(turtlebotNeighbourhood.begin(), turtlebotNeighbourhood.end(), packageNo);

                if (it != turtlebotNeighbourhood.end()) 
                {
                    //packages to assign to turtlebot
                    neighbouringPackages.push_back(packageNo);
                }
            }
            //find the closest neighbouringPackages to turtlebot
            
            for (auto assignedPackage : neighbouringPackages)
            {
                distance = multiRobot.getDistanceToPackage(turtlebot, assignedPackage);
                distances.push_back(distance);
            }
            distances.push_back(99999);//for find algorithm to work

            for (int i = 0; i < distances.size()-1; i++)
            {
                if(distances.at(i+1)<distances.at(i)){
                    closestDistance = distances.at(i+1);
                }
            }

            //get the index of closestDistance and assign correlated package number to turtlebot
            auto itDistance = std::find(distances.begin(), distances.end(), closestDistance);
            if (itDistance != distances.end()) 
            {
                //closest assignable package
                int index = itDistance - distances.end();
                assignPackageToTurtlebot(neighbouringPackages.at(index));
            }      
            
            
        }
        
    }
    
    
    // assign the packages to the turtlebot that are in the neighbourhood AND nearest to turtlebot
    return executed;
}
*/


// packagePoseAssociation??? (not sure depends on how MultiRobot works)


