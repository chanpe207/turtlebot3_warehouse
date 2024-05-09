#include "turtlebot3_warehouse/MultiBot.h"
#include "turtlebot3_warehouse/TurtleBot3.h"
#include "turtlebot3_warehouse/taskallocation.h"
#include <cmath>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <fstream>
#include "turtlebot3_warehouse/order.h"

MultiBot::MultiBot(ros::NodeHandle* nh, TurtleBot3Interface tb3Interface, TaskAllocation taskAllocation, std::string include_file_path) : 
    nh_(*nh), turtleBot3Interface_(tb3Interface), taskAllocation_(taskAllocation), include_file_path_(include_file_path) 
{
    ROS_INFO("MultiBot initialised with TurtleBot3");
}

double MultiBot::calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double MultiBot::calculatePlanDistance(const nav_msgs::Path& path) {
    double total_distance = 0.0;
    if (path.poses.size() > 1) {
        for (size_t i = 0; i < path.poses.size() - 1; ++i) {
            total_distance += calculateDistance(path.poses[i], path.poses[i + 1]);
        }
    }
    return total_distance;
}

void MultiBot::calculateDepotPlans() {
    // DO NOT USE
    std::vector<TurtleBot3*> turtlebots = turtleBot3Interface_.getTurtleBotsList();
    int num_tb = turtleBot3Interface_.getNumTurtlebots();
    
    // Open the CSV file
    std::ofstream file(plans_file_path_);

    // Check if you cannot open it
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file to write plans.");
        return;
    }
    // Top row of the CSV file for the columns
    file << "Start,End,Distance\n";

    // define the depot position
    geometry_msgs::PoseStamped depot_pose;
    // use the dropOffCoords for any Order
    std::vector<Order> orders = taskAllocation_.getOrders();
    std::vector<double> depotCoords = orders.at(0).getDropOffCoords();
    depot_pose.pose.position.x = depotCoords.front();
    depot_pose.pose.position.y = depotCoords.back();
    depot_pose.pose.position.z = 0.0;
    depot_pose.pose.orientation.w = 1.0;

    // for (int tb = 0; tb < num_tb; ++tb) {
        std::string service_name = "/tb3_0/move_base/make_plan";
        ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>(service_name);

        std::string startLocation = "Depot";

        for (size_t i = 0; i < packageCoordinates.size(); ++i) { // Loop through each package location
            const auto& coord = packageCoordinates[i];

            nav_msgs::GetPlan srv; // Outline the goal position using data from packageCoordinates
            srv.request.start.header.frame_id = "map";
            srv.request.start.pose = depot_pose.pose;
            srv.request.goal.header.frame_id = "map";
            srv.request.goal.pose.position.x = std::get<0>(coord);
            srv.request.goal.pose.position.y = std::get<1>(coord);
            srv.request.goal.pose.position.z = 0;
            srv.request.goal.pose.orientation.w = 1;
            if (client.call(srv)) {
                double distance = calculatePlanDistance(srv.response.plan); // Get path distance
                std::string packageNumber = "Package" + std::to_string(i + 1);

                if (distance == 0.00) {
                    ROS_WARN("Depot Plan to %s may be off the map or unreachable.", packageNumber.c_str());
                } else {
                    // Save the path length to the CSV file
                    ROS_INFO_STREAM(startLocation << " plan distance to " << packageNumber << ": " << distance);
                    file << startLocation << "," << packageNumber << "," << distance << "\n";
                }
            } else {
                ROS_ERROR("Depot Failed to call service %s", service_name.c_str());
            }
            ros::Duration(0.1).sleep(); //delay service calls
        }
    // }
    
    file.close(); //close the csv for later use after all saved

}


void MultiBot::calculateFuturePlans() {
    // Open the CSV file for writing future plan distances
    std::ofstream file(plans_file_path_);
    std::string startLocation;
    std::string endLocation;
    double distance;
    nav_msgs::GetPlan srv;
    std::vector<double> distanceMatrixRows(packageCoordinates.size(),0);
    std::vector<std::vector<double>> distanceMatrix(packageCoordinates.size(),distanceMatrixRows);

     // Check if you cannot open it
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file to write plans.");
        return;
    }
    // Top row of the CSV file for the columns
    file << "Start,End,Distance\n";

    ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetPlan>("/tb3_0/move_base/make_plan");

    // Calculate distances between all possible pairs of goals
    for (size_t i = 0; i < packageCoordinates.size(); ++i) {
        for (size_t j = 0; j < packageCoordinates.size(); ++j) {
        //loop through each package between each package

            if (i == 0)
            {
                startLocation = "Depot";
            }
            else
            {
                startLocation = "Package" + std::to_string(i);
            }

            if (j == 0)
            {
                endLocation = "Depot";
            }
            else
            {
                endLocation = "Package" + std::to_string(j);
            }
            
                             
            //set the start and end point for the make_plan
            srv.request.start.header.frame_id = "map";
            srv.request.start.pose.position.x = std::get<0>(packageCoordinates[i]);
            srv.request.start.pose.position.y = std::get<1>(packageCoordinates[i]);
            srv.request.start.pose.position.z = 0;
            srv.request.start.pose.orientation.w = 1;

            srv.request.goal.header.frame_id = "map";
            srv.request.goal.pose.position.x = std::get<0>(packageCoordinates[j]);
            srv.request.goal.pose.position.y = std::get<1>(packageCoordinates[j]);
            srv.request.goal.pose.position.z = 0;
            srv.request.goal.pose.orientation.w = 1;
            
            if (client.call(srv)) {
                if (i == j)
                { // if the package is calculating with itself make it 0
                    distance = 0;
                        
                    //save to CSV file
                    ROS_INFO("plan distance from package %zu to package %zu: %.2f meters", i, j, distance);
                    file << startLocation << "," << endLocation << "," << distance << std::endl;
                }
                else if (i>j)
                { // if the plan has been calculated in one direction already, use that distance
                    distance = distanceMatrix.at(j).at(i);

                    //save to CSV file
                    ROS_INFO("plan distance from package %zu to package %zu: %.2f meters", i, j, distance);
                    file << startLocation << "," << endLocation << "," << distance << std::endl;
                }
                else
                {
                    //calculate distance between start and end points using make_plan
                    distance = calculatePlanDistance(srv.response.plan);
                    
                    //save to CSV file
                    ROS_INFO("plan distance from package %zu to package %zu: %.2f meters", i, j, distance);
                    file << startLocation << "," << endLocation << "," << distance << std::endl;
                }
                distanceMatrix.at(i).at(j) = distance; //add distance to the distanceMatrix
            }
            else 
            {
                ROS_ERROR("Failed to call service make_plan for future plan calculation between package %zu and %zu", i, j);
            }
            ros::Duration(0.1).sleep();
        }
    }

    file.close();
    ROS_INFO("future plan calculations completed and saved.");
}

void MultiBot::loadPackages() {
    // Retrieve the list of orders from taskAllocation
    std::vector<Order> orders = taskAllocation_.getOrders();
    std::cout<<"num orders: "<<orders.size()<<std::endl;

    // Assuming packageCoordinates is a vector of tuples (x, y, z)
    // Clear existing coordinates
    packageCoordinates.clear();

    // Place depot coordinates at the start (using any Order)
    double pickUpX = orders.at(0).getDropOffCoords().at(0);
    double pickUpY = orders.at(0).getDropOffCoords().at(1); 
    double pickUpZ = 0;
    packageCoordinates.emplace_back(std::make_tuple(pickUpX, pickUpY, pickUpZ));

    // Process each order to extract package coordinates
    for (const Order& order : orders) {
      
        // Retrieve pick-up location coordinates
        pickUpX = order.getPickUpCoords().at(0);
        pickUpY = order.getPickUpCoords().at(1); 
        pickUpZ = 0;
        // Add to packageCoordinates
        packageCoordinates.emplace_back(std::make_tuple(pickUpX, pickUpY, pickUpZ));

        // Retrieve drop-off location coordinates if more than one depot
        // double dropOffX = order.getDropOffCoords().at(0); 
        // double dropOffY = order.getDropOffCoords().at(1); 
        // double dropOffZ = 0; 
        // // Add to packageCoordinates
        // packageCoordinates.emplace_back(std::make_tuple(dropOffX, dropOffY, dropOffZ));
    }
    std::cout<<"num packageCoordinates: "<<packageCoordinates.size()<<std::endl;
    ROS_INFO("loaded package coordinates in Multibot.cpp");
}

/*
Start,End,Distance
Depot,Depot,0
Depot,Package1,3.3412
Depot,Package2,7.60412
Depot,Package3,5.60466
Package1,Depot,3.3412
Package1,Package1,0
Package1,Package2,9.02491
Package1,Package3,7.04203
Package2,Depot,7.60412
Package2,Package1,9.02491
Package2,Package2,0
Package2,Package3,2.38358
Package3,Depot,5.60466
Package3,Package1,7.04203
Package3,Package2,2.38358
Package3,Package3,0
*/