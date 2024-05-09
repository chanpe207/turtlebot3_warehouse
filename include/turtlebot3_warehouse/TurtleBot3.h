#ifndef TURTLEBOT3_H
#define TURTLEBOT3_H

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <vector>
#include <array>

class TurtleBot3 {
public:
    explicit TurtleBot3(ros::NodeHandle* nh, std::string topic_string_);
    geometry_msgs::Pose getCurrentPose(void) const;
    bool hasPoseBeenUpdated(void) const;
    void resetPoseUpdateFlag(void);
    std::vector<std::vector<double>> getCurrentAllocation(void);
    unsigned int getCurrentAllocationIndex(void);
    void setCurrentAllocation(std::vector<std::vector<double>> currentAllocation);
    void setCurrentAllocationIndex(unsigned int currentAllocationIndex);

private:
    ros::NodeHandle nh_;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    geometry_msgs::Pose current_pose_;
    bool pose_updated_ = false;
    ros::Subscriber pose_sub_;
    std::vector<std::vector<double>> currentAllocation_; //A set of allocated goals to achieve in a single trip, including the depot stop
    unsigned int currentAllocationIndex_ = 0; //The index of the currentAllocations_
};

#endif // TURTLEBOT3_H
