#include "turtlebot3_warehouse/TurtleBot3.h"

TurtleBot3::TurtleBot3(ros::NodeHandle* nh, std::string topic_string_) :
    nh_(*nh)
{
    pose_sub_ = nh_.subscribe(topic_string_, 10, &TurtleBot3::poseCallback, this);
}

void TurtleBot3::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    current_pose_ = msg->pose.pose;
    pose_updated_ = true;
}

geometry_msgs::Pose TurtleBot3::getCurrentPose(void) const {
    return current_pose_;
}

bool TurtleBot3::hasPoseBeenUpdated(void) const {
    return pose_updated_;
}

void TurtleBot3::resetPoseUpdateFlag(void) {
    pose_updated_ = false;
}

std::vector<std::vector<double>> TurtleBot3::getCurrentAllocation(void) {
    return currentAllocation_;
}

unsigned int TurtleBot3::getCurrentAllocationIndex(void) {
    return currentAllocationIndex_;
}

void TurtleBot3::setCurrentAllocation(std::vector<std::vector<double>> currentAllocation) {
    currentAllocation_ = currentAllocation;
}

void TurtleBot3::setCurrentAllocationIndex(unsigned int currentAllocationIndex) {
    currentAllocationIndex_ = currentAllocationIndex;
}

/*
void TurtleBot3::passNewGoalCallback(service, request, response) {
    response -> goal pose
}
*/
