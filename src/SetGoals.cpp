
#include "turtlebot3_warehouse/SetGoals.h"

SetGoals::SetGoals(ros::NodeHandle* nodehandle): nh(*nodehandle) {
    // Initialize publishers for both robots
    goal_pub_tb3_0 = nh.advertise<geometry_msgs::PoseStamped>("/tb3_0/move_base_simple/goal", 10);
    goal_pub_tb3_1 = nh.advertise<geometry_msgs::PoseStamped>("/tb3_1/move_base_simple/goal", 10);
}

void SetGoals::publishGoals() {
    // Publish goals for the robots using the member publishers
    publishGoal(5.0, 0.0, "map", goal_pub_tb3_0);
    publishGoal(3.0, 0.0, "map", goal_pub_tb3_1);
}

void SetGoals::publishGoal(double x, double y, std::string frame_id, ros::Publisher& pub) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = frame_id;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0; // Assuming no orientation change

    ROS_INFO_STREAM("Publishing goal to " << pub.getTopic());
    pub.publish(goal);
    ros::spinOnce();
    //if i add ros::spin(); here it publioshes a single goal. What is th eproblem???
}

