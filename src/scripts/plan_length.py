#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import math

def calculate_distance(pose1, pose2):
    """Calculate the Euclidean distance between two geometry_msgs/PoseStamped messages."""
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx**2 + dy**2)

def path_callback(path_msg):
    """Callback function for the path message."""
    total_distance = 0.0
    if len(path_msg.poses) > 1:
        for i in range(len(path_msg.poses) - 1):
            total_distance += calculate_distance(path_msg.poses[i].pose, path_msg.poses[i+1].pose)
    rospy.loginfo("Total path length: {:.2f} meters".format(total_distance))

def path_length_calculator():
    """Initialize the node, subscriber, and start listening to the path topic."""
    rospy.init_node('path_length_calculator', anonymous=True)
    rospy.Subscriber("/tb3_0/move_base/NavfnROS/plan", Path, path_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        path_length_calculator()
    except rospy.ROSInterruptException:
        pass

