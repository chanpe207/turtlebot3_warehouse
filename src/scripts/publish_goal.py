#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist

def spin_robot(publisher, circles=1, angular_speed=1.0):
    # Create a Twist message to spin the robot
    twist = Twist()
    twist.angular.z = angular_speed  # Set angular speed

    # Calculate the time required to complete 10 circles
    # Assuming 2*pi radian for one circle and given angular speed
    time_for_one_circle = 2 * 3.14159 / angular_speed
    total_time = time_for_one_circle * circles

    rate = rospy.Rate(10)  # 10Hz
    start_time = rospy.Time.now()

    # Spin the robot for the calculated total time
    while rospy.Time.now() - start_time < rospy.Duration(total_time):
        publisher.publish(twist)
        rate.sleep()

    # Stop the robot after spinning
    twist.angular.z = 0
    publisher.publish(twist)

def publish_goal():
    # Initialize the ROS Node
    rospy.init_node('publish_goal_node', anonymous=True)

    # Publisher for tb3_0 to control its velocity
    cmd_vel_publisher_tb3_0 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)

    # Wait a bit for the publisher to establish connection to subscribers
    rospy.sleep(1)

    # Make tb3_0 spin in 10 circles
    rospy.loginfo("Making tb3_0 spin in 10 circles")
    spin_robot(cmd_vel_publisher_tb3_0)

    # Publisher for tb3_0's goal
    goal_publisher_tb3_0 = rospy.Publisher('/tb3_0/move_base_simple/goal', PoseStamped, queue_size=10)

    # Wait for the publisher to establish connection to subscribers
    rospy.sleep(1)

    # Create the goal message for tb3_0
    goal_tb3_0 = PoseStamped()
    goal_tb3_0.header.seq = 0
    goal_tb3_0.header.stamp = rospy.Time.now()
    goal_tb3_0.header.frame_id = "map"
    goal_tb3_0.pose.position.x = 5.0
    goal_tb3_0.pose.position.y = 0.0
    goal_tb3_0.pose.position.z = 0.0
    goal_tb3_0.pose.orientation.x = 0.0
    goal_tb3_0.pose.orientation.y = 0.0
    goal_tb3_0.pose.orientation.z = 0.0
    goal_tb3_0.pose.orientation.w = 1.0

    # Log info and publish the goal for tb3_0
    rospy.loginfo("Publishing goal to /tb3_0/move_base_simple/goal")
    goal_publisher_tb3_0.publish(goal_tb3_0)

    # Publisher for tb3_1's goal
    goal_publisher_tb3_1 = rospy.Publisher('/tb3_1/move_base_simple/goal', PoseStamped, queue_size=10)

    # Wait for the publisher to establish connection to subscribers
    rospy.sleep(1)

    # Create the goal message for tb3_1
    goal_tb3_1 = PoseStamped()
    goal_tb3_1.header.seq = 0
    goal_tb3_1.header.stamp = rospy.Time.now()
    goal_tb3_1.header.frame_id = "map"
    goal_tb3_1.pose.position.x = 3.0
    goal_tb3_1.pose.position.y = 0.0
    goal_tb3_1.pose.position.z = 0.0
    goal_tb3_1.pose.orientation.x = 0.0
    goal_tb3_1.pose.orientation.y = 0.0
    goal_tb3_1.pose.orientation.z = 0.0
    goal_tb3_1.pose.orientation.w = 1.0

    # Log info and publish the goal for tb3_1
    rospy.loginfo("Publishing goal to /tb3_1/move_base_simple/goal")
    goal_publisher_tb3_1.publish(goal_tb3_1)

    # Keep the script alive for a while
    rospy.sleep(3)

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass

