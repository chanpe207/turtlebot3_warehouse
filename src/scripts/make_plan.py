#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def request_plan(start_x, start_y, goal_x, goal_y, frame_id="map", tolerance=0.5):
    rospy.wait_for_service('/move_base/make_plan')
    try:
        make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        
        start = PoseStamped()
        start.header.frame_id = frame_id
        start.pose.position.x = start_x
        start.pose.position.y = start_y
        start.pose.orientation.w = 1.0  # Neutral orientation
        
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0  # Neutral orientation
        
        plan = make_plan(start=start, goal=goal, tolerance=tolerance)
        
        if plan.plan.poses:
            rospy.loginfo("Plan received with %d poses.", len(plan.plan.poses))
            return plan.plan
        else:
            rospy.logwarn("Received an empty plan.")
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def publish_plan(plan, publisher):
    if plan:
        publisher.publish(plan)
    else:
        rospy.logwarn("No plan to publish.")

if __name__ == "__main__":
    rospy.init_node('plan_requester', anonymous=True)
    plan_publisher = rospy.Publisher('/visualization_plan', Path, queue_size=10)
    
    # Requesting a plan from (0, 0) to (15, 15)
    plan = request_plan(0.0, 0.0, 15.0, 15.0)
    
    if plan:
        publish_plan(plan, plan_publisher)
        rospy.loginfo("Published the plan for visualization.")
    else:
        rospy.loginfo("No plan was generated.")

