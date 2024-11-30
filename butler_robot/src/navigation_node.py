#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)

        # Action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Subscriber for navigation goals
        rospy.Subscriber('/navigation_goal', Pose, self.navigate_to_goal)

    def navigate_to_goal(self, pose_msg):
        goal = self.create_move_base_goal(pose_msg)
        rospy.loginfo("Sending goal to move_base.")
        self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        
        rospy.loginfo("Waiting for result...")
        self.client.wait_for_result()

    def create_move_base_goal(self, pose_msg):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal position and orientation
        goal.target_pose.pose = pose_msg
        return goal

    def done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn(f"Goal failed with status: {status}")

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Feedback: Current location: {feedback.base_position.pose.position}")
        
    def navigate_to_goal(self, pose_msg):
        goal = self.create_move_base_goal(pose_msg)
        rospy.loginfo("Sending goal to move_base.")
        self.client.send_goal(goal, done_cb=self.done_callback,     feedback_cb=self.feedback_callback)

        timeout = rospy.Duration(30.0)  # 30 seconds timeout
        if not self.client.wait_for_result(timeout):
            rospy.logwarn("Goal timed out. Returning to home position.")
            self.send_home_goal()

    def send_home_goal(self):
        home_goal = MoveBaseGoal()
        home_goal.target_pose.header.frame_id = "map"
        home_goal.target_pose.header.stamp = rospy.Time.now()
        home_goal.target_pose.pose.position.x = 0.0
        home_goal.target_pose.pose.position.y = 0.0
        home_goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(home_goal)
        self.client.wait_for_result()


if __name__ == "__main__":
    try:
        NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")

