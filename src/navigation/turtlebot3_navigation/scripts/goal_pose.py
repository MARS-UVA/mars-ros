#!/usr/bin/env python3

import rospy
import actionlib # service that sends goal commands to the move_base package
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Callbacks definition

# called when the goal that you've sent is being analyzed (path is being planned)
def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

# once path is generated and robot is traversing it, this feedback callback is called
def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

# three results that would end navigation
def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
    

rospy.init_node('goal_pose')

navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

# Example of navigation goal
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

# these numbers are arbitrary, but ofc they have to indicate a pose that the robot could reach
goal.target_pose.pose.position.x = -2.16
goal.target_pose.pose.position.y = 0.764
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = 0.662
goal.target_pose.pose.orientation.w = 0.750

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result() # hang here until we receive a result

if not finished:
    rospy.logerr("Action server not available!")
else:
    rospy.loginfo ( navclient.get_result())