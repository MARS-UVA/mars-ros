#!/usr/bin/env python3
# This code heavily inspired by Lab 3 (AprilTag Navigation)
# in Peter Yu's Sept 2016 Intro to Robotics Lab at MIT
# https://people.csail.mit.edu/peterkty/teaching/Lab3Handout_Fall_2016.pdf 

## THREADING: https://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread
# idea: terminate functions based on global autonomy_state var and call other function at end

# general imports
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import threading
import tf_conversions
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# imports from our ros workspace
from hero_board.msg import MotorCommand
from navigation.msg import AutonomyState
from actions.srv import StartAction, StartActionRequest

nTfRetry = 1 # number of retries if a tf operation fails
retryTime = 0.05
tag_refresh_time = 1 # in seconds
arrived_buffer_time = 5 # in seconds

autonomy_mode = True # todo back to False

rospy.init_node('naive_navigator', anonymous=True)
motor_command_mode = rospy.get_param('~motor_command_mode')
twist_mode = rospy.get_param('~twist_mode')
debug_mode = rospy.get_param('~debug_mode')
simulation_mode = rospy.get_param('~simulation_mode')

if simulation_mode:
    multiplier = -1
else:
    multiplier = 1

tfBuffer = tf2_ros.Buffer()
lr = tf2_ros.TransformListener(tfBuffer)

if debug_mode:
    debug_publisher = rospy.Publisher("/debug_messages", String, queue_size=1)
    debug_msg = String()
    
def main():
    global autonomy_mode
    autonomy_mode_sub = rospy.Subscriber("/autonomy_state", AutonomyState, autonomy_state_callback, queue_size=1)
    rospy.sleep(1)
    rospy.loginfo("STARTING NAV NODE")
    
    if not autonomy_mode:
        thread = threading.Thread(target = stall_loop, args=(lambda: autonomy_mode,))
    else:
        thread = threading.Thread(target = nav_loop, args=(lambda: autonomy_mode,))
    thread.start()

    rospy.spin()

## does nothing while we wait for autonomy mode to be toggled on
def stall_loop(stop_check):
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown() :
        if stop_check():
            break
        rate.sleep() 

    if not rospy.is_shutdown():
        instigate_thread(False)
        
## handles toggling autonomy mode on/off
def autonomy_state_callback(data):
    global autonomy_mode
    if data.naive:
        if debug_mode and not autonomy_mode:
            debug_msg.data = "Switching to autonomous mode"
        autonomy_mode = True
    else:
        if debug_mode and autonomy_mode:
            debug_msg.data = "Switching to non-autonomous mode"
        autonomy_mode = False
    debug_publisher.publish(debug_msg)

## determines which function to start (the stall loop or the navigation loop)
def instigate_thread(stall=True):
    global autonomy_mode
    if stall:
        thread = threading.Thread(target = stall_loop, args=(lambda: autonomy_mode,))
    else:
        thread = threading.Thread(target = nav_loop, args=(lambda: autonomy_mode,))
    thread.start()

## does the navigation
def nav_loop(autonomy_check):
    global motor_command_mode
    global twist_mode

    if motor_command_mode:
        command_publisher = rospy.Publisher("/motor/output", MotorCommand, queue_size=1)
        mc = MotorCommand()

    if twist_mode:
        twist_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        tw = Twist()

    target_pose2d = [0, 0, 0] # [x, y, yaw] this will be at the origin of the map for now
    rate = rospy.Rate(100) # 100hz
    
    arrived = False
    arrived_position = False
    arrived_time = None
    
    while not rospy.is_shutdown():
        if not autonomy_check(): # If we should no longer be doing autonomy
            # stop the robot
            if motor_command_mode:
                mc.values = np.multiply(np.array([100, 100, 100, 100, 100, 100, 100, 100, 100]), multiplier).tolist()
                command_publisher.publish(mc)
            if twist_mode:
                tw.linear.x = 0
                tw.linear.y = 0
                tw.linear.z = 0
                tw.angular.x = 0
                tw.angular.y = 0
                tw.angular.z = 0
                twist_publisher.publish(tw)
                # break out of this while loop
            break

        # get robot pose [*translation(x, y, z), *rotation_quat(x, y, z, w)]
        robot_pose3d = lookupTransform('map', 'robot_base') # we want to line up the *front* of the robot with the goal
        if robot_pose3d is None:
            if not arrived:
                if debug_mode:
                    debug_msg.data = "1. Tag not in view, turn right slowly until you see it again"
                    debug_publisher.publish(debug_msg)
                if motor_command_mode:
                    mc.values = np.multiply(np.array([120, 80, 120, 80, 100, 100, 100, 100, 100]), multiplier).tolist()
                    command_publisher.publish(mc)
                if twist_mode:
                    tw.linear.x = 0
                    tw.linear.y = 0
                    tw.linear.z = 0
                    tw.angular.x = 0
                    tw.angular.y = 0
                    tw.angular.z = 0.5 * multiplier
                    twist_publisher.publish(tw)
            rate.sleep()
            continue
        
        robot_position2d  = robot_pose3d[0:2] # translation.x, translation.y
        target_position2d = target_pose2d[0:2]

        delimiter = ", "
        list1 = [str(element) for element in robot_pose3d]
        list2 = [str(element) for element in target_position2d]
        list3 = [str(element) for element in robot_position2d]
        robot_3d_pose_str = delimiter.join(list1)
        target_2d_pose_str = delimiter.join(list2)
        robot_2d_pose_str = delimiter.join(list3)

        if debug_mode:
            debug_msg.data = "This is the robot 3d pos:" + robot_3d_pose_str
            debug_publisher.publish(debug_msg)
            debug_msg.data = "This is the target 2d pos:" + target_2d_pose_str
            debug_publisher.publish(debug_msg)
            debug_msg.data = "This is the robot 2d pos:" + robot_2d_pose_str
            debug_publisher.publish(debug_msg)
        
        robot_yaw    = tf_conversions.transformations.euler_from_quaternion(robot_pose3d[3:7]) [2]
        robot_pose2d = robot_position2d + [robot_yaw] # this is [translation.x, translation.y, translation.z, yaw]
        
        # 2. navigation policy
        # 2.1 if       in the target, stop
        # 2.2 else if  close to target position, turn to the target orientation
        # 2.3 else if  in the correct heading, go straight to the target position,
        # 2.4 else     turn in the direction of the target position
        
        # Get how far we are from the goal (x, y)
        position_delta         = np.array(target_position2d) - np.array(robot_position2d)
        # Get the direction in which the robot is currently facing (yaw)
        robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
        # Take the vector representing how far we are from the target position and convert to a unit vector
        pos_delta_unit_vec = position_delta / np.linalg.norm(position_delta)
        # Take cross product of current heading with correct position delta heading
        # to see how far off our current yaw is from the goal yaw (0 means yaws are equal)
        heading_err_cross = cross2d( robot_heading_vec, pos_delta_unit_vec)
        
        """
        if debug_mode:
            debug_msg.data = "robot yaw: " + str(robot_yaw) + ", target yaw: " + str(target_pose2d[2]) + ", difference in radians: " + str(diffrad(robot_yaw, target_pose2d[2]))
            debug_publisher.publish(debug_msg)
        """
        # print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
        # print 'pos_delta', pos_delta
        # print 'robot_yaw', robot_yaw
        # print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d[2])
        # print 'heading_err_cross', heading_err_cross
        
        # if robot is at the target position and facing the right direction
        if arrived or (np.linalg.norm( position_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
            if debug_mode:
                debug_msg.data = "Case 2.1  Stop"
                debug_publisher.publish(debug_msg)
            if motor_command_mode:
                mc.values = np.multiply(np.array([120, 80, 120, 80, 100, 100, 100, 100, 100]), multiplier).tolist()
            if twist_mode:
                tw.linear.x = 0
                tw.linear.y = 0
                tw.linear.z = 0
                tw.angular.x = 0
                tw.angular.y = 0
                tw.angular.z = 0

            # we will only mark the robot as "arrived" if it has been at the target position for a certain # of seconds
            # this is to prevent false readings from prematurely stopping the navigation
            if not arrived and arrived_time is not None: 
                now = rospy.Time.now()
                time_difference = now - arrived_time
                if time_difference.to_sec() > arrived_buffer_time:
                    if debug_mode:
                        debug_msg.data = "Arrived!"
                        debug_publisher.publish(debug_msg)
                    arrived = True
                    rospy.wait_for_service("/start_action")
                    start_action = rospy.ServiceProxy('/start_action', StartAction)
                    # go forward with twist message
                    # wait
                    # stop
                    # call raise ladder service
                    try:
                        request = StartActionRequest()
                        request.action_description_json = "{\
                            \"name\": \"raise_ladder\",\
                            \"update_delay\": 0.05,\
                            \"speed\": 30,\
                            \"raised_angle\": 52.0\
                        }"
                        if debug_mode:
                            debug_msg.data = "raising ladder"
                            debug_publisher.publish(debug_msg)
                        response = start_action(request)
                    except rospy.ServiceException as exc:
                        if debug_mode:
                            debug_msg.data = "Service did not process request: " + str(exc)
                            debug_publisher.publish(debug_msg)
                    # call raise bin
                    try:
                        request = StartActionRequest()
                        request.action_description_json = "{\
                            \"name\": \"raise_bin\",\
                            \"update_delay\": 0.05,\
                            \"speed\": 30\
                        }"
                        if debug_mode:
                            debug_msg.data = "raising bin"
                            debug_publisher.publish(debug_msg)
                        response = start_action(request)
                    except rospy.ServiceException as exc:
                        if debug_mode:
                            debug_msg.data = "Service did not process request: " + str(exc)
                            debug_publisher.publish(debug_msg)
                    rospy.sleep(20.0) # sleeps for 20 seconds
                    # call lower bin
                    try:
                        request = StartActionRequest()
                        request.action_description_json = "{ \
                            \"name\": \"lower_bin\",\
                            \"update_delay\": 0.05,\
                            \"speed\": 30,\
                            \"lowered_angle\": 40\
                        }"
                        if debug_mode:
                            debug_msg.data = "lowering bin"
                            debug_publisher.publish(debug_msg)
                        response = start_action(request)
                    except rospy.ServiceException as exc:
                        if debug_mode:
                            debug_msg.data = "Service did not process request: " + str(exc)
                            debug_publisher.publish(debug_msg)

            else:
                arrived_time = rospy.Time.now()

        # if robot is close to target position
        elif np.linalg.norm( position_delta ) < 0.08: # checks the magnitude of the distance to the target position
            arrived_position = True
            arrived_time = None
            if diffrad(robot_yaw, target_pose2d[2]) > 0:  
                if debug_mode:
                    debug_msg.data = "Case 2.2.1  Turn right slowly"
                    debug_publisher.publish(debug_msg) 
                if motor_command_mode:
                    mc.values = np.multiply(np.array([110, 90, 110, 90, 100, 100, 100, 100, 100]), multiplier).tolist()
                if twist_mode:
                    tw.linear.x = 0
                    tw.linear.y = 0
                    tw.linear.z = 0
                    tw.angular.x = 0
                    tw.angular.y = 0
                    tw.angular.z = 0.5 * multiplier
            else:
                if debug_mode:
                    debug_msg.data = "Case 2.2.2  Turn left slowly"
                    debug_publisher.publish(debug_msg)
                if motor_command_mode:
                    mc.values = np.multiply(np.array([90, 110, 90, 110, 100, 100, 100, 100, 100]), multiplier).tolist()
                tw.linear.x = 0
                tw.linear.y = 0
                tw.linear.z = 0
                tw.angular.x = 0
                tw.angular.y = 0
                tw.angular.z = -0.5 * multiplier

        # if neither of those cases worked, then we need to get to the right position!!
        # if robot has correct heading
        elif np.fabs( heading_err_cross ) < 0.2:
            arrived_time = None
            if debug_mode:
                debug_msg.data = "Case 2.3  Straight forward"
                debug_publisher.publish(debug_msg)
            if motor_command_mode:
                mc.values = np.multiply(np.array([140, 140, 140, 140, 100, 100, 100, 100, 100]), multiplier).tolist()
            tw.linear.x = -0.2 * multiplier
            tw.linear.y = 0
            tw.linear.z = 0
            tw.angular.x = 0
            tw.angular.y = 0
            tw.angular.z = 0

        # if robot does NOT have correct heading, then turn to the correct heading
        else:
            arrived_time = None
            if debug_mode:
                debug_msg.data = "This is the heading error:" + str(heading_err_cross)
                debug_publisher.publish(debug_msg)

            if heading_err_cross < 0: 
                # based on right hand rule, the cross product of robot heading X target heading will be negative if the robot heading is TO THE LEFT of the target
                if debug_mode:
                    debug_msg.data = "Case 2.4.1  Turn right" # therefore we would need to turn right
                    debug_publisher.publish(debug_msg)
                if motor_command_mode:
                    mc.values = np.multiply(np.array([140, 60, 140, 60, 100, 100, 100, 100, 100]), multiplier).tolist()
                if twist_mode:
                    tw.linear.x = 0
                    tw.linear.y = 0
                    tw.linear.z = 0
                    tw.angular.x = 0
                    tw.angular.y = 0
                    tw.angular.z = 1.0 * multiplier
            else:
                if debug_mode:
                    debug_msg.data = "Case 2.4.2  Turn left"
                    debug_publisher.publish(debug_msg)
                if motor_command_mode:
                    mc.values = [60, 140, 60, 140, 100, 100, 100, 100, 100] # TOFO: fix mc arrays. just multiplying by -1 won't do
                if twist_mode:
                    tw.linear.x = 0
                    tw.linear.y = 0
                    tw.linear.z = 0
                    tw.angular.x = 0
                    tw.angular.y = 0
                    tw.angular.z = -1.0 * multiplier
                
        # publish motor commands
        if motor_command_mode:
            command_publisher.publish(mc)
        if twist_mode:
            twist_publisher.publish(tw)

        rate.sleep()
    
    if not rospy.is_shutdown():
        instigate_thread()

"""
HELPER FUNCTIONS
"""

## Two dimensional cross product
def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

## Computes the difference between two angles in radians (from -pi to pi)
def diffrad(a,b):
    diff = (a-b)
    while diff < -np.pi:
        diff += 2*np.pi
    while diff > np.pi:
        diff -= 2*np.pi
    return diff

## Looks up an existing tf transform
## http://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28Python%29
def lookupTransform(sourceFrame, targetFrame):
    for i in range(nTfRetry):
        try:
            trans = tfBuffer.lookup_transform(sourceFrame, targetFrame, rospy.Time(0))
            now = rospy.Time.now()
            time_difference = now - trans.header.stamp
            if time_difference.to_sec() > tag_refresh_time: # if the last tag detection was too long ago
                continue
            transform_list = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            return transform_list
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(retryTime)
            continue
    return None

if __name__=='__main__':
    main()
