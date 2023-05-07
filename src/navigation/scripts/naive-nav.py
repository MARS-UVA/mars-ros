#!/usr/bin/env python
# This code heavily inspired by Lab 3 (AprilTag Navigation)
# in Peter Yu's Sept 2016 Intro to Robotics Lab at MIT
# https://people.csail.mit.edu/peterkty/teaching/Lab3Handout_Fall_2016.pdf 

import rospy
import tf2_ros
import tf2_geometry_msgs # so that geometry_msgs are automatically converted to tf2_geometry_msgs
import numpy as np
import threading
import tf_conversions
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from hero_board.msg import MotorCommand
from apriltag_ros.msg import AprilTagDetectionArray

nTfRetry = 1
retryTime = 0.05

rospy.init_node('naive_navigator', anonymous=True)
tfBuffer = tf2_ros.Buffer()
lr = tf2_ros.TransformListener(tfBuffer)
br = tf2_ros.TransformBroadcaster()
debug_publisher = rospy.Publisher("/debug_messages", String, queue_size=1)
debug_msg = String()
    
def main():
    apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    rospy.sleep(1)
    
    constant_vel = False
    if constant_vel:
        thread = threading.Thread(target = constant_vel_loop)
    else:
        thread = threading.Thread(target = nav_loop)
    thread.start()
    
    rospy.spin()

## does nothing
def constant_vel_loop():
    # Create a publisher here
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown() :
        # for our robot, I will publish MotorCommand messages
        # for use in a simulation, I will publish Twist messages (at least, I think so -- would need to look into what turtlebot sim needs)
        
        rate.sleep() 

## apriltag msg handling function
def apriltag_callback(data):
    # use apriltag pose detection to find where the robot is
    for detection in data.detections:
        if 1 in detection.id:   # our apriltag ID is 1
            poselist_tag_cam = [detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z, detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w]

            # transform the tag -> camera tf into a tag -> base tf
            poselist_tag_base = transformPose(poselist_tag_cam, 'usb_cam', 'robot_base')
            # calculate base -> tag
            poselist_base_tag = invPoselist(poselist_tag_base)
            # transform base -> tag into base -> map
            poselist_base_map = transformPose(poselist_base_tag, 'tag_1', 'map')
            # publish base -> map
            pubFrame(pose = poselist_base_map, frame_id = 'robot_base', parent_frame_id = 'map')

## navigation control loop
def nav_loop():
    # create a publisher here to send motor commands
    target_pose2d = [0, 0, 0] # this will be at the origin of the map for now
    rate = rospy.Rate(10) # 100hz
    
    arrived = False
    arrived_position = False
    
    while not rospy.is_shutdown() :
        # get robot pose
        robot_pose3d = lookupTransform('map', 'robot_base')
        
        if robot_pose3d is None:
            debug_msg.data = "1. Tag not in view, Stop"
            debug_publisher.publish(debug_msg)
            # wcv.desiredWV_R = 0  # right, left
            # wcv.desiredWV_L = 0
            # velcmd_pub.publish(wcv)  
            rate.sleep()
            continue
        
        robot_position2d  = robot_pose3d[0:2]
        target_position2d = target_pose2d[0:2]
        
        robot_yaw    = tf_conversions.transformations.euler_from_quaternion(robot_pose3d[3:7]) [2]
        robot_pose2d = robot_position2d + [robot_yaw]
        
        # 2. navigation policy
        # 2.1 if       in the target, stop
        # 2.2 else if  close to target position, turn to the target orientation
        # 2.3 else if  in the correct heading, go straight to the target position,
        # 2.4 else     turn in the direction of the target position
        
        pos_delta         = np.array(target_position2d) - np.array(robot_position2d)
        robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
        heading_err_cross = cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
        
        # print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
        # print 'pos_delta', pos_delta
        # print 'robot_yaw', robot_yaw
        # print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d[2])
        # print 'heading_err_cross', heading_err_cross
        
        if arrived or (np.linalg.norm( pos_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
            debug_msg.data = "Case 2.1  Stop"
            debug_publisher.publish(debug_msg)
            # wcv.desiredWV_R = 0  
            # wcv.desiredWV_L = 0
            # arrived = True # for now, comment this out, because i want to keep doing stuff even if we get to the target once
        elif np.linalg.norm( pos_delta ) < 0.08:
            arrived_position = True
            if diffrad(robot_yaw, target_pose2d[2]) > 0:  
                debug_msg.data = "Case 2.2.1  Turn right slowly"
                debug_publisher.publish(debug_msg) 
                # wcv.desiredWV_R = -0.05 
                # wcv.desiredWV_L = 0.05
            else:
                debug_msg.data = "Case 2.1  Stop"
                debug_publisher.publish(debug_msg)
                # wcv.desiredWV_R = 0.05  
                # wcv.desiredWV_L = -0.05
                
        elif arrived_position or np.fabs( heading_err_cross ) < 0.2:
            debug_msg.data = "Case 2.3  Straight forward"
            debug_publisher.publish(debug_msg)
            # wcv.desiredWV_R = 0.1
            # wcv.desiredWV_L = 0.1
        else:
            if heading_err_cross < 0:
                debug_msg.data = "Case 2.4.1  Turn right"
                debug_publisher.publish(debug_msg)
                # wcv.desiredWV_R = -0.1
                # wcv.desiredWV_L = 0.1
            else:
                debug_msg.data = "Case 2.4.2  Turn left"
                debug_publisher.publish(debug_msg)
                # wcv.desiredWV_R = 0.1
                # wcv.desiredWV_L = -0.1
                
        # publish motor commands here 
        
        rate.sleep()

def cross2d(a, b):
    return a[0]*b[1] - a[1]*b[0]

def diffrad(a,b):
    diff = (a-b)
    while diff < -np.pi:
        diff += 2*np.pi
    while diff > np.pi:
        diff -= 2*np.pi
    return diff

def pubFrame(pose=[0,0,0,0,0,0,1], frame_id='obj', parent_frame_id='map', npub=1):
    if len(pose) == 7:
        ori = tuple(pose[3:7])
    elif len(pose) == 6:
        ori = tf_conversions.transformations.quaternion_from_euler(*pose[3:6])
    else:
        print('Bad length of pose')
        return None
    
    pos = tuple(pose[0:3])
    
    for j in range(npub):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = frame_id
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = ori[0]
        t.transform.rotation.y = ori[1]
        t.transform.rotation.z = ori[2]
        t.transform.rotation.w = ori[3]

        br.sendTransform(t)
        rospy.sleep(0.01)

# http://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28Python%29
def lookupTransform(sourceFrame, targetFrame):
    for i in range(nTfRetry):
        try:
            trans = tfBuffer.lookup_transform(sourceFrame, targetFrame, rospy.Time())
            transform_list = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            return transform_list
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(retryTime)
            continue
    return None

# https://answers.ros.org/question/323075/transform-the-coordinate-frame-of-a-pose-from-one-fixed-frame-to-another/ 
def transformPose(pose, fromFrame, toFrame):
    _pose = tf2_geometry_msgs.PoseStamped()
    _pose.header.frame_id = fromFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = tf_conversions.transformations.quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()
    
    _pose.pose.position.x = pose[0]
    _pose.pose.position.y = pose[1]
    _pose.pose.position.z = pose[2]
    _pose.pose.orientation.x = pose[3]
    _pose.pose.orientation.y = pose[4]
    _pose.pose.orientation.z = pose[5]
    _pose.pose.orientation.w = pose[6]
    
    for i in range(nTfRetry):
        try:
            t = rospy.Time.now()
            _pose.header.stamp = t
            _pose_target = tfBuffer.transform(_pose, toFrame, rospy.Duration(1))
            p = _pose_target.pose.position
            o = _pose_target.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except: 
            print('[transformPose] failed to transform targetFrame %s sourceFrame %s, retry %d' % (targetFrame, sourceFrame, i))
            rospy.sleep(retryTime)
            
    return None

def invPoselist(poselist):
    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(poselist)))

def xyzquat_from_matrix(matrix):
    return tf_conversions.transformations.translation_from_matrix(matrix).tolist() + tf_conversions.transformations.quaternion_from_matrix(matrix).tolist()

def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(tf_conversions.transformations.compose_matrix(translate=translate) ,
                   tf_conversions.transformations.quaternion_matrix(quaternion))

if __name__=='__main__':
    main()