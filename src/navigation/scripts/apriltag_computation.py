#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs # so that geometry_msgs are automatically converted to tf2_geometry_msgs
import numpy as np
import tf_conversions
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

rospy.init_node('apriltag_computation', anonymous=True)
apriltag_ids = rospy.get_param('~apriltag_ids').split()

tfBuffer = tf2_ros.Buffer()
lr = tf2_ros.TransformListener(tfBuffer)
br = tf2_ros.TransformBroadcaster()

nTfRetry = 1
retryTime = 0.05

def main():
    apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    rospy.sleep(1)
    rospy.spin()

## apriltag msg handling function
def apriltag_callback(data):
    # use apriltag pose detection to find where the robot is
    aggregated_base_map_transform = np.zeros(7)
    count = 0
    for detection in data.detections:
        if str(detection.id[0]) in apriltag_ids:
            count +=1 
            apriltag_frame_id = "tag_" + str(detection.id[0])
            poselist_tag_from_cam = [detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z, detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y, detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w]
            # transform the tag -> camera tf into a tag -> base tf
            poselist_tag_from_base = transformPose(poselist_tag_from_cam, 'usb_cam', 'robot_base')
            # calculate base -> tag
            poselist_base_tag = invPoselist(poselist_tag_from_base)
            # transform base -> tag into base -> map
            poselist_base_map = transformPose(poselist_base_tag, apriltag_frame_id, 'map')
            # publish base -> map
            aggregated_base_map_transform = np.add(aggregated_base_map_transform, poselist_base_map)
    if count > 0:
        base_map_transform = np.divide(np.array(aggregated_base_map_transform), count)
        pubFrame(pose = base_map_transform, frame_id = 'robot_base', parent_frame_id = 'map')
        
"""
HELPER FUNCTIONS
"""
def pubFrame(pose=[0,0,0,0,0,0,1], frame_id='obj', parent_frame_id='map', npub=1):
    if len(pose) == 7:
        ori = tuple(pose[3:7])
    elif len(pose) == 6:
        ori = tf_conversions.transformations.quaternion_from_euler(*pose[3:6])
    else:
        # bad length of pose
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