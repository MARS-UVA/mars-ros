#!/usr/bin/env python3

import rospy

pub = None

def setup_node():
    pub = rospy.Publisher("ir-adc", String, queue_size=0)
    rospy.init_node("ir-adc-node", anonymous=True)
    
def publish_ir():
    pub.publish("test")

if __name__ == "__main__":
    setup_node()
    while not rospy.is_shutdown():
        publish_ir()
    
    rospy.spin()
