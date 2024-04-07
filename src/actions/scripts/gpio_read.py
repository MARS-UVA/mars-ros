import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO

pub = None
def setup_node():
    global pub
    pub = rospy.Publisher("gpio", String, queue_size=0)
    rospy.init_node("gpio_read", anonymous = True)

def publish():
    global pub
    #pub.publish("dog")

if __name__ == "__main__":
    setup_node()
    while not rospy.is_shutdown():
        publish()
    rospy.spin()