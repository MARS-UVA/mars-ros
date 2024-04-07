import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO

pub = None
channel = 12 #GIOP pin number

def setup_node():
    global pub
    pub = rospy.Publisher("gpio", String, queue_size=0)
    rospy.init_node("gpio_read", anonymous = True)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(channel, GPIO.IN)

def publish():
    global pub
    gpio_state = GPIO.input(channel)
    pub.publish(str(gpio_state))
    rospy.loginfo("GPIO state: %s" % gpio_state)

if __name__ == "__main__":
    setup_node()
    while not rospy.is_shutdown():
        publish()
    rospy.spin()
    GPIO.cleanup()
