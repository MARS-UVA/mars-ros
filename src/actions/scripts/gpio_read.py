import rospy
import Jetson.GPIO as GPIO
from actions.msg import DigitalFeedbackGpio

pub = None

#Limit Switches:
channel_bucket = 12 
channel_bin = 16 

feedback = DigitalFeedbackGpio()

def setup_node():
    global pub
    pub = rospy.Publisher("gpio", DigitalFeedbackGpio, queue_size=0)
    rospy.init_node("gpio_read", anonymous = True)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(channel_bucket, GPIO.IN)
    GPIO.setup(channel_bin, GPIO.IN)

def publish():
    global pub
    gpio_state_bucket = GPIO.input(channel_bucket)
    gpio_state_bin = GPIO.input(channel_bin)
    feedback.bucket_contact = gpio_state_bucket
    #if gpio_state_bucket == 1:
    feedback.publish_timestamp = rospy.Time.now()
    feedback.construction_bin_contact = gpio_state_bin
    #if gpio_state_bin == 1:
    #feedback.construction_bin_raised_timestamp = rospy.Time.now()
    pub.publish(feedback)
    rospy.loginfo("bucket state: %s, bin state: %s" % (feedback.bucket_contact, feedback.construction_bin_contact))

if __name__ == "__main__":
    setup_node()
    while not rospy.is_shutdown():
        publish()
    rospy.spin()
    GPIO.cleanup()
