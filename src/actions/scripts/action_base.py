from hero_board.srv import GetState, GetStateResponse
from hero_board.msg import HeroFeedback, MotorCommand
import rospy
import time
from std_msgs.msg import Int32
from actions.msg import DigitalFeedbackGpio

class ActionBase:

    def __init__(self, description):
        self.description = description

        rospy.wait_for_service("/get_state")
        self.get_state = rospy.ServiceProxy("/get_state", GetState)

        self.feedback_data = None
        self.ir_sub = rospy.Subscriber("/ir_adc_readings", Int32, self.ir_callback)
        self.gpio_sub = rospy.Subscriber("/gpio", DigitalFeedbackGpio, self.gpio_callback)
        self.motor_sub = rospy.Subscriber("/motor/feedback", HeroFeedback, self.motor_callback)

        # For now, all the actions publish motor commmands as direct drive commands, mainly 
        # because the hero board isn't set up for autonomy messages (messages that rely on feedback)
        self.pub = rospy.Publisher("/motor/cmd_vel", MotorCommand, queue_size=0) 

    def ir_callback(self, data):
        self.ir_data = data

    def gpio_callback(self, data):
        self.gpio_data = data
    
    def motor_callback(self, data):
        self.feedback_data = data
        # print("callback setting data to " + str(list(self.feedback_data.currents)))

    def is_running(self):
        if self.get_state().state != GetStateResponse.AUTONOMY:
            return False
        elif self.is_completed():
            return False

        return True

    def is_completed(self):
        # Determines whether the action is completed/satisfied/expired
        # Raising an error like this is a way to enforce virtual methods in Python
        raise NotImplementedError()

    def execute(self):
        # Publishes motor commands corresponding to the current robot's reported state
        raise NotImplementedError()

    def cleanup(self):
        msg = [100]*9
        self.pub.publish(MotorCommand(msg))

    def start(self):
        # print("action_base start()")
        if self.get_state().state != GetStateResponse.AUTONOMY:
            rospy.logwarn("Tried to start an action when not in autonomy mode! Ignoring action.")
            return

        while self.feedback_data is None:
            rospy.logwarn("Action hasn't received feedback data! Waiting for feedback before executing.")
            time.sleep(0.1)

        while self.is_running():
            # print("action looping...")
            self.execute()

        self.cleanup()

        self.motor_sub.unregister()
        rospy.loginfo("Action finished executing")
