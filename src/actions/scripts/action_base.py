from hero_board.srv import GetState, GetStateResponse
from hero_board.msg import HeroFeedback, MotorCommand
import rospy
import time

class ActionBase:

    def __init__(self, description):
        self.description = description

        rospy.wait_for_service("/get_state")
        self.get_state = rospy.ServiceProxy("/get_state", GetState)

        self.feedback_data = None
        self.sub = rospy.Subscriber("/motor/feedback", HeroFeedback, self.callback)

        # For now, all the actions publish motor commmands as direct drive commands, mainly 
        # because the hero board isn't set up for autonomy messages (messages that rely on feedback)
        self.pub = rospy.Publisher("/motor/cmd_vel", MotorCommand, queue_size=0) 

    def callback(self, data):
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

        self.sub.unregister()
        rospy.loginfo("Action finished executing")
