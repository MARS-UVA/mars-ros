from action_base import ActionBase
from hero_board.msg import MotorCommand
import rospy
import time

class ActionRaiseBin(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action raisebin executing...")
        msg = [100]*7 + [100 - self.description["speed"]] + [100]
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinRaised == True)


class ActionLowerBin(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action lowerbin executing...")
        msg = [100]*7 + [100 + self.description["speed"]] + [100]
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinLowered == True)


class ActionRaiseLadder(ActionBase):
    def __init__(self, description):
        super().__init__(description)
        self.left_raised = False
        self.right_raised = False

        if self.description["raised_angle"] > 9999: # TODO
            self.left_raised = True
            self.right_raised = True

    def execute(self):
        rospy.loginfo("action raiseladder executing...")

        # Note that the measured angle increases as the ladder is raised ("raised" meaning the ladder becoming vertical)
        # The angle should be between around TODO and TODO
        msg = [100]*9
        if self.feedback_data.bucketLadderAngleL > self.description["raised_angle"]:
            self.left_raised = True
        else:
            msg[0] = 100 + self.description["speed"]

        if self.feedback_data.bucketLadderAngleR > self.description["raised_angle"]:
            self.right_raised = True
        else:
            msg[0] = 100 - self.description["speed"]

        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return self.left_raised and self.right_raised


class ActionLowerLadder(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action lowerladder executing...")
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return True

# msg format:
'''
0: raise/lower ladder
1:
2:
3:
4:
5:
6:
7: raise/lower collection bin
8:
'''
class ActionDig(ActionBase):
    def __init__(self, description):
        super().__init__(description)
        self.initial_time = time.time() #gets the time when the action was started
        #the dig action description has three fields: name, update_delay, and duration

    def execute(self):
        rospy.loginfo("action dig executing...")
        # msg = [100]*7 + [100] + [100] #set these values based on which index of msg corresponds to the bucket ladder chain
        msg = [100]*9 # TODO
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"]) #delay for the specified amount of time before you

    def is_completed(self):
        #the way we check that the action is completed is if we've been digging for the specified duration
        current_time = time.time()
        return current_time - self.initial_time >= self.description["duration"]
