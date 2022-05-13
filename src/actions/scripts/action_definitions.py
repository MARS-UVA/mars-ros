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

        if self.description["raised_angle"] > 51:
            self.left_raised = True
            self.right_raised = True

    def execute(self):
        rospy.loginfo("action raiseladder executing...")

        # Note that the measured angle increases as the ladder is raised ("raised" meaning the ladder becoming vertical)
        # The angle should be between around 10 and 52
        msg = [100]*9
        if not self.left_raised and self.feedback_data.bucketLadderAngleL > self.description["raised_angle"]:
            self.left_raised = True
        else:
            msg[4] = 100 + self.description["speed"]

        if not self.right_raised and self.feedback_data.bucketLadderAngleR > self.description["raised_angle"]:
            self.right_raised = True
        else:
            msg[4] = 100 - self.description["speed"]

        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return self.left_raised and self.right_raised


class ActionLowerLadder(ActionBase):
    def __init__(self, description):
        super().__init__(description)
        self.left_lowered = False
        self.right_lowered = False

        if self.description["lowered_angle"] < 10:
            self.left_lowered = True
            self.right_lowered = True

    def execute(self):
        rospy.loginfo("action lowerladder executing...")

        msg = [100]*9
        if not self.left_lowered and self.feedback_data.bucketLadderAngleL < self.description["lowered_angle"]:
            self.left_lowered = True
        else:
            msg[4] = 100 - self.description["speed"]

        if not self.right_lowered and self.feedback_data.bucketLadderAngleR < self.description["lowered_angle"]:
            self.right_lowered = True
        else:
            msg[4] = 100 + self.description["speed"]

        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return self.left_lowered and self.right_lowered

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
        self.initial_time = int(time.time()) #gets the time when the action was started
        #the dig action description has fields: name, update_delay, duration, speed

    def execute(self):
        rospy.loginfo("action dig executing...")
        msg = [100]*9
        msg[6] = 100 - self.description["speed"]
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"]) #delay for the specified amount of time before you

    def is_completed(self):
        #the way we check that the action is completed is if we've been digging for the specified duration
        current_time = time.time()
        return current_time - self.initial_time >= self.description["duration"]
