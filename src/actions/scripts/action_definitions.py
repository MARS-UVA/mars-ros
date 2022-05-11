from action_base import ActionBase
from hero_board.msg import MotorCommand
import rospy
import time

class ActionRaiseBin(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action raisebin executing...")
        # msg = [100]*9 + [0] + [0]
        # self.pub.publish(MotorCommand(msg)) # TODO determine the correct message
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinRaised == True)


class ActionLowerBin(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action lowerbin executing...")
        # msg = [100]*9 + [2] + [0]
        # self.pub.publish(MotorCommand(msg)) # TODO determine the correct message
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinLowered == True)


class ActionRaiseLadder(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action raiseladder executing...")
        # Note that the angle decreases as the ladder is raised ("raised" meaning the ladder becoming vertical)
        # msg = [100]*9 + [2] + [0]
        # if self.feedback_data.bucketLadderAngleL > self.description["raised_angle"]:
        #     msg[0] = 0
        # if self.feedback_data.bucketLadderAngleR > self.description["raised_angle"]:
        #     msg[0] = 0
        # self.pub.publish(MotorCommand(msg)) # TODO determine the correct message
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        # a = self.description["raised_angle"]
        # return (self.feedback_data.bucketLadderAngleL < a) and (self.feedback_data.bucketLadderAngleR < a)
        return True


class ActionLowerLadder(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action lowerladder executing...")
        # msg = [100]*9 + [2] + [0]
        # self.pub.publish(MotorCommand(msg)) # TODO determine the correct message
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        # a = self.description["lowered_angle"]
        # return (self.feedback_data.bucketLadderAngleL < a) and (self.feedback_data.bucketLadderAngleR < a)
        return True
