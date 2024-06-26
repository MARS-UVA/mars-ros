from action_base import ActionBase
from hero_board.msg import MotorCommand
from hero_board.msg import HeroFeedback
import rospy
import time
from math import sin


# Old msg format:
'''
0: raise/lower ladder
1:
2:
3:
4:
5:
6:
7: raise/lower collection floor
8:is_completed(
'''

# New msg format:
'''
0: front left wheel
1: front right wheel
2: back left wheel
3: back right wheel
4: raise/lower ladder - left actuator
5: rotate ladder chain
6: raise/lower collection bin floor
7: ir sensor servo motor
8: webcam servo
'''
WEBCAM_ANGLE = 0
IR_ANGLE = 0

IR_DITANCE_THRESHOLD  = 30
IR_SERVO_ANGLE_HOP = 5  #Change later, in degrees for now
CHAIN_CURRENT_THRESHOLD  = 40
SLIGHT_ROTATION = 2

class ActionRaiseBin(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action raisebin executing...")
        '''
        msg1 = [100]*8 + [100 + (0.15*self.description["speed"])]
        self.pub.publish(MotorCommand(msg1))
        time.sleep(2)

        msg2 = [100]*7 + [100 + self.description["speed"]] + [100 + (0.15*self.description["speed"])]
        self.pub.publish(MotorCommand(msg2))
        time.sleep(self.description["update_delay"])
        '''

        msg = [100]*6 + [100 - self.description["speed"]] + [IR_ANGLE, WEBCAM_ANGLE]
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinRaised == True)


class ActionLowerBin(ActionBase):
    def __init__(self, description):
        super().__init__(description)

    def execute(self):
        rospy.loginfo("action lowerbin executing...")
        '''
        msg1 = [100]*7 + [100 + self.description["speed"]] + [100]
        self.pub.publish(MotorCommand(msg1))
        time.sleep(self.description["update_delay"])

        msg2 = [100]*8 + [100 + (0.15*self.description["speed"])]
        self.pub.publish(MotorCommand(msg2))
        time.sleep(self.description["update_delay"])
        '''

        msg = [100]*6 + [100 + self.description["speed"]] + [IR_ANGLE, WEBCAM_ANGLE]
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinLowered == True)

class ActionDump(ActionBase):
    def __init__(self, description):
        super().__init__(description)
        # self.raise_bin_action = ActionRaiseBin(ActionBase)
        # self.lower_bin_action = ActionLowerBin(ActionBase)
        self.move_forward_completed = False
        self.lower_bin_completed = False
        self.dump_completed = False
        self.raise_bin_completed = False
        self.move_backward_completed = False
        self.initial_time = int(time.time())

    def execute(self):
        rospy.loginfo("action dump executing...")
        #moving forward and backward parts should be taken out once navigation is implemented
        
        if (not self.move_forward_completed):
            rospy.loginfo("moving forward...")
            msg = [100 + self.description["forward_speed"]]*4 + [100]*3 + [IR_ANGLE, WEBCAM_ANGLE]
            self.pub.publish(MotorCommand(msg))
            current_time = time.time()
            if (current_time - self.initial_time >= self.description["forward_duration"]):
                self.move_forward_completed = True
                self.initial_time = int(time.time())     
        
        elif (not self.lower_bin_completed):
            rospy.loginfo("lowering bin...")
            msg = [100]*6 + [100 + self.description["lower_bin_speed"]] + [IR_ANGLE, WEBCAM_ANGLE]
            self.pub.publish(MotorCommand(msg))
            current_time = time.time()
            if (current_time - self.initial_time >= self.description["lower_bin_duration"] or self.gpio_data.construction_bin_contact):
                self.lower_bin_completed = True
                self.initial_time = int(time.time()) 
                
        elif (not self.dump_completed):
            rospy.loginfo("dumping...")
            msg = [100]*7 + [IR_ANGLE, WEBCAM_ANGLE]
            self.pub.publish(MotorCommand(msg))
            current_time = time.time()
            if (current_time - self.initial_time >= self.description["dump_duration"]):
                self.dump_completed = True
                self.initial_time = int(time.time())
        
        elif (not self.raise_bin_completed):
            rospy.loginfo("raising bin...")
            msg = [100]*6 + [100 - self.description["raise_bin_speed"]] + [IR_ANGLE, WEBCAM_ANGLE]
            self.pub.publish(MotorCommand(msg))
            current_time = time.time()
            if (current_time - self.initial_time >= self.description["raise_bin_duration"]):
                self.raise_bin_completed = True
                self.initial_time = int(time.time()) 
            
        elif (not self.move_backward_completed):
            rospy.loginfo("moving backward...")
            msg = [100 - self.description["backward_speed"]]*4 + [100]*3 + [IR_ANGLE, WEBCAM_ANGLE]
            self.pub.publish(MotorCommand(msg))
            current_time = time.time()
            if (current_time - self.initial_time >= self.description["backward_duration"]):
                self.move_backward_completed = True
            
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.move_forward_completed and self.lower_bin_completed and self.raise_bin_completed and self.move_backward_completed)


class ActionRaiseLadder(ActionBase):
    def __init__(self, description):
        super().__init__(description)
        self.left_raised = False
        self.right_raised = False

        # if self.description["raised_angle"] > 51:
        #     self.left_raised = True
        #     self.right_raised = True

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

        # if self.description["lowered_angle"] < 10:
        #     self.left_lowered = True
        #     self.right_lowered = True

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

'''
IR sensor action description:
1) Run bucket ladder chain to dig
2) Once the chain runs, start IR scan
3) Once the IR sensor detects something within threshold, set a condition to complete
'''
class ActionDig(ActionBase):
    def __init__(self, description):
        super().__init__(description)
        rospy.loginfo("initializing dig action")
        self.initial_time = int(time.time()) #gets the time when the action was started
        self.ir_servo_angle = 0  #Initial angle - change to whatever starting reference angle is 
        #the dig action description has fields: name, update_delay, duration, speed
        self.stop_digging = False

    def execute(self):
        rospy.loginfo("action dig executing...")
        self.ir_servo_angle = (self.ir_servo_angle + IR_SERVO_ANGLE_HOP) % 180
        msg = [100]*9
        msg[5] = 100 + self.description["speed"]
        msg[7] = self.ir_servo_angle
        rospy.loginfo("dig sending command %s", list(msg))
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"]) #delay for the specified amount of time before you
        self.ir_scan()
        self.check_hitting_ebox()

    def ir_scan(self):
        #Control servo to spin to a certain point
        if(self.ir_data is not None):
            height = self.ir_data * sin(self.ir_servo_angle)
            if height < IR_DITANCE_THRESHOLD:
                self.stop_digging = True

    def check_hitting_ebox(self):
        msg = [100]*9
        rospy.loginfo("Bucket limit switch status: %d" % self.gpio_data.bucket_contact)
        if self.gpio_data.bucket_contact == 1:
            msg[4] = 100 + SLIGHT_ROTATION
        else:
            msg[4] = 100 - SLIGHT_ROTATION
            #msg[4] = 100 - self.description["speed"]
        self.pub.publish(MotorCommand(msg))

    def is_chain_stuck(self):
        return self.feedback_data.bucketLadderChainCurrent > CHAIN_CURRENT_THRESHOLD

    def is_completed(self):
        #the way we check that the action is completed is if we've been digging for the specified duration
        current_time = time.time()
        return (current_time - self.initial_time >= self.description["duration"]) or self.stop_digging or self.is_chain_stuck()
