from action_base import ActionBase
from hero_board.msg import MotorCommand
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
8:
'''

# New msg format:
'''
0: front left wheel
1: front right wheel
2: back left wheel
3: back right wheel
4: raise/lower ladder - both actuator
5: rotate ladder chain
6: raise/lower collection bin floor
7: ir sensor servo motor
'''


IR_DITANCE_THRESHOLD  = 30 #Change later after measurement, distance in cm
IR_SERVO_ANGLE_HOP = 5  #Change later, in degrees for now
BUCKET_LIMIT_CONTACT_TIME_LOWER_THRESH = 0.5  #0.5 seconds - unit is in nanoseconds
BUCKET_LIMIT_CONTACT_TIME_UPPER_THRESH = 2  #2 seconds - unit is in nanoseconds

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

        msg = [100]*6 + [100 - self.description["speed"]]  #Add servos
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

        msg = [100]*6 + [100 + self.description["speed"]]  #Add servos
        self.pub.publish(MotorCommand(msg))
        time.sleep(self.description["update_delay"])

    def is_completed(self):
        return (self.feedback_data.depositBinLowered == True or self.gpio_data.construction_bin_contact == 1)


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
        msg = [100]*8
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

        msg = [100]*8
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
ActionDig class description -
IR sensor action description:
1) Run bucket ladder chain to dig
2) Once the chain runs, start IR scan
3) Once the IR sensor detects something within threshold, set a condition to complete
'''
class ActionDig(ActionBase):
    def __init__(self, description):
        #the dig action description has fields: name, update_delay, duration, speed
        super().__init__(description)
        self.initial_time = int(time.time()) #gets the time when the action was started
        self.last_bucket_contact_time = None
        self.ir_servo_angle = 0  #Initial angle - change to whatever starting reference angle is 
        self.stop_digging = False

    def execute(self):
        rospy.loginfo("action dig executing...")
        self.ir_servo_angle = (self.ir_servo_angle + IR_SERVO_ANGLE_HOP) % 180
        rospy.loginfo("")
        msg = [100]*8
        msg[5] = 100 - self["speed"]
        msg[7] = self.ir_servo_angle
        #self.pub.publish(MotorCommand(msg)) - commented out so buckets do not move during test
        time.sleep(self.description["update_delay"]) #delay for the specified amount of time before you update the motors
        self.check_sensors()

    def check_sensors(self):
        '''Checking IR sensor readings:'''
        rospy.loginfo("Scanning with IR sensor and limit switch status...")
        rospy.loginfo("IR sensor angle: %d" % self.ir_servo_angle)
        height = self.ir_data * sin(self.ir_servo_angle)
        rospy.loginfo("IR sensor data: %d" % self.ir_data)
        if height < IR_DITANCE_THRESHOLD:
            self.stop_digging = True
            rospy.loginfo("IR sensor detected something within threshold, stopping dig action")
        
        '''Checking bucket ladder limit switch readings:'''
        rospy.loginfo("Bucket Limit Switch status: %d" % self.gpio_data.bucket_contact)
        rospy.loginfo("Bin Limit Switch status: %d" % self.gpio_data.construction_bin_contact)
        if self.last_bucket_contact_time is None and self.gpio_data.bucket_contact == 1:
            self.last_bucket_contact_time = self.gpio_data.publish_timestamp
        elif self.normal_behavior:
            rospy.loginfo("Bucket ladder belt operating normally")
        else:
            rospy.loginfo("Bucket ladder belt operating abnormally, check if bucket may be jammed")


    '''Description: If bucket ladder switch contact within an allowable/normal operation period (0.5-2 seconds - may need to change period later)'''
    def normal_behavior(self):
        return (self.gpio_data.bucket_contact == 1 and 
                BUCKET_LIMIT_CONTACT_TIME_LOWER_THRESH <= 
                self.elapsed_since_last_contact <= 
                BUCKET_LIMIT_CONTACT_TIME_UPPER_THRESH)  

    def abnormal_behavior(self):  #This funcion is not usd as of yet but could be used to check whether buckets ar jammed
        return (self.gpio_data.bucket_contact == 0 and
                self.elapsed_since_last_contact > BUCKET_LIMIT_CONTACT_TIME_UPPER_THRESH)

    def elapsed_since_last_contact(self):
        seconds = self.gpio_data.publish_timestamp.sec - self.last_bucket_contact_time.sec
        nanosecs = self.gpio_data.publish_timestamp.nanosec - self.last_bucket_contact_time.nanosec
        elapsed_time = seconds + nanosecs/1e9
        return elapsed_time

    def is_completed(self):
        #the way we check that the action is completed is if we've been digging for the specified duration or if we've been told to stop digging
        current_time = time.time()
        return (current_time - self.initial_time >= self.description["duration"]) or self.stop_digging
