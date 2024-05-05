import Jetson.GPIO as GPIO
import time
import rospy
from hero_board.msg import MotorCommand
from actions.msg import LadderAngle
import math

# later on we can change this code to account for both encoder pins

# Pin Definitions
leftActuator = 15 # pin 15 on Jetson, connected to one of the schmitt trigger outputs
rightActuator = 16 # pin 16 on Jetson, connected to another schmitt trigger output
highVoltagePulses = 0
pulsesToInches = 0.0022626 #convert to inches
# official specs say we get 17.4 pulses per mm of travel. This is equal to 0.0022626 inches per pulse
isExtending = False # If actuator is extending. Get this value from some other part of our code
pos = 0 # absolute position for the linear actuator in inches 
# 0 when not extended, 8 when fully extended
# for this code to work: start running linear actator from a fully retracted state
angle = 0 # angle of the ladder chain

PIVOT_TO_ACTUATOR_SIDE = 12 # distance from pivot point to the side of the actuator in inches
PIVOT_TO_BASE_SIDE = 13 # distance from pivot point to the side of the base in inches

def read_extending_status(data):
    global isExtending
    if data[4] > 100:
        isExtending = True
    elif data[4] < 100:
        isExtending = False

def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(leftActuator, GPIO.IN)
    GPIO.setup(rightActuator, GPIO.IN)
    global auto_motor_sub
    auto_motor_sub = rospy.Subscriber("/motor/cmd_vel", MotorCommand, read_extending_status)
    global direct_motor_sub
    direct_motor_sub = rospy.Subscriber("/motor/output", MotorCommand, read_extending_status)
    global angle_pub
    angle_pub = rospy.Publisher("/ladder_angle", LadderAngle, queue_size=1) 

    print("Starting demo now! Press CTRL+C to exit")
    # curr_value = GPIO.HIGH
    global highVoltagePulses
    global angle
    try:
        while True:
            #add to count every time a rising edge is detected
            GPIO.wait_for_edge(leftActuator, GPIO.RISING)
            highVoltagePulses += 1
            if highVoltagePulses % 10 == 0: # print count every 10 pulses
                # update position
                updatePosition()
                print("Position: ", pos)
    finally:
        GPIO.cleanup()

def updatePosition():
    global pos
    global highVoltagePulses
    global pulsesToInches
    if isExtending:
        pos += (highVoltagePulses * pulsesToInches)
        highVoltagePulses = 0
    else:
        pos -= (highVoltagePulses * pulsesToInches)
        highVoltagePulses = 0
    angle = math.acos((PIVOT_TO_ACTUATOR_SIDE^2 + PIVOT_TO_BASE_SIDE^2 - pos^2) / (2 * PIVOT_TO_ACTUATOR_SIDE * PIVOT_TO_BASE_SIDE))
    angle_pub.publish(angle)
# todo: a function used to calibrate the position value
# set position to 0 when actuator is fully retracted
# or set position to 8 when fully extedned

if __name__ == '__main__':
    rospy.init_node("ladder_actuator_read", anonymous=True)
    main()
    rospy.spin()