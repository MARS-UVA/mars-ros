#!/usr/bin/env python3
import math
import rospy
import struct
import time
import traceback
from actions.msg import DigitalFeedbackGpio
from hero_board.msg import HeroFeedback, MotorCommand
from hero_board.srv import GetState, GetStateResponse, SetState, SetStateRequest, SetStateResponse
from serial_manager import SerialManager
from utils.protocol import var_len_proto_recv, var_len_proto_send, Opcode
from geometry_msgs.msg import Twist
from enum import Enum


NODE_NAME = "hero_send_recv"
DIRECT_DRIVE_CONTROL_TOPIC = "/motor/output" # subscribing
AUTONOMY_TWIST_CONTROL_TOPIC = "/cmd_vel" # subscribing
AUTONOMY_MOTOR_CONTROL_TOPIC = "/motor/cmd_vel"
HERO_FEEDBACK_TOPIC = "/motor/feedback" # publishing (data like motor currents and arm position read by the hero board)
GPIO_FEEDBACK_TOPIC = "/gpio"

SAFE_CURRENT_THRESHOLD = 10.0

NUM_MOTOR_CURRENTS = 4 # how many motor currents the hero sends back over serial. One byte corresponds to one motor current. 
NUM_ACTUATOR_FEEDBACK_VALUES = 3 # 2 angles for bucket ladder angle, one from each actuator
NUM_BOOLS = 0 # 2 bools total for bucket pressing upper and lower limit switch

TOTAL_NUM_FLOATS = NUM_MOTOR_CURRENTS + NUM_ACTUATOR_FEEDBACK_VALUES
FEEDBACK_PACKET_LENGTH = 4*NUM_MOTOR_CURRENTS + 4*NUM_ACTUATOR_FEEDBACK_VALUES + 1*NUM_BOOLS # currents and angles are 4 bytes each, bools are 1 byte each

SPECIAL_PACKET_LENGTH = 1
PAYLOAD_PACKET_LENGTH = 4

class Special_signal(Enum):
    IR_START = 0xA0
    IR_STOP = 0xA1

manual_sub = None
auto_twist_sub = None
auto_actions_sub = None
current_state = SetStateRequest.IDLE

serial_manager = None

averaged_converted_angle_L = 45.0 # By setting these intial values to a constant, the readings will start off inaccurate
averaged_converted_angle_R = 45.0

bucket_chain_current = None
bucket_ladder_contact = False
construction_bin_contact = False

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#converts one potentiometer value to an angle. Can be used for either side of the bucket ladder, but assumes they have the same geometry
def convert_ladder_pot_to_angle(old_average, pot):
    # print("converting pot= {0:.4f}".format(pot))
    """
    Approximate potentiometer readings: top=0.362, bottom=0.160
    Bottom length (along frame): 16 in
    Bottom to top of actuator: 14.5
        added length when short: 1
        added length when long: 10

    Bucket ladder length: 31?
    """
    # To find the angle between the ladder and the actuator, use law of cosines:
    act_length = map(pot, 0.160, 0.362, 1.0, 10.0) + 14.5 # side c
    a = 31
    b = 17
    angle_deg = old_average
    try:
        cosc = (pow(a, 2) + pow(b, 2) - pow(act_length, 2)) / (2*a*b)
        angle_rad = math.acos(cosc)
        angle_deg = angle_rad * (180.0/3.14)
    except ValueError:
        rospy.logwarn("Failed converting angle with actuator reading=%f, skipping this reading" % pot)

    # angle_deg is now either the old average angle (if there was a math error) or the new angle just calculated
    # next, return and save a weighted average of the previous angle and the new angle. more weight is put on previous calculations

    new_average = 0.95 * old_average + 0.05 * angle_deg
    # print("act_length={:.3f}, pot={:.3f}, angle_deg={:.3f}".format(act_length, pot, new_average))
    return new_average

def stop_motors_non_emergency():
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, [100]*8)) # 100 is the neutral value

# Both of these subscriber functions take a MotorCommand ROS msg as a parameter
def process_manual_motor_values(command): 
    # command type is hero_board::MotorCommand
    vals = command.values

    # modify vals to provide some safety checks:

    # if the bucket ladder current spikes, slow bucket rotations
        # this comes from the serial info that we get from the HERO
    if bucket_chain_current > SAFE_CURRENT_THRESHOLD:
        vals[5] = 100
    # if the bucket ladder limit switch is hit, raise bucket ladder just a little instead of moving the ladder as intended
        # this comes from GPIO feedback
    #if bucket_ladder_contact:
       # vals[4] = 104
    # if the construction bin limit switch is hit and you're trying to open the bin, don't keep opening the bin
        # this comes from GPIO feedback
    #if construction_bin_contact and vals[6] > 100:
        #vals[6] = 100
    # if the bucket ladder is down, don't drive unless you're spinning
        # this comes from ladder angle feedback

    rospy.loginfo("writing 'direct_drive' values: %s", list(vals))
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, vals))

def rad_per_sec_to_motor_power(a):
    # The following equations account for a 100:1 gear ratio between the motors and the wheels
    # and were made using the rubber wheels on the old robot

    # This will have to be adjusted based on ground material, wheel size, robot dimensions, power source, etc. 

    # return 63.386 * (a / 6.28) + 10.161 # best fitting experimental equation (counting wheel revolutions per time), but has non-zero intercept which doesn't make sense
    # return 72.818 * (a / 6.28) # best fitting experimental equation with forced zero intercept
    return 90 * (a / 6.28) # adjusted equation based on actually driving the robot around

def process_autonomy_motor_values(command):
    # command type is geometry_msgs/Twist
    # see https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
    rospy.loginfo(command)
    linear = command.linear.x
    angular = command.angular.z
    b = 0.457 #width of vehicle base
    r = 0.127 #radius of wheels

    angular_left = (linear - angular * b / 2.0) / r
    angular_right = (linear + angular * b / 2.0) / r

    angular_left_scaled = 100 + int(rad_per_sec_to_motor_power(angular_left)) # add 100 because neutral motor value is 100 and not 0
    angular_right_scaled = 100 + int(rad_per_sec_to_motor_power(angular_right))

    vals = [100]*9
    vals[0] = angular_left_scaled
    vals[1] = angular_right_scaled
    vals[2] = angular_left_scaled
    vals[3] = angular_right_scaled

    # rospy.loginfo("writing 'autonomy' motor value: linear=%s, angular=%s, angular_left=%s, angular_left_scaled=%s, vals=%s", \
        # linear, angular, angular_left, angular_left_scaled, vals)
    rospy.loginfo("writing 'autonomy' values: %s", vals)
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, vals))


def get_state_service(req):
    global current_state
    # print("hero get_state_service... current_state={}".format(current_state))
    return GetStateResponse(current_state)

def set_state_service(req):
    global auto_twist_sub, auto_actions_sub, manual_sub, current_state
    # print("hero set_state_service... req.state={}, current_state={}".format(req.state, current_state))

    to_change = req.state
    if to_change == current_state:
        return SetStateResponse('no change in state')
    current_state = to_change

    if to_change == SetStateRequest.DIRECT_DRIVE:
        rospy.loginfo('changing to drive state DIRECT_DRIVE')
        if auto_twist_sub:
            auto_twist_sub.unregister()
            auto__twist_sub = None
        if auto_actions_sub:
            auto_actions_sub.unregister()
            auto_actions_sub = None
        manual_sub = rospy.Subscriber(DIRECT_DRIVE_CONTROL_TOPIC, MotorCommand, process_manual_motor_values)
        return SetStateResponse('changing to manual control')

    elif to_change == SetStateRequest.AUTONOMY:
        rospy.loginfo('changing to drive state AUTONOMY')
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        stop_motors_non_emergency()
        auto_twist_sub = rospy.Subscriber(AUTONOMY_TWIST_CONTROL_TOPIC, Twist, process_autonomy_motor_values)
        auto_actions_sub = rospy.Subscriber(AUTONOMY_MOTOR_CONTROL_TOPIC, MotorCommand, process_manual_motor_values)
        return SetStateResponse('changing to autonomy control')
    
    elif to_change == SetStateRequest.IDLE:
        rospy.loginfo('changing to drive state IDLE')
        if auto_twist_sub:
            auto_twist_sub.unregister()
            auto_twist_sub = None
        if auto_actions_sub:
            auto_actions_sub.unregister()
            auto_actions_sub = None
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        stop_motors_non_emergency()
        return SetStateResponse('changing to drive state IDLE')
    
    else:
        rospy.logwarn('tried to change drive state to an unknown state')

def save_gpio_feedback(feedback):
    bucket_contact = feedback.bucket_contact
    construction_bin_contact = feedback.construction_bin_contact

if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        serial_manager = SerialManager(is_dummy=False)

        s_s_serv = rospy.Service("/set_state", SetState, set_state_service)
        g_s_serv = rospy.Service("/get_state", GetState, get_state_service)

        # Temporary code to start the robot in autonomy mode:
        # rospy.loginfo('changing to drive state AUTONOMY')
        # current_state = SetStateRequest.AUTONOMY
        # auto_sub = rospy.Subscriber(AUTONOMY_CONTROL_TOPIC, Twist, process_autonomy_motor_values)

        # no need to used a thread here. As per testing, main thread works fine
        # ros publisher queue can be used to limit the number of messages
        rospy.loginfo("starting hero status publisher")
        pub = rospy.Publisher(HERO_FEEDBACK_TOPIC, HeroFeedback, queue_size=5)

        # 
        hero_feedback = rospy.Subscriber(GPIO_FEEDBACK_TOPIC, DigitalFeedbackGpio, save_gpio_feedback)

        in_ir_stream = False
        while not rospy.is_shutdown():
            # to_send_raw = ser.read(ser.inWaiting()) # I moved this line down after the if statement. Is that a problem?
            if pub.get_num_connections() == 0: # don't publish if there are no subscribers
                time.sleep(0.01)
                continue
            
            to_send_raw = serial_manager.read_in_waiting()
            to_send = var_len_proto_recv(to_send_raw) #0xff, opcode (00, 01, 10, 11), length of package
            val = HeroFeedback()

            for packet in to_send:

                packet_opcode = packet[0]
                packet_data = packet[1]
                if packet_opcode != Opcode.FEEDBACK:
                    rospy.logwarn("got packet from hero with the wrong opcode, not publishing this packet")
                    continue
                            
                if len(packet_data) == FEEDBACK_PACKET_LENGTH:
                    # rospy.loginfo("processing values")
                    # unpack bytes as encoded binary data -- in this case, the binary data is an array of floats
                    floats_combined = struct.unpack("%df"%(TOTAL_NUM_FLOATS), packet_data[0:(TOTAL_NUM_FLOATS*4)]) # each float is 4 bytes
                    # val.currents = [max(0, min(255, int(c*30.0))) for c in floats_combined[0:NUM_MOTOR_CURRENTS]] # because the rpc message uses bytes instead of floats, convert
                        
                    val.forwardLWheelCurrent = floats_combined[0]
                    val.rearLWheelCurrent = floats_combined[1]
                    val.forwardRWheelCurrent = floats_combined[2]
                    val.rearRWheelCurrent = floats_combined[3]

                    val.bucketLadderChainCurrent = floats_combined[4]
                    bucket_chain_current = val.bucketLadderChainCurrent

                    # angle calculation will need to change based on new robot dimensions
                    # averaged_converted_angle_L = convert_ladder_pot_to_angle(averaged_converted_angle_L, floats_combined[NUM_MOTOR_CURRENTS + 0])
                    # averaged_converted_angle_R = convert_ladder_pot_to_angle(averaged_converted_angle_R, floats_combined[NUM_MOTOR_CURRENTS + 1])
                    val.bucketLadderActuatorCurrent = floats_combined[5]
                    val.constructionBinActuatorCurrent = floats_combined[6]

                    pub.publish(val)
                    
                elif len(packet_data) == SPECIAL_PACKET_LENGTH:
                    if packet_data[0] == Special_signal.IR_START:
                        in_ir_stream = True
                        ir_readings = []
                    
                    if packet_data[0] == Special_signal.IR_STOP:
                        in_ir_stream = False
                        #-------------------output data to method-----------------------------
                        del ir_readings
                        
                elif len(packet_data) == PAYLOAD_PACKET_LENGTH:
                    if in_ir_stream:
                        ir_readings.append(packet_data)
                    
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    except Exception as e:
        traceback.print_exc()
    finally:
        serial_manager.close()
        exit(-1)
