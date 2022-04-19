#!/usr/bin/env python3
import rospy
import struct
import time
import traceback
from hero_board.msg import HeroFeedback, MotorCommand
from hero_board.srv import GetState, GetStateResponse, SetState, SetStateRequest, SetStateResponse
from serial_manager import SerialManager
from utils.protocol import var_len_proto_recv, var_len_proto_send, Opcode


NODE_NAME = "hero_send_recv"
DIRECT_DRIVE_CONTROL_TOPIC = "/motor/output" # subscribing
AUTONOMY_CONTROL_TOPIC = "/motor/cmd_vel" # subscribing
HERO_FEEDBACK_TOPIC = "/motor/feedback" # publishing (data like motor currents and arm position read by the hero board)

NUM_MOTOR_CURRENTS = 11 # how many motor currents the hero sends back over serial. One byte corresponds to one motor current. 
NUM_ANGLES = 2 # 2 angles for bucket ladder angle, one from each actuator
NUM_BOOLS = 2 # 2 bools total for bucket pressing upper and lower limit switch
EXPECTED_PACKET_LENGTH = 4*NUM_MOTOR_CURRENTS + 4*NUM_ANGLES + 1*NUM_BOOLS # currents and angles are 4 bytes each, bools are 1 byte each

manual_sub = None
auto_sub = None
current_state = SetStateRequest.IDLE

serial_manager = None


def stop_motors_non_emergency():
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, [100]*8)) # 100 is the neutral value

# Both of these subscriber functions take a MotorCommand ROS msg as a parameter, not anything related to RPC
def process_manual_motor_values(command): 
    vals = command.values
    rospy.loginfo("writing direct_drive motor value: %s", list(vals))
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, vals))

def process_autonomy_motor_values(command):
    vals = command.values
    rospy.loginfo("writing autonomy motor value: %s", list(vals))
    serial_manager.write(var_len_proto_send(Opcode.AUTONOMY, vals))


def get_state_service(req):
    global current_state
    # print("hero get_state_service... current_state={}".format(current_state))
    return GetStateResponse(current_state)

def set_state_service(req):
    global auto_sub, manual_sub, current_state
    # print("hero set_state_service... req.state={}, current_state={}".format(req.state, current_state))

    to_change = req.state
    if to_change == current_state:
        return SetStateResponse('no change in state')
    current_state = to_change

    if to_change == SetStateRequest.DIRECT_DRIVE:
        rospy.loginfo('changing to drive state DIRECT_DRIVE')
        if auto_sub:
            auto_sub.unregister()
            auto_sub = None
        manual_sub = rospy.Subscriber(DIRECT_DRIVE_CONTROL_TOPIC, MotorCommand, process_manual_motor_values)
        return SetStateResponse('changing to manual control')

    elif to_change == SetStateRequest.AUTONOMY:
        rospy.loginfo('changing to drive state AUTONOMY')
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        stop_motors_non_emergency()
        auto_sub = rospy.Subscriber(AUTONOMY_CONTROL_TOPIC, MotorCommand, process_autonomy_motor_values)
        return SetStateResponse('changing to autonomy control')
    
    elif to_change == SetStateRequest.IDLE:
        rospy.loginfo('changing to drive state IDLE')
        if auto_sub:
            auto_sub.unregister()
            auto_sub = None
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        stop_motors_non_emergency()
        return SetStateResponse('changing to drive state IDLE')
    
    else:
        rospy.logwarn('tried to change drive state to an unknown state')


if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        serial_manager = SerialManager(is_dummy=False)

        s_s_serv = rospy.Service("/set_state", SetState, set_state_service)
        g_s_serv = rospy.Service("/get_state", GetState, get_state_service)

        # no need to used a thread here. As per testing, main thread works fine
        # ros publisher queue can be used to limit the number of messages
        rospy.loginfo("starting hero status publisher")
        pub = rospy.Publisher(HERO_FEEDBACK_TOPIC, HeroFeedback, queue_size=5)
        while not rospy.is_shutdown():
            # to_send_raw = ser.read(ser.inWaiting()) # I moved this line down after the if statement. Is that a problem?
            if pub.get_num_connections() == 0: # don't publish if there are no subscribers
                time.sleep(0.01)
                continue
            to_send_raw = serial_manager.read_in_waiting()
            to_send = var_len_proto_recv(to_send_raw)
            val = HeroFeedback()
            for packet in to_send:
                packet_opcode = packet[0]
                packet_data = packet[1]
                if packet_opcode != Opcode.FEEDBACK:
                    rospy.logwarn("got packet from hero with the wrong opcode, not publishing this packet")
                    continue
                if len(packet_data) != EXPECTED_PACKET_LENGTH:
                    rospy.logwarn("got packet from hero with an unexpected length, not publishing this packet (got length {}, expected {})".format(len(packet_data), EXPECTED_PACKET_LENGTH))
                    continue

                # TODO verify the order of the floats and bools
                floats_combined = struct.unpack("%df"%(NUM_MOTOR_CURRENTS+NUM_ANGLES), packet_data[0:(NUM_MOTOR_CURRENTS*4 + NUM_ANGLES*4)]) # each float is 4 bytes
                val.currents = [max(0, min(255, int(c))) for c in floats_combined[0:NUM_MOTOR_CURRENTS]] # because the rpc message uses bytes instead of floats, convert 
                                                                                                # float to int and make sure it fits in one byte by clamping to range [0, 255]
                # val.currents = packet_data[0:NUM_MOTOR_CURRENTS] # first X bytes
                val.bucketLadderAngleL = floats_combined[NUM_MOTOR_CURRENTS + 0]
                val.bucketLadderAngleR = floats_combined[NUM_MOTOR_CURRENTS + 1]
                val.depositBinRaised = (packet_data[-2] != 0) # second to last value
                val.depositBinLowered = (packet_data[-1] != 0) # last value
                pub.publish(val)
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    except Exception as e:
        traceback.print_exc()
    finally:
        serial_manager.close()
        exit(-1)
