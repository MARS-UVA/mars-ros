#!/usr/bin/env python3
import rospy
from serial_manager import SerialManager
import traceback
from hero_board.msg import MotorVal
from hero_board.srv import GetState, GetStateResponse, SetState, SetStateRequest, SetStateResponse
from utils.protocol import var_len_proto_recv, var_len_proto_send, Opcode
import time
import struct


NODE_NAME = "hero_send_recv"
DIRECT_DRIVE_CONTROL_TOPIC = "/motor/output" # subscribing
AUTONOMY_CONTROL_TOPIC = "/motor/cmd_vel" # subscribing
HERO_FEEDBACK_TOPIC = "/motor/feedback" # publishing (data like motor currents and arm position read by the hero board)

NUM_MOTOR_CURRENTS = 8 # how many motor currents the hero sends back over serial. One byte corresponds to one motor current. 
# NUM_FLOATS = 1 # currently, 1 angle for bucket ladder angle
# NUM_BOOLS = 2 # currently, 2 bools total for bucket pressing upper and lower limit switch
NUM_FLOATS = 0
NUM_BOOLS = 0
EXPECTED_PACKET_LENGTH = NUM_MOTOR_CURRENTS + 4*NUM_FLOATS + NUM_BOOLS # currents and bools are 1 byte each, floats are 4 bytes each

manual_sub = None
auto_sub = None
current_state = SetStateRequest.IDLE

serial_manager = None


def stop_motors_non_emergency():
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, [100]*8)) # 100 is the neutral value

def process_manual_motor_values(motor_vals):
    m_val = motor_vals.motorval
    rospy.loginfo("writing direct_drive motor value: %s", list(m_val))
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, m_val))

def process_autonomy_motor_values(motor_vals):
    m_val = motor_vals.motorval
    rospy.loginfo('writing autonomy motor value: %s', list(m_val))
    serial_manager.write(var_len_proto_send(Opcode.AUTONOMY, m_val))


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
        manual_sub = rospy.Subscriber(DIRECT_DRIVE_CONTROL_TOPIC, MotorVal, process_manual_motor_values)
        return SetStateResponse('changing to manual control')

    elif to_change == SetStateRequest.AUTONOMY:
        rospy.loginfo('changing to drive state AUTONOMY')
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        stop_motors_non_emergency()
        auto_sub = rospy.Subscriber(AUTONOMY_CONTROL_TOPIC, MotorVal, process_autonomy_motor_values)
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
        pub = rospy.Publisher(HERO_FEEDBACK_TOPIC, MotorVal, queue_size=5)
        while not rospy.is_shutdown():
            # to_send_raw = ser.read(ser.inWaiting()) # I moved this line down after the if statement. Is that a problem?
            if pub.get_num_connections() == 0: # don't publish if there are no subscribers
                time.sleep(0.01)
                continue
            to_send_raw = serial_manager.read_in_waiting()
            to_send = var_len_proto_recv(to_send_raw)
            val = MotorVal()
            for packet in to_send:
                packet_opcode = packet[0]
                packet_data = packet[1]
                if packet_opcode != Opcode.FEEDBACK:
                    rospy.logwarn("got packet from hero with the wrong opcode, not publishing this packet")
                    continue
                if len(packet_data) != EXPECTED_PACKET_LENGTH:
                    rospy.logwarn("got packet from hero with an unexpected length, not publishing this packet (got {}, expected {})".format(len(packet_data), EXPECTED_PACKET_LENGTH))
                    continue
                # print("  sending packet data=" + str(packet_data))
                val.currents = packet_data[0:NUM_MOTOR_CURRENTS] # first X bytes
                # val.bucketLadderAngle = struct.unpack("%df"%NUM_FLOATS, packet_data[NUM_MOTOR_CURRENTS:(NUM_FLOATS*4)]) # (each float is 4 bytes and there is X of them)
                # val.depositBinRaised = (packet_data[-2] == 1) # second to last value
                # val.depositBinLowered = (packet_data[-1] == 1) # last value
                val.bucketLadderAngle = 45.0 
                val.depositBinRaised = False
                val.depositBinLowered = True
                pub.publish(val)
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    except Exception as e:
        traceback.print_exc()
    finally:
        serial_manager.close()
        exit(-1)
