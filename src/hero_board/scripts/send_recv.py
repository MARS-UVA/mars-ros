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
MANUAL_CONTROL_TOPIC = "/motor/output" # subscribing
AUTO_CONTROL_TOPIC = "/motor/cmd_vel" # subscribing
HERO_STATUS_TOPIC = "/motor/status" # publishing

manual_sub = None
auto_sub = None
current_state = SetStateRequest.MANUAL

serial_manager = None


def process_manual_motor_values(motor_vals):
    m_val = motor_vals.motorval
    rospy.loginfo("writing motor value manual: %s", list(m_val))
    serial_manager.write(var_len_proto_send(Opcode.DIRECT_DRIVE, m_val))

def process_auto_motor_values(motor_vals):
    m_val = motor_vals.motorval
    rospy.loginfo('writing motor value autonomy: %s', list(m_val))
    serial_manager.write(var_len_proto_send(Opcode.PID, m_val))


def get_state_service(req):
    global current_state
    return GetStateResponse(current_state)

def set_state_service(req):
    global auto_sub, manual_sub, current_state

    to_change = req.state
    if to_change == current_state:
        return SetStateResponse("no change in state")

    if to_change == SetStateRequest.MANUAL:
        rospy.loginfo('changing to manual control')
        if auto_sub:
            auto_sub.unregister()
            auto_sub = None
        manual_sub = rospy.Subscriber(MANUAL_CONTROL_TOPIC, MotorVal, process_manual_motor_values)
        return SetStateResponse('changing to manual control')

    elif to_change == SetStateRequest.AUTONOMY:
        rospy.loginfo('changing to autonomy control')
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        auto_sub = rospy.Subscriber(AUTO_CONTROL_TOPIC, MotorVal, process_auto_motor_values)
        return SetStateResponse('changing to autonomy control')

    current_state = to_change


if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        serial_manager = SerialManager(is_dummy=False)

        s_s_serv = rospy.Service("/set_state", SetState, set_state_service)
        g_s_serv = rospy.Service("/get_state", GetState, get_state_service)

        manual_sub = rospy.Subscriber(MANUAL_CONTROL_TOPIC, MotorVal, process_manual_motor_values, queue_size=1) # initialize the manual control subscriber because that's the state we start in
        
        # no need to used a thread here. As per testing, main thread works fine
        # ros publisher queue can be used to limit the number of messages
        rospy.loginfo("starting hero status publisher")
        pub = rospy.Publisher(HERO_STATUS_TOPIC, MotorVal, queue_size=5)
        while not rospy.is_shutdown():
            # to_send_raw = ser.read(ser.inWaiting()) # I moved this line down after the if statement. Is that a problem?
            if pub.get_num_connections() == 0: # don't publish if there are no subscribers
                time.sleep(0.01)
                continue
            to_send_raw = serial_manager.read_in_waiting()
            to_send = var_len_proto_recv(to_send_raw)
            val = MotorVal()
            for x in to_send:
                val.motorval = x[:-8] # everything except last 8
                val.angle, val.translation = struct.unpack("2f", x[-8:]) # last 8 (these values get reinterpreted as 2 floats)
                pub.publish(val)
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    except Exception as e:
        traceback.print_exc()
    finally:
        serial_manager.close()
        exit(-1)
