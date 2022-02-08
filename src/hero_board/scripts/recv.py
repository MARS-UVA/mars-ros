#!/usr/bin/env python3
import rospy
import traceback
from hero_board.msg import MotorVal
from hero_board.srv import GetState, GetStateResponse, SetState, SetStateRequest, SetStateResponse
from utils.protocol import var_len_proto_recv, var_len_proto_send
import time
import struct
from math import pi

MOTOR_REC_NAME = "motor_commands"
MOTOR_COMMAND_PUB = "/motor/output"
MOTOR_VOLT_NAME = "/motor/status"
AI_MOTOR_CONTROL = "/motor/cmd_vel"

manual_sub = None
auto_sub = None
current_state = SetStateRequest.MANUAL



def handle_state_request(req):
    global current_state
    return GetStateResponse(current_state)

    
def process_motor_values(motor_vals):
    '''
    takes in the MotorVal message as a parameter and sends the bytes
    to serial
    '''
    m_val = motor_vals.motorval
    rospy.loginfo("motor value manual: %s", m_val)
    

def process_auto_motor_values(motor_vals):
    m_val = motor_vals.motorval
    rospy.loginfo('motor value autonomy: %s', m_val)
    


def change_control(req):
    global auto_sub, manual_sub, current_state

    to_change = req.state
    if to_change == current_state:
        return SetStateResponse("no change in state")

    if to_change == SetStateRequest.MANUAL:
        print('changing to manual control')
        if auto_sub:
            auto_sub.unregister()
            auto_sub = None
        manual_sub = rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)
        return SetStateResponse('changing to manual control')

    elif to_change == SetStateRequest.AUTONOMY:
        print('changing to autonomy')
        if manual_sub:
            manual_sub.unregister()
            manual_sub = None
        
        auto_sub = rospy.Subscriber(AI_MOTOR_CONTROL, MotorVal, process_auto_motor_values)
        return SetStateResponse('changing to autonomy control')

    current_state = to_change

def control_server():
    serv = rospy.Service("/set_state", SetState, change_control)
    state_serv = rospy.Service("/get_state", GetState, handle_state_request)

if __name__ == "__main__":
    try:
        '''
        subscribes to the motor command publisher and passes the MotorVal
        message to the callback
        '''
        control_server()
        rospy.init_node(MOTOR_REC_NAME, anonymous=True)
        manual_sub = rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values, queue_size=1)
        
        rospy.loginfo("starting publisher")
        # no need to used a thread here. As per testing, main thread works fine
        # ros publisher queue can be used to limit the number of messages
        pub = rospy.Publisher(MOTOR_VOLT_NAME, MotorVal, queue_size=5)
        while not rospy.is_shutdown():
            motor_vals = ser.read(ser.inWaiting())
            if pub.get_num_connections() == 0: # don't publish if there're subscribers
                time.sleep(0.01)
                continue
            to_send = var_len_proto_recv(motor_vals)
            val = MotorVal()
            for x in to_send:
                val.motorval = x[:-8]
                val.angle, val.translation = struct.unpack("2f", x[-8:])
                pub.publish(val)
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    finally:
       
        exit(-1)
