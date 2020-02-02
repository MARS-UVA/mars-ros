#!/usr/bin/env python

import rospy
import threading
import queue
import serial
from hero_baord.msg import MotorVal
from utils.protocol import var_len_proto_recv


motor_signals = queue.Queue()
should_terminate = threading.Event()
def hero_recv():
    ser  = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) 
    while not rospy.is_shutdown():
        motor_vals = ser.read(ser.inWaiting())
        motor_signals.put(var_len_proto_recv(motor_vals))

def motor_pub():
    pub = rospy.Publisher('motor_volts', MotorVal, queue_size=10)
    rospy.init_node('motor_volts', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if motor_signals.empty():
            continue
        data = motor_signals.get()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()


if __name__=="__main__":
    recv_thread = threading.Thread(target=hero_recv)
    recv_thread.start()
    try:
        motor_pub()
    except rospy.ROSInterruptException:
        