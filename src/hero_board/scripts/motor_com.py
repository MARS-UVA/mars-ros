#!/usr/bin/env python
import rospy
import threading
import queue
import serial
from hero_board.msg import MotorVal
from utils.protocol import var_len_proto_send, var_len_proto_recv

# TODO Subscribes to motor commands (motor percent outputs) topics
# (published by) other nodes, encode them using our variable-length protocol,
# and send them to the Hero board
# TODO Receives data (e.g. motor current, IMU data) from the HERO,
# decode data using our variable-length protocol and publish them
# Note: the motor commands and motor currents will be an array of 7 bytes (uint8).
# In other words, they share the same ROS message structure but are interpreted
# differently


MOTOR_REC_NAME = "moto_commands"
MOTOR_COMMAND_PUB = "motor_pub"
QUEUE_SIZE = 100

motor_values_queue = queue.Queue()

should_terminate = threading.Event()


def send_commands_to_hero():
    try:

        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    except Exception:
        rospy.signal_shutdown()

    try:

        while not should_terminate.is_set():
            if(motor_values_queue.empty()):
                continue
            motor_val = motor_values_queue.get(block=True)
            rospy.loginfo(motor_val)
            ser.write(var_len_proto_send(motor_val))
    except Exception:
        ser.close()
        rospy.signal_shutdown()


def process_motor_values(motor_vals):
    '''takes in the MotorVal message as a parameter and sends the bytes
    to the serial thread'''
    motor_values_queue.put(motor_vals.motorval)


def motor_listener():
    rospy.init_node(MOTOR_REC_NAME, anonymous=True)
    rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)
    rospy.spin()


if __name__ == "__main__":
    hero_thread = threading.Thread(target=send_commands_to_hero)
    hero_thread.start()

    try:
        motor_listener()
    except KeyboardInterrupt as k:
        should_terminate.set()
    except Exception:
        should_terminate.set()
