#!/usr/bin/env python3
from datetime import datetime
import math
import rospy
import serial
import time
import traceback
import numpy as np
from utils.protocol import var_len_proto_send, Opcode

"""
This class wraps around the normal serial interface that send_recv.py uses. It allows using either
the normal serial connection, or instead writing all data to the console so that you can still run
this node without needing the hero board to be plugged in. 
"""


class SerialManager:

    def __init__(self, is_dummy=False):
        self.is_dummy = is_dummy

        if not self.is_dummy:
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
                # self.ser = serial.Serial('/dev/pts/3', 115200, timeout=None) # Debugging option - virtual serial device: https://stackoverflow.com/questions/52187/virtual-serial-port-for-linux
            except OSError as e: # OSError includes FileNotFoundError
                traceback.print_exc()
                # exit(-1)
                # TODO would it make more sense to crash everything instead of switching to dummy mode?
                print(e)
                rospy.logfatal("Could not open serial connection /dev/ttyUSB0 in serial_manager.py. Maybe the file permissions aren't right? (try 'sudo chmod 666 /dev/ttyUSB0'). Switching to dummy output mode. ")
                self.is_dummy = True
                self.ser = None

    def write(self, data):
        if self.is_dummy:
            # rospy.loginfo("[DUMMY] serial_manager writing data: {}".format(data))
            pass
        else:
            self.ser.write(data)

    def read_in_waiting(self):
        if self.is_dummy:
            time.sleep(1)
            # Emulate data received from the hero board. This data will get passed into var_len_proto_recv
            # Lengths and count should correspond with the constants at the top of send_recv.py
            t = datetime.now().microsecond
            a1 = (math.sin(t//1500) + 1) * 45
            a2 = a1 + 1
            limit = 1 if (t % 8) > 4 else 0
            data = list(np.array([1.1*i for i in range(11)] + [a1, a2], np.float32).tobytes()) + [limit, limit]
            # data = list(np.array([1.1*i for i in range(11)], np.float32).tobytes()) + [0]*4*2 + [0, 1] # (just constant feedback)

            # For manually replicating the protocol encoding: 
            # length = 0b11000000 | len(data)
            # cs = (255 + sum(data) + length) % 256
            # ret = [255] + [length] + data + [cs]
            # return bytes(ret)
            return var_len_proto_send(Opcode.FEEDBACK, data)
        else:
            waiting = self.ser.inWaiting()
            while waiting == 0:
                time.sleep(0.01)
                waiting = self.ser.inWaiting()
            return self.ser.read(waiting)

    def close(self):
        if self.is_dummy:
            pass
        else:
            self.ser.close()
