#!/usr/bin/env python3
import rospy
import serial
import time
import traceback

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
            except OSError as e: # OSError includes FileNotFoundError
                traceback.print_exc()
                # exit(-1)
                # TODO would it make more sense to crash everything instead of switching to dummy mode?
                print(e)
                rospy.logfatal("ERROR: could not open serial connection /dev/ttyUSB0 in serial_manager.py. Maybe the file permissions aren't right? (see permission.sh). Switching to dummy output mode. ")
                self.is_dummy = True
                self.ser = None
        
    def write(self, data):
        if self.is_dummy:
            rospy.loginfo("[DUMMY] serial_manager writing data: {}".format(data))
        else:
            self.ser.write(data)

    def read_in_waiting(self):
        print("sm read")
        if self.is_dummy:
            time.sleep(1)
            # emulate data received from the hero board. This data will get passed into var_len_proto_recv
            data = [0, 1, 2, 3, 4, 5, 6, 7] + [0]*8
            length = 0b11000000 | len(data)
            cs = (255 + sum(data) + length) % 256
            ret = [255] + [length] + data + [cs]
            # print("read dummy returning: ", ret)
            return bytes(ret)
        else:
            return self.ser.read(self.ser.inWaiting())

    def close(self):
        if self.is_dummy:
            pass
        else:
            self.ser.close()
