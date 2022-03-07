#!/usr/bin/env python3
import utils.protocol
import rospy
import serial

"""
This class wraps around the normal serial interface that send_recv.py uses. It allows using either
the normal serial connection, or instead writing all data to the console so that you can still run
this node without needing the hero board to be plugged in. 
"""


class SerialManager:

    def __init__(self, is_dummy):
        self.is_dummy = is_dummy

        if not self.is_dummy:
            try:
                self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
            except Exception as e:
                traceback.print_exc()
                # exit(-1)
                # TODO would it make more sense to crash instead of switching to dummy mode?
                rospy.logerr("ERROR: could not open serial connection /dev/ttyUSB0 in serial_manager.py. \
                    Maybe the file permissions aren't right? (see permission.sh). Switching to dummy output mode. ")
                self.ser = None
                self.is_dummy = True
        
    def write(self, data):
        if self.is_dummy:
            rospy.loginfo("[DUMMY] serial_manager writing data: {} (decoded={})".format(data, ",".join(protocol.var_len_proto_recv(data))))
        else:
            self.ser.write(data)

    def read_in_waiting(self):
        if self.is_dummy:
            # emulate data received from the hero board. This data will get passed into var_len_proto_recv
            data = [0, 1, 2, 3, 4, 5, 6, 7] + [0]*8
            length = 0b11000000 | len(data)
            cs = (sum(data) + length) % 255
            return [bytes([255] + [length] + data + [cs])]
        else:
            return self.ser.read(self.ser.inWaiting())

    def close(self):
        if self.is_dummy:
            pass
        else:
            self.ser.close()
