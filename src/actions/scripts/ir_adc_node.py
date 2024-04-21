#!/usr/bin/env python3

# note: if you're getting a permissions error, you will need to give the user who starts this launchfile 
# permissions to read and write on the i2c port. See https://lexruee.ch/setting-i2c-permissions-for-non-root-users.html 

import rospy
import ctypes
import pylibi2c
from std_msgs.msg import Int32

pub = None

# bytes that are written to config register. These configure the conversion register
# to store data from channel 0 on the ADC
buf = bytes([0xC4,0x83])

def setup():
    # read from i2c bus 1 (pin 3: SDA, pin5: SCL)  on Jetson. 0x48 is the I2C addresss of the ADC
    global adc
    adc = pylibi2c.I2CDevice('/dev/i2c-1', 0x48) # initialize a I2C device

    # Set delay
    adc.delay = 10

    # Set page_bytes
    adc.page_bytes = 8

    # Set flags
    adc.flags = pylibi2c.I2C_M_IGNORE_NAK

    global pub
    pub = rospy.Publisher("ir_adc_readings", Int32, queue_size=0)

    # set config register
    adc.write(0x1, buf)
    
def publish_ir():
    global pub
    # From i2c 0x0(internal address) read 2 bytes of data. 
    data = adc.ioctl_read(0x0, 2)
    result = (data[0] << 8) | data[1] # combine the 2 bytes of data into a single integer to output
    
    pub.publish(result)
    rospy.loginfo(result)

if __name__ == "__main__":
    rospy.init_node("ir_adc_node", anonymous=True)
    rospy.loginfo("Starting IR")
    setup()
    rospy.loginfo("Passed setup")
    while not rospy.is_shutdown():
        publish_ir()
        rospy.spin()