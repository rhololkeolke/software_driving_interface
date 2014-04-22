#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_feedback
import struct
from smbus import SMBus

ARDUINO_ADDRESS = 0x04
I2C_DEVICE = 1

bus = SMBus(I2C_DEVICE)

i2c_rate = None
RATE = 1

def callback(msg):
   rospy.loginfo(rospy.get_name() + " received the following: wa: %f wf: %f v: %f" % (msg.wheel_angle, msg.wheel_force, msg.vibration))
   packed_msg = struct.pack('<ddd', msg.wheel_angle, msg.wheel_force, msg.vibration)
   rospy.loginfo(rospy.get_name() + " packed message into %s" % packed_msg.encode('hex'))
   byte_list = map(ord, packed_msg)
   print byte_list
   bus.write_i2c_block_data(ARDUINO_ADDRESS, 0, byte_list)
   i2c_rate.sleep()



def listener():
   global i2c_rate
   rospy.init_node('HDI_feedback_listener')
   i2c_rate = rospy.Rate(RATE) 
   bus.write_quick(ARDUINO_ADDRESS)
   rospy.Subscriber("HDI/state", HDI_feedback, callback)
   rospy.spin()



if __name__ == '__main__':
   listener()
