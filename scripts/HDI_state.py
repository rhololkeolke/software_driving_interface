#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_feedback
import struct
from smbus import SMBus

ARDUINO_ADDRESS = 0x04

bus = SMBus(ARDUINO_ADDRESS)


def callback(msg):
   rospy.loginfo(rospy.get_name() + " received the following: wa: %f wf: %f v: %f" % (msg.wheel_angle, msg.wheel_force, msg.vibration))
   packed_msg = struct.pack('<ddd', msg.wheel_angle, msg.wheel_force, msg.vibration)
   rospy.loginfo(rospy.get_name() + " packed message into %s" % packed_msg.encode('hex'))
   for byte in packed_msg:
      bus.write_byte(byte)



def listener():
   rospy.init_node('HDI_feedback_listener')
   rospy.Subscriber("HDI/state", HDI_feedback, callback)
   rospy.spin()



if __name__ == '__main__':
   listener()
