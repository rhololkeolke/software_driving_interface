#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_control
import struct
from smbus import SMBus

ARDUINO_ADDRESS = 0x04
I2C_DEVICE = 1

WHEEL_ANGLE_ID = 0x00
GAS_POS_ID = 0x01
BRAKE_POS_ID = 0x02
GEAR_ID = 0x03
VIBRATION_ID = 0x04
KEY_TURNED_ID = 0x05

bus = SMBus(I2C_DEVICE)

i2c_rate = None
RATE = 10

def talker():
   global i2c_rate
   pub = rospy.Publisher('HDI/cmd', HDI_control)
   rospy.init_node('HDI_control_talker')
   rate = rospy.Rate(RATE) # 100 Hz

   while not rospy.is_shutdown():

      # Assign Message Values
      msg = HDI_control()
      for i in range(6):
         id = -1
         double_data = [0]*8
         int_data = -1
         try:
            raw_data = bus.read_i2c_block_data(ARDUINO_ADDRESS, 0)
            id = raw_data[0]
            double_data = [x << 4 | y for x,y in zip(raw_data[1:17:2], raw_data[2:17:2])]
            int_data = raw_data[1]
         except IOError:
            pass
         # print id
         # print int_data
         # print double_data
         if id == WHEEL_ANGLE_ID:
            msg.wheel_angle = struct.unpack('<d', ''.join(map(chr, double_data)))[0]
         elif id == GAS_POS_ID:
            msg.gas_pos = struct.unpack('<d', ''.join(map(chr, double_data)))[0]
         elif id == BRAKE_POS_ID:
            msg.brake_pos = struct.unpack('<d', ''.join(map(chr, double_data)))[0]
         elif id == GEAR_ID:
            msg.gear = int_data
         elif id == VIBRATION_ID:
            msg.vibration = struct.unpack('<d', ''.join(map(chr, double_data)))[0]

      # Print to screen and log
      rospy.loginfo("wa: %f gp: %f bp: %f g: %d v: %f" % (msg.wheel_angle, msg.gas_pos, msg.brake_pos, msg.gear, msg.vibration))

      pub.publish(msg)
      rate.sleep()


if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
