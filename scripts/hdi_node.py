#!/usr/bin/env python

import rospy
from software_driving_interface.msg import HDI_control, HDI_feedback
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

RATE = 100

def callback(msg):
    rospy.loginfo(rospy.get_name() + " received the following: wa %f wf: %f v: %f" % (msg.wheel_angle, msg.wheel_force, msg.vibration))
    packed_msg = struct.pack('<ddd', msg.wheel_angle, msg.wheel_force, msg.vibration)
    byte_list = map(ord, packed_msg)
    bus.write_i2c_block_data(ARDUINO_ADDRESS, 0, byte_list)

if __name__ == '__main__':
    rospy.init_node('hdi_node')
    pub = rospy.Publisher('HDI/cmd', HDI_control)
    rospy.Subscriber("HDI/state", HDI_feedback, callback)
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
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

        rospy.loginfo("wa: %f gp: %f bp: %f g: %d v: %f" % (msg.wheel_angle, msg.gas_pos, msg.brake_pos, msg.gear, msg.vibration))

        pub.publish(msg)
        rate.sleep()
