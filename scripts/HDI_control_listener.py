#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_control


def callback(msg):
   rospy.loginfo(rospy.get_name() + " received the following: wa: %f gp: %f bp: %f g: %d v: %f" % (msg.wheel_angle, msg.gas_pos, msg.brake_pos, msg.gear, msg.vibration))



def listener():
   rospy.init_node('HDI_control_listener')
   rospy.Subscriber("HDI_control", HDI_control, callback)
   rospy.spin()



if __name__ == '__main__':
   listener()
