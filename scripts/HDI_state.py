#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_feedback


def callback(msg):
   rospy.loginfo(rospy.get_name() + " received the following: wa: %f wf: %f v: %f" % (msg.wheel_angle, msg.wheel_force, msg.vibration))



def listener():
   rospy.init_node('HDI_feedback_listener')
   rospy.Subscriber("HDI_feedback", HDI_feedback, callback)
   rospy.spin()



if __name__ == '__main__':
   listener()
