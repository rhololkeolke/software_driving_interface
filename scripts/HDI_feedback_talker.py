#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_feedback

def talker():
   pub = rospy.Publisher('HDI_feedback', HDI_feedback)
   rospy.init_node('HDI_feedback_talker')
   rate = rospy.Rate(500) # 500 Hz

   while not rospy.is_shutdown():

      # Assign Message Values
      msg = HDI_feedback()
      msg.wheel_angle = 1.0 # radians
      msg.wheel_force = 0.0 # N-m
      msg.vibration = 0.0

      # Print to screen and log
      rospy.loginfo("wa: %f wf: %f v: %f" % (msg.wheel_angle, msg.wheel_force, msg.vibration))

      pub.publish(msg)
      rate.sleep()


if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
