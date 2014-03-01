#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_control

def talker():
   pub = rospy.Publisher('HDI/cmd', HDI_control)
   rospy.init_node('HDI_control_talker')
   rate = rospy.Rate(100) # 100 Hz

   while not rospy.is_shutdown():

      # Assign Message Values
      msg = HDI_control()
      msg.wheel_angle = 1.0 # radians
      msg.gas_pos = 0.4
      msg.brake_pos = 0.0
      msg.gear = 1 # drive
      msg.vibration = 0.0

      # Print to screen and log
      rospy.loginfo("wa: %f gp: %f bp: %f g: %d v: %f" % (msg.wheel_angle, msg.gas_pos, msg.brake_pos, msg.gear, msg.vibration))

      pub.publish(msg)
      rate.sleep()


if __name__ == '__main__':
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
