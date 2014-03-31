/*
 * testListener.h
 *
 *  Created on: Mar 21, 2014
 *      Author: democritus5589
 */

#ifndef _SDI_STATE_H_
#define _SDI_STATE_H_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "software_driving_interface/HDI_feedback.h"
//#include "driving_msgs/HDI_feedback.h"
#include <string>

using namespace std;

namespace sdi
{
   class SDI_Talker
   {
   public:
	   int run(int argc, char **argv);

   private:
      bool messageProcessed;
      vector<double> atlasPosition;
      vector<double> atlasLinearVelocity;
      vector<double> atlasAngularVelocity;
      double wheelAngle;
      double wheelForce;
      double vibration;

      software_driving_interface::HDI_feedback feedbackMsg;

      void setWheelAngle(const std_msgs::Float64::ConstPtr& msg);

      void setWheelForce(const std_msgs::Float64::ConstPtr& msg);

      void setVibration(const std_msgs::Int8::ConstPtr& msg);

      void logMessage(string name, const std_msgs::Int8::ConstPtr& msg);
      void logMessage(string name, const std_msgs::Float64::ConstPtr& msg);
      void logMessage(software_driving_interface::HDI_feedback& msg);//const software_driving_interface::HDI_feedback::ConstPtr& msg);
      void setVelocity(const nav_msgs::Odometry::ConstPtr& msg);
      void logMessage(const nav_msgs::Odometry::ConstPtr& msg);
      void setHandWheelForce();
      void fillFeedbackMsg();

   }; // end of SDI_Talker class
} // end of sdi namespace



#endif /* _TEST_LISTENER_H_ */
